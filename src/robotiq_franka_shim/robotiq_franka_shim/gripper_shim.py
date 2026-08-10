#!/usr/bin/env python3
"""franka_gripper-compatible shim in front of the Robotiq 2F-85.

Re-exposes the action servers that franka_ros2 provides for the Franka Hand and
forwards them to the ros2_control GripperActionController of the Robotiq driver,
so that teleop / data-collection / inference code written against the Franka Hand
keeps working unchanged after the hardware swap.

The gripper is driven as a binary open/close device: any commanded width above
`binary_threshold` opens, anything below closes. This matches a policy whose
gripper action is binary, and it sidesteps the nonlinear angle-to-opening
relationship of the 2F-85 four-bar linkage entirely.

Published state is expressed in Franka units (per-finger displacement, 0..0.04 m)
so that any observation vector built from it keeps the scale the policy was
normalized on.
"""

import threading

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState

try:
    from franka_msgs.action import Grasp, Homing, Move

    HAVE_FRANKA_MSGS = True
except ImportError:  # allow running on a machine without franka_ros2 installed
    HAVE_FRANKA_MSGS = False


class RobotiqFrankaShim(Node):
    def __init__(self):
        super().__init__("robotiq_franka_shim")

        self.declare_parameter("robotiq_action", "/robotiq_gripper_controller/gripper_cmd")
        self.declare_parameter("robotiq_joint", "robotiq_85_left_knuckle_joint")
        self.declare_parameter("open_position", 0.0)  # rad, 2F-85 fully open
        self.declare_parameter("closed_position", 0.7929)  # rad, 2F-85 fully closed
        self.declare_parameter("max_effort", 40.0)
        # Widths below are in Franka semantics (metres of total opening).
        self.declare_parameter("franka_max_width", 0.08)
        self.declare_parameter("binary_threshold", 0.04)
        # Below this measured opening after a grasp we assume nothing is held.
        self.declare_parameter("grasp_min_width", 0.002)
        self.declare_parameter("finger_joints", ["fr3_finger_joint1", "fr3_finger_joint2"])
        self.declare_parameter("command_timeout", 5.0)

        p = self.get_parameter
        self._robotiq_joint = p("robotiq_joint").value
        self._open_pos = p("open_position").value
        self._closed_pos = p("closed_position").value
        self._max_effort = p("max_effort").value
        self._max_width = p("franka_max_width").value
        self._threshold = p("binary_threshold").value
        self._grasp_min_width = p("grasp_min_width").value
        self._finger_joints = list(p("finger_joints").value)
        self._timeout = p("command_timeout").value

        self._theta = self._open_pos
        cb = ReentrantCallbackGroup()

        self._client = ActionClient(self, GripperCommand, p("robotiq_action").value, callback_group=cb)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self._state_pub = self.create_publisher(JointState, "/franka_gripper/joint_states", 10)
        self.create_timer(0.02, self._publish_state, callback_group=cb)

        ActionServer(
            self,
            GripperCommand,
            "/franka_gripper/gripper_action",
            self._on_gripper_command,
            callback_group=cb,
        )
        if HAVE_FRANKA_MSGS:
            ActionServer(self, Grasp, "/franka_gripper/grasp", self._on_grasp, callback_group=cb)
            ActionServer(self, Move, "/franka_gripper/move", self._on_move, callback_group=cb)
            ActionServer(self, Homing, "/franka_gripper/homing", self._on_homing, callback_group=cb)
        else:
            self.get_logger().warn("franka_msgs not found; only /franka_gripper/gripper_action is served")

        self.get_logger().info("Robotiq <-> franka_gripper shim ready (binary open/close)")

    # --- state -----------------------------------------------------------

    def _on_joint_states(self, msg: JointState):
        try:
            self._theta = msg.position[msg.name.index(self._robotiq_joint)]
        except ValueError:
            pass

    def _width(self) -> float:
        """Current opening, rescaled to the Franka 0..0.08 m range.

        Linear in the knuckle angle, which is an approximation of the real
        four-bar kinematics. Good enough for binary use: the value only has to be
        monotonic and land near the extremes. If you ever switch to continuous
        widths, replace this with a calibrated fit.
        """
        span = self._closed_pos - self._open_pos
        frac = (self._theta - self._open_pos) / span
        return max(0.0, min(1.0, 1.0 - frac)) * self._max_width

    def _publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._finger_joints
        msg.position = [self._width() / 2.0] * len(self._finger_joints)
        msg.velocity = [0.0] * len(self._finger_joints)
        msg.effort = [0.0] * len(self._finger_joints)
        self._state_pub.publish(msg)

    # --- driving the Robotiq --------------------------------------------

    def _drive(self, target_width: float) -> bool:
        """Send one binary open/close goal and block until the controller is done."""
        closing = target_width < self._threshold
        goal = GripperCommand.Goal()
        goal.command.position = self._closed_pos if closing else self._open_pos
        goal.command.max_effort = self._max_effort

        if not self._client.wait_for_server(timeout_sec=self._timeout):
            self.get_logger().error("Robotiq gripper action server unavailable")
            return False

        done = threading.Event()
        result_holder = {}

        def on_result(fut):
            result_holder["result"] = fut.result()
            done.set()

        def on_goal(fut):
            handle = fut.result()
            if not handle.accepted:
                done.set()
                return
            handle.get_result_async().add_done_callback(on_result)

        self._client.send_goal_async(goal).add_done_callback(on_goal)

        if not done.wait(timeout=self._timeout):
            self.get_logger().warn("Timed out waiting for the Robotiq controller")
            return False
        return "result" in result_holder

    # --- franka-facing action servers ------------------------------------

    def _on_gripper_command(self, handle):
        # franka_gripper interprets command.position as per-finger displacement.
        ok = self._drive(2.0 * handle.request.command.position)
        handle.succeed() if ok else handle.abort()
        result = GripperCommand.Result()
        result.position = self._width() / 2.0
        result.effort = self._max_effort
        result.stalled = False
        result.reached_goal = ok
        return result

    def _on_move(self, handle):
        ok = self._drive(handle.request.width)
        handle.succeed() if ok else handle.abort()
        result = Move.Result()
        result.success = ok
        return result

    def _on_grasp(self, handle):
        ok = self._drive(handle.request.width)
        # A successful grasp means the fingers stopped on an object rather than
        # closing all the way, so a non-zero remaining opening is the signal.
        grasped = ok and self._width() > self._grasp_min_width
        handle.succeed() if grasped else handle.abort()
        result = Grasp.Result()
        result.success = grasped
        if ok and not grasped:
            result.error = "gripper closed fully, no object detected"
        return result

    def _on_homing(self, handle):
        # The 2F-85 auto-calibrates on activation; a full open is the closest
        # equivalent and avoids dropping whatever is held during a re-activation.
        ok = self._drive(self._max_width)
        handle.succeed() if ok else handle.abort()
        result = Homing.Result()
        result.success = ok
        return result


def main():
    rclpy.init()
    node = RobotiqFrankaShim()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
