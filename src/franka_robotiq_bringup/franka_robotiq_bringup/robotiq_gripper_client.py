#!/usr/bin/env python3
"""Bridge the GELLO gripper command topic to the Robotiq GripperActionController.

Sits where ``franka_umdc_gripper_client`` sat in the Franka Hand pipeline: it
consumes the very same ``std_msgs/Float32`` that ``franka_gello_state_publisher``
already publishes, so nothing upstream of it has to change when the Franka Hand
is swapped for a Robotiq 2F-85.

Input semantics (from ``gello_publisher.py``): the value is a *percent open*,
``input_open_value`` when the trigger is released and ``input_closed_value`` when
it is squeezed. The publisher binarises it, but this node does not rely on that:
it normalises and clamps, so a continuous input degrades gracefully into a
continuous gripper opening.

Output semantics: ``control_msgs/GripperCommand`` in *knuckle joint radians*,
where 0.0 is fully open and ``gripper_closed_position`` (0.7929 rad) is fully
closed -- i.e. the opposite direction from the input, hence the ``1 - frac``.

The upstream client this replaces sent ``1 - percent`` straight through as if the
percent were already radians. That both ignored the configured closed position
and, at full close, commanded 1.0 rad against a joint whose URDF upper limit is
0.7929 rad.
"""

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float32

DEFAULT_COMMAND_TOPIC = "gripper_client/target_gripper_width_percent"
DEFAULT_ACTION_NAME = "robotiq_gripper_controller/gripper_cmd"


class RobotiqGripperClient(Node):
    def __init__(self):
        super().__init__("robotiq_gripper_client")

        self.declare_parameter("gripper_command_topic", DEFAULT_COMMAND_TOPIC)
        self.declare_parameter("gripper_action", DEFAULT_ACTION_NAME)
        # 2F-85 knuckle angle at the extremes, in radians.
        self.declare_parameter("gripper_open_position", 0.0)
        self.declare_parameter("gripper_closed_position", 0.7929)
        # Range of the incoming percent, matching gello_publisher's
        # gripper_{open,closed}_output.
        self.declare_parameter("input_open_value", 1.0)
        self.declare_parameter("input_closed_value", 0.0)
        self.declare_parameter("max_effort", 40.0)
        # Ignore command changes smaller than this, in normalised open fraction.
        # The GELLO publisher emits a binary 0/1, so anything below 1.0 suppresses
        # only duplicate messages -- it exists for the continuous-input case.
        self.declare_parameter("command_deadband", 0.02)
        # 0 or below means wait forever for the controller to come up.
        self.declare_parameter("server_wait_timeout", 0.0)

        p = self.get_parameter
        self._topic = p("gripper_command_topic").value
        self._action_name = p("gripper_action").value
        self._open_pos = float(p("gripper_open_position").value)
        self._closed_pos = float(p("gripper_closed_position").value)
        self._input_open = float(p("input_open_value").value)
        self._input_closed = float(p("input_closed_value").value)
        self._max_effort = float(p("max_effort").value)
        self._deadband = float(p("command_deadband").value)
        self._server_timeout = float(p("server_wait_timeout").value)

        if abs(self._input_open - self._input_closed) < 1e-9:
            raise ValueError("input_open_value and input_closed_value must differ")

        self._last_frac = None
        self._goal_in_flight = False

        self._action_client = ActionClient(self, GripperCommand, self._action_name)
        self._wait_for_controller()

        self.create_subscription(Float32, self._topic, self._on_command, 10)
        self.get_logger().info(
            f"Robotiq gripper client ready: '{self._topic}' -> '{self._action_name}' "
            f"(open={self._open_pos:.4f} rad, closed={self._closed_pos:.4f} rad, "
            f"max_effort={self._max_effort:.1f})"
        )

    def _wait_for_controller(self) -> None:
        """Block until the GripperActionController is up.

        Spawning the controller and this node happen concurrently, so on a cold
        start the server is legitimately absent for a few seconds. Logging while
        waiting beats a silent hang.
        """
        if self._server_timeout > 0.0:
            if not self._action_client.wait_for_server(timeout_sec=self._server_timeout):
                raise RuntimeError(
                    f"Gripper action server '{self._action_name}' did not come up "
                    f"within {self._server_timeout:.1f}s"
                )
            return

        while not self._action_client.wait_for_server(timeout_sec=5.0):
            if not rclpy.ok():
                raise RuntimeError("Shut down while waiting for the gripper action server")
            self.get_logger().warning(
                f"Waiting for gripper action server '{self._action_name}'..."
            )

    def _to_open_fraction(self, raw: float) -> float:
        span = self._input_open - self._input_closed
        return max(0.0, min(1.0, (raw - self._input_closed) / span))

    def _to_joint_position(self, open_fraction: float) -> float:
        # open_fraction 1.0 -> open_pos, 0.0 -> closed_pos.
        return self._open_pos + (1.0 - open_fraction) * (self._closed_pos - self._open_pos)

    def _on_command(self, msg: Float32) -> None:
        frac = self._to_open_fraction(float(msg.data))

        if self._last_frac is not None and abs(frac - self._last_frac) < self._deadband:
            return
        # A goal already on the wire means the previous target is still being
        # driven. Dropping the new one would strand the gripper at a stale
        # target, so let it through: the controller preempts its own goal.
        self._last_frac = frac

        goal = GripperCommand.Goal()
        goal.command.position = self._to_joint_position(frac)
        goal.command.max_effort = self._max_effort

        self.get_logger().debug(
            f"raw={msg.data:.3f} -> open_frac={frac:.3f} -> "
            f"position={goal.command.position:.4f} rad"
        )
        self._goal_in_flight = True
        self._action_client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._goal_in_flight = False
            self.get_logger().error(f"Failed to send gripper goal: {exc}")
            return

        if not handle.accepted:
            self._goal_in_flight = False
            # Never raise from a callback: it would tear down the executor and
            # leave the gripper stuck at whatever it was last told to do.
            self.get_logger().warning("Gripper goal rejected by the controller")
            # Forget the last command so the next identical one is not deadbanded
            # away -- otherwise a single rejection could wedge the gripper.
            self._last_frac = None
            return

        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        self._goal_in_flight = False
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Gripper goal failed: {exc}")
            self._last_frac = None
            return

        # The 2F-85 stalls by design when it grasps an object, so `stalled` is
        # the normal outcome of a successful grasp, not an error.
        self.get_logger().debug(
            f"position={result.position:.4f} stalled={result.stalled} "
            f"reached_goal={result.reached_goal}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RobotiqGripperClient()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
