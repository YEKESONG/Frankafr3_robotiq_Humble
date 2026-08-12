#!/usr/bin/env python3
"""端到端自检：发 GELLO 的百分比话题，看夹爪关节角是否跟随。

覆盖的正是最容易搞反/搞错的两件事：
  1. 方向 —— GELLO 松开(1.0) 必须对应张开(0 rad)，捏紧(0.0) 对应闭合(0.7929 rad)
  2. 话题命名空间对不对得上 —— client 订阅的相对话题是否落在 GELLO 实际发布的绝对话题上

用法（先在另一个终端把 robotiq_teleop.launch.py 起起来）：

    python3 scripts/test_gripper_chain.py            # 默认 left
    python3 scripts/test_gripper_chain.py right      # 换臂命名空间

fake 模式下 mock_components 会立刻把命令回读成状态，四项应当全 PASS。
接实物时闭合那一步可能停在物体上而到不了 0.7929，属正常（2F-85 夹住即停）。
"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

JOINT = "robotiq_85_left_knuckle_joint"
CLOSED_RAD = 0.7929
TOLERANCE = 0.02


class Probe(Node):
    def __init__(self, arm_ns):
        super().__init__("robotiq_chain_probe")
        self.command_topic = f"/{arm_ns}/gripper/gripper_client/target_gripper_width_percent"
        self.state_topic = f"/{arm_ns}/gripper/joint_states"
        self.pub = self.create_publisher(Float32, self.command_topic, 10)
        self.pos = None
        self.create_subscription(JointState, self.state_topic, self._on_state, 10)

    def _on_state(self, msg):
        if JOINT in msg.name:
            self.pos = msg.position[msg.name.index(JOINT)]

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive(self, percent, label, expected):
        self.pub.publish(Float32(data=percent))
        self.spin(3.0)
        ok = self.pos is not None and abs(self.pos - expected) < TOLERANCE
        print(
            f"  percent={percent:<4} ({label:<6}) -> {JOINT}={self.pos} "
            f"期望~{expected:.4f}  {'PASS' if ok else 'FAIL'}"
        )
        return ok


def main():
    arm_ns = sys.argv[1] if len(sys.argv) > 1 else "left"
    rclpy.init()
    node = Probe(arm_ns)
    print(f"命令话题: {node.command_topic}")
    print(f"状态话题: {node.state_topic}\n等待 joint_states...")
    node.spin(5.0)
    if node.pos is None:
        print(f"FAIL: {node.state_topic} 上没有数据。夹爪 launch 起来了吗？")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print(f"初始 {JOINT}={node.pos}")
    results = [
        node.drive(0.0, "闭合", CLOSED_RAD),
        node.drive(1.0, "张开", 0.0),
        node.drive(0.0, "闭合", CLOSED_RAD),
        node.drive(0.5, "半开", CLOSED_RAD / 2),
    ]
    node.destroy_node()
    rclpy.shutdown()
    print("\nALL PASS" if all(results) else "\nSOME FAILED")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
