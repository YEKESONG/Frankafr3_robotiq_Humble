# robotiq_franka_shim

FR3 上把 Franka Hand 换成 Robotiq 2F-85 的最小集成，面向**关节空间 + 二值夹爪动作**的栈。

目标平台：**Ubuntu 22.04 / ROS 2 Humble**。

两个部分：

- `urdf/robotiq_2f_85_minimal.xacro` —— 加载串口驱动的 `<ros2_control>` 块，外加两个空 link 的桩，
  让 `robot_state_publisher` 不刷警告。没有 mesh、没有 mimic 关节、没有碰撞体：这套栈里没有任何消费者会读它们。
- `robotiq_franka_shim/gripper_shim.py` —— 对外提供 `franka_gripper` 的 action 接口，对内转发到
  Robotiq 的 `GripperActionController`。这样遥操、数据采集、推理这些原本针对 Franka Hand 写的代码一行都不用改。

## 集成步骤

1. 在 FR3 的 xacro 里，把 `franka_hand` 宏替换为：

   ```xml
   <xacro:include filename="$(find robotiq_franka_shim)/urdf/robotiq_2f_85_minimal.xacro"/>
   <xacro:robotiq_2f_85_minimal parent="fr3_link8" com_port="/dev/robotiq"/>
   ```

2. 把 `config/robotiq_gripper_controllers.yaml` 合并进 FR3 的 `ros2_control_node` 已经加载的参数文件，
   然后 spawn 两个控制器：

   ```bash
   ros2 run controller_manager spawner robotiq_gripper_controller -c /controller_manager
   ros2 run controller_manager spawner robotiq_activation_controller -c /controller_manager
   ```

3. 启动 shim：`ros2 run robotiq_franka_shim gripper_shim`

4. 给 FR3 设置负载 —— **URDF 不负责这件事**。2F-85 加原厂转接法兰约 1.1 kg，
   而 Franka Hand 是 0.73 kg，不设的话重力补偿不对、碰撞阈值会误触发：

   ```bash
   ros2 service call /service_server/set_load franka_msgs/srv/SetLoad "{mass: 1.1, ...}"
   ```

   （franka_ros2 的 Humble 分支上服务名可能不同，用 `ros2 service list | grep load` 确认。）

## 接口语义对照

| | Franka Hand | 2F-85 | shim 的处理 |
|---|---|---|---|
| 指令 | 单指位移 0~0.04 m | 关节角 0~0.7929 rad | 以 0.04 m 开口为阈值二值化 |
| 方向 | 越大越开 | 越大越**闭** | 反向 |
| 状态输出 | `fr3_finger_joint{1,2}` | `robotiq_85_left_knuckle_joint` | 换算回 Franka 量纲后重新发布 |

发布的开口宽度**刻意保持在 Franka 的 0~0.08 m 刻度上**：如果 observation 向量里含夹爪状态，
请沿用**训练时的** normalization 统计量，不要用新数据重算，否则 policy 会直接 OOD。

装了 franka_ros2 时 shim 提供 `/franka_gripper/` 下的 `grasp`、`move`、`homing`、`gripper_action`
四个 action；没装 `franka_msgs` 时只提供 `gripper_action`，节点降级运行而不是报错退出。

## 上电验证顺序

1. `use_fake_hardware:=true` —— 只验证控制器链路，不碰串口。
2. `<param name="use_dummy">true</param>` —— 走驱动内置的 fake backend。
3. 接实物，先裸测串口：`robotiq_hardware_tests` 里的 `gripper_interface_test`。
4. `ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.7929, max_effort: 40.0}}"`
5. 同样的动作走 shim：`ros2 action send_goal /franka_gripper/grasp ...`
6. GELLO 遥操 —— 确认扳机方向没有反。
7. VLA 推理，降速运行，手放在急停上。

## 参数调节

xacro 里的初值给得保守：`gripper_force_multiplier:=0.20`（约 47 N；2F-85 最大 235 N，
而 Franka Hand 只有约 70 N）、`gripper_speed_multiplier:=0.25`，
目的是让闭合耗时接近 policy 训练时见到的时序。

## Humble 特有的注意点

- 控制器类型是 `position_controllers/GripperActionController`（来自 `gripper_controllers` 包）。
  Jazzy 以后改名为 `parallel_gripper_action_controller/GripperActionController`。
- Humble 的 GripperActionController 会强制 claim position **和** velocity 两个状态接口，
  所以 xacro 里的 `<state_interface name="velocity"/>` 不能删，尽管驱动从不更新它。
- 上游 `ros2_robotiq_gripper` 必须用 `humble` 分支，`main` 在 Humble 上编不过。
