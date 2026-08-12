# franka_robotiq_22.04

GELLO → Franka FR3 单臂遥操，末端用 **Robotiq 2F-85** 取代 Franka Hand。

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green)

[English](README.md) | **简体中文**

---

## 概述

完整遥操所需的一切都在这一个工作空间里，不依赖任何其他 ROS 工作空间。三路并行：

| 路 | 包 | 职责 |
| --- | --- | --- |
| 主手 | `franka_gello_state_publisher` | 读 GELLO 的 Dynamixel，发布 `/left/gello/joint_states` 和二值夹爪百分比 `/left/gripper/gripper_client/target_gripper_width_percent` |
| 机械臂 | `franka_fr3_arm_controllers` | 关节阻抗控制器跟随 GELLO 关节，经 `franka_hardware` 走 FCI 驱动 FR3 |
| 夹爪 | `franka_robotiq_bringup` | 把百分比换算成 `control_msgs/GripperCommand`，由自己的 `ros2_control` 经 Modbus RTU / USB-RS485 驱动 |

夹爪自己开一个 `controller_manager`，和臂那一路彻底解耦：改夹爪不用重启臂，也不用动
FR3 的 xacro（臂的配置本来就是 `load_gripper: "false"`）。

## 环境要求

**软件** —— Ubuntu 22.04，ROS 2 Humble。

**硬件**
- Franka FR3，已开启 FCI（默认 `172.16.0.3`）
- Robotiq 2F-85，24 V 供电 + USB-RS485 转换器
- GELLO 主手，Dynamixel U2D2 / OpenRB-150 转换器

## 安装

```bash
git clone https://github.com/YEKESONG/Frankafr3_robotiq_Humble.git ~/Desktop/franka_robotiq_22.04
cd ~/Desktop/franka_robotiq_22.04

# 系统依赖（一次性）
sudo apt install -y python3-vcstool \
    ros-humble-gripper-controllers ros-humble-libfranka \
    ros-humble-franka-description ros-humble-franka-msgs \
    ros-humble-joint-state-publisher ros-humble-joint-state-broadcaster \
    ros-humble-moveit-core ros-humble-moveit-msgs
pip install dynamixel_sdk pyserial

# 拉上游 + 屏蔽无关包 + 编译（幂等，可反复跑）
./scripts/setup_workspace.sh
```

上游仓库在 `fr3_robotiq.humble.repos` 里按 commit 锁定，用 `vcs import` 拉取，不 fork
也不打补丁。`franka_ros2` 只编 `franka_hardware`、`franka_msgs`、
`franka_semantic_components`、`franka_robot_state_broadcaster` 四个包，其余打
`COLCON_IGNORE` 屏蔽。

每个新终端：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/franka_robotiq_22.04/install/setup.bash
```

> **务必在干净环境里编译。** colcon 会把构建时的 `AMENT_PREFIX_PATH` 烧进
> `install/setup.sh`。如果 `.bashrc` 自动 source 了别的 franka/gello 工作空间，本工作空间
> 就会反过来链到它们身上。source 之后 `echo $AMENT_PREFIX_PATH | tr : "\n"` 里除了
> `/opt/ros/humble` 和本仓库不应有别的路径。

## 使用

### 一键启动

```bash
./scripts/start_single_arm_robotiq.sh
```

弹出一个 terminator 窗口，三个标签（GELLO / 机械臂 / 夹爪）。机械臂标签会先等
`/left/gello/joint_states` 真正开始发布，再等你按**空格**才启动控制器 —— 臂是力矩控制，
没有有效目标就启动会下坠。按空格前请把 GELLO 摆到与机械臂当前位姿接近的位置。

### 分级验证

出问题时按顺序做，每一级只引入一个新变量。

```bash
# 1. 只验控制器链路，不碰串口、不接硬件
ros2 launch franka_robotiq_bringup robotiq_teleop.launch.py config_file:=fake_left_robotiq.yaml
python3 scripts/test_gripper_chain.py          # 应当 4/4 PASS

# 2. 裸测 Modbus RTU，绕开整个 ROS —— 接实物后第一件事
python3 scripts/probe_robotiq_serial.py        # 任何带 ★ 的行都说明夹爪应答了

# 3. 驱动的内置 dummy 后端（验插件加载和接口 claim）
ros2 launch franka_robotiq_bringup robotiq_gripper.launch.py use_dummy:=true

# 4. 接实物，不经 GELLO 直接打 action
ros2 launch franka_robotiq_bringup robotiq_teleop.launch.py config_file:=single_left_robotiq.yaml
ros2 action send_goal /left/gripper/robotiq_gripper_controller/gripper_cmd \
    control_msgs/action/GripperCommand "{command: {position: 0.7929, max_effort: 40.0}}"
```

### 数据采集

`record_*_lerobot.py` 不需要任何改动。这些脚本的夹爪维度取自 GELLO 的指令话题再二值化，
读的不是真实夹爪反馈，所以换硬件不改变数据格式和语义。

## 配置

### 串口

```bash
ls -l /dev/serial/by-id/
```

**永远用 `/dev/serial/by-id/...` 路径，不要用 `/dev/ttyUSBn`。** GELLO 的转换器也是 FTDI
设备，两者的 ttyUSB 编号会随插拔顺序互换。把查到的路径填进
`src/franka_robotiq_bringup/config/single_left_robotiq.yaml` 的 `com_port`。
需要 dialout 组权限：`sudo usermod -aG dialout $USER`。

### 夹爪参数

都在 `config/single_left_robotiq.yaml` 里：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `gripper_force_multiplier` | `0.20` | 占 2F-85 最大 235 N 的比例，约 47 N；被替换的 Franka Hand 约 70 N |
| `gripper_speed_multiplier` | `0.75` | 占最大 150 mm/s 的比例 |
| `input_open_value` / `input_closed_value` | `1.0` / `0.0` | 必须与 `gello_publisher` 的 `gripper_open_output` / `gripper_closed_output` 一致 |
| `max_effort` | `40.0` | 每条 `GripperCommand` goal 带的力上限 |

`namespace` 必须是 `<臂命名空间>/gripper`（例如 `left/gripper`），否则 client 的指令话题
解析不到 GELLO 发布器发出的那个。

## 目录结构

```
franka_robotiq_22.04/
├── fr3_robotiq.humble.repos          # 上游依赖，锁 commit
├── scripts/
│   ├── setup_workspace.sh            # 拉依赖 + 屏蔽无关包 + 编译
│   ├── start_single_arm_robotiq.sh   # 遥操一键启动
│   ├── test_gripper_chain.py         # 端到端自检：方向 + 话题命名空间
│   └── probe_robotiq_serial.py       # 裸 Modbus RTU 诊断（绕开 ROS）
└── src/
    ├── franka_robotiq_bringup/       # 自有包：夹爪这一路
    ├── franka_gello_state_publisher/ # 从 Franka GELLO 集成拷入，含本机标定值
    ├── franka_fr3_arm_controllers/   # 从 Franka GELLO 集成拷入，含本机臂配置
    ├── ros2_robotiq_gripper/         # 上游，vcs import，无改动
    ├── serial/                       # 上游，vcs import，无改动
    └── franka_ros2/                  # 上游 v2.3.0，vcs import，无改动
```

两个 GELLO 包随仓库走而不是 vcs 拉取，因为它们的 config 里是本机标定出来的关节符号、
装配偏置、夹爪量程和机械臂 IP —— 属于本地配置而非上游代码。

## 排障

**先看夹爪的 LED，它一句话就能把排查范围切一半：**

| LED | 含义 |
| --- | --- |
| 熄灭 | 无 24 V。注意 USB-RS485 转换器吃 USB 电，夹爪没电时它照样枚举出 `/dev/ttyUSB*` |
| 常亮红 | 已上电但通信不通 —— 电源不用查了，问题在数据链路 |
| 常亮蓝 | 上电且通信正常 |
| 红蓝闪烁 | 启动中，或有故障需重新激活 |

跑 `probe_robotiq_serial.py` 时红灯也是正常的：探针只读状态寄存器，从不写激活位。
**判据只看有没有 ★，不要看 LED。** 红转蓝要等 ROS 驱动启动时执行激活
（日志里的 `Robotiq Gripper successfully activated!`）。

**日志里同时出现 `Controller already loaded` 和 `no controller with this name exists`**
不是竞态，是上一次的 `ros2_control_node` 还活着 —— Ctrl-C 打在 `ros2 launch` 上有时只杀
掉外壳：

```bash
pkill -9 -f lib/controller_manager/ros2_control_node
pkill -9 -f lib/franka_robotiq_bringup/robotiq_gripper_client
```

<details>
<summary><b>RS-485 不通（夹爪红灯 + 全程无应答）</b></summary>

先跑 `scripts/probe_robotiq_serial.py` 把软件侧排干净 —— 它会扫 5 种波特率 × 3 种校验位、
从站地址 1–247、RTS 方向控制和总线空闲电平。这些全部无应答，就只剩物理层。
按可能性从高到低：

1. **没有共地（散线接法最容易漏）。** RS-485 是差分信号，但仍需要一个共同的 0V 参考。
   典型错误是夹爪的黑线（0V）只接到电源适配器，转换器这一侧的 GND 悬空、仅通过 USB
   接到开发机的地。把转换器的 GND 接到夹爪黑线 0V 的同一个点 —— 只接 A/B 两根不够。

   ⚠️ **接了一根叫 "G" 的线不等于共地。** 法兰散线里往往还有一根屏蔽/裸线，外观很像地线，
   但它在夹爪那一侧可能根本没接到 0V（屏蔽层通常只在一端接地）。用万用表电阻档量这根线
   和黑色 0V 线之间的通断：接近 0 Ω 说明确实是 0V；开路说明它是屏蔽层，当信号地用等于没接。

2. **A/B 接反。** 不同厂商对 A/B 的定义相反，看丝印猜不可靠。直接把两根信号线对调再测，
   只有两种组合，10 秒穷举完。换线时开着 `probe_robotiq_serial.py --watch`，通了立刻会
   看到（还会响一声）。

3. **断线 / 虚接。** 万用表量每根线端到端的通断，尤其是散线压接的地方。

探针报「完全静默」**不代表接线没问题**：总线空闲时没有任何驱动器在推电平，收发器的失效
保护偏置会把悬空的线拉到 mark，接反了也照样静默。只有「持续 0x00 噪声」才是接反的阳性证据。

</details>

<details>
<summary><b>上游的坑（本工作空间已规避）</b></summary>

1. 上游 `main` 分支在 Humble 上编不过 —— 用了 Kilted 才有的
   `on_init(HardwareComponentInterfaceParams&)`。锁在 `humble` 分支的 `a29c69b`。
2. `robotiq_control.launch.py` 引用了不存在的 `config/robotiq_update_rate.yaml`，参数文件
   缺失会让 `ros2_control_node` 直接起不来。本工作空间不使用该 launch，`update_rate` 写在
   自己的 controllers yaml 里。
3. `use_effort_interface` / `use_speed_interface` 在 Humble 的 `GripperActionController` 里
   根本不存在，配置里写了会被静默忽略 —— 看起来像在配力和速度，其实没有。力和速度改在
   xacro 的 `gripper_{force,speed}_multiplier` 里一次设死。
4. 驱动从不更新 velocity 状态，它恒为 0；但 Humble 的 `GripperActionController` 强制 claim
   position 和 velocity 两个状态接口，所以 xacro 里的 `<state_interface name="velocity"/>`
   不能删。又因为 velocity 恒为 0，每条 goal 都会被判成 stalled，所以 `allow_stalling: true`
   是必须的。这也正好符合 2F-85 的物理行为：夹住物体就停，到不了指令位置。
5. YAML 里重复的 `/**:` 顶层键会让除最后一个之外的所有段被静默丢弃。本工作空间只用一个。

另外，本工作空间的 gripper client 按 `(1 - percent) * gripper_closed_position` 换算，而不是
把百分比直接当弧度发 —— 后者全闭时发 1.0 rad，超过该关节 0.8 rad 的 URDF 上限。

</details>

## 当前状态

**完整遥操已在真机上跑通。** 三路由 `start_single_arm_robotiq.sh` 一起启动：FR3 在力矩控制
下跟随 GELLO，Robotiq 2F-85 由同一个主手扳机张合。

其余已验证项：驱动激活、三个控制器全 active、端到端方向与话题 4/4 PASS、连续重启 6/6、
RS-485 静止 soak 730/730 @ 240 s。干净环境下 `AMENT_PREFIX_PATH` 只含 `/opt/ros/humble`
和本仓库。

实物闭合到 0.7824 rad 而非指令的 0.7929 属正常：两指空载互顶后停住，`allow_stalling: true`
正是为此。

RS-485 的 soak 是在线缆静止时做的。现在线缆会跟着机械臂一起动，建议再跑一次
`probe_robotiq_serial.py --soak 120`，同时用手拨动、轻拽、弯折信号线，确认运动状态下链路
也不掉。

## 不在本工作空间范围内

VLA 部署（`~/franka_deploy/remote_acot_1.py` 等）没有动。那些脚本用 `franky.Gripper(ip)`
经 FCI 直连 Franka Hand，拔掉官方夹爪后必然连不上，需要单独写一个同 API 的 Robotiq 适配器。
另外末端长度从约 107 mm 变成约 160 mm，同一组关节角现在指尖多伸出约 5 cm —— 这不是改代码
能解决的，需要重采数据或微调。

## 许可

MIT，见 [LICENSE](LICENSE)。
