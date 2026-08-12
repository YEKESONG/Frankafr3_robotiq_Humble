# franka_robotiq_22.04

GELLO → Franka FR3 单臂遥操，末端用 **Robotiq 2F-85** 取代 Franka Hand。

目标平台：**Ubuntu 22.04 / ROS 2 Humble**。

## 自包含

**克隆本仓库 + 配好环境即可完成完整遥操，不依赖任何其他工作空间。**
GELLO 发布器、机械臂控制器、Robotiq 夹爪三路全在这一个 workspace 里。

```
franka_gello_state_publisher            GELLO 主手 → Dynamixel 读关节
  ├─ sensor_msgs/JointState  /left/gello/joint_states ──────┐
  └─ std_msgs/Float32                                       │
     /left/gripper/gripper_client/target_gripper_width_percent
         │        （1.0 = 松开/张开, 0.0 = 捏紧/闭合，二值）  │
         ▼                                                  ▼
  robotiq_gripper_client                        franka_fr3_arm_controllers
     └─ control_msgs/GripperCommand（knuckle 关节弧度）  joint_impedance_controller
        /left/gripper/robotiq_gripper_controller/gripper_cmd    │
            │                                                   ▼
            ▼                                        franka_hardware（FCI）
     ros2_control（夹爪独立的 controller_manager）         → FR3 172.16.0.3
       └─ robotiq_driver → Modbus RTU → USB-RS485 → 2F-85
```

臂的配置本来就是 `load_gripper: "false"`，所以拔掉 Franka Hand 对它没有影响。
原先的 Franka Hand 夹爪链路是 `franka_umdc_control`（libfranka 直连）+
`franka_umdc_gripper_client`，本方案整体替换掉这两个节点。

### 为什么夹爪要自己开一个 controller_manager

臂由 franka_ros2 的 ros2_control 节点驱动。把夹爪并进去意味着改 FR3 的 xacro
和 franka 那边的启动流程。夹爪单开一个 controller_manager 之后两边彻底解耦：
改夹爪不用重启臂，改臂也不用动夹爪。

## 目录

```
franka_robotiq_22.04/
├── fr3_robotiq.humble.repos                # 上游依赖，锁 commit
├── scripts/
│   ├── setup_workspace.sh                  # 一键搭建：拉依赖 + 屏蔽无关包 + 编译
│   ├── start_single_arm_robotiq.sh         # 遥操一键启动（terminator 3 标签）
│   ├── test_gripper_chain.py               # 端到端自检：方向 + 话题命名空间
│   └── probe_robotiq_serial.py             # 裸 Modbus RTU 诊断（绕开整个 ROS）
└── src/
    ├── franka_robotiq_bringup/             # 自有包：夹爪这一路
    │   ├── config/{robotiq_controllers,single_left_robotiq,fake_left_robotiq}.yaml
    │   ├── launch/{robotiq_gripper,robotiq_teleop}.launch.py
    │   ├── urdf/robotiq_2f_85_gripper.urdf.xacro
    │   └── franka_robotiq_bringup/robotiq_gripper_client.py
    ├── franka_gello_state_publisher/       # 从 Franka GELLO 集成拷入，含本机标定值
    ├── franka_fr3_arm_controllers/         # 从 Franka GELLO 集成拷入，含本机臂配置
    ├── ros2_robotiq_gripper/               # 上游，vcs import，无改动
    ├── serial/                             # 上游，vcs import，无改动
    └── franka_ros2/                        # 上游 v2.3.0，vcs import，无改动
```

`franka_gello_state_publisher` 和 `franka_fr3_arm_controllers` 随仓库走而不是
vcs 拉取，因为它们的 config 里是**本机标定出来的** GELLO 关节符号、装配偏置、
夹爪量程和机械臂 IP，换一台机器要重新标，属于本地配置而非上游代码。

## 搭建

```bash
git clone <本仓库> ~/Desktop/franka_robotiq_22.04
cd ~/Desktop/franka_robotiq_22.04

# 1. 系统依赖（一次性）
sudo apt install -y python3-vcstool \
    ros-humble-gripper-controllers ros-humble-libfranka \
    ros-humble-franka-description ros-humble-franka-msgs \
    ros-humble-joint-state-publisher ros-humble-joint-state-broadcaster \
    ros-humble-moveit-core ros-humble-moveit-msgs
pip install dynamixel_sdk pyserial

# 2. 拉上游 + 屏蔽无关包 + 编译（幂等，可反复跑）
./scripts/setup_workspace.sh
```

`setup_workspace.sh` 会先检查上面这些依赖，缺了直接告诉你缺哪个，然后
`vcs import` 拉上游、给 franka_ros2 里用不到的包打 `COLCON_IGNORE`、最后编译。

只需要 franka_ros2 的 4 个包：`franka_hardware`（FCI 硬件插件）、`franka_msgs`、
`franka_semantic_components`、`franka_robot_state_broadcaster`。
其余（gazebo bringup、moveit config、example controllers、franka_gripper）
会拖进一堆用不到的依赖，编译慢且容易失败，所以屏蔽掉。

每个新终端：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/franka_robotiq_22.04/install/setup.bash
```

> **注意 colcon 会把构建时的 `AMENT_PREFIX_PATH` 烧进 `install/setup.sh`。**
> 如果你的 `.bashrc` 自动 source 了别的 franka/gello 工作空间，本工作空间就会
> 反过来链到它们身上，"自包含"就名存实亡了。要么把 `.bashrc` 里那些
> source 行去掉，要么在干净环境里编译：
>
> ```bash
> env -i HOME=$HOME PATH=/usr/bin:/bin bash --noprofile --norc -c \
>   'cd ~/Desktop/franka_robotiq_22.04 && source /opt/ros/humble/setup.bash && \
>    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
> ```
>
> 验证：source 之后 `echo $AMENT_PREFIX_PATH | tr : "\n"` 里除了
> `/opt/ros/humble` 和本仓库不应有别的路径。

## 串口

```bash
ls -l /dev/serial/by-id/
```

**永远用 `/dev/serial/by-id/...` 路径，不要用 `/dev/ttyUSBn`。**
GELLO 的 Dynamixel 转换器也是 FTDI 设备，两者的 ttyUSB 编号会随插拔顺序互换
（本机实测：Robotiq `usb-FTDI_USB_TO_RS-485_DAAL8Y3V-if00-port0`，
GELLO `usb-FTDI_USB__-__Serial_Converter_FTBMP0HP-if00-port0`）。
by-id 路径本身就能区分二者，不需要额外写 udev 规则。

把查到的路径填进 `src/franka_robotiq_bringup/config/single_left_robotiq.yaml`
的 `com_port`。当前值已经是本机实测的那一个。

需要 dialout 组权限：`sudo usermod -aG dialout $USER`（本机已在组内）。

## 分级验证

按顺序做，别跳。每一级只引入一个新变量。

**① 只验控制器链路，不碰串口**（mock_components，不需要接夹爪）

```bash
ros2 launch franka_robotiq_bringup robotiq_teleop.launch.py \
    config_file:=fake_left_robotiq.yaml
# 另开一个终端
python3 scripts/test_gripper_chain.py
```

四项应当全 PASS。这一步验的是方向没反、话题命名空间对得上、控制器能激活。

**② 裸测串口，绕开整个 ROS**（接实物之后第一件事）

```bash
python3 scripts/probe_robotiq_serial.py
```

任何带 ★ 的行就说明夹爪在总线上应答了。**全程无应答说明问题在 ROS 之下。**

先看夹爪自己的 LED，它一句话就能把范围切一半：

| LED | 含义 |
|---|---|
| 熄灭 | 无 24V。注意 USB-RS485 转换器吃 USB 电，夹爪没电时它照样枚举出 `/dev/ttyUSB*`，很容易误判成"接好了" |
| **常亮红** | **已上电，但通信不通** —— 电源不用查了，问题在数据链路 |
| 常亮蓝 | 上电且通信正常 |
| 红蓝闪烁 | 启动中，或有故障需重新激活 |

红灯 + 全程无应答时，见下面的「排障 / RS-485 不通」。

**③ 走驱动的内置 fake 后端**（验插件加载和接口 claim，仍不需要真夹爪）

```bash
ros2 launch franka_robotiq_bringup robotiq_gripper.launch.py use_dummy:=true
```

**④ 接实物**

```bash
ros2 launch franka_robotiq_bringup robotiq_teleop.launch.py \
    config_file:=single_left_robotiq.yaml
# 直接打 action（不经过 GELLO）
ros2 action send_goal /left/gripper/robotiq_gripper_controller/gripper_cmd \
    control_msgs/action/GripperCommand "{command: {position: 0.7929, max_effort: 40.0}}"
```

**⑤ 完整遥操**

```bash
./scripts/start_single_arm_robotiq.sh
```

3 个标签：GELLO 发布 / 机械臂 / Robotiq 夹爪。机械臂标签会先等
`/left/gello/joint_states` 真正开始发布、再等你按空格才启动
（力矩控制，没有有效目标就启动会下坠）。

## 采集

`record_*_lerobot.py` **不需要任何改动**。这些脚本的夹爪维度取自 GELLO 主手的
指令话题（`/left/gripper/gripper_client/target_gripper_width_percent`）再二值化，
读的不是真实夹爪状态，所以换硬件不改变数据格式和语义。

## 排障

**日志里同时出现这两行：**

```
[spawner] Controller already loaded, skipping load_controller
[controller_manager] Could not configure controller with name '...' because no controller with this name exists
```

这不是竞态，是**上一次的 `ros2_control_node` 还活着**。Ctrl-C 打在 `ros2 launch`
上有时只杀掉外壳，留下子进程；残留的 controller_manager 和新起的那个占用同一个
节点名，spawner 的 `list_controllers` 问到了旧的（所以说"已加载"），
`configure_controller` 落到了新的（所以说"不存在"）。

确认并清理：

```bash
ps -eo pid,args | grep -E "ros2_control_node|robotiq_gripper_client" | grep -v grep
pkill -9 -f lib/controller_manager/ros2_control_node
pkill -9 -f lib/franka_robotiq_bringup/robotiq_gripper_client
```

清干净再重启。本工作空间连续重启 6 次实测 6/6 正常，前提就是每次残留清零。

### RS-485 不通（夹爪红灯 + 全程无应答）

用散线手工接到通用 USB-RS485 转换器（而不是 Robotiq 原厂 ACC-ADT-USB-RS485
套件）时，A/B 极性和共地都要自己保证，这两条是最常见的失败点。

先跑 `scripts/probe_robotiq_serial.py` 把软件侧排除干净。它会依次确认：
转换器是否真的按波特率发送字节、5 种波特率 × 3 种校验位、从站地址 1–247 全扫、
RTS 方向控制、以及总线空闲电平。**这些全部无应答，就只剩物理层。**

按可能性从高到低：

**1. 没有共地（散线接法最容易漏）**

RS-485 是差分信号，但仍然需要一个共同的 0V 参考。典型的错误接法是：
夹爪的黑线（0V）只接到了电源适配器，而转换器这一侧的 GND 悬空、
仅通过 USB 接到开发机的地。两边地不同 → 差分接收器的共模电压跑出范围 → 完全不通。

**把转换器的 GND 端子接到夹爪的黑线 0V（也就是电源适配器负极的同一个点）。**
只接 A/B 两根线是不够的。

⚠️ **接了一根叫"G"的线不等于共地。** 法兰散线里除了 0V，往往还有一根屏蔽/裸线，
外观上很像地线，但它在夹爪那一侧可能根本没接到 0V（屏蔽层通常只在一端接地）。
拿它当信号地接，结果和完全不接一样。

用万用表电阻档量一下**这根"G"和黑色 0V 线之间的通断**：

- 接近 0 Ω → 确实是 0V，共地成立
- 开路 → 它是屏蔽层，不是信号地。把转换器的 GND 改接到黑线 0V（即电源适配器负极）

**2. A/B 接反**

不同厂商对 A/B 的定义相反（有的 A=D+，有的 A=D−），靠看丝印猜不可靠。
**直接把两根信号线对调再测一次**，只有两种组合，10 秒就能穷举完，比查手册快。

开着实时监视模式换线，通了立刻会看到（还会响一声），不用来回敲命令：

```bash
python3 scripts/probe_robotiq_serial.py --watch
```

**3. 断线 / 虚接**

万用表量夹爪端到转换器端每根线的通断，尤其是散线压接的地方。

关于空闲电平的一个坑：`probe_robotiq_serial.py` 报「完全静默」**不代表接线没问题**。
总线空闲时没有任何驱动器在推电平，接反了也照样静默 —— 收发器的失效保护偏置会把
悬空的线拉到 mark。只有「收到持续 0x00 噪声」才是接反的阳性证据，静默是无信息的。

**判据只看有没有 ★，不要看 LED。** `probe_robotiq_serial.py` 是只读的，
它只读状态寄存器、从不写激活位（rACT），所以链路即使完全正常，
跑探针时 LED 也会一直是红的。红转蓝要等 ROS 驱动启动时执行激活
（日志里的 `Robotiq Gripper successfully activated!`）。
上电即红、还没通信就是红，是完全正常的初始状态。

## 参数

都在 `config/single_left_robotiq.yaml` 里：

| 参数 | 默认 | 说明 |
|---|---|---|
| `gripper_force_multiplier` | 0.20 | 占 2F-85 最大 235 N 的比例，约 47 N。Franka Hand 只有约 70 N，起步先保守 |
| `gripper_speed_multiplier` | 0.25 | 占最大 150 mm/s 的比例。压低是为了让闭合耗时接近 Franka Hand 的时序 |
| `input_open_value` / `input_closed_value` | 1.0 / 0.0 | 对应 gello_publisher 的 `gripper_open_output` / `gripper_closed_output` |
| `max_effort` | 40.0 | 每条 GripperCommand goal 带的力上限 |

## 上游的坑（本工作空间已规避）

读上游和 gello 工作空间里那份没跑通的 Robotiq 代码时发现的，记下来免得重踩：

1. **上游 `main` 分支在 Humble 上编不过** —— 用了 Kilted 才有的
   `on_init(HardwareComponentInterfaceParams&)` 接口。repos 文件锁在 `humble`
   分支的 `a29c69b`。不要换成 `main`。
2. **`robotiq_control.launch.py` 引用了不存在的 `config/robotiq_update_rate.yaml`**
   —— 参数文件缺失会让 `ros2_control_node` 直接起不来。本工作空间不使用该 launch，
   `update_rate` 写在自己的 controllers yaml 里。
3. **`use_effort_interface` / `use_speed_interface` 在 Humble 上根本不存在** ——
   Humble 的 `GripperActionController` 参数只有 joint / goal_tolerance /
   max_effort / allow_stalling / stall_timeout / stall_velocity_threshold /
   action_monitor_rate，且只 claim `<joint>/position` 一个命令接口。上游和 gello
   那份配置里写的这两个键会被静默忽略，看起来像在配力和速度，其实没有。
   力和速度改在 xacro 的 `gripper_{force,speed}_multiplier` 里一次设死。
4. **驱动从不更新 velocity 状态**，它恒为 0；但 Humble 的 `GripperActionController`
   强制 claim position **和** velocity 两个状态接口，所以 xacro 里的
   `<state_interface name="velocity"/>` 不能删。又因为 velocity 恒为 0，每条 goal
   都会被判成 stalled，所以 `allow_stalling: true` 是必须的 —— 否则每条 goal 都
   abort。这同时也正好符合 2F-85 的物理行为：夹住物体就停，到不了指令位置。
5. **gello 工作空间的 `franka_gripper_manager/config/robotiq_controllers.yaml`
   有 YAML 重复键** —— 三个 `/**:` 顶层键，解析后只剩最后一个，
   `controller_manager` 和 `robotiq_gripper_controller` 两整段被静默丢弃。
   本工作空间只用一个 `/**:` 顶层键。

另外，gello 那份 `robotiq_gripper_client.py` 把 0~1 的百分比**直接当弧度**发
（`position = 1 - percent`），既忽略了配置的闭合角，全闭时又发 1.0 rad，
**超过该关节 0.8 rad 的 URDF 上限**。本工作空间的 client 按
`(1 - percent) * gripper_closed_position` 换算。

## 当前验证状态

| 项 | 状态 |
|---|---|
| 12 个包在干净环境编译 | ✅ 通过 |
| xacro 渲染（fake / 实物两种模式） | ✅ 通过 |
| 控制器链路（mock_components） | ✅ `joint_state_broadcaster`、`robotiq_gripper_controller` 均 configured + activated |
| 重复启动 6 次 | ✅ 6/6 两个控制器都起来 |
| 端到端方向与话题（`test_gripper_chain.py`，fake） | ✅ 4/4 PASS |
| **接实物：驱动激活** | ✅ `Robotiq Gripper successfully activated!` |
| **接实物：三个控制器** | ✅ 均 active（含只有实物才有的 activation controller） |
| **接实物：端到端** | ✅ 4/4 PASS，夹爪实际张合，方向正确 |
| **RS-485 链路稳定性** | ✅ 静止 soak 730/730 @ 240s，无中断 |
| **自包含** | ✅ 干净环境下 `AMENT_PREFIX_PATH` 只含 `/opt/ros/humble` 和本仓库；三路 launch 均可解析 |

实物闭合到 0.7824 rad 而非指令的 0.7929，是两指空载互顶后停住，属正常
——`allow_stalling: true` 正是为此。

尚未验证：**接上 FR3 通电之后的完整遥操**。臂那一路的代码已在本工作空间里编过、
launch 可解析，但没有在真机上跑过力矩控制。

RS-485 的 soak 是在**线缆静止**时做的。夹爪装到机械臂上之后线会跟着动，
装之前建议再做一次动态验证：跑 `probe_robotiq_serial.py --soak 120`，
同时用手拨动、轻拽、弯折三根信号线和端子，掉一次它就会打出时间戳。

## 不在本工作空间范围内

VLA 部署（`~/franka_deploy/remote_acot_1.py` 等）**没有动**。那些脚本用
`franky.Gripper(ip)` 经 FCI 直连 Franka Hand，拔掉官方夹爪后必然连不上，
需要单独写一个和 `franky.Gripper` 同 API 的 Robotiq 适配器。
另外末端长度从约 107 mm 变成约 160 mm，而策略输出的是关节角，
同一组关节角现在指尖多伸出约 5 cm —— 这不是改代码能解决的，需要重采数据或微调。
