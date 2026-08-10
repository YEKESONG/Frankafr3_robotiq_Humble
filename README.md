# fr3_robotiq_bringup

把 Franka FR3 上的官方夹爪（Franka Hand）替换为 **Robotiq 2F-85** 的集成代码。

面向的栈：**GELLO 关节空间遥操 + VLA 直接输出关节指令 + 二值夹爪动作**，
不用 MoveIt、不用 RViz、observation 里没有 EE 位姿。

目标平台：**Ubuntu 22.04 / ROS 2 Humble**。

## 这个仓库的定位

**只放自己的改动，不 fork 上游。** 上游的 `ros2_robotiq_gripper` 和 `serial`
通过 [`fr3_robotiq.humble.repos`](fr3_robotiq.humble.repos) 以源码依赖的方式拉取并锁定版本，
本仓库不含它们的任何副本或补丁。好处是上游升级只需要改 repos 文件里的一行 commit hash，
不用处理 merge 冲突，也不需要向上游提 PR。

```
fr3_robotiq_bringup/
├── fr3_robotiq.humble.repos      # 上游依赖，锁版本
└── src/
    └── robotiq_franka_shim/      # 唯一的自有包
```

> **注意**：上游的 `main` 分支在 Humble 上编不过（用了 Kilted 才有的
> `on_init(HardwareComponentInterfaceParams&)` 接口）。repos 文件已经锁在
> `humble` 分支的 `a29c69b`。不要自己换成 `main`。

## 工作空间搭建

```bash
mkdir -p ~/fr3_ws/src && cd ~/fr3_ws/src
git clone <本仓库地址> fr3_robotiq_bringup

# 拉上游依赖（ros2_robotiq_gripper@humble + serial）
vcs import . < fr3_robotiq_bringup/fr3_robotiq.humble.repos

cd ~/fr3_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

本仓库的包在 `src/fr3_robotiq_bringup/src/` 下，colcon 会递归扫到，无需额外配置。

### 和 franka_ros2 工作空间的关系

建议**直接把 `fr3_robotiq_bringup` clone 进你现有的 franka_ros2 工作空间的 `src/`**，
一起 build，省掉 overlay 的 source 顺序问题。

如果坚持分开两个 ws，每次开终端必须先 source franka 的、再 source 这个的。

## 硬件准备

| 项 | 说明 |
|---|---|
| 转接法兰 | Robotiq 原厂随箱件 |
| 供电 | 24V，**至少 2A**。2F-85 常态约 150 mA，但激活自标定和全力夹持时峰值可到 1A；1A 适配器会出现"平时正常、一用力就掉电重启"的偶发故障 |
| 通信 | USB-RS485（ACC-ADT-USB-RS485），115200 8N1，slave 地址 0x09 |

### udev 规则

GELLO 的 U2D2 大概率也是 FTDI（`idVendor=0403`），**必须用序列号区分**，
否则两个设备会抢同一个符号链接：

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|serial" | head

sudo tee /etc/udev/rules.d/99-robotiq.rules <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{serial}=="<填你的序列号>", SYMLINK+="robotiq", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/robotiq
```

## 集成与验证

见 [`src/robotiq_franka_shim/README.md`](src/robotiq_franka_shim/README.md)。

## 已知的上游问题（本仓库已规避）

1. `robotiq_description/config/robotiq_controllers.yaml` 里的 effort/velocity
   接口名和硬件实际导出的名字对不上（应为 `set_gripper_max_effort` /
   `set_gripper_max_velocity`）。本仓库不使用这两个接口。
2. 驱动从不更新 velocity 状态，它恒为 0。但 Humble 的 GripperActionController
   会强制 claim 它，所以 xacro 里必须保留声明。
3. `robotiq_control.launch.py` 引用了不存在的 `config/robotiq_update_rate.yaml`。
   本仓库不使用该 launch 文件。
