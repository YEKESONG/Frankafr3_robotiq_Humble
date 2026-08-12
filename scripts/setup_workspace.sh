#!/usr/bin/env bash
# ============================================================================
# 一键搭建：拉上游依赖 -> 屏蔽用不到的包 -> 编译
#
#   cd ~/Desktop/franka_robotiq_22.04
#   ./scripts/setup_workspace.sh
#
# 幂等，可重复跑。只做本仓库内的事，不碰系统、不碰其他工作空间。
#
# 前置（需要 sudo，脚本不会替你做，会检查并提示）：
#   sudo apt install -y ros-humble-gripper-controllers ros-humble-libfranka \
#                       ros-humble-franka-description ros-humble-joint-state-publisher \
#                       ros-humble-moveit-core ros-humble-moveit-msgs \
#                       ros-humble-joint-state-broadcaster python3-vcstool
#   pip install dynamixel_sdk pyserial
# ============================================================================
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"

echo ">> 工作空间: $WS"

# --- 前置检查 -------------------------------------------------------------
missing=()
for pkg in ros-humble-gripper-controllers ros-humble-libfranka \
           ros-humble-franka-description ros-humble-joint-state-publisher \
           ros-humble-moveit-core ros-humble-moveit-msgs \
           ros-humble-joint-state-broadcaster; do
  dpkg -s "$pkg" >/dev/null 2>&1 || missing+=("$pkg")
done
if [ ${#missing[@]} -gt 0 ]; then
  echo "!! 缺少 apt 依赖，请先执行："
  echo "   sudo apt install -y ${missing[*]}"
  exit 1
fi

python3 -c "import dynamixel_sdk" 2>/dev/null || {
  echo "!! 缺少 dynamixel_sdk（GELLO 的 Dynamixel 驱动）：pip install dynamixel_sdk"; exit 1; }
python3 -c "import serial" 2>/dev/null || {
  echo "!! 缺少 pyserial（夹爪串口诊断脚本用）：pip install pyserial"; exit 1; }
command -v vcs >/dev/null || {
  echo "!! 缺少 vcstool：sudo apt install python3-vcstool"; exit 1; }

# --- 上游依赖 -------------------------------------------------------------
echo ">> 拉取上游依赖（已存在则更新到锁定版本）..."
vcs import src < fr3_robotiq.humble.repos

# --- 屏蔽 franka_ros2 里用不到的包 ----------------------------------------
# 只需要 franka_hardware(FCI 硬件插件) / franka_msgs / franka_semantic_components /
# franka_robot_state_broadcaster 四个。其余会拖进 gazebo、moveit config 等一堆
# 本方案完全用不到的依赖，编译慢且容易因缺依赖失败。
# 注意 franka_gripper 也屏蔽掉：官方夹爪已被 Robotiq 取代，
# 且臂的配置是 load_gripper:=false，从不加载它。
NEEDED="franka_hardware franka_msgs franka_semantic_components franka_robot_state_broadcaster"
echo ">> 屏蔽 franka_ros2 中用不到的包（保留: $NEEDED）..."
for d in src/franka_ros2/*/; do
  pkg="$(basename "$d")"
  [ -f "$d/package.xml" ] || continue
  if echo "$NEEDED" | grep -qw "$pkg"; then
    rm -f "$d/COLCON_IGNORE"
  else
    touch "$d/COLCON_IGNORE"
    echo "   忽略 $pkg"
  fi
done

# --- 编译 -----------------------------------------------------------------
echo ">> 编译..."
# ROS 的 setup.bash 会引用未定义变量（AMENT_TRACE_SETUP_FILES 等），
# 在 set -u 下会直接报错退出，所以 source 期间临时关掉。
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "========================================================================"
echo ">> 完成。每个新终端需要:"
echo "   source /opt/ros/humble/setup.bash"
echo "   source $WS/install/setup.bash"
echo
echo ">> 启动遥操: $WS/scripts/start_single_arm_robotiq.sh"
echo "========================================================================"
