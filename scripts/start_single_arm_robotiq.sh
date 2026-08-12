#!/usr/bin/env bash
# ============================================================================
# GELLO -> Franka FR3 + Robotiq 2F-85 单臂遥操一键启动（terminator 多标签版）
#
#   ./start_single_arm_robotiq.sh
#
# 3 标签：T1 GELLO发布 / T2 机械臂 / T3 Robotiq夹爪
#
# 与原 Franka Hand 流程（gello 工作空间的 start_dual_arm.sh）的区别只有夹爪那一路：
#   旧：L1 franka_umdc_control(libfranka 连 Franka Hand) + L4 franka_umdc_gripper_client
#   新：T3 franka_robotiq_bringup(ros2_control + 串口 Modbus RTU 连 2F-85)
# GELLO 发布器和机械臂控制器完全沿用 gello 工作空间里已经跑通的那套，一行没改。
# 两边话题在 /left/gripper/gripper_client/target_gripper_width_percent 上对接。
#
# 顺序保证：机械臂标签不会盲目按时间启动，而是先主动等待 /left/gello/joint_states
# 真正开始发布，确认后再等用户按空格才 launch 机械臂。
# 因为机械臂是力矩控制、目标全靠 GELLO；没有有效目标就启动会导致下坠。
#
# 一个 terminator 窗口，每个节点一个标签页。节点退出后标签停在 shell
# （看日志 / 原地 ↑回车 重跑），不会自动关。想停某个节点：切到那个标签 Ctrl-C。
# 想停全部：关掉窗口。用 -g 临时配置启动，不改动 ~/.config/terminator/config。
# ============================================================================

# ---- 可改参数 -----------------------------------------------------------
# 已跑通的 GELLO 工作空间（本脚本只读不改）
WS_GELLO="$HOME/Desktop/gello/gello_software-main/ros2"
# 本工作空间（Robotiq 夹爪）
WS_ROBOTIQ="$HOME/Desktop/franka_robotiq_22.04"

# 臂的命名空间。夹爪配置里的 namespace 必须是 "<这个值>/gripper"，
# 否则 GELLO 发的百分比话题和夹爪 client 订阅的话题对不上。
ARM_NS="left"

GELLO_CFG="example_single_l.yaml"        # franka_gello_state_publisher
ARM_CFG="single_left_fr3_config.yaml"    # franka_fr3_arm_controllers
GRIPPER_CFG="single_left_robotiq.yaml"   # franka_robotiq_bringup（本工作空间）

# 机械臂控制器启动前，最多等多少秒确认 GELLO 数据真正开始发布。
GELLO_WAIT_TIMEOUT=120
# ------------------------------------------------------------------------

node_title() {
  case "$1" in
    T1) echo "T1 GELLO发布" ;;
    T2) echo "T2 机械臂" ;;
    T3) echo "T3 Robotiq夹爪" ;;
  esac
}

node_cmd() {
  case "$1" in
    T1) echo "ros2 launch franka_gello_state_publisher main.launch.py config_file:=$GELLO_CFG" ;;
    T2) echo "wait_for_gello $ARM_NS; wait_for_space; ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py robot_config_file:=$ARM_CFG" ;;
    T3) echo "ros2 launch franka_robotiq_bringup robotiq_teleop.launch.py config_file:=$GRIPPER_CFG" ;;
  esac
}

# 主动等待指定命名空间的 GELLO 关节话题真正开始发布后才返回。
# 这是"保证机械臂在 GELLO 就绪后才启动"的核心，替代不可靠的固定 sleep。
wait_for_gello() {
  local ns topic remaining deadline=$(( SECONDS + GELLO_WAIT_TIMEOUT ))
  for ns in "$@"; do
    topic="/$ns/gello/joint_states"
    echo ">> 等待 $topic 开始发布（机械臂启动前必须先有有效目标，否则会下坠）..."
    remaining=$(( deadline - SECONDS ))
    [ "$remaining" -le 0 ] && remaining=1
    # 单进程常驻订阅：建立一次订阅后持续监听，收到第一条立即返回(0)；到总超时仍无
    # 数据则返回非 0。相比反复冷启动 `ros2 topic echo --once`，省掉每轮 ~1.5s 的进程
    # 冷启动 + DDS discovery，也消除"订阅还没建好就被 3s timeout 杀掉"导致的误判空转。
    if timeout "$(( remaining + 5 ))" env WAIT_TOPIC="$topic" WAIT_TIMEOUT="$remaining" python3 -c '
import os, sys, time, rclpy
from sensor_msgs.msg import JointState
topic = os.environ["WAIT_TOPIC"]
deadline = time.monotonic() + float(os.environ["WAIT_TIMEOUT"])
rclpy.init()
node = rclpy.create_node("wait_gello_probe")
got = []
node.create_subscription(JointState, topic, lambda m: got.append(1), 10)
next_note = time.monotonic() + 3.0
while rclpy.ok() and not got and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
    if not got and time.monotonic() >= next_note:
        print(f"   {topic} 还没数据，继续等待...", flush=True)
        next_note += 3.0
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if got else 1)
'; then
      echo "   ✓ $topic 已在发布"
    else
      echo "!! 超时（${GELLO_WAIT_TIMEOUT}s）：$topic 仍无数据，放弃启动机械臂以策安全。"
      echo "   请先确认 GELLO 发布器(T1)标签是否正常在发数据，再重跑本标签。"
      exec bash
    fi
  done
  echo ">> GELLO 已就绪。"
}

# GELLO 就绪后，等待用户按【空格键】才真正启动机械臂控制器（开始遥操作）。
# 把使机械臂跟随的动作交给人手动触发，更可控更安全。
# 启动前请把 GELLO 摆到与机械臂当前位姿接近的位置，避免按下瞬间机械臂大幅跳动。
wait_for_space() {
  echo
  echo "========================================================================"
  echo ">> 准备就绪：请把 GELLO 摆到与机械臂当前位姿接近的位置。"
  echo ">> 按【空格键】开始遥操作（机械臂将开始跟随 GELLO）；按 Ctrl-C 取消。"
  echo "========================================================================"
  local key
  while IFS= read -rsn1 key; do
    [ "$key" = " " ] && break
  done
  echo ">> 已确认，启动机械臂控制器，开始遥操作。"
}

# --- 自分发：被 terminator 标签调用，在当前终端里跑某个节点 ---------------
if [ "$1" = "__node" ]; then
  ID="$2"
  printf '\033]0;%s\007' "$(node_title "$ID")"
  source /opt/ros/humble/setup.bash
  # 两个工作空间都 source：GELLO 发布器/机械臂控制器在 gello 那边，
  # Robotiq 夹爪在这边。两边包名不重叠，先后顺序无所谓。
  source "$WS_GELLO/install/setup.bash"
  source "$WS_ROBOTIQ/install/setup.bash"
  cd "$WS_ROBOTIQ" || exit 1
  eval "$(node_cmd "$ID")"
  echo; echo "====== 本节点已退出：↑回车重跑，或关闭本标签 ======"
  exec bash
fi
# ------------------------------------------------------------------------

for ws in "$WS_GELLO" "$WS_ROBOTIQ"; do
  if [ ! -f "$ws/install/setup.bash" ]; then
    echo "!! 找不到 $ws/install/setup.bash，请先在该工作空间 colcon build"
    exit 1
  fi
done

IDS=(T1 T2 T3)

SELF="$(readlink -f "$0")"
CFG="$WS_ROBOTIQ/.robotiq_terminator_layout.cfg"

# 生成本次专用的 terminator 配置（含一个多标签 layout）
{
  echo "[global_config]"
  echo "  title_use_system_font = True"
  echo "[profiles]"
  echo "  [[default]]"
  echo "    scrollback_lines = 5000"
  echo "[layouts]"
  echo "  [[robotiq]]"
  echo "    [[[window0]]]"
  echo "      type = Window"
  echo "      parent = \"\""
  echo "    [[[notebook0]]]"
  echo "      type = Notebook"
  echo "      parent = window0"
  labels=""
  for id in "${IDS[@]}"; do labels="$labels$(node_title "$id"), "; done
  echo "      labels = ${labels%, }"
  echo "      active_page = 0"
  i=0
  for id in "${IDS[@]}"; do
    echo "    [[[term$i]]]"
    echo "      type = Terminal"
    echo "      parent = notebook0"
    echo "      order = $i"
    echo "      profile = default"
    echo "      command = bash $SELF __node $id"
    i=$((i+1))
  done
  echo "[plugins]"
} > "$CFG"

echo ">> 启动单臂 GELLO + Robotiq 链路，共 ${#IDS[@]} 个节点标签（terminator）..."
# -u: 独立实例（不复用已有 terminator），否则 -g/-l 会被已有实例忽略，只弹一个空白终端
terminator -u -g "$CFG" -l robotiq
