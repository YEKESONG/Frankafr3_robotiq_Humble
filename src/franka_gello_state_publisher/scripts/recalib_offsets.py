#!/usr/bin/env python3
"""
对比式重标定：只告诉你哪个关节的 assembly_offset 变了、变了多少，不覆盖配置。

用途：拆装过某个关节后，确认该关节零位偏了几个 90°，同时保留其余关节手调过的微调值。

用法（GELLO 摆到标定姿势后运行，注意先关掉 gello_publisher 节点，串口独占）：
    python3 recalib_offsets.py --config ../config/example_duo.yaml --section LEFT
    python3 recalib_offsets.py --config ../config/example_single_l.yaml --section SINGLE \
        --start-joints 0 0 0 -1.57 0 1.57 0
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

# 允许不 source install/setup.bash 直接运行
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from franka_gello_state_publisher.dynamixel.driver import DynamixelDriver  # noqa: E402
from franka_gello_state_publisher.gello_hardware import GelloHardware  # noqa: E402

BAUDRATE = 57600
# README 里各装配形态推荐的标定姿势
CALIB_POSES = {
    "single": [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0],
    "duo_left": [-1.57, -0.80, 1.80, -3.00, 1.40, 1.50, -2.10],
    "duo_right": [1.57, -0.80, -1.80, -3.00, -1.40, 1.50, 2.10],
}


def wrap_to_pi(x):
    """把角度折到 [-pi, pi)。"""
    return np.mod(x + np.pi, 2 * np.pi) - np.pi


def solve_offsets(raw, start_joints, signs):
    """
    反解 assembly_offsets。

    normalize_joint_positions 里 offset 是在乘 joint_signs 之前减掉的，
    所以 f(o) = normalize(raw, o, sign) 对 o 的斜率是 -sign。
    由 f(o+d) = f(o) - sign*d 得：把 f(0) 的残差乘上 sign 就是精确 offset。
    这样对 sign=+1 和 sign=-1 都成立（官方 get_offsets.py 对负号关节的
    ±pi/2 offset 会给出反号结果，这里绕开了那个坑）。
    """
    zeros = np.zeros_like(raw)
    residual = wrap_to_pi(
        GelloHardware.normalize_joint_positions(raw, zeros, signs) - start_joints
    )
    exact = np.mod(residual * signs, 2 * np.pi)
    snapped = np.mod(np.round(exact / (np.pi / 2)) * (np.pi / 2), 2 * np.pi)
    return exact, snapped


def verify(raw, offsets, signs, start_joints):
    """把候选 offsets 代回正向计算，返回与标定姿势的残差（度）。"""
    got = GelloHardware.normalize_joint_positions(raw, np.asarray(offsets), signs)
    return np.degrees(wrap_to_pi(got - start_joints))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", required=True, help="GELLO 配置 yaml 路径")
    p.add_argument("--section", required=True, help="yaml 顶层键：SINGLE / LEFT / RIGHT")
    p.add_argument(
        "--start-joints",
        nargs="+",
        type=float,
        default=None,
        help="你把 GELLO 摆成的标定姿势(rad)。不填则按 section 自动选 README 推荐姿势。",
    )
    p.add_argument("--port", default=None, help="覆盖 yaml 里的 com_port")
    args = p.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text())[args.section]
    signs = np.array(cfg["joint_signs"])
    n = cfg["num_arm_joints"]
    current = np.array(cfg["assembly_offsets"], dtype=float)

    if args.start_joints is not None:
        start = np.array(args.start_joints)
    else:
        key = {"SINGLE": "single", "LEFT": "duo_left", "RIGHT": "duo_right"}[args.section]
        start = np.array(CALIB_POSES[key])
    assert len(start) == n, f"start-joints 需要 {n} 个值"

    port = args.port or f"/dev/serial/by-id/{cfg['com_port']}"
    n_total = n + (1 if cfg.get("gripper") else 0)
    driver = DynamixelDriver(list(range(1, n_total + 1)), port=port, baudrate=BAUDRATE)
    joints_raw = np.array(driver.get_joints())
    raw = joints_raw[:n]

    exact, snapped = solve_offsets(raw, start, signs)
    res_cur = verify(raw, current[:n], signs, start)
    res_new = verify(raw, snapped, signs, start)
    delta = wrap_to_pi(snapped - current[:n])

    print(f"\n标定姿势 (rad): {list(np.round(start, 3))}")
    print(f"joint_signs:    {list(signs)}\n")
    print(f"{'关节':<5}{'配置中offset':>13}{'实测精确':>10}{'吸附90°':>10}"
          f"{'变化(°)':>10}{'旧残差(°)':>11}{'新残差(°)':>11}")
    print("-" * 71)
    for i in range(n):
        flag = "  <<< 变了" if abs(delta[i]) > np.radians(10) else ""
        print(f"J{i + 1:<4}{current[i]:>13.3f}{exact[i]:>10.3f}{snapped[i]:>10.3f}"
              f"{np.degrees(delta[i]):>10.1f}{res_cur[i]:>11.1f}{res_new[i]:>11.1f}{flag}")

    print("\n说明：")
    print("  · 『旧残差』大而『新残差』≈0 的关节，就是拆装后零位跑掉的那个。")
    print("  · 只把那一个关节的值换成『吸附90°』列，其余关节保持配置中原值不动")
    print("    （原值里的非 90° 部分是手工微调，不要一起覆盖）。")
    print("  · 『实测精确』只作参考——它把你摆姿势的目测误差也算进去了，不要直接抄。")

    if cfg.get("gripper"):
        gr = cfg["gripper_range_rad"]
        g_raw = joints_raw[-1]
        pct = (g_raw - gr[0]) / (gr[1] - gr[0])
        print(f"\n夹爪：当前 raw={g_raw:.3f} rad，按现配置换算 percent={pct:.3f}")
        print(f"  （完全松开时应≈1.0。若明显偏离，扳机侧也动过，"
              f"gripper_range_rad 改为 [{g_raw - 1.22:.3f}, {g_raw:.3f}]）")

    driver.close()


if __name__ == "__main__":
    main()
