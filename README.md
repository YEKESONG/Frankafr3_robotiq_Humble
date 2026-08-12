# franka_robotiq_22.04

Single-arm GELLO teleoperation for the Franka FR3, with a **Robotiq 2F-85** replacing the Franka Hand.

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green)

**English** | [简体中文](README.zh-CN.md)

---

## Overview

Everything needed for a full teleoperation session lives in this one workspace — no other
ROS workspace is required. Three legs run in parallel:

| Leg | Package | Role |
| --- | --- | --- |
| Leader | `franka_gello_state_publisher` | Reads the GELLO Dynamixels; publishes `/left/gello/joint_states` and a binary gripper percent on `/left/gripper/gripper_client/target_gripper_width_percent` |
| Arm | `franka_fr3_arm_controllers` | Joint-impedance controller tracking the GELLO joints, driving the FR3 over FCI via `franka_hardware` |
| Gripper | `franka_robotiq_bringup` | Converts the percent into a `control_msgs/GripperCommand`, driven by its own `ros2_control` stack over Modbus RTU / USB-RS485 |

The gripper runs its own `controller_manager`, separate from the arm's. That keeps the two
decoupled: changing the gripper never requires restarting the arm, and the FR3 xacro stays
untouched (`load_gripper: "false"` was already the arm's configuration).

## Requirements

**Software** — Ubuntu 22.04, ROS 2 Humble.

**Hardware**
- Franka FR3 with FCI enabled (default `172.16.0.3`)
- Robotiq 2F-85 with 24 V supply and a USB-RS485 adapter
- GELLO leader arm on a Dynamixel U2D2/OpenRB-150 adapter

## Installation

```bash
git clone https://github.com/YEKESONG/Frankafr3_robotiq_Humble.git ~/Desktop/franka_robotiq_22.04
cd ~/Desktop/franka_robotiq_22.04

# System dependencies (once)
sudo apt install -y python3-vcstool \
    ros-humble-gripper-controllers ros-humble-libfranka \
    ros-humble-franka-description ros-humble-franka-msgs \
    ros-humble-joint-state-publisher ros-humble-joint-state-broadcaster \
    ros-humble-moveit-core ros-humble-moveit-msgs
pip install dynamixel_sdk pyserial

# Fetch upstream sources, mask unused packages, build (idempotent)
./scripts/setup_workspace.sh
```

Upstream repositories are pinned by commit in `fr3_robotiq.humble.repos` and pulled with
`vcs import`; none of them are forked or patched. Of `franka_ros2`, only `franka_hardware`,
`franka_msgs`, `franka_semantic_components` and `franka_robot_state_broadcaster` are built —
the rest is masked with `COLCON_IGNORE`.

Then, in every new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/franka_robotiq_22.04/install/setup.bash
```

> **Build in a clean environment.** colcon bakes the build-time `AMENT_PREFIX_PATH` into
> `install/setup.sh`. If your `.bashrc` sources another franka/gello workspace, this one will
> silently link against it. After sourcing, `echo $AMENT_PREFIX_PATH | tr : "\n"` should show
> nothing but `/opt/ros/humble` and this repository.

## Usage

### Quick start

```bash
./scripts/start_single_arm_robotiq.sh
```

Opens a terminator window with three tabs (GELLO / arm / gripper). The arm tab waits until
`/left/gello/joint_states` is actually publishing, then waits for you to press **space** before
starting the controller — the arm is torque-controlled and will sag if started without a valid
target. Align the GELLO with the robot's current pose before pressing space.

### Staged bring-up

Useful when something is wrong; each step adds exactly one variable.

```bash
# 1. Controller chain only, no serial port, no hardware
ros2 launch franka_robotiq_bringup robotiq_teleop.launch.py config_file:=fake_left_robotiq.yaml
python3 scripts/test_gripper_chain.py          # expects 4/4 PASS

# 2. Raw Modbus RTU, bypassing ROS entirely — first thing to run on real hardware
python3 scripts/probe_robotiq_serial.py        # any ★ line means the gripper answered

# 3. Driver's built-in dummy backend (plugin loading, interface claims)
ros2 launch franka_robotiq_bringup robotiq_gripper.launch.py use_dummy:=true

# 4. Real gripper, commanded directly without GELLO
ros2 launch franka_robotiq_bringup robotiq_teleop.launch.py config_file:=single_left_robotiq.yaml
ros2 action send_goal /left/gripper/robotiq_gripper_controller/gripper_cmd \
    control_msgs/action/GripperCommand "{command: {position: 0.7929, max_effort: 40.0}}"
```

### Data recording

The `record_*_lerobot.py` scripts need no changes. Their gripper dimension is taken from the
GELLO command topic and binarised, not from gripper feedback, so swapping the hardware leaves
the dataset format and semantics unchanged.

## Configuration

### Serial port

```bash
ls -l /dev/serial/by-id/
```

Always use a `/dev/serial/by-id/...` path, never `/dev/ttyUSBn`: the GELLO adapter is an FTDI
device too, and the two swap numbers across replugs. Put the path in the `com_port` field of
`src/franka_robotiq_bringup/config/single_left_robotiq.yaml`. Membership in `dialout` is
required (`sudo usermod -aG dialout $USER`).

### Gripper parameters

All in `config/single_left_robotiq.yaml`:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `gripper_force_multiplier` | `0.20` | Fraction of the 2F-85's 235 N, i.e. ~47 N. The Franka Hand it replaces has ~70 N |
| `gripper_speed_multiplier` | `0.75` | Fraction of the maximum 150 mm/s |
| `input_open_value` / `input_closed_value` | `1.0` / `0.0` | Must match `gripper_open_output` / `gripper_closed_output` of `gello_publisher` |
| `max_effort` | `40.0` | Effort cap carried by each `GripperCommand` goal |

`namespace` must be `<arm namespace>/gripper` (e.g. `left/gripper`), otherwise the client's
command topic will not resolve onto what the GELLO publisher emits.

## Repository layout

```
franka_robotiq_22.04/
├── fr3_robotiq.humble.repos          # Upstream dependencies, pinned by commit
├── scripts/
│   ├── setup_workspace.sh            # Fetch, mask, build
│   ├── start_single_arm_robotiq.sh   # One-shot teleoperation launch
│   ├── test_gripper_chain.py         # End-to-end check: direction and namespaces
│   └── probe_robotiq_serial.py       # Bare Modbus RTU diagnostics (no ROS)
└── src/
    ├── franka_robotiq_bringup/       # This project: the gripper leg
    ├── franka_gello_state_publisher/ # From Franka's GELLO integration, holds local calibration
    ├── franka_fr3_arm_controllers/   # From Franka's GELLO integration, holds local arm config
    ├── ros2_robotiq_gripper/         # Upstream, vcs import, unmodified
    ├── serial/                       # Upstream, vcs import, unmodified
    └── franka_ros2/                  # Upstream v2.3.0, vcs import, unmodified
```

The two GELLO packages ship with the repository rather than being pulled by `vcs`, because
their configs hold machine-specific calibration — joint signs, assembly offsets, gripper range,
robot IP — which is local configuration, not upstream code.

## Troubleshooting

**The gripper's LED halves the search space before you touch anything:**

| LED | Meaning |
| --- | --- |
| Off | No 24 V. Note that the USB-RS485 adapter enumerates `/dev/ttyUSB*` from USB power alone, even with the gripper unpowered |
| Solid red | Powered, but not communicating — stop checking power, the fault is in the data link |
| Solid blue | Powered and communicating |
| Blinking red/blue | Booting, or a fault requiring re-activation |

Red is also the normal state while `probe_robotiq_serial.py` runs: the probe is read-only and
never writes the activation bit. Judge by the ★ lines, not by the LED. Red turns blue when the
ROS driver activates the gripper (`Robotiq Gripper successfully activated!`).

**`Controller already loaded` together with `no controller with this name exists`** is not a
race — a previous `ros2_control_node` is still alive. Ctrl-C on `ros2 launch` sometimes kills
only the wrapper:

```bash
pkill -9 -f lib/controller_manager/ros2_control_node
pkill -9 -f lib/franka_robotiq_bringup/robotiq_gripper_client
```

<details>
<summary><b>RS-485 dead: solid red LED and no reply at all</b></summary>

Run `scripts/probe_robotiq_serial.py` first — it sweeps 5 baud rates × 3 parities, slave
addresses 1–247, RTS direction control and the idle line level. If none of that answers, the
fault is physical. In order of likelihood:

1. **No common ground.** RS-485 is differential but still needs a shared 0 V reference. The
   classic mistake with loose wiring is to connect the gripper's black 0 V lead only to the
   power supply, leaving the adapter's GND floating on USB ground. Tie the adapter's GND to the
   same 0 V point as the gripper's black lead — A/B alone is not enough.

   ⚠️ A wire labelled "G" is not necessarily ground. Flange harnesses often carry a shield/drain
   wire that looks like one but is bonded only at the far end. Measure resistance between that
   wire and the black 0 V lead: near 0 Ω means it is really 0 V; open circuit means it is a
   shield and is useless as a signal ground.

2. **A/B swapped.** Vendors disagree on whether A is D+ or D−, so silkscreen is unreliable. Just
   swap the two signal wires and retest — two combinations, ten seconds. Keep
   `probe_robotiq_serial.py --watch` running while you rewire and it will beep the moment the
   bus comes up.

3. **Broken or cold-crimped wire.** Meter each conductor end to end.

A "completely silent" report does *not* clear the wiring: an idle bus has no driver on it, and
failsafe biasing pulls the floating pair to mark, so a reversed pair is silent too. Only
sustained 0x00 noise is positive evidence of reversal.

</details>

<details>
<summary><b>Upstream pitfalls already worked around here</b></summary>

1. Upstream `main` does not build on Humble — it uses the Kilted-only
   `on_init(HardwareComponentInterfaceParams&)`. Pinned to `a29c69b` on the `humble` branch.
2. `robotiq_control.launch.py` references a nonexistent `config/robotiq_update_rate.yaml`, which
   prevents `ros2_control_node` from starting. This workspace does not use that launch file and
   sets `update_rate` in its own controllers YAML.
3. `use_effort_interface` / `use_speed_interface` do not exist in Humble's
   `GripperActionController`; configs that set them are silently ignored. Force and speed are
   fixed once in the xacro's `gripper_{force,speed}_multiplier`.
4. The driver never updates the velocity state — it stays 0 — yet Humble's
   `GripperActionController` claims both position and velocity, so the xacro's
   `<state_interface name="velocity"/>` must stay. Because velocity is always 0, every goal is
   judged stalled, which makes `allow_stalling: true` mandatory. That also matches the 2F-85's
   physical behaviour: it stops on contact and never reaches the commanded position.
5. Duplicate top-level `/**:` keys in a YAML silently discard all but the last block. This
   workspace uses exactly one.

The gripper client here converts the percent as `(1 - percent) * gripper_closed_position`,
rather than treating the percent as radians directly — the latter emits 1.0 rad when fully
closed, past the joint's 0.8 rad URDF limit.

</details>

## Status

Verified on real hardware: driver activation, all three controllers active, end-to-end direction
and topics (4/4 PASS), 6/6 clean restarts, and a 730/730 RS-485 soak over 240 s. In a clean
environment `AMENT_PREFIX_PATH` contains only `/opt/ros/humble` and this repository.

Closing to 0.7824 rad instead of the commanded 0.7929 is expected — the fingers meet unloaded
and stall, which is what `allow_stalling: true` is for.

**Not yet verified: full teleoperation with a powered FR3.** The arm leg builds and its launch
files resolve, but torque control has not been run on the real robot. The RS-485 soak was also
done with the cable at rest; before mounting the gripper on the arm, repeat it with
`probe_robotiq_serial.py --soak 120` while flexing and tugging the signal wires.

## Out of scope

VLA deployment (`~/franka_deploy/remote_acot_1.py` and friends) is untouched. Those scripts talk
to the Franka Hand through `franky.Gripper(ip)` over FCI and cannot work once it is removed; a
Robotiq adapter with the same API is needed. Separately, the end-effector grows from ~107 mm to
~160 mm, so the same joint angles now put the fingertips ~5 cm further out — that requires
re-recording data or fine-tuning, not a code change.

## License

MIT. See [LICENSE](LICENSE).
