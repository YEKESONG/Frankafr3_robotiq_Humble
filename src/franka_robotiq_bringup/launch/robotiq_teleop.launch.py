"""Bring up one or more Robotiq 2F-85 grippers from a config YAML.

Mirrors how franka_gello_state_publisher and franka_fr3_arm_controllers are
configured in the gello workspace -- one top-level key per arm -- so a duo setup
is a single launch, and the single-arm case is the same file with one entry.

    ros2 launch franka_robotiq_bringup robotiq_teleop.launch.py \
        config_file:=single_left_robotiq.yaml

Every key other than `namespace` is optional and falls through to the default
declared in robotiq_gripper.launch.py.
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

PACKAGE = "franka_robotiq_bringup"

# Forwarded verbatim to robotiq_gripper.launch.py when present in the config.
FORWARDED_KEYS = (
    "com_port",
    "use_fake_hardware",
    "use_dummy",
    "gripper_speed_multiplier",
    "gripper_force_multiplier",
    "gripper_closed_position",
    "gripper_open_position",
    "max_effort",
    "input_open_value",
    "input_closed_value",
    "command_deadband",
    "gripper_command_topic",
    "launch_client",
    "launch_robot_state_publisher",
)


def load_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Config file not found: {file_path}")
    with open(file_path, "r") as handle:
        return yaml.safe_load(handle)


def generate_nodes(context):
    config_file_name = LaunchConfiguration("config_file").perform(context)
    config_dir = FindPackageShare(PACKAGE).perform(context)
    configs = load_yaml(os.path.join(config_dir, "config", config_file_name))

    included = []
    for entry_name, config in configs.items():
        if "namespace" not in config:
            raise KeyError(f"Config entry '{entry_name}' is missing the required 'namespace' key")

        launch_arguments = {"namespace": str(config["namespace"])}
        for key in FORWARDED_KEYS:
            if key in config:
                # Launch arguments are strings; bools have to survive the trip
                # as "true"/"false" rather than Python's "True"/"False".
                value = config[key]
                launch_arguments[key] = (
                    str(value).lower() if isinstance(value, bool) else str(value)
                )

        included.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare(PACKAGE), "launch", "robotiq_gripper.launch.py"]
                    )
                ),
                launch_arguments=launch_arguments.items(),
            )
        )

    return included


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="single_left_robotiq.yaml",
                description="Config YAML in this package's config/ directory.",
            ),
            OpaqueFunction(function=generate_nodes),
        ]
    )
