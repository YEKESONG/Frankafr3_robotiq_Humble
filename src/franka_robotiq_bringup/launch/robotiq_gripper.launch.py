"""Bring up one Robotiq 2F-85: controller_manager, controllers and the GELLO bridge.

Everything lands inside `namespace`, so the command topic the client subscribes
to resolves to <namespace>/gripper_client/target_gripper_width_percent. With
namespace:=left/gripper that is exactly what franka_gello_state_publisher
already publishes for the left arm -- no remapping needed on either side.

For more than one gripper, use robotiq_teleop.launch.py, which includes this
file once per entry in a config YAML.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PACKAGE = "franka_robotiq_bringup"

ARGUMENTS = [
    DeclareLaunchArgument(
        "namespace",
        default_value="left/gripper",
        description=(
            "Namespace for every node. Must match what the GELLO publisher uses: "
            "<arm namespace>/gripper, e.g. left/gripper."
        ),
    ),
    DeclareLaunchArgument(
        "com_port",
        default_value="/dev/ttyUSB0",
        description=(
            "Serial port of the USB-RS485 adapter. Always prefer a stable "
            "/dev/serial/by-id/... path: the GELLO Dynamixel adapter is an FTDI "
            "device too, so /dev/ttyUSBn numbering swaps between them on replug."
        ),
    ),
    DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="Use mock_components instead of the driver. No serial port is opened.",
    ),
    DeclareLaunchArgument(
        "use_dummy",
        default_value="false",
        description="Load the real driver but talk to its built-in fake backend.",
    ),
    DeclareLaunchArgument(
        "gripper_speed_multiplier",
        default_value="0.25",
        description="Fraction of the 2F-85's 150 mm/s maximum closing speed.",
    ),
    DeclareLaunchArgument(
        "gripper_force_multiplier",
        default_value="0.20",
        description=(
            "Fraction of the 2F-85's 235 N maximum grip force. 0.20 is ~47 N, "
            "in the neighbourhood of the Franka Hand's ~70 N."
        ),
    ),
    DeclareLaunchArgument("gripper_closed_position", default_value="0.7929"),
    DeclareLaunchArgument("gripper_open_position", default_value="0.0"),
    DeclareLaunchArgument("max_effort", default_value="40.0"),
    DeclareLaunchArgument(
        "input_open_value",
        default_value="1.0",
        description="Value of the GELLO percent topic meaning 'fully open'.",
    ),
    DeclareLaunchArgument(
        "input_closed_value",
        default_value="0.0",
        description="Value of the GELLO percent topic meaning 'fully closed'.",
    ),
    DeclareLaunchArgument("command_deadband", default_value="0.02"),
    DeclareLaunchArgument(
        "gripper_command_topic",
        default_value="gripper_client/target_gripper_width_percent",
        description="Relative to `namespace`.",
    ),
    DeclareLaunchArgument(
        "launch_client",
        default_value="true",
        description="Set false to drive the gripper by hand via the action, without GELLO.",
    ),
    DeclareLaunchArgument(
        "launch_robot_state_publisher",
        default_value="false",
        description=(
            "Off by default: nothing in this stack consumes gripper TF, and two "
            "grippers would both publish a `world` -> robotiq_85_base_link "
            "transform and fight over the global /tf tree."
        ),
    ),
]


def generate_nodes(context):
    namespace = LaunchConfiguration("namespace")

    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare(PACKAGE), "urdf", "robotiq_2f_85_gripper.urdf.xacro"]
                ),
                " use_fake_hardware:=", LaunchConfiguration("use_fake_hardware"),
                " use_dummy:=", LaunchConfiguration("use_dummy"),
                " com_port:=", LaunchConfiguration("com_port"),
                " gripper_speed_multiplier:=", LaunchConfiguration("gripper_speed_multiplier"),
                " gripper_force_multiplier:=", LaunchConfiguration("gripper_force_multiplier"),
                " gripper_closed_position:=", LaunchConfiguration("gripper_closed_position"),
            ]
        ),
        value_type=str,
    )

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(PACKAGE), "config", "robotiq_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        output="screen",
        parameters=[{"robot_description": robot_description}, controllers_file],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{"robot_description": robot_description}],
        condition=IfCondition(LaunchConfiguration("launch_robot_state_publisher")),
    )

    def spawner(controllers, condition):
        """One spawner process for the whole list, rather than one process each.

        The spawner takes any number of controller names and loads them in
        order, so a single process is both fewer moving parts and free of
        concurrent load/configure service calls against the same
        controller_manager.
        """
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            output="screen",
            # Relative name: resolves to <namespace>/controller_manager.
            arguments=[*controllers, "--controller-manager", "controller_manager",
                       "--controller-manager-timeout", "30"],
            condition=condition,
        )

    # The activation controller drives the driver's reactivate_gripper GPIO,
    # which only the real hardware interface exports -- mock_components has no
    # such interface to claim, so it is left out in fake mode.
    nodes = [
        control_node,
        robot_state_publisher_node,
        spawner(
            ["joint_state_broadcaster", "robotiq_gripper_controller"],
            IfCondition(LaunchConfiguration("use_fake_hardware")),
        ),
        spawner(
            [
                "joint_state_broadcaster",
                "robotiq_gripper_controller",
                "robotiq_activation_controller",
            ],
            UnlessCondition(LaunchConfiguration("use_fake_hardware")),
        ),
        Node(
            package=PACKAGE,
            executable="robotiq_gripper_client",
            name="robotiq_gripper_client",
            namespace=namespace,
            output="screen",
            parameters=[
                {
                    "gripper_command_topic": LaunchConfiguration("gripper_command_topic"),
                    "gripper_open_position": ParameterValue(
                        LaunchConfiguration("gripper_open_position"), value_type=float
                    ),
                    "gripper_closed_position": ParameterValue(
                        LaunchConfiguration("gripper_closed_position"), value_type=float
                    ),
                    "input_open_value": ParameterValue(
                        LaunchConfiguration("input_open_value"), value_type=float
                    ),
                    "input_closed_value": ParameterValue(
                        LaunchConfiguration("input_closed_value"), value_type=float
                    ),
                    "max_effort": ParameterValue(
                        LaunchConfiguration("max_effort"), value_type=float
                    ),
                    "command_deadband": ParameterValue(
                        LaunchConfiguration("command_deadband"), value_type=float
                    ),
                }
            ],
            condition=IfCondition(LaunchConfiguration("launch_client")),
        ),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=generate_nodes)])
