from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    control_share = FindPackageShare("lidar_mapping_drone_control")
    controller_config = PathJoinSubstitution(
        [control_share, "config", "flight_controller.yaml"]
    )
    motor_bridge_config = PathJoinSubstitution(
        [control_share, "config", "motor_bridge.yaml"]
    )

    mode = LaunchConfiguration("mode")
    enable_keyboard = LaunchConfiguration("enable_keyboard")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="manual_keyboard",
                description=(
                    "Controller mode: altitude_only, attitude_hold, "
                    "position_hold, or manual_keyboard."
                ),
            ),
            DeclareLaunchArgument(
                "enable_keyboard",
                default_value="false",
                description="Start the terminal keyboard helper for manual testing.",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="motorCommand_bridge",
                arguments=["--ros-args", "-p", ["config_file:=", motor_bridge_config]],
                output="screen",
            ),
            Node(
                package="lidar_mapping_drone_control",
                executable="flight_controller_node",
                name="flight_controller",
                parameters=[controller_config, {"controller_mode": mode}],
                output="screen",
            ),
            Node(
                package="lidar_mapping_drone_control",
                executable="keyboard_control_node",
                name="keyboard_control",
                parameters=[controller_config],
                output="screen",
                emulate_tty=True,
                condition=IfCondition(enable_keyboard),
            ),
        ]
    )
