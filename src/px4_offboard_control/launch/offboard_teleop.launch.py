from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "takeoff_altitude_m", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "movement_step_m", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "altitude_step_m", default_value="0.25"
            ),
            DeclareLaunchArgument(
                "yaw_step_deg", default_value="10.0"
            ),
            Node(
                package="px4_offboard_control",
                executable="offboard_teleop",
                name="offboard_teleop",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "takeoff_altitude_m": ParameterValue(
                            LaunchConfiguration("takeoff_altitude_m"),
                            value_type=float,
                        ),
                        "movement_step_m": ParameterValue(
                            LaunchConfiguration("movement_step_m"),
                            value_type=float,
                        ),
                        "altitude_step_m": ParameterValue(
                            LaunchConfiguration("altitude_step_m"),
                            value_type=float,
                        ),
                        "yaw_step_deg": ParameterValue(
                            LaunchConfiguration("yaw_step_deg"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
