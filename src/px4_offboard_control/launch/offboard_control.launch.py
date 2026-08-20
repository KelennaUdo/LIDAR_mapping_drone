from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument(
                "target_altitude_m", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "flight_duration_s", default_value="15.0"
            ),
            Node(
                package="px4_offboard_control",
                executable="offboard_control",
                name="offboard_control",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "auto_start": ParameterValue(
                            LaunchConfiguration("auto_start"),
                            value_type=bool,
                        ),
                        "target_altitude_m": ParameterValue(
                            LaunchConfiguration("target_altitude_m"),
                            value_type=float,
                        ),
                        "flight_duration_s": ParameterValue(
                            LaunchConfiguration("flight_duration_s"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
