from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


TELEMETRY_TOPICS = [
    "/tf",
    "/tf_static",
    "/laser_scan",
    "/x3_lidar/imu",
    "/x3_lidar/range/down",
]


def generate_launch_description():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_output = f"bags/telemetry_sensors_{timestamp}"
    output = LaunchConfiguration("output")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output",
                default_value=default_output,
                description="Output bag directory.",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    output,
                    "--topics",
                    *TELEMETRY_TOPICS,
                ],
                output="screen",
            ),
        ]
    )
