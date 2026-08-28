from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


# Together, these topics preserve what the drone sensed, how PX4 estimated its
# motion, and the shared simulation timeline that ties those observations together.
RECORDED_TOPICS = (
    "/clock",
    "/x500/lidar/points",
    "/fmu/out/vehicle_odometry",
    "/fmu/out/vehicle_status_v1",
    "/tf",
    "/tf_static",
)


def generate_launch_description():
    """Record one clock-complete X500 SLAM experiment."""
    output_path = LaunchConfiguration("output")

    # --use-sim-time makes the bag's receive timestamps follow Gazebo's clock.
    recorder = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--use-sim-time",
            "-o",
            output_path,
            "--topics",
            *RECORDED_TOPICS,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output",
                description="Destination directory for this experiment's rosbag",
            ),
            recorder,
        ]
    )
