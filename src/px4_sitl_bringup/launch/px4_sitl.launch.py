from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("px4_sitl_bringup")
    runner = PathJoinSubstitution(
        [bringup_share, "scripts", "run_px4_sitl.sh"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "px4_source_dir",
                default_value=EnvironmentVariable(
                    "PX4_SOURCE_DIR",
                    default_value="/mnt/px4-workspace/PX4-Autopilot",
                ),
                description="Mounted PX4-Autopilot checkout",
            ),
            DeclareLaunchArgument(
                "px4_image",
                default_value="px4-sitl:v1.17.0",
            ),
            DeclareLaunchArgument("model", default_value="gz_x500"),
            DeclareLaunchArgument("world", default_value="default"),
            DeclareLaunchArgument("headless", default_value="0"),
            SetEnvironmentVariable(
                "PX4_SOURCE_DIR", LaunchConfiguration("px4_source_dir")
            ),
            SetEnvironmentVariable(
                "PX4_IMAGE", LaunchConfiguration("px4_image")
            ),
            SetEnvironmentVariable(
                "PX4_SIM_MODEL", LaunchConfiguration("model")
            ),
            SetEnvironmentVariable(
                "PX4_GZ_WORLD", LaunchConfiguration("world")
            ),
            SetEnvironmentVariable("HEADLESS", LaunchConfiguration("headless")),
            ExecuteProcess(
                cmd=["bash", runner],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
