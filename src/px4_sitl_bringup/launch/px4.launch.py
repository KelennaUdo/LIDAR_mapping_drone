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
        [bringup_share, "scripts", "run_px4.sh"]
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
                "px4_agent_dir",
                default_value=EnvironmentVariable(
                    "PX4_AGENT_DIR",
                    default_value=(
                        "/mnt/px4-workspace/Micro-XRCE-DDS-Agent"
                    ),
                ),
                description="Built Micro XRCE-DDS Agent checkout",
            ),
            DeclareLaunchArgument(
                "qgc_appimage",
                default_value=EnvironmentVariable(
                    "QGC_APPIMAGE",
                    default_value=PathJoinSubstitution(
                        [
                            EnvironmentVariable("HOME"),
                            "Applications",
                            "QGroundControl",
                            "QGroundControl.AppImage",
                        ]
                    ),
                ),
                description="QGroundControl AppImage executable",
            ),
            DeclareLaunchArgument(
                "px4_image",
                default_value="px4-sitl:v1.17.0",
            ),
            DeclareLaunchArgument("model", default_value="gz_x500"),
            DeclareLaunchArgument("world", default_value="default"),
            DeclareLaunchArgument("headless", default_value="0"),
            DeclareLaunchArgument("start_qgc", default_value="1"),
            DeclareLaunchArgument("dds_agent_port", default_value="8888"),
            DeclareLaunchArgument("dds_agent_verbose", default_value="4"),
            SetEnvironmentVariable(
                "PX4_SOURCE_DIR", LaunchConfiguration("px4_source_dir")
            ),
            SetEnvironmentVariable(
                "PX4_AGENT_DIR", LaunchConfiguration("px4_agent_dir")
            ),
            SetEnvironmentVariable(
                "QGC_APPIMAGE", LaunchConfiguration("qgc_appimage")
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
            SetEnvironmentVariable(
                "START_QGC", LaunchConfiguration("start_qgc")
            ),
            SetEnvironmentVariable(
                "DDS_AGENT_PORT", LaunchConfiguration("dds_agent_port")
            ),
            SetEnvironmentVariable(
                "DDS_AGENT_VERBOSE",
                LaunchConfiguration("dds_agent_verbose"),
            ),
            ExecuteProcess(
                cmd=["bash", runner],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
