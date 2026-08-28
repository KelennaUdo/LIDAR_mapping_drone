from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch offline LiDAR odometry and graph-based SLAM."""
    bringup_share = FindPackageShare("px4_sitl_bringup")
    kiss_install = LaunchConfiguration("kiss_icp_install_dir")

    kiss_config = PathJoinSubstitution(
        [kiss_install, "kiss_icp", "share", "kiss_icp", "config", "config.yaml"]
    )
    rtabmap_config = PathJoinSubstitution(
        [bringup_share, "config", "rtabmap_slam.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [bringup_share, "rviz", "x500_slam.rviz"]
    )

    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    odometry_topic = LaunchConfiguration("odometry_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "kiss_icp_install_dir",
                default_value=EnvironmentVariable(
                    "KISS_ICP_INSTALL_DIR",
                    default_value="/mnt/px4-workspace/kiss_icp_ws/install",
                ),
                description="Built KISS-ICP ROS 2 install space",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value=EnvironmentVariable(
                    "RTABMAP_DATABASE_PATH",
                    default_value=(
                        "/mnt/px4-workspace/rtabmap_maps/x500_mapping.db"
                    ),
                ),
                description="Persistent RTAB-Map database file",
            ),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/x500/lidar/points",
            ),
            DeclareLaunchArgument(
                "odometry_topic",
                default_value="/kiss/odometry",
            ),
            DeclareLaunchArgument("start_rviz", default_value="1"),
            Node(
                package="kiss_icp",
                executable="kiss_icp_node",
                name="kiss_icp_node",
                output="screen",
                parameters=[
                    kiss_config,
                    {
                        "base_frame": "lidar_link",
                        "lidar_odom_frame": "odom_lidar",
                        "publish_odom_tf": True,
                        "invert_odom_tf": False,
                        "publish_debug_clouds": True,
                        "use_sim_time": True,
                        "position_covariance": 0.1,
                        "orientation_covariance": 0.1,
                    },
                ],
                remappings=[("pointcloud_topic", pointcloud_topic)],
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[
                    rtabmap_config,
                    {"database_path": LaunchConfiguration("database_path")},
                ],
                remappings=[
                    ("odom", odometry_topic),
                    ("scan_cloud", pointcloud_topic),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_rviz")),
            ),
        ]
    )
