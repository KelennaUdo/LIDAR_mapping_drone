from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("lidar_mapping_drone_bringup")
    sim_share = FindPackageShare("lidar_mapping_drone_sim")

    # The bringup package owns runtime configuration; the sim package owns assets.
    world = PathJoinSubstitution([sim_share, "worlds", "lidar_robot.world.sdf"])
    model_path = PathJoinSubstitution([sim_share, "models"])
    bridge_config = PathJoinSubstitution(
        [bringup_share, "config", "bridge_lidar.yaml"]
    )
    rviz_config = PathJoinSubstitution([sim_share, "rviz", "lidar_view.rviz"])

    return LaunchDescription(
        [
            # Gazebo needs this so model://x3_lidar resolves to our local model.
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path),

            # Start Gazebo Sim. This is a normal process, not a ROS node.
            ExecuteProcess(
                cmd=["gz", "sim", "-v", "4", "-r", world],
                output="screen",
                additional_env={
                    "__NV_PRIME_RENDER_OFFLOAD": "1",
                    "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
                    "__VK_LAYER_NV_optimus": "NVIDIA_only",
                },
            ),

            # Bridge Gazebo /lidar2 into ROS 2 /laser_scan.
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["--ros-args", "-p", ["config_file:=", bridge_config]],
                output="screen",
            ),

            # Gazebo uses the SDF world name as the root pose frame. This
            # identity transform only aliases that root to RViz's conventional
            # "world" frame. The drone and LiDAR transforms remain dynamic.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--frame-id",
                    "world",
                    "--child-frame-id",
                    "lidar_robot_world",
                ],
                output="screen",
            ),

            # Start RViz with the saved LaserScan display configuration.
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
