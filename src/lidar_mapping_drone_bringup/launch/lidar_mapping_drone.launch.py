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
            # Gazebo needs this so model://lidar_bot resolves to our local model.
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path),

            # Start Gazebo Sim. This is a normal process, not a ROS node.
            ExecuteProcess(
                cmd=["gz", "sim", "-v", "4", "-r", world],
                output="screen",
            ),


            # Bridge Gazebo /lidar2 into ROS 2 /laser_scan.
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["--ros-args", "-p", ["config_file:=", bridge_config]],
                output="screen",
            ),

            # Publish the fixed transform RViz needs to place /laser_scan.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0.38",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "world",
                    "--child-frame-id",
                    "lidar_bot/lidar_link/lidar",
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
