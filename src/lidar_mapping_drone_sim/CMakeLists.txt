cmake_minimum_required(VERSION 3.10)
project(lidar_mapping_drone_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
