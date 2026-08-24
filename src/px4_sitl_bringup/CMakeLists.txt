cmake_minimum_required(VERSION 3.10)
project(px4_sitl_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz-transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(x500_tf_bridge src/x500_tf_bridge.cpp)
add_executable(
  pointcloud_clock_adapter
  src/pointcloud_clock_adapter.cpp
)

target_link_libraries(
  x500_tf_bridge
  PRIVATE
    geometry_msgs::geometry_msgs
    gz-msgs::core
    gz-transport::core
    rclcpp::rclcpp
    tf2::tf2
    tf2_ros::tf2_ros
)

target_link_libraries(
  pointcloud_clock_adapter
  PRIVATE
    rclcpp::rclcpp
    rosgraph_msgs::rosgraph_msgs
    sensor_msgs::sensor_msgs
)

install(
  TARGETS
    pointcloud_clock_adapter
    x500_tf_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch models rviz scripts worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
