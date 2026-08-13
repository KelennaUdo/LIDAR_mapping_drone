cmake_minimum_required(VERSION 3.10)
project(px4_offboard_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(offboard_control src/offboard_control.cpp)
target_link_libraries(
  offboard_control
  PRIVATE rclcpp::rclcpp px4_msgs::px4_msgs
)

install(
  TARGETS offboard_control
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/run_offboard_control.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
