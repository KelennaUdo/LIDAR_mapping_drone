#include <memory>

#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rclcpp/rclcpp.hpp"


class PointCloudClockAdapter : public rclcpp::Node
{
private:

  // ============================================================
  // ROS 2 OBJECTS
  // ============================================================

  // Receives the point cloud replayed directly from the rosbag.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    recorded_pointcloud_subscription_;

  // Publishes the recorded Gazebo timestamp as the shared ROS simulation clock.
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr
    clock_publisher_;

  // Republishes the unchanged cloud after its timestamp becomes the current clock.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pointcloud_publisher_;


public:

  // ============================================================
  // CONSTRUCTOR
  // ============================================================

  // Connects the recorded cloud input to the synchronized clock and cloud outputs.
  PointCloudClockAdapter()
  : Node("pointcloud_clock_adapter")
  {
    clock_publisher_ =
      create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock",
      rclcpp::ClockQoS());

    pointcloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      "pointcloud",
      rclcpp::SensorDataQoS());

    recorded_pointcloud_subscription_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
      "recorded_pointcloud",
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        publish_synchronized_cloud(*message);
      });

    RCLCPP_INFO(
      get_logger(),
      "Waiting for recorded point clouds to provide Gazebo simulation time");
  }


private:

  // ============================================================
  // CLOCK AND POINT-CLOUD FORWARDING
  // ============================================================

  // Makes the cloud's Gazebo timestamp current before forwarding the cloud.
  void publish_synchronized_cloud(
    const sensor_msgs::msg::PointCloud2 & pointcloud)
  {
    rosgraph_msgs::msg::Clock clock_message{};
    clock_message.clock = pointcloud.header.stamp;

    clock_publisher_->publish(clock_message);
    pointcloud_publisher_->publish(pointcloud);
  }
};


// ============================================================
// PROGRAM ENTRY POINT
// ============================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudClockAdapter>());
  rclcpp::shutdown();
  return 0;
}
