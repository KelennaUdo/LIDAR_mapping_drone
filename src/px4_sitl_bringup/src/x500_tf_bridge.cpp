#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gz/msgs/pose_v.pb.h"
#include "gz/transport/Node.hh"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_broadcaster.hpp"


class X500TfBridge : public rclcpp::Node
{
private:

  // ============================================================
  // FRAME AND TOPIC NAMES
  // ============================================================

  // Gazebo topic containing the X500 model and link poses.
  std::string pose_topic_;

  // Entity names used to select only the relevant X500 transforms.
  std::string model_name_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string world_frame_;


  // ============================================================
  // BRIDGE OBJECTS AND STATE
  // ============================================================

  // Receives Gazebo Transport messages directly from the simulator.
  gz::transport::Node gazebo_node_;

  // Publishes the moving world-to-drone relationship on ROS /tf.
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;

  // Publishes the fixed drone-to-LiDAR relationship on ROS /tf_static.
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  // The rigid LiDAR mount only needs to be published once per node startup.
  bool static_transform_published_{false};

  // Prevents a missing-entity warning from flooding the terminal at 50 Hz.
  bool missing_entity_warning_printed_{false};


public:

  // ============================================================
  // CONSTRUCTOR
  // ============================================================

  // Loads frame names, creates TF broadcasters, and subscribes to Gazebo poses.
  X500TfBridge()
  : Node("x500_tf_bridge")
  {
    pose_topic_ = declare_parameter<std::string>(
      "pose_topic",
      "/world/mapping_test/dynamic_pose/info");

    model_name_ = declare_parameter<std::string>("model_name", "x500_0");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar_link");

    dynamic_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    static_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    if (!gazebo_node_.Subscribe(pose_topic_, &X500TfBridge::pose_callback, this)) {
      throw std::runtime_error(
              "Could not subscribe to Gazebo pose topic: " + pose_topic_);
    }

    RCLCPP_INFO(
      get_logger(),
      "Waiting for Gazebo poses on %s",
      pose_topic_.c_str());
  }


private:

  // ============================================================
  // GAZEBO POSE PROCESSING
  // ============================================================

  // Selects the model, body, and LiDAR poses and converts them into a ROS TF chain.
  void pose_callback(const gz::msgs::Pose_V & message)
  {
    const gz::msgs::Pose * model_pose = nullptr;
    const gz::msgs::Pose * base_pose = nullptr;
    const gz::msgs::Pose * lidar_pose = nullptr;

    for (const auto & pose : message.pose()) {
      if (pose.name() == model_name_) {
        model_pose = &pose;
      } else if (pose.name() == base_frame_) {
        base_pose = &pose;
      } else if (pose.name() == lidar_frame_) {
        lidar_pose = &pose;
      }
    }

    if (model_pose == nullptr || base_pose == nullptr || lidar_pose == nullptr) {
      if (!missing_entity_warning_printed_) {
        RCLCPP_WARN(
          get_logger(),
          "Waiting for Gazebo entities %s, %s, and %s",
          model_name_.c_str(),
          base_frame_.c_str(),
          lidar_frame_.c_str());
        missing_entity_warning_printed_ = true;
      }
      return;
    }

    missing_entity_warning_printed_ = false;

    const tf2::Transform world_to_model = to_tf_transform(*model_pose);
    const tf2::Transform model_to_base = to_tf_transform(*base_pose);
    const tf2::Transform model_to_lidar = to_tf_transform(*lidar_pose);

    const tf2::Transform world_to_base = world_to_model * model_to_base;
    const tf2::Transform base_to_lidar =
      model_to_base.inverse() * model_to_lidar;

    const auto stamp = message_stamp(message);

    dynamic_broadcaster_->sendTransform(
      to_ros_transform(
        world_to_base,
        stamp,
        world_frame_,
        base_frame_));

    if (!static_transform_published_) {
      static_broadcaster_->sendTransform(
        to_ros_transform(
          base_to_lidar,
          stamp,
          base_frame_,
          lidar_frame_));

      static_transform_published_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Publishing TF chain %s -> %s -> %s",
        world_frame_.c_str(),
        base_frame_.c_str(),
        lidar_frame_.c_str());
    }
  }


  // ============================================================
  // MESSAGE CONVERSION HELPERS
  // ============================================================

  // Converts one Gazebo pose into the transform representation used for composition.
  static tf2::Transform to_tf_transform(const gz::msgs::Pose & pose)
  {
    const tf2::Vector3 translation(
      pose.position().x(),
      pose.position().y(),
      pose.position().z());

    const tf2::Quaternion rotation(
      pose.orientation().x(),
      pose.orientation().y(),
      pose.orientation().z(),
      pose.orientation().w());

    return tf2::Transform(rotation, translation);
  }

  // Converts a composed transform into the ROS message sent by TF broadcasters.
  static geometry_msgs::msg::TransformStamped to_ros_transform(
    const tf2::Transform & transform,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & parent_frame,
    const std::string & child_frame)
  {
    geometry_msgs::msg::TransformStamped message{};
    message.header.stamp = stamp;
    message.header.frame_id = parent_frame;
    message.child_frame_id = child_frame;

    message.transform.translation.x = transform.getOrigin().x();
    message.transform.translation.y = transform.getOrigin().y();
    message.transform.translation.z = transform.getOrigin().z();

    message.transform.rotation.x = transform.getRotation().x();
    message.transform.rotation.y = transform.getRotation().y();
    message.transform.rotation.z = transform.getRotation().z();
    message.transform.rotation.w = transform.getRotation().w();

    return message;
  }

  // Preserves Gazebo simulation time so TF and PointCloud2 timestamps agree in RViz.
  static builtin_interfaces::msg::Time message_stamp(
    const gz::msgs::Pose_V & message)
  {
    builtin_interfaces::msg::Time stamp{};

    if (message.has_header() && message.header().has_stamp()) {
      stamp.sec = static_cast<std::int32_t>(message.header().stamp().sec());
      stamp.nanosec =
        static_cast<std::uint32_t>(message.header().stamp().nsec());
    }

    return stamp;
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<X500TfBridge>());
  rclcpp::shutdown();
  return 0;
}
