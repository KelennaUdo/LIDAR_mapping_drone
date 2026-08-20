#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


class OffboardControl : public rclcpp::Node
{
private:

  // ============================================================
  // NODE STATE
  // ============================================================

  // Enables or disables automatic flight; read from the ROS 2 auto_start parameter.
  bool auto_start_{false};

  // Desired flight altitude in metres; used when creating the PX4 position setpoint.
  double target_altitude_m_{2.0};

  // How long to remain in flight before landing; used by the flight timer logic.
  double flight_duration_s_{15.0};


  // Remembers whether Offboard mode and arming have already been requested.
  bool flight_started_{false};

  // Remembers whether the land command has already been sent, preventing repeat commands.
  bool landing_requested_{false};

  // Counts heartbeat/setpoint messages so PX4 receives a valid Offboard stream before arming.
  std::uint64_t warmup_message_count_{0};

  // Stores the moment the flight began so elapsed flight time can be calculated.
  std::chrono::steady_clock::time_point flight_started_at_{};

  // Number of 10 Hz messages needed for roughly one second of Offboard warm-up.
  static constexpr std::uint64_t kWarmupMessages = 10;


  // ============================================================
  // ROS 2 OBJECTS
  // ============================================================

  // Calls control_loop() every 100 ms so the Offboard messages keep being published.
  rclcpp::TimerBase::SharedPtr timer_;

  // Publishes the Offboard heartbeat that tells PX4 which control mode ROS 2 is using.
  rclcpp::Publisher<
    px4_msgs::msg::OffboardControlMode>::SharedPtr
    offboard_mode_publisher_;

  // Publishes the desired drone position to PX4 using TrajectorySetpoint messages.
  rclcpp::Publisher<
    px4_msgs::msg::TrajectorySetpoint>::SharedPtr
    trajectory_publisher_;

  // Publishes discrete PX4 commands such as mode changes, arming, and landing.
  rclcpp::Publisher<
    px4_msgs::msg::VehicleCommand>::SharedPtr
    vehicle_command_publisher_;


public:

  // ============================================================
  // CONSTRUCTOR
  // ============================================================

  // Creates the ROS 2 node, loads parameters, creates publishers, and starts the 10 Hz control timer.
  OffboardControl()
  : Node("offboard_control")
  {
    // Read whether automatic flight should start; defaults to enabled here.
    auto_start_ =
      declare_parameter<bool>("auto_start", true);

    // Read the target altitude; defaults to 2 metres.
    target_altitude_m_ =
      declare_parameter<double>("target_altitude_m", 2.0);

    // Read the desired flight duration; defaults to 15 seconds.
    flight_duration_s_ =
      declare_parameter<double>("flight_duration_s", 15.0);


    // Reject invalid altitude values before attempting flight.
    if (target_altitude_m_ <= 0.0) {
      throw std::invalid_argument(
        "target_altitude_m must be greater than zero");
    }

    // Reject invalid flight durations before attempting flight.
    if (flight_duration_s_ <= 0.0) {
      throw std::invalid_argument(
        "flight_duration_s must be greater than zero");
    }


    // Create the publisher that sends Offboard control-mode heartbeats to PX4.
    offboard_mode_publisher_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode",
        10);

    // Create the publisher that sends position targets to PX4.
    trajectory_publisher_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint",
        10);

    // Create the publisher that sends arm, mode-change, and land commands to PX4.
    vehicle_command_publisher_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command",
        10);


    // Run control_loop() every 100 ms, giving a 10 Hz Offboard update rate.
    timer_ = create_wall_timer(
      100ms,
      [this]() {
        control_loop();
      });


    // Print a warning if automatic flight is enabled.
    if (auto_start_) {
      RCLCPP_WARN(
        get_logger(),
        "Automatic flight enabled: target %.1f m for %.1f seconds",
        target_altitude_m_,
        flight_duration_s_);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Automatic flight disabled. Set auto_start:=true to fly.");
    }
  }


private:

  // ============================================================
  // MAIN FLIGHT SEQUENCE
  // ============================================================

  // Runs the flight sequence repeatedly: warm up Offboard, arm, hold position, then land.
  void control_loop()
  {
    // Stop here if automatic flight is disabled or landing has already been requested.
    if (!auto_start_ || landing_requested_) {
      return;
    }

    // Continuously tell PX4 that ROS 2 is alive and still commanding the same position.
    publish_offboard_heartbeat();
    publish_position_target();


    // Build up roughly one second of valid Offboard messages before requesting flight.
    if (!flight_started_) {
      ++warmup_message_count_;

      // Once enough messages have been sent, request Offboard mode and arm the drone.
      if (warmup_message_count_ >= kWarmupMessages) {
        request_offboard_mode();
        request_arm();

        // Save the start time so the program knows when the flight duration has expired.
        flight_started_at_ = std::chrono::steady_clock::now();
        flight_started_ = true;

        RCLCPP_INFO(
          get_logger(),
          "Requested Offboard mode and arming");
      }

      return;
    }


    // Request landing once the configured flight duration has passed.
    if (seconds_since_flight_started() >= flight_duration_s_) {
      request_land();
      landing_requested_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Requested landing");
    }
  }


  // ============================================================
  // OFFBOARD HEARTBEAT
  // ============================================================

  // Publishes OffboardControlMode so PX4 knows ROS 2 is alive and requesting position control.
  void publish_offboard_heartbeat()
  {
    // Create a new empty PX4 OffboardControlMode message.
    px4_msgs::msg::OffboardControlMode message{};

    // Timestamp the message using the current ROS 2 clock.
    message.timestamp = timestamp_us();

    // Enable position control because TrajectorySetpoint will contain a position target.
    message.position = true;

    // Disable control modes that this node is not using.
    message.velocity = false;
    message.acceleration = false;
    message.attitude = false;
    message.body_rate = false;
    message.thrust_and_torque = false;
    message.direct_actuator = false;

    // Send the heartbeat through ROS 2 to PX4.
    offboard_mode_publisher_->publish(message);
  }


  // ============================================================
  // POSITION TARGET
  // ============================================================

  // Publishes the desired NED position that PX4 should fly toward and hold.
  void publish_position_target()
  {
    // Create a new empty PX4 trajectory setpoint message.
    px4_msgs::msg::TrajectorySetpoint message{};

    // Timestamp the setpoint so PX4 knows when it was generated.
    message.timestamp = timestamp_us();

    /*
     * PX4 uses NED coordinates:
     * x = North
     * y = East
     * z = Down
     *
     * Negative z therefore means upward.
     */
    message.position = {
      0.0F,
      0.0F,
      -static_cast<float>(target_altitude_m_)
    };


    // NaN tells PX4 to ignore fields that this controller is not commanding.
    const float unused =
      std::numeric_limits<float>::quiet_NaN();

    message.velocity = {
      unused,
      unused,
      unused
    };

    message.acceleration = {
      unused,
      unused,
      unused
    };

    message.jerk = {
      unused,
      unused,
      unused
    };

    // Hold a yaw angle of zero radians.
    message.yaw = 0.0F;

    // Do not command a yaw rotation speed.
    message.yawspeed = unused;

    // Send the target position through ROS 2 to PX4.
    trajectory_publisher_->publish(message);
  }


  // ============================================================
  // VEHICLE COMMANDS
  // ============================================================

  // Requests PX4 Offboard flight mode using the generic VehicleCommand publisher.
  void request_offboard_mode()
  {
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
      1.0F,
      6.0F);
  }


  // Requests vehicle arming so PX4 is allowed to spin motors and fly.
  void request_arm()
  {
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::
      VEHICLE_CMD_COMPONENT_ARM_DISARM,
      1.0F);
  }


  // Requests PX4's built-in landing behavior.
  void request_land()
  {
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  }


  // Builds and sends a generic PX4 VehicleCommand used by arm, mode-change, and land helpers.
  void publish_vehicle_command(
    std::uint32_t command,
    float param1 = 0.0F,
    float param2 = 0.0F)
  {
    // Create a new empty PX4 command message.
    px4_msgs::msg::VehicleCommand message{};

    // Timestamp the command using the ROS 2 clock.
    message.timestamp = timestamp_us();

    // Store the command type and any command-specific parameters.
    message.command = command;
    message.param1 = param1;
    message.param2 = param2;

    // Identify PX4 system 1, component 1 as the command destination.
    message.target_system = 1;
    message.target_component = 1;

    // Identify the sender of the command.
    message.source_system = 1;
    message.source_component = 1;

    // Tell PX4 that this command came from an external controller such as ROS 2.
    message.from_external = true;

    // Send the completed command to PX4.
    vehicle_command_publisher_->publish(message);
  }


  // ============================================================
  // TIME HELPERS
  // ============================================================

  // Calculates elapsed flight time by comparing the current steady clock with flight_started_at_.
  double seconds_since_flight_started() const
  {
    // Subtract the saved flight start time from the current time.
    const auto elapsed =
      std::chrono::steady_clock::now() - flight_started_at_;

    // Convert the elapsed duration into seconds as a double.
    return std::chrono::duration<double>(elapsed).count();
  }


  // Returns the current ROS 2 clock time in microseconds for PX4 message timestamps.
  std::uint64_t timestamp_us() const
  {
    return static_cast<std::uint64_t>(
      get_clock()->now().nanoseconds() / 1000);
  }
};


// ============================================================
// PROGRAM ENTRY POINT
// ============================================================

// Starts ROS 2, creates the OffboardControl node, and keeps it running until shutdown.
int main(int argc, char * argv[])
{
  // Initialize ROS 2 and process any ROS-specific command-line arguments.
  rclcpp::init(argc, argv);

  // Create one OffboardControl node and store it in a shared smart pointer.
  auto node = std::make_shared<OffboardControl>();

  // Keep the node alive so ROS 2 can repeatedly run its timer callback.
  rclcpp::spin(node);

  // Shut ROS 2 down cleanly when the node stops.
  rclcpp::shutdown();

  return 0;
}