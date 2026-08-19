#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


// Temporarily puts this terminal into single-key mode and restores it on exit.
class TerminalInput
{
private:
  termios original_settings_{};
  bool settings_changed_{false};

public:
  TerminalInput()
  {
    if (!isatty(STDIN_FILENO)) {
      throw std::runtime_error(
              "Keyboard input requires an interactive terminal");
    }

    if (tcgetattr(STDIN_FILENO, &original_settings_) != 0) {
      throw std::runtime_error("Could not read terminal settings");
    }

    termios single_key_settings = original_settings_;
    single_key_settings.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    single_key_settings.c_cc[VMIN] = 0;
    single_key_settings.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &single_key_settings) != 0) {
      throw std::runtime_error("Could not enable single-key input");
    }

    settings_changed_ = true;
  }

  ~TerminalInput()
  {
    if (settings_changed_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_settings_);
    }
  }

  // Returns one pressed key, or zero when no key is waiting.
  char read_key() const
  {
    char key = 0;
    const ssize_t bytes_read = read(STDIN_FILENO, &key, 1);
    return bytes_read == 1 ? key : 0;
  }
};


class OffboardTeleop : public rclcpp::Node
{
private:

  // ============================================================
  // NODE STATE
  // ============================================================

  // Initial height target used when T requests takeoff.
  double takeoff_altitude_m_{2.0};

  // Distance added to the horizontal target by each movement key press.
  double movement_step_m_{0.5};

  // Distance added to the altitude target by each R or F key press.
  double altitude_step_m_{0.25};

  // Angle added to the yaw target by each Q or E key press.
  double yaw_step_deg_{10.0};

  // Persistent NED position and yaw targets sent to PX4 at 10 Hz.
  float target_x_m_{0.0F};
  float target_y_m_{0.0F};
  float target_z_m_{-2.0F};
  float target_yaw_rad_{0.0F};

  // Flight requests are separated so pressing 'T' never arms before heartbeat warm-up.
  bool takeoff_requested_{false};
  bool flight_enabled_{false};
  bool landing_requested_{false};
  bool emergency_stop_requested_{false};

  // PX4 requires a valid Offboard stream before it will enter Offboard mode.
  std::uint64_t warmup_message_count_{0};
  static constexpr std::uint64_t kWarmupMessages = 10;

  // Conservative target bounds for this simulation learning tool.
  static constexpr float kMinimumAltitudeM = 0.3F;
  static constexpr float kMaximumAltitudeM = 10.0F;


  // ============================================================
  // ROS 2 AND TERMINAL OBJECTS
  // ============================================================

  // Owns the terminal mode for this node's lifetime.
  TerminalInput terminal_input_;

  // Runs keyboard polling and Offboard publishing at 10 Hz.
  rclcpp::TimerBase::SharedPtr timer_;

  // Publishes the heartbeat declaring that position control is active.
  rclcpp::Publisher<
    px4_msgs::msg::OffboardControlMode>::SharedPtr
    offboard_mode_publisher_;

  // Publishes the position and yaw target changed by keyboard input.
  rclcpp::Publisher<
    px4_msgs::msg::TrajectorySetpoint>::SharedPtr
    trajectory_publisher_;

  // Publishes mode, arm, land, and emergency disarm requests.
  rclcpp::Publisher<
    px4_msgs::msg::VehicleCommand>::SharedPtr
    vehicle_command_publisher_;


public:

  // ============================================================
  // CONSTRUCTOR
  // ============================================================

  OffboardTeleop()
  : Node("offboard_teleop")
  {
    takeoff_altitude_m_ =
      declare_parameter<double>("takeoff_altitude_m", 2.0);
    movement_step_m_ =
      declare_parameter<double>("movement_step_m", 0.5);
    altitude_step_m_ =
      declare_parameter<double>("altitude_step_m", 0.25);
    yaw_step_deg_ =
      declare_parameter<double>("yaw_step_deg", 10.0);

    validate_parameters();
    target_z_m_ = -static_cast<float>(takeoff_altitude_m_);

    offboard_mode_publisher_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);

    trajectory_publisher_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);

    vehicle_command_publisher_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    timer_ = create_wall_timer(
      100ms,
      [this]() {
        control_loop();
      });

    print_controls();
    print_target();
  }


private:

  // ============================================================
  // MAIN CONTROL LOOP
  // ============================================================

  // Reads one key and keeps the PX4 Offboard stream alive at 10 Hz.
  void control_loop()
  {
    handle_key(terminal_input_.read_key());

    if (landing_requested_ || emergency_stop_requested_) {
      return;
    }

    publish_offboard_heartbeat();
    publish_position_target();

    if (warmup_message_count_ < kWarmupMessages) {
      ++warmup_message_count_;
    }

    if (takeoff_requested_ &&
      !flight_enabled_ &&
      warmup_message_count_ >= kWarmupMessages)
    {
      request_offboard_mode();
      request_arm();
      flight_enabled_ = true;

      RCLCPP_WARN(
        get_logger(),
        "Requested Offboard mode and arming");
    }
  }


  // ============================================================
  // KEYBOARD INPUT
  // ============================================================

  // Converts a key press into a flight request or a target adjustment.
  void handle_key(char key)
  {
    if (key == 0) {
      return;
    }

    if (key == 'h' || key == 'H' || key == '?') {
      print_controls();
      return;
    }

    if (key == 't' || key == 'T') {
      if (!flight_enabled_ && !takeoff_requested_) {
        takeoff_requested_ = true;
        RCLCPP_WARN(get_logger(), "Takeoff requested");
      }
      return;
    }

    if (key == 'l' || key == 'L') {
      if (flight_enabled_) {
        request_land();
        landing_requested_ = true;
        RCLCPP_WARN(get_logger(), "Landing requested");
      }
      return;
    }

    // Uppercase X is deliberately harder to press because it cuts motor power.
    if (key == 'X') {
      request_emergency_disarm();
      emergency_stop_requested_ = true;
      RCLCPP_ERROR(get_logger(), "EMERGENCY MOTOR STOP requested");
      return;
    }

    if (!flight_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "Press T to take off before sending movement commands");
      return;
    }

    const float movement_step = static_cast<float>(movement_step_m_);
    const float altitude_step = static_cast<float>(altitude_step_m_);
    const float yaw_step =
      static_cast<float>(yaw_step_deg_ * std::acos(-1.0) / 180.0);

    switch (key) {
      case 'w':
      case 'W':
        move_forward(movement_step);
        break;
      case 's':
      case 'S':
        move_forward(-movement_step);
        break;
      case 'a':
      case 'A':
        move_right(-movement_step);
        break;
      case 'd':
      case 'D':
        move_right(movement_step);
        break;
      case 'r':
      case 'R':
        target_z_m_ = std::max(
          target_z_m_ - altitude_step,
          -kMaximumAltitudeM);
        break;
      case 'f':
      case 'F':
        target_z_m_ = std::min(
          target_z_m_ + altitude_step,
          -kMinimumAltitudeM);
        break;
      case 'q':
      case 'Q':
        target_yaw_rad_ -= yaw_step;
        break;
      case 'e':
      case 'E':
        target_yaw_rad_ += yaw_step;
        break;
      default:
        return;
    }

    print_target();
  }

  // Moves the target forward relative to its commanded yaw direction.
  void move_forward(float distance_m)
  {
    target_x_m_ += std::cos(target_yaw_rad_) * distance_m;
    target_y_m_ += std::sin(target_yaw_rad_) * distance_m;
  }

  // Moves the target right relative to its commanded yaw direction.
  void move_right(float distance_m)
  {
    target_x_m_ -= std::sin(target_yaw_rad_) * distance_m;
    target_y_m_ += std::cos(target_yaw_rad_) * distance_m;
  }


  // ============================================================
  // PX4 SETPOINTS
  // ============================================================

  // Publishes the heartbeat that requests PX4 position control.
  void publish_offboard_heartbeat()
  {
    px4_msgs::msg::OffboardControlMode message{};
    message.timestamp = timestamp_us();
    message.position = true;
    message.velocity = false;
    message.acceleration = false;
    message.attitude = false;
    message.body_rate = false;
    message.thrust_and_torque = false;
    message.direct_actuator = false;
    offboard_mode_publisher_->publish(message);
  }

  // Publishes the persistent target; no key press means PX4 holds this point.
  void publish_position_target()
  {
    const float unused = std::numeric_limits<float>::quiet_NaN();

    px4_msgs::msg::TrajectorySetpoint message{};
    message.timestamp = timestamp_us();
    message.position = {target_x_m_, target_y_m_, target_z_m_};
    message.velocity = {unused, unused, unused};
    message.acceleration = {unused, unused, unused};
    message.jerk = {unused, unused, unused};
    message.yaw = target_yaw_rad_;
    message.yawspeed = unused;
    trajectory_publisher_->publish(message);
  }


  // ============================================================
  // VEHICLE COMMANDS
  // ============================================================

  void request_offboard_mode()
  {
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
      1.0F,
      6.0F);
  }

  void request_arm()
  {
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      1.0F);
  }

  void request_land()
  {
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  }

  // Uses PX4's force-disarm code; this is for simulation emergencies only.
  void request_emergency_disarm()
  {
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      0.0F,
      21196.0F);
  }

  void publish_vehicle_command(
    std::uint32_t command,
    float param1 = 0.0F,
    float param2 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand message{};
    message.timestamp = timestamp_us();
    message.command = command;
    message.param1 = param1;
    message.param2 = param2;
    message.target_system = 1;
    message.target_component = 1;
    message.source_system = 1;
    message.source_component = 1;
    message.from_external = true;
    vehicle_command_publisher_->publish(message);
  }


  // ============================================================
  // VALIDATION AND OUTPUT
  // ============================================================

  void validate_parameters() const
  {
    if (takeoff_altitude_m_ < kMinimumAltitudeM ||
      takeoff_altitude_m_ > kMaximumAltitudeM)
    {
      throw std::invalid_argument(
              "takeoff_altitude_m must be between 0.3 and 10.0");
    }

    if (movement_step_m_ <= 0.0 ||
      altitude_step_m_ <= 0.0 ||
      yaw_step_deg_ <= 0.0)
    {
      throw std::invalid_argument("teleop step sizes must be positive");
    }
  }

  void print_controls() const
  {
    RCLCPP_INFO(
      get_logger(),
      "Controls: T take off | W/S forward/back | A/D left/right | "
      "R/F up/down | Q/E yaw | L land | Shift+X EMERGENCY STOP | H help");
    RCLCPP_INFO(
      get_logger(),
      "Wait for QGroundControl to show Ready before pressing T");
  }

  void print_target() const
  {
    RCLCPP_INFO(
      get_logger(),
      "Target NED: x=%.2f m y=%.2f m z=%.2f m yaw=%.1f deg",
      target_x_m_,
      target_y_m_,
      target_z_m_,
      target_yaw_rad_ * 180.0F / static_cast<float>(std::acos(-1.0)));
  }

  std::uint64_t timestamp_us() const
  {
    return static_cast<std::uint64_t>(
      get_clock()->now().nanoseconds() / 1000);
  }
};


// ============================================================
// PROGRAM ENTRY POINT
// ============================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<OffboardTeleop>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "offboard_teleop: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
