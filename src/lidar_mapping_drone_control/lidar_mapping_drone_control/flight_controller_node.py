"""FlightControllerNode wiring block.

This node connects the project-level control architecture:

position_reference -> OuterLoopXYPositionController -> reference_pitch_roll_angle
reference_pitch_roll_angle + estimated_states -> InnerLoopPitchRollController
yaw_reference + estimated_states -> YawController
altitude_reference + estimated_states -> AltitudeController
thrust/torques -> MotorMixer -> four rotor velocity commands

The node publishes actuator_msgs/Actuators.velocity[] to the ROS side of the
Gazebo motor bridge for `/X3/gazebo/command/motor_speed`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from .altitude_controller import AltitudeController, AltitudeControllerConfig
from .common import PIDGains, clamp, wrap_pi
from .inner_loop_pitch_roll_controller import (
    InnerLoopPitchRollController,
    PitchRollControllerConfig,
)
from .motor_mixer import MotorMixer, MotorMixerConfig
from .outer_loop_xy_position_controller import (
    OuterLoopXYPositionController,
    PitchRollReference,
    XYPositionControllerConfig,
)
from .safety_limiter import SafetyLimiter, SafetyLimiterConfig, zero_motor_command
from .state_estimator import StateEstimator
from .yaw_controller import YawController, YawControllerConfig


VALID_MODES = {
    "altitude_only",
    "attitude_hold",
    "position_hold",
    "manual_keyboard",
}


@dataclass
class ReferenceState:
    x_m: float
    y_m: float
    altitude_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float


class FlightControllerNode(Node):
    """ROS node that composes the inspectable controller blocks."""

    def __init__(self) -> None:
        super().__init__("flight_controller")
        self._declare_parameters()

        self._mode = self._parameter_string("controller_mode")
        if self._mode not in VALID_MODES:
            self.get_logger().warning(
                f"Unknown controller_mode '{self._mode}', falling back to altitude_only."
            )
            self._mode = "altitude_only"

        self._references = ReferenceState(
            x_m=self._parameter_float("target_x_m"),
            y_m=self._parameter_float("target_y_m"),
            altitude_m=self._parameter_float("target_altitude_m"),
            roll_rad=math.radians(self._parameter_float("target_roll_deg")),
            pitch_rad=math.radians(self._parameter_float("target_pitch_deg")),
            yaw_rad=math.radians(self._parameter_float("target_yaw_deg")),
        )
        self._emergency_stop = self._parameter_bool("emergency_stop")
        self._last_update_s = self.get_clock().now().nanoseconds * 1.0e-9
        self._last_safety_reason = ""

        gravity = self._parameter_float("gravity_mps2")
        mass = self._parameter_float("mass_kg")
        min_motor = self._parameter_float("motor_mixer.min_motor_speed_rad_s")
        max_motor = self._parameter_float("motor_mixer.max_motor_speed_rad_s")

        self._state_estimator = StateEstimator(
            node=self,
            tf_topic=self._parameter_string("tf_topic"),
            tracked_child_frame=self._parameter_string("tracked_child_frame"),
        )
        self._outer_loop = OuterLoopXYPositionController(
            XYPositionControllerConfig(
                kp=self._parameter_float("xy_position_controller.kp"),
                kd=self._parameter_float("xy_position_controller.kd"),
                max_accel_mps2=self._parameter_float(
                    "xy_position_controller.max_accel_mps2"
                ),
                max_tilt_rad=math.radians(
                    self._parameter_float("xy_position_controller.max_tilt_deg")
                ),
                gravity_mps2=gravity,
            )
        )
        self._inner_loop = InnerLoopPitchRollController(
            PitchRollControllerConfig(
                roll_gains=PIDGains(
                    kp=self._parameter_float("pitch_roll_controller.roll.kp"),
                    ki=self._parameter_float("pitch_roll_controller.roll.ki"),
                    kd=self._parameter_float("pitch_roll_controller.roll.kd"),
                    integral_limit=self._parameter_float(
                        "pitch_roll_controller.roll.integral_limit"
                    ),
                ),
                pitch_gains=PIDGains(
                    kp=self._parameter_float("pitch_roll_controller.pitch.kp"),
                    ki=self._parameter_float("pitch_roll_controller.pitch.ki"),
                    kd=self._parameter_float("pitch_roll_controller.pitch.kd"),
                    integral_limit=self._parameter_float(
                        "pitch_roll_controller.pitch.integral_limit"
                    ),
                ),
                torque_limit_nm=self._parameter_float(
                    "pitch_roll_controller.torque_limit_nm"
                ),
            )
        )
        self._yaw_controller = YawController(
            YawControllerConfig(
                gains=PIDGains(
                    kp=self._parameter_float("yaw_controller.kp"),
                    ki=self._parameter_float("yaw_controller.ki"),
                    kd=self._parameter_float("yaw_controller.kd"),
                    integral_limit=self._parameter_float(
                        "yaw_controller.integral_limit"
                    ),
                ),
                torque_limit_nm=self._parameter_float("yaw_controller.torque_limit_nm"),
            )
        )
        self._altitude_controller = AltitudeController(
            AltitudeControllerConfig(
                gains=PIDGains(
                    kp=self._parameter_float("altitude_controller.kp"),
                    ki=self._parameter_float("altitude_controller.ki"),
                    kd=self._parameter_float("altitude_controller.kd"),
                    integral_limit=self._parameter_float(
                        "altitude_controller.integral_limit"
                    ),
                ),
                mass_kg=mass,
                gravity_mps2=gravity,
                min_thrust_n=self._parameter_float("altitude_controller.min_thrust_n"),
                max_thrust_n=self._parameter_float("altitude_controller.max_thrust_n"),
            )
        )
        self._motor_mixer = MotorMixer(
            MotorMixerConfig(
                motor_constant_n_per_radps2=self._parameter_float(
                    "motor_mixer.motor_constant_n_per_radps2"
                ),
                moment_constant_m=self._parameter_float("motor_mixer.moment_constant_m"),
                rotor_x_m=self._parameter_float_array("motor_mixer.rotor_x_m"),
                rotor_y_m=self._parameter_float_array("motor_mixer.rotor_y_m"),
                rotor_spin_sign=self._parameter_float_array(
                    "motor_mixer.rotor_spin_sign"
                ),
                min_motor_speed_rad_s=min_motor,
                max_motor_speed_rad_s=max_motor,
            )
        )
        self._safety_limiter = SafetyLimiter(
            SafetyLimiterConfig(
                min_motor_speed_rad_s=min_motor,
                max_motor_speed_rad_s=max_motor,
                max_altitude_m=self._parameter_float("safety.max_altitude_m"),
                max_tilt_rad=math.radians(self._parameter_float("safety.max_tilt_deg")),
                state_timeout_s=self._parameter_float("safety.state_timeout_s"),
            )
        )

        self._motor_pub = self.create_publisher(
            Actuators,
            self._parameter_string("motor_command_topic"),
            10,
        )
        self._manual_sub = self.create_subscription(
            Twist,
            self._parameter_string("manual_reference_delta_topic"),
            self._handle_manual_delta,
            10,
        )
        self._estop_sub = self.create_subscription(
            Bool,
            self._parameter_string("emergency_stop_topic"),
            self._handle_emergency_stop,
            10,
        )
        self.add_on_set_parameters_callback(self._handle_parameter_update)

        control_rate_hz = self._parameter_float("control_rate_hz")
        self._timer = self.create_timer(1.0 / control_rate_hz, self._control_step)
        self.get_logger().info(
            f"Flight controller started in {self._mode} mode. "
            "Start the base simulation before enabling motor commands."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("controller_mode", "manual_keyboard")
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("mass_kg", 1.62)
        self.declare_parameter("gravity_mps2", 9.80665)
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("tracked_child_frame", "x3_lidar")
        self.declare_parameter(
            "motor_command_topic",
            "/X3/gazebo/command/motor_speed",
        )
        self.declare_parameter(
            "manual_reference_delta_topic",
            "/flight_controller/manual_reference_delta",
        )
        self.declare_parameter(
            "emergency_stop_topic",
            "/flight_controller/emergency_stop",
        )
        self.declare_parameter("emergency_stop", False)

        self.declare_parameter("target_altitude_m", 1.0)
        self.declare_parameter("target_x_m", 0.0)
        self.declare_parameter("target_y_m", 0.0)
        self.declare_parameter("target_yaw_deg", 0.0)
        self.declare_parameter("target_roll_deg", 0.0)
        self.declare_parameter("target_pitch_deg", 0.0)

        self.declare_parameter("xy_position_controller.kp", 0.30)
        self.declare_parameter("xy_position_controller.kd", 0.20)
        self.declare_parameter("xy_position_controller.max_accel_mps2", 1.0)
        self.declare_parameter("xy_position_controller.max_tilt_deg", 8.0)

        self.declare_parameter("pitch_roll_controller.roll.kp", 0.12)
        self.declare_parameter("pitch_roll_controller.roll.ki", 0.0)
        self.declare_parameter("pitch_roll_controller.roll.kd", 0.035)
        self.declare_parameter("pitch_roll_controller.roll.integral_limit", 0.0)
        self.declare_parameter("pitch_roll_controller.pitch.kp", 0.12)
        self.declare_parameter("pitch_roll_controller.pitch.ki", 0.0)
        self.declare_parameter("pitch_roll_controller.pitch.kd", 0.035)
        self.declare_parameter("pitch_roll_controller.pitch.integral_limit", 0.0)
        self.declare_parameter("pitch_roll_controller.torque_limit_nm", 0.18)

        self.declare_parameter("yaw_controller.kp", 0.05)
        self.declare_parameter("yaw_controller.ki", 0.0)
        self.declare_parameter("yaw_controller.kd", 0.015)
        self.declare_parameter("yaw_controller.integral_limit", 0.0)
        self.declare_parameter("yaw_controller.torque_limit_nm", 0.08)

        self.declare_parameter("altitude_controller.kp", 1.1)
        self.declare_parameter("altitude_controller.ki", 0.0)
        self.declare_parameter("altitude_controller.kd", 0.55)
        self.declare_parameter("altitude_controller.integral_limit", 1.0)
        self.declare_parameter("altitude_controller.min_thrust_n", 0.0)
        self.declare_parameter("altitude_controller.max_thrust_n", 35.0)

        self.declare_parameter(
            "motor_mixer.motor_constant_n_per_radps2",
            8.54858e-06,
        )
        self.declare_parameter("motor_mixer.moment_constant_m", 0.016)
        self.declare_parameter(
            "motor_mixer.rotor_x_m",
            [0.13, -0.13, 0.13, -0.13],
        )
        self.declare_parameter(
            "motor_mixer.rotor_y_m",
            [-0.22, 0.20, 0.22, -0.20],
        )
        self.declare_parameter(
            "motor_mixer.rotor_spin_sign",
            [1.0, 1.0, -1.0, -1.0],
        )
        self.declare_parameter("motor_mixer.min_motor_speed_rad_s", 0.0)
        self.declare_parameter("motor_mixer.max_motor_speed_rad_s", 800.0)

        self.declare_parameter("safety.max_altitude_m", 3.0)
        self.declare_parameter("safety.max_tilt_deg", 35.0)
        self.declare_parameter("safety.state_timeout_s", 0.5)

        self.declare_parameter("manual_keyboard.altitude_step_m", 0.10)
        self.declare_parameter("manual_keyboard.attitude_step_deg", 1.0)
        self.declare_parameter("manual_keyboard.yaw_step_deg", 5.0)

    def _control_step(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        dt_s = max(1.0e-3, min(0.1, now_s - self._last_update_s))
        self._last_update_s = now_s
        self._emergency_stop = self.get_parameter("emergency_stop").value or self._emergency_stop
        self._mode = self.get_parameter("controller_mode").value

        state = self._state_estimator.state
        state_age_s = self._state_estimator.state_age_s(now_s)
        if state is None:
            result = self._safety_limiter.apply(
                zero_motor_command(),
                state,
                state_age_s,
                self._emergency_stop,
            )
            self._publish_motor_command(result.command)
            self._log_safety_result(result.triggered, result.reason)
            return

        pitch_roll_ref = self._reference_pitch_roll_for_mode(state)
        torque_command = self._inner_loop.update(
            reference_roll_rad=pitch_roll_ref.roll_rad,
            reference_pitch_rad=pitch_roll_ref.pitch_rad,
            state=state,
            dt_s=dt_s,
        )
        yaw_torque = self._yaw_controller.update(
            self._references.yaw_rad,
            state,
            dt_s,
        )
        thrust = self._altitude_controller.update(
            self._references.altitude_m,
            state,
            dt_s,
        )
        raw_motor_command = self._motor_mixer.mix(
            thrust_command_n=thrust,
            roll_torque_command_nm=torque_command.roll_torque_nm,
            pitch_torque_command_nm=torque_command.pitch_torque_nm,
            yaw_torque_command_nm=yaw_torque,
        )
        safe_result = self._safety_limiter.apply(
            raw_motor_command,
            state,
            state_age_s,
            self._emergency_stop,
        )
        self._publish_motor_command(safe_result.command)
        self._log_safety_result(safe_result.triggered, safe_result.reason)

    def _reference_pitch_roll_for_mode(self, state) -> PitchRollReference:
        if self._mode == "position_hold":
            return self._outer_loop.update(self._references.x_m, self._references.y_m, state)
        if self._mode == "manual_keyboard":
            return PitchRollReference(
                pitch_rad=self._references.pitch_rad,
                roll_rad=self._references.roll_rad,
            )
        if self._mode == "attitude_hold":
            return PitchRollReference(
                pitch_rad=self._references.pitch_rad,
                roll_rad=self._references.roll_rad,
            )
        if self._mode != "altitude_only":
            self.get_logger().warning(
                f"Unknown controller_mode '{self._mode}', using altitude_only behavior."
            )
        return PitchRollReference(pitch_rad=0.0, roll_rad=0.0)

    def _handle_manual_delta(self, msg: Twist) -> None:
        if self._mode != "manual_keyboard":
            return
        max_tilt = math.radians(self._parameter_float("xy_position_controller.max_tilt_deg"))
        self._references.altitude_m = clamp(
            self._references.altitude_m + msg.linear.z,
            0.0,
            self._parameter_float("safety.max_altitude_m"),
        )
        self._references.pitch_rad = clamp(
            self._references.pitch_rad + msg.linear.x,
            -max_tilt,
            max_tilt,
        )
        self._references.roll_rad = clamp(
            self._references.roll_rad + msg.linear.y,
            -max_tilt,
            max_tilt,
        )
        self._references.yaw_rad = wrap_pi(self._references.yaw_rad + msg.angular.z)
        self.get_logger().info(
            "Manual refs: "
            f"z={self._references.altitude_m:.2f}m "
            f"pitch={math.degrees(self._references.pitch_rad):.1f}deg "
            f"roll={math.degrees(self._references.roll_rad):.1f}deg "
            f"yaw={math.degrees(self._references.yaw_rad):.1f}deg"
        )

    def _handle_emergency_stop(self, msg: Bool) -> None:
        self._emergency_stop = bool(msg.data)
        level = (
            self.get_logger().warning
            if self._emergency_stop
            else self.get_logger().info
        )
        level(f"Emergency stop set to {self._emergency_stop}.")

    def _handle_parameter_update(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "emergency_stop":
                self._emergency_stop = bool(parameter.value)
            elif parameter.name == "controller_mode":
                if parameter.value not in VALID_MODES:
                    return SetParametersResult(
                        successful=False,
                        reason=f"controller_mode must be one of {sorted(VALID_MODES)}",
                    )
                self._mode = str(parameter.value)
            elif parameter.name == "target_altitude_m":
                self._references.altitude_m = float(parameter.value)
            elif parameter.name == "target_x_m":
                self._references.x_m = float(parameter.value)
            elif parameter.name == "target_y_m":
                self._references.y_m = float(parameter.value)
            elif parameter.name == "target_yaw_deg":
                self._references.yaw_rad = math.radians(float(parameter.value))
            elif parameter.name == "target_roll_deg":
                self._references.roll_rad = math.radians(float(parameter.value))
            elif parameter.name == "target_pitch_deg":
                self._references.pitch_rad = math.radians(float(parameter.value))
        return SetParametersResult(successful=True)

    def _publish_motor_command(self, command) -> None:
        msg = Actuators()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.velocity = command.as_velocity_list()
        self._motor_pub.publish(msg)

    def _log_safety_result(self, triggered: bool, reason: str) -> None:
        if triggered and reason != self._last_safety_reason:
            self.get_logger().warning(f"Safety limiter stopped motors: {reason}")
        elif not triggered and self._last_safety_reason:
            self.get_logger().info("Safety limiter cleared; motor commands enabled.")
        self._last_safety_reason = reason if triggered else ""

    def _parameter_string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _parameter_bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def _parameter_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _parameter_float_array(self, name: str) -> list[float]:
        return [float(value) for value in self.get_parameter(name).value]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlightControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
