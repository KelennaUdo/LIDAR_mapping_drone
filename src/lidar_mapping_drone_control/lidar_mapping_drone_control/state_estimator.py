"""StateEstimator block.

This block represents the "estimated_states" signal in the flight-control
architecture. Its current input is the bridged Gazebo TF stream. Its output is
an EstimatedState object containing position, attitude, and numerically
estimated velocities. Later, IMU or odometry inputs can be added here without
changing the downstream controller blocks.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from .common import wrap_pi


@dataclass
class EstimatedState:
    """Clean state object consumed by the controller blocks."""

    timestamp_s: float
    received_time_s: float
    parent_frame_id: str
    child_frame_id: str
    x_m: float
    y_m: float
    z_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    vx_mps: float
    vy_mps: float
    vz_mps: float
    roll_rate_radps: float
    pitch_rate_radps: float
    yaw_rate_radps: float


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Convert quaternion to roll, pitch, yaw using the ROS ENU convention."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class StateEstimator:
    """Subscribes to TF and exposes the latest estimated drone state."""

    def __init__(
        self,
        node: Node,
        tf_topic: str,
        tracked_child_frame: str,
    ) -> None:
        self._node = node
        self._tracked_child_frame = tracked_child_frame
        self._state: Optional[EstimatedState] = None
        self._previous_state: Optional[EstimatedState] = None
        self._subscription = node.create_subscription(
            TFMessage,
            tf_topic,
            self._handle_tf,
            20,
        )

    @property
    def state(self) -> Optional[EstimatedState]:
        return self._state

    def state_age_s(self, now_s: float) -> Optional[float]:
        if self._state is None:
            return None
        return max(0.0, now_s - self._state.received_time_s)

    def _handle_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            if transform.child_frame_id != self._tracked_child_frame:
                continue

            self._previous_state = self._state
            self._state = self._state_from_transform(transform)

    def _state_from_transform(self, transform) -> EstimatedState:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        roll, pitch, yaw = quaternion_to_euler(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )

        stamp_s = stamp_to_seconds(transform.header.stamp)
        if stamp_s <= 0.0:
            stamp_s = self._node.get_clock().now().nanoseconds * 1.0e-9
        received_s = self._node.get_clock().now().nanoseconds * 1.0e-9

        vx = vy = vz = 0.0
        roll_rate = pitch_rate = yaw_rate = 0.0
        if self._previous_state is not None:
            dt = stamp_s - self._previous_state.timestamp_s
            if dt <= 0.0:
                dt = received_s - self._previous_state.received_time_s
            if dt > 1.0e-6:
                vx = (translation.x - self._previous_state.x_m) / dt
                vy = (translation.y - self._previous_state.y_m) / dt
                vz = (translation.z - self._previous_state.z_m) / dt
                roll_rate = wrap_pi(roll - self._previous_state.roll_rad) / dt
                pitch_rate = wrap_pi(pitch - self._previous_state.pitch_rad) / dt
                yaw_rate = wrap_pi(yaw - self._previous_state.yaw_rad) / dt

        return EstimatedState(
            timestamp_s=stamp_s,
            received_time_s=received_s,
            parent_frame_id=transform.header.frame_id,
            child_frame_id=transform.child_frame_id,
            x_m=translation.x,
            y_m=translation.y,
            z_m=translation.z,
            roll_rad=roll,
            pitch_rad=pitch,
            yaw_rad=yaw,
            vx_mps=vx,
            vy_mps=vy,
            vz_mps=vz,
            roll_rate_radps=roll_rate,
            pitch_rate_radps=pitch_rate,
            yaw_rate_radps=yaw_rate,
        )
