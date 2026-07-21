"""Hybrid state estimator used directly by the flight controller.

The estimator assembles one EstimatedState from three sources:
- Gazebo TF supplies world-frame x/y position and horizontal velocity.
- The simulated IMU supplies body attitude and angular velocity.
- The downward range sensor supplies altitude and vertical velocity.

TF still carries the fixed model-to-sensor mounting transforms needed to put
the IMU and range measurements into the x3_lidar model frame. TF altitude and
attitude are not used in the controller state.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Range
from tf2_msgs.msg import TFMessage

from .common import clamp


Quaternion = tuple[float, float, float, float]
Vector3 = tuple[float, float, float]


class EstimatorPhase(str, Enum):
    """Startup and flight phases for interpreting downward range."""

    UNINITIALIZED = "uninitialized"
    LANDED = "landed"
    AIRBORNE = "airborne"


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


@dataclass
class PoseSample:
    timestamp_s: float
    received_time_s: float
    parent_frame_id: str
    child_frame_id: str
    x_m: float
    y_m: float
    vx_mps: float
    vy_mps: float


@dataclass
class ImuSample:
    timestamp_s: float
    received_time_s: float
    orientation: Quaternion
    angular_velocity: Vector3


@dataclass
class RangeSample:
    timestamp_s: float
    received_time_s: float
    range_m: float
    min_range_m: float
    max_range_m: float

    @property
    def valid(self) -> bool:
        return (
            math.isfinite(self.range_m)
            and self.min_range_m <= self.range_m <= self.max_range_m
        )


@dataclass
class RigidTransform:
    translation: Vector3
    rotation: Quaternion


@dataclass
class HybridEstimatorConfig:
    tracked_child_frame: str
    imu_link_frame: str
    range_link_frame: str
    ground_z_m: float
    vertical_velocity_filter_alpha: float


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def normalize_quaternion(value: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(component * component for component in value))
    if norm <= 1.0e-12:
        return 0.0, 0.0, 0.0, 1.0
    return tuple(component / norm for component in value)


def quaternion_conjugate(value: Quaternion) -> Quaternion:
    return -value[0], -value[1], -value[2], value[3]


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def rotate_vector(rotation: Quaternion, vector: Vector3) -> Vector3:
    rotation = normalize_quaternion(rotation)
    pure = vector[0], vector[1], vector[2], 0.0
    rotated = quaternion_multiply(
        quaternion_multiply(rotation, pure),
        quaternion_conjugate(rotation),
    )
    return rotated[0], rotated[1], rotated[2]


def quaternion_to_euler(value: Quaternion) -> tuple[float, float, float]:
    """Convert a quaternion to roll, pitch, yaw using ROS ENU conventions."""
    x, y, z, w = normalize_quaternion(value)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(clamp(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert roll, pitch, yaw into an x/y/z/w quaternion."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return normalize_quaternion(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


def model_altitude_from_range(
    range_m: float,
    ground_z_m: float,
    model_orientation: Quaternion,
    model_to_range: RigidTransform,
) -> Optional[float]:
    """Recover model-frame world altitude from a downward beam measurement."""
    model_orientation = normalize_quaternion(model_orientation)
    sensor_offset_world = rotate_vector(
        model_orientation,
        model_to_range.translation,
    )
    sensor_orientation_world = quaternion_multiply(
        model_orientation,
        normalize_quaternion(model_to_range.rotation),
    )
    # Gazebo gpu_lidar rays point along the sensor frame's local +X axis.
    beam_world = rotate_vector(sensor_orientation_world, (1.0, 0.0, 0.0))
    if beam_world[2] >= -1.0e-6:
        return None
    return ground_z_m - sensor_offset_world[2] - range_m * beam_world[2]


class HybridEstimatorCore:
    """Pure state assembly and landed/airborne transition logic."""

    def __init__(self, config: HybridEstimatorConfig) -> None:
        if not 0.0 < config.vertical_velocity_filter_alpha <= 1.0:
            raise ValueError("vertical_velocity_filter_alpha must be in (0, 1].")
        self.config = config
        self.phase = EstimatorPhase.UNINITIALIZED
        self.state: Optional[EstimatedState] = None
        self.pose: Optional[PoseSample] = None
        self.imu: Optional[ImuSample] = None
        self.downward_range: Optional[RangeSample] = None
        self.model_to_imu: Optional[RigidTransform] = None
        self.model_to_range: Optional[RigidTransform] = None
        self._previous_tf_timestamp_s: Optional[float] = None
        self._previous_tf_x_m = 0.0
        self._previous_tf_y_m = 0.0
        self._last_valid_range_received_s: Optional[float] = None
        self._last_processed_range_timestamp_s: Optional[float] = None
        self._altitude_m = config.ground_z_m
        self._vertical_velocity_mps = 0.0
        self._previous_altitude_m: Optional[float] = None
        self._previous_altitude_timestamp_s: Optional[float] = None

    def update_mount_transform(
        self,
        parent_frame_id: str,
        child_frame_id: str,
        translation: Vector3,
        rotation: Quaternion,
    ) -> None:
        if parent_frame_id != self.config.tracked_child_frame:
            return
        rigid_transform = RigidTransform(
            translation=translation,
            rotation=normalize_quaternion(rotation),
        )
        if child_frame_id == self.config.imu_link_frame:
            self.model_to_imu = rigid_transform
        elif child_frame_id == self.config.range_link_frame:
            self.model_to_range = rigid_transform

    def update_pose(
        self,
        timestamp_s: float,
        received_time_s: float,
        parent_frame_id: str,
        child_frame_id: str,
        x_m: float,
        y_m: float,
    ) -> None:
        vx_mps = 0.0
        vy_mps = 0.0
        if self._previous_tf_timestamp_s is not None:
            dt_s = timestamp_s - self._previous_tf_timestamp_s
            if dt_s > 1.0e-6:
                vx_mps = (x_m - self._previous_tf_x_m) / dt_s
                vy_mps = (y_m - self._previous_tf_y_m) / dt_s
        self.pose = PoseSample(
            timestamp_s=timestamp_s,
            received_time_s=received_time_s,
            parent_frame_id=parent_frame_id,
            child_frame_id=child_frame_id,
            x_m=x_m,
            y_m=y_m,
            vx_mps=vx_mps,
            vy_mps=vy_mps,
        )
        self._previous_tf_timestamp_s = timestamp_s
        self._previous_tf_x_m = x_m
        self._previous_tf_y_m = y_m
        self._refresh_state(received_time_s)

    def update_imu(
        self,
        timestamp_s: float,
        received_time_s: float,
        orientation: Quaternion,
        angular_velocity: Vector3,
    ) -> None:
        self.imu = ImuSample(
            timestamp_s=timestamp_s,
            received_time_s=received_time_s,
            orientation=normalize_quaternion(orientation),
            angular_velocity=angular_velocity,
        )
        self._refresh_state(received_time_s)

    def update_range(
        self,
        timestamp_s: float,
        received_time_s: float,
        range_m: float,
        min_range_m: float,
        max_range_m: float,
    ) -> None:
        self.downward_range = RangeSample(
            timestamp_s=timestamp_s,
            received_time_s=received_time_s,
            range_m=range_m,
            min_range_m=min_range_m,
            max_range_m=max_range_m,
        )
        self._refresh_state(received_time_s)

    def state_age_s(self, now_s: float) -> Optional[float]:
        if self.state is None or self.pose is None or self.imu is None:
            return None
        required_receipts = [
            self.pose.received_time_s,
            self.imu.received_time_s,
        ]
        if self.phase == EstimatorPhase.LANDED:
            if self.downward_range is None:
                return None
            required_receipts.append(self.downward_range.received_time_s)
        else:
            if self._last_valid_range_received_s is None:
                return None
            required_receipts.append(self._last_valid_range_received_s)
        return max(0.0, now_s - min(required_receipts))

    def status_text(self, now_s: float) -> str:
        missing = []
        if self.pose is None:
            missing.append("tf_xy")
        if self.imu is None:
            missing.append("imu")
        if self.downward_range is None:
            missing.append("range")
        if self.model_to_imu is None:
            missing.append("imu_mount_tf")
        if self.model_to_range is None:
            missing.append("range_mount_tf")
        if missing:
            return f"{self.phase.value}:missing={','.join(missing)}"
        range_state = "valid" if self.downward_range.valid else "invalid"
        age_s = self.state_age_s(now_s)
        age_text = "none" if age_s is None else f"{age_s:.3f}s"
        return f"{self.phase.value}:range={range_state}:state_age={age_text}"

    def _refresh_state(self, received_time_s: float) -> None:
        if (
            self.pose is None
            or self.imu is None
            or self.downward_range is None
            or self.model_to_imu is None
            or self.model_to_range is None
        ):
            return

        imu_to_model = quaternion_conjugate(self.model_to_imu.rotation)
        model_orientation = quaternion_multiply(
            self.imu.orientation,
            imu_to_model,
        )
        angular_velocity_model = rotate_vector(
            self.model_to_imu.rotation,
            self.imu.angular_velocity,
        )

        if self.phase == EstimatorPhase.UNINITIALIZED:
            self.phase = (
                EstimatorPhase.AIRBORNE
                if self.downward_range.valid
                else EstimatorPhase.LANDED
            )

        if self.phase == EstimatorPhase.LANDED:
            self._altitude_m = self.config.ground_z_m
            self._vertical_velocity_mps = 0.0
            if self.downward_range.valid:
                altitude_m = model_altitude_from_range(
                    self.downward_range.range_m,
                    self.config.ground_z_m,
                    model_orientation,
                    self.model_to_range,
                )
                if altitude_m is not None:
                    self.phase = EstimatorPhase.AIRBORNE
                    self._accept_altitude(altitude_m)
        elif self.downward_range.valid:
            altitude_m = model_altitude_from_range(
                self.downward_range.range_m,
                self.config.ground_z_m,
                model_orientation,
                self.model_to_range,
            )
            if altitude_m is not None:
                self._accept_altitude(altitude_m)

        roll, pitch, yaw = quaternion_to_euler(model_orientation)
        self.state = EstimatedState(
            timestamp_s=max(
                self.pose.timestamp_s,
                self.imu.timestamp_s,
                self.downward_range.timestamp_s,
            ),
            received_time_s=received_time_s,
            parent_frame_id=self.pose.parent_frame_id,
            child_frame_id=self.pose.child_frame_id,
            x_m=self.pose.x_m,
            y_m=self.pose.y_m,
            z_m=self._altitude_m,
            roll_rad=roll,
            pitch_rad=pitch,
            yaw_rad=yaw,
            vx_mps=self.pose.vx_mps,
            vy_mps=self.pose.vy_mps,
            vz_mps=self._vertical_velocity_mps,
            roll_rate_radps=angular_velocity_model[0],
            pitch_rate_radps=angular_velocity_model[1],
            yaw_rate_radps=angular_velocity_model[2],
        )

    def _accept_altitude(self, altitude_m: float) -> None:
        if self.downward_range is None:
            return
        timestamp_s = self.downward_range.timestamp_s
        if timestamp_s == self._last_processed_range_timestamp_s:
            return

        raw_vertical_velocity_mps = 0.0
        if (
            self._previous_altitude_m is not None
            and self._previous_altitude_timestamp_s is not None
        ):
            dt_s = timestamp_s - self._previous_altitude_timestamp_s
            if dt_s > 1.0e-6:
                raw_vertical_velocity_mps = (
                    altitude_m - self._previous_altitude_m
                ) / dt_s
                alpha = self.config.vertical_velocity_filter_alpha
                self._vertical_velocity_mps = (
                    alpha * raw_vertical_velocity_mps
                    + (1.0 - alpha) * self._vertical_velocity_mps
                )
        else:
            self._vertical_velocity_mps = 0.0

        self._altitude_m = altitude_m
        self._previous_altitude_m = altitude_m
        self._previous_altitude_timestamp_s = timestamp_s
        self._last_processed_range_timestamp_s = timestamp_s
        self._last_valid_range_received_s = self.downward_range.received_time_s


class StateEstimator:
    """ROS subscriptions around HybridEstimatorCore."""

    def __init__(
        self,
        node: Node,
        tf_topic: str,
        imu_topic: str,
        range_topic: str,
        tracked_child_frame: str,
        imu_link_frame: str,
        range_link_frame: str,
        ground_z_m: float,
        vertical_velocity_filter_alpha: float,
    ) -> None:
        self._node = node
        self._core = HybridEstimatorCore(
            HybridEstimatorConfig(
                tracked_child_frame=tracked_child_frame,
                imu_link_frame=imu_link_frame,
                range_link_frame=range_link_frame,
                ground_z_m=ground_z_m,
                vertical_velocity_filter_alpha=vertical_velocity_filter_alpha,
            )
        )
        self._last_phase = self._core.phase
        self._tf_subscription = node.create_subscription(
            TFMessage,
            tf_topic,
            self._handle_tf,
            20,
        )
        self._imu_subscription = node.create_subscription(
            Imu,
            imu_topic,
            self._handle_imu,
            qos_profile_sensor_data,
        )
        self._range_subscription = node.create_subscription(
            Range,
            range_topic,
            self._handle_range,
            qos_profile_sensor_data,
        )

    @property
    def state(self) -> Optional[EstimatedState]:
        return self._core.state

    @property
    def phase(self) -> EstimatorPhase:
        return self._core.phase

    def state_age_s(self, now_s: float) -> Optional[float]:
        return self._core.state_age_s(now_s)

    def status_text(self, now_s: float) -> str:
        return self._core.status_text(now_s)

    def _now_s(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1.0e-9

    def _message_time_s(self, stamp, fallback_s: float) -> float:
        timestamp_s = stamp_to_seconds(stamp)
        return timestamp_s if timestamp_s > 0.0 else fallback_s

    def _handle_tf(self, msg: TFMessage) -> None:
        received_s = self._now_s()
        tracked_transform = None
        for transform in msg.transforms:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self._core.update_mount_transform(
                transform.header.frame_id,
                transform.child_frame_id,
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w),
            )
            if transform.child_frame_id == self._core.config.tracked_child_frame:
                tracked_transform = transform

        if tracked_transform is not None:
            translation = tracked_transform.transform.translation
            self._core.update_pose(
                timestamp_s=self._message_time_s(
                    tracked_transform.header.stamp,
                    received_s,
                ),
                received_time_s=received_s,
                parent_frame_id=tracked_transform.header.frame_id,
                child_frame_id=tracked_transform.child_frame_id,
                x_m=translation.x,
                y_m=translation.y,
            )
        self._log_phase_transition()

    def _handle_imu(self, msg: Imu) -> None:
        received_s = self._now_s()
        self._core.update_imu(
            timestamp_s=self._message_time_s(msg.header.stamp, received_s),
            received_time_s=received_s,
            orientation=(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ),
            angular_velocity=(
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ),
        )
        self._log_phase_transition()

    def _handle_range(self, msg: Range) -> None:
        received_s = self._now_s()
        self._core.update_range(
            timestamp_s=self._message_time_s(msg.header.stamp, received_s),
            received_time_s=received_s,
            range_m=msg.range,
            min_range_m=msg.min_range,
            max_range_m=msg.max_range,
        )
        self._log_phase_transition()

    def _log_phase_transition(self) -> None:
        if self._core.phase == self._last_phase:
            return
        self._node.get_logger().info(
            f"Hybrid state estimator phase: {self._core.phase.value}"
        )
        self._last_phase = self._core.phase
