#!/usr/bin/env python3
"""Plot recorded X3 telemetry against Gazebo TF ground truth."""

from __future__ import annotations

import argparse
import math
import os
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


TF_TOPIC = "/tf"
IMU_TOPIC = "/x3_lidar/imu"
RANGE_TOPIC = "/x3_lidar/range/down"
MODEL_FRAME = "x3_lidar"
RANGE_LINK_FRAME = "x3_lidar/downward_range_link"


@dataclass
class PoseSeries:
    time_ns: np.ndarray
    position: np.ndarray
    orientation: np.ndarray


@dataclass
class ImuSeries:
    time_ns: np.ndarray
    orientation: np.ndarray
    angular_velocity: np.ndarray
    linear_acceleration: np.ndarray
    frame_id: str


@dataclass
class RangeSeries:
    time_ns: np.ndarray
    value: np.ndarray
    min_range: np.ndarray
    max_range: np.ndarray
    frame_id: str


@dataclass
class RigidTransform:
    translation: np.ndarray
    rotation: np.ndarray


@dataclass
class BagData:
    pose: PoseSeries
    imu: ImuSeries | None
    downward_range: RangeSeries | None
    model_to_range: RigidTransform | None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plot X3 IMU and downward-range rosbag data against the recorded "
            "Gazebo TF pose. The bag is never modified."
        )
    )
    parser.add_argument("bag", type=Path, help="Path to a rosbag2 directory")
    parser.add_argument(
        "--output",
        type=Path,
        help=(
            "Output image path. Defaults to "
            "<bag>/analysis/telemetry_vs_tf.png."
        ),
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open the interactive Matplotlib window after saving the image.",
    )
    parser.add_argument(
        "--ground-z",
        type=float,
        default=0.0,
        help="World-frame ground-plane height in metres (default: 0.0).",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=150,
        help="Saved image resolution (default: 150).",
    )
    return parser.parse_args()


def stamp_to_ns(stamp: object, fallback_ns: int) -> int:
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else fallback_ns


def quaternion(message: object) -> np.ndarray:
    return np.array([message.x, message.y, message.z, message.w], dtype=float)


def normalize_quaternion(value: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(value)
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return value / norm


def quaternion_conjugate(value: np.ndarray) -> np.ndarray:
    return np.array([-value[0], -value[1], -value[2], value[3]])


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.array(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ]
    )


def rotate_vector(rotation: np.ndarray, vector: np.ndarray) -> np.ndarray:
    pure = np.array([vector[0], vector[1], vector[2], 0.0])
    rotated = quaternion_multiply(
        quaternion_multiply(rotation, pure), quaternion_conjugate(rotation)
    )
    return rotated[:3]


def quaternion_to_euler(quaternions: np.ndarray) -> np.ndarray:
    result = np.empty((len(quaternions), 3), dtype=float)
    for index, raw in enumerate(quaternions):
        x, y, z, w = normalize_quaternion(raw)
        sin_roll = 2.0 * (w * x + y * z)
        cos_roll = 1.0 - 2.0 * (x * x + y * y)
        sin_pitch = 2.0 * (w * y - z * x)
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        result[index] = (
            math.atan2(sin_roll, cos_roll),
            math.asin(np.clip(sin_pitch, -1.0, 1.0)),
            math.atan2(sin_yaw, cos_yaw),
        )
    return np.rad2deg(np.unwrap(result, axis=0))


def slerp(left: np.ndarray, right: np.ndarray, fraction: float) -> np.ndarray:
    left = normalize_quaternion(left)
    right = normalize_quaternion(right)
    dot = float(np.dot(left, right))
    if dot < 0.0:
        right = -right
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return normalize_quaternion(left + fraction * (right - left))
    angle = math.acos(dot)
    scale = math.sin(angle)
    return (
        math.sin((1.0 - fraction) * angle) / scale * left
        + math.sin(fraction * angle) / scale * right
    )


def interpolate_quaternions(
    source_time_ns: np.ndarray,
    source_quaternions: np.ndarray,
    query_time_ns: np.ndarray,
) -> np.ndarray:
    result = np.empty((len(query_time_ns), 4), dtype=float)
    indices = np.searchsorted(source_time_ns, query_time_ns, side="left")
    for result_index, source_index in enumerate(indices):
        if source_index <= 0:
            result[result_index] = source_quaternions[0]
            continue
        if source_index >= len(source_time_ns):
            result[result_index] = source_quaternions[-1]
            continue
        before = source_index - 1
        duration = source_time_ns[source_index] - source_time_ns[before]
        fraction = (
            0.0
            if duration == 0
            else (query_time_ns[result_index] - source_time_ns[before]) / duration
        )
        result[result_index] = slerp(
            source_quaternions[before], source_quaternions[source_index], fraction
        )
    return result


def quaternion_error_degrees(
    reference: np.ndarray, measurement: np.ndarray
) -> np.ndarray:
    errors = np.empty(len(reference), dtype=float)
    for index, (reference_q, measurement_q) in enumerate(
        zip(reference, measurement, strict=True)
    ):
        difference = quaternion_multiply(
            quaternion_conjugate(normalize_quaternion(reference_q)),
            normalize_quaternion(measurement_q),
        )
        errors[index] = math.degrees(
            2.0 * math.acos(float(np.clip(abs(difference[3]), 0.0, 1.0)))
        )
    return errors


def derive_tf_angular_velocity(pose: PoseSeries) -> tuple[np.ndarray, np.ndarray]:
    time_s = pose.time_ns.astype(float) / 1e9
    output_time = []
    output_rate = []
    for index in range(1, len(time_s)):
        duration = time_s[index] - time_s[index - 1]
        if duration <= 0.0:
            continue
        previous = normalize_quaternion(pose.orientation[index - 1])
        current = normalize_quaternion(pose.orientation[index])
        difference = quaternion_multiply(quaternion_conjugate(previous), current)
        if difference[3] < 0.0:
            difference = -difference
        vector_norm = np.linalg.norm(difference[:3])
        if vector_norm < 1e-12:
            rate = np.zeros(3)
        else:
            angle = 2.0 * math.atan2(vector_norm, difference[3])
            rate = difference[:3] / vector_norm * angle / duration
        output_time.append((time_s[index] + time_s[index - 1]) / 2.0)
        output_rate.append(rate)
    return np.asarray(output_time), np.asarray(output_rate)


def storage_id_from_metadata(bag: Path) -> str:
    metadata_path = bag / "metadata.yaml"
    if not metadata_path.is_file():
        raise RuntimeError(f"No rosbag metadata file found: {metadata_path}")
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        return metadata["rosbag2_bagfile_information"]["storage_identifier"]
    except (KeyError, TypeError, yaml.YAMLError) as error:
        raise RuntimeError(f"Could not read storage identifier from {metadata_path}") from error


def read_bag(bag: Path) -> BagData:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Python bag libraries are unavailable. Source "
            "/opt/ros/lyrical/setup.bash and install/setup.bash first."
        ) from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag), storage_id=storage_id_from_metadata(bag)
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {
        metadata.name: get_message(metadata.type)
        for metadata in reader.get_all_topics_and_types()
    }

    pose_samples: list[tuple[int, np.ndarray, np.ndarray]] = []
    imu_samples: list[tuple[int, np.ndarray, np.ndarray, np.ndarray, str]] = []
    range_samples: list[tuple[int, float, float, float, str]] = []
    model_to_range: RigidTransform | None = None

    while reader.has_next():
        if hasattr(reader, "read_next_ext"):
            topic, serialized, recorded_ns, _ = reader.read_next_ext()
        else:
            topic, serialized, recorded_ns = reader.read_next()
        if topic not in {TF_TOPIC, IMU_TOPIC, RANGE_TOPIC}:
            continue
        message = deserialize_message(serialized, topic_types[topic])
        if topic == TF_TOPIC:
            for transform in message.transforms:
                sample_ns = stamp_to_ns(transform.header.stamp, recorded_ns)
                if transform.child_frame_id == MODEL_FRAME:
                    translation = transform.transform.translation
                    pose_samples.append(
                        (
                            sample_ns,
                            np.array(
                                [translation.x, translation.y, translation.z],
                                dtype=float,
                            ),
                            quaternion(transform.transform.rotation),
                        )
                    )
                elif (
                    transform.header.frame_id == MODEL_FRAME
                    and transform.child_frame_id == RANGE_LINK_FRAME
                    and model_to_range is None
                ):
                    translation = transform.transform.translation
                    model_to_range = RigidTransform(
                        translation=np.array(
                            [translation.x, translation.y, translation.z], dtype=float
                        ),
                        rotation=quaternion(transform.transform.rotation),
                    )
        elif topic == IMU_TOPIC:
            sample_ns = stamp_to_ns(message.header.stamp, recorded_ns)
            imu_samples.append(
                (
                    sample_ns,
                    quaternion(message.orientation),
                    np.array(
                        [
                            message.angular_velocity.x,
                            message.angular_velocity.y,
                            message.angular_velocity.z,
                        ],
                        dtype=float,
                    ),
                    np.array(
                        [
                            message.linear_acceleration.x,
                            message.linear_acceleration.y,
                            message.linear_acceleration.z,
                        ],
                        dtype=float,
                    ),
                    message.header.frame_id,
                )
            )
        else:
            sample_ns = stamp_to_ns(message.header.stamp, recorded_ns)
            range_samples.append(
                (
                    sample_ns,
                    float(message.range),
                    float(message.min_range),
                    float(message.max_range),
                    message.header.frame_id,
                )
            )

    if not pose_samples:
        raise RuntimeError(
            f"No transform with child frame '{MODEL_FRAME}' was found on {TF_TOPIC}."
        )

    pose_samples.sort(key=lambda sample: sample[0])
    pose = PoseSeries(
        time_ns=np.asarray([sample[0] for sample in pose_samples], dtype=np.int64),
        position=np.asarray([sample[1] for sample in pose_samples]),
        orientation=np.asarray([sample[2] for sample in pose_samples]),
    )

    imu = None
    if imu_samples:
        imu_samples.sort(key=lambda sample: sample[0])
        imu = ImuSeries(
            time_ns=np.asarray([sample[0] for sample in imu_samples], dtype=np.int64),
            orientation=np.asarray([sample[1] for sample in imu_samples]),
            angular_velocity=np.asarray([sample[2] for sample in imu_samples]),
            linear_acceleration=np.asarray([sample[3] for sample in imu_samples]),
            frame_id=imu_samples[0][4],
        )

    downward_range = None
    if range_samples:
        range_samples.sort(key=lambda sample: sample[0])
        downward_range = RangeSeries(
            time_ns=np.asarray([sample[0] for sample in range_samples], dtype=np.int64),
            value=np.asarray([sample[1] for sample in range_samples]),
            min_range=np.asarray([sample[2] for sample in range_samples]),
            max_range=np.asarray([sample[3] for sample in range_samples]),
            frame_id=range_samples[0][4],
        )

    return BagData(
        pose=pose,
        imu=imu,
        downward_range=downward_range,
        model_to_range=model_to_range,
    )


def predicted_range_from_tf(
    pose: PoseSeries,
    model_to_range: RigidTransform,
    ground_z: float,
) -> np.ndarray:
    predicted = np.full(len(pose.time_ns), np.nan)
    sensor_beam = np.array([1.0, 0.0, 0.0])
    for index, (position, orientation) in enumerate(
        zip(pose.position, pose.orientation, strict=True)
    ):
        model_rotation = normalize_quaternion(orientation)
        sensor_position = position + rotate_vector(
            model_rotation, model_to_range.translation
        )
        sensor_rotation = quaternion_multiply(
            model_rotation, normalize_quaternion(model_to_range.rotation)
        )
        beam_world = rotate_vector(sensor_rotation, sensor_beam)
        if beam_world[2] < -1e-6:
            distance = (ground_z - sensor_position[2]) / beam_world[2]
            if distance >= 0.0:
                predicted[index] = distance
    return predicted


def series_rate_hz(time_ns: np.ndarray) -> float:
    if len(time_ns) < 2 or time_ns[-1] <= time_ns[0]:
        return float("nan")
    return (len(time_ns) - 1) / ((time_ns[-1] - time_ns[0]) / 1e9)


def relative_seconds(time_ns: np.ndarray, origin_ns: int) -> np.ndarray:
    return (time_ns.astype(float) - float(origin_ns)) / 1e9


def configure_axis(axis: object, title: str, ylabel: str) -> None:
    axis.set_title(title)
    axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.25)


def mark_unavailable(axis: object, title: str, message: str) -> None:
    axis.set_title(title)
    axis.text(0.5, 0.5, message, ha="center", va="center", transform=axis.transAxes)
    axis.set_xticks([])
    axis.set_yticks([])


def plot_dashboard(
    data: BagData,
    bag: Path,
    output: Path,
    ground_z: float,
    dpi: int,
    show: bool,
) -> None:
    cache_directory = Path(tempfile.gettempdir()) / "lidar_mapping_drone_matplotlib"
    cache_directory.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(cache_directory))

    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    all_start_times = [data.pose.time_ns[0]]
    if data.imu is not None:
        all_start_times.append(data.imu.time_ns[0])
    if data.downward_range is not None:
        all_start_times.append(data.downward_range.time_ns[0])
    origin_ns = int(min(all_start_times))

    pose_time = relative_seconds(data.pose.time_ns, origin_ns)
    pose_euler = quaternion_to_euler(data.pose.orientation)
    figure, axes = plt.subplots(4, 2, figsize=(16, 18), sharex=False)
    axes = axes.ravel()
    tf_line_style = {
        "linewidth": 2.6,
        "alpha": 0.58,
        "zorder": 1,
    }
    imu_line_style = {
        "linewidth": 1.35,
        "linestyle": (0, (5, 2)),
        "marker": "o",
        "markevery": 200,
        "markersize": 3.8,
        "markerfacecolor": "white",
        "markeredgewidth": 1.0,
        "alpha": 0.95,
        "zorder": 3,
    }

    range_residual = None
    range_valid_count = 0
    if data.downward_range is not None:
        measured = data.downward_range
        range_time = relative_seconds(measured.time_ns, origin_ns)
        valid = (
            np.isfinite(measured.value)
            & (measured.value >= measured.min_range)
            & (measured.value <= measured.max_range)
        )
        range_valid_count = int(np.count_nonzero(valid))
        axes[0].plot(
            range_time[valid],
            measured.value[valid],
            color="#0096c7",
            linewidth=1.6,
            label="Measured downward range",
        )
        axes[0].plot(
            pose_time,
            data.pose.position[:, 2],
            color="#6c757d",
            linewidth=1.3,
            linestyle=":",
            label="TF model z (different origin)",
        )
        if data.model_to_range is not None:
            expected = predicted_range_from_tf(
                data.pose, data.model_to_range, ground_z
            )
            axes[0].plot(
                pose_time,
                expected,
                color="#d62828",
                linewidth=1.5,
                label="Expected beam range from TF",
            )
            expected_at_range = np.interp(
                measured.time_ns[valid].astype(float),
                data.pose.time_ns.astype(float),
                expected,
            )
            range_residual = measured.value[valid] - expected_at_range
            axes[1].plot(
                range_time[valid], range_residual, color="#7b2cbf", linewidth=1.3
            )
            axes[1].axhline(0.0, color="#333333", linewidth=1.0)
            configure_axis(axes[1], "Range residual", "measured - TF expected (m)")
        else:
            mark_unavailable(
                axes[1],
                "Range residual",
                f"Missing TF relation {MODEL_FRAME} -> {RANGE_LINK_FRAME}",
            )
        if np.any(~valid):
            axes[0].scatter(
                range_time[~valid],
                data.pose.position[0, 2] * np.ones(np.count_nonzero(~valid)),
                marker="x",
                color="#b00020",
                label="Invalid range sample",
            )
        configure_axis(axes[0], "Downward range against TF geometry", "distance (m)")
        axes[0].legend(loc="best")
    else:
        mark_unavailable(axes[0], "Downward range against TF", f"Missing {RANGE_TOPIC}")
        mark_unavailable(axes[1], "Range residual", f"Missing {RANGE_TOPIC}")

    attitude_error = None
    if data.imu is not None:
        imu_time = relative_seconds(data.imu.time_ns, origin_ns)
        imu_euler = quaternion_to_euler(data.imu.orientation)
        for component, color in ((0, "#d62828"), (1, "#2a9d8f")):
            name = ("roll", "pitch")[component]
            axes[2].plot(
                pose_time,
                pose_euler[:, component],
                color=color,
                label=f"TF {name}",
                **tf_line_style,
            )
            axes[2].plot(
                imu_time,
                imu_euler[:, component],
                color=color,
                label=f"IMU {name}",
                **imu_line_style,
            )
        configure_axis(axes[2], "Roll and pitch", "angle (deg)")
        axes[2].legend(loc="best", ncols=2)

        axes[3].plot(
            pose_time,
            pose_euler[:, 2],
            color="#d62828",
            label="TF yaw",
            **tf_line_style,
        )
        axes[3].plot(
            imu_time,
            imu_euler[:, 2],
            color="#4361ee",
            label="IMU yaw",
            **imu_line_style,
        )
        configure_axis(axes[3], "Yaw (unwrapped)", "angle (deg)")
        axes[3].legend(loc="best")

        tf_rate_time, tf_rates = derive_tf_angular_velocity(data.pose)
        tf_rate_time -= origin_ns / 1e9
        for component, name, color in (
            (0, "x", "#d62828"),
            (1, "y", "#2a9d8f"),
            (2, "z", "#4361ee"),
        ):
            axes[4].plot(
                tf_rate_time,
                tf_rates[:, component],
                color=color,
                label=f"TF-derived {name}",
                **tf_line_style,
            )
            axes[4].plot(
                imu_time,
                data.imu.angular_velocity[:, component],
                color=color,
                label=f"IMU {name}",
                **imu_line_style,
            )
        configure_axis(axes[4], "Body angular rates", "rad/s")
        axes[4].legend(loc="best", ncols=2, fontsize=8)

        acceleration_magnitude = np.linalg.norm(data.imu.linear_acceleration, axis=1)
        for component, name, color in (
            (0, "x", "#d62828"),
            (1, "y", "#2a9d8f"),
            (2, "z", "#4361ee"),
        ):
            axes[5].plot(
                imu_time,
                data.imu.linear_acceleration[:, component],
                color=color,
                linewidth=0.9,
                label=f"a{name}",
            )
        axes[5].plot(
            imu_time,
            acceleration_magnitude,
            color="#111111",
            linewidth=1.3,
            label="magnitude",
        )
        configure_axis(axes[5], "IMU linear acceleration", "m/s^2")
        axes[5].legend(loc="best", ncols=2)

        tf_at_imu = interpolate_quaternions(
            data.pose.time_ns, data.pose.orientation, data.imu.time_ns
        )
        attitude_error = quaternion_error_degrees(tf_at_imu, data.imu.orientation)
        axes[6].plot(
            imu_time, attitude_error, color="#7b2cbf", linewidth=1.1
        )
        configure_axis(axes[6], "IMU attitude error against TF", "rotation error (deg)")
    else:
        for index, title in (
            (2, "Roll and pitch"),
            (3, "Yaw"),
            (4, "Body angular rates"),
            (5, "IMU linear acceleration"),
            (6, "IMU attitude error against TF"),
        ):
            mark_unavailable(axes[index], title, f"Missing {IMU_TOPIC}")

    axes[7].plot(
        data.pose.position[:, 0],
        data.pose.position[:, 1],
        color="#d62828",
        linewidth=1.5,
    )
    axes[7].scatter(
        data.pose.position[0, 0],
        data.pose.position[0, 1],
        color="#2a9d8f",
        label="start",
        zorder=3,
    )
    axes[7].scatter(
        data.pose.position[-1, 0],
        data.pose.position[-1, 1],
        color="#b00020",
        label="end",
        zorder=3,
    )
    axes[7].set_xlabel("world x (m)")
    configure_axis(axes[7], "TF ground-truth XY path", "world y (m)")
    axes[7].axis("equal")
    axes[7].legend(loc="best")

    for index in range(7):
        axes[index].set_xlabel("time (s)")

    summary_parts = [
        f"TF: {len(data.pose.time_ns)} samples at {series_rate_hz(data.pose.time_ns):.1f} Hz"
    ]
    if data.imu is not None:
        summary_parts.append(
            f"IMU: {len(data.imu.time_ns)} samples at {series_rate_hz(data.imu.time_ns):.1f} Hz"
        )
    if data.downward_range is not None:
        summary_parts.append(
            f"Range: {range_valid_count}/{len(data.downward_range.time_ns)} valid "
            f"at {series_rate_hz(data.downward_range.time_ns):.1f} Hz"
        )
    if range_residual is not None:
        range_rms_mm = np.sqrt(np.mean(range_residual**2)) * 1000.0
        summary_parts.append(f"range RMS error: {range_rms_mm:.2f} mm")
    if attitude_error is not None:
        summary_parts.append(f"attitude RMS error: {np.sqrt(np.mean(attitude_error**2)):.3f} deg")

    figure.suptitle(
        "X3 LiDAR Drone Telemetry vs TF Ground Truth\n"
        + bag.name
        + "\n"
        + " | ".join(summary_parts),
        fontsize=15,
    )
    figure.legend(
        handles=[
            Line2D(
                [0],
                [0],
                color="#444444",
                label="TF reference",
                **tf_line_style,
            ),
            Line2D(
                [0],
                [0],
                color="#444444",
                label="IMU measurement",
                **imu_line_style,
            ),
        ],
        loc="upper center",
        bbox_to_anchor=(0.5, 0.935),
        ncols=2,
        frameon=False,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.91))
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=dpi, bbox_inches="tight")
    print(f"Telemetry plot written to: {output.resolve()}")
    print("Comparison summary:")
    for part in summary_parts:
        print(f"  - {part}")
    if data.imu is None:
        print(f"  - Warning: {IMU_TOPIC} was not found in the bag.")
    if data.downward_range is None:
        print(f"  - Warning: {RANGE_TOPIC} was not found in the bag.")
    if data.downward_range is not None and data.model_to_range is None:
        print(
            "  - Warning: range geometry could not be reconstructed because "
            f"TF did not contain {MODEL_FRAME} -> {RANGE_LINK_FRAME}."
        )
    if show:
        plt.show()
    else:
        plt.close(figure)


def main() -> int:
    args = parse_args()
    bag = args.bag.expanduser().resolve()
    if not bag.is_dir():
        print(f"error: rosbag directory does not exist: {bag}", file=sys.stderr)
        return 2
    output = (
        args.output.expanduser().resolve()
        if args.output is not None
        else bag / "analysis" / "telemetry_vs_tf.png"
    )
    try:
        data = read_bag(bag)
        plot_dashboard(
            data=data,
            bag=bag,
            output=output,
            ground_z=args.ground_z,
            dpi=args.dpi,
            show=args.show,
        )
    except (RuntimeError, OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
