#!/usr/bin/env python3
"""Compare KISS-ICP position estimates with PX4 vehicle odometry."""

import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class OdometryComparison(Node):
    """Collect corresponding KISS-ICP and PX4 position estimates."""

    # ============================================================
    # CONSTRUCTOR
    # ============================================================

    def __init__(self, kiss_topic: str, px4_topic: str, output_root: Path) -> None:
        super().__init__("kiss_px4_odometry_comparison")

        # Topic names describe the two independent estimates being compared.
        self.kiss_topic_ = kiss_topic
        self.px4_topic_ = px4_topic

        # Each run receives its own timestamped output directory.
        self.output_root_ = output_root

        # Samples store shared receipt time, source time, and XYZ position.
        self.kiss_samples_: list[tuple[float, float, float, float, float]] = []
        self.px4_samples_: list[tuple[float, float, float, float, float]] = []

        # Prevent repeated warnings when PX4 publishes an unsupported pose frame.
        self.unsupported_px4_frame_warning_printed_ = False

        # Best-effort subscriptions match the sensor-style and PX4 DDS publishers.
        topic_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.kiss_subscription_ = self.create_subscription(
            Odometry,
            self.kiss_topic_,
            self.kiss_callback,
            topic_qos,
        )

        self.px4_subscription_ = self.create_subscription(
            VehicleOdometry,
            self.px4_topic_,
            self.px4_callback,
            topic_qos,
        )

        self.get_logger().info("Collecting position estimates")
        self.get_logger().info(f"  KISS-ICP: {self.kiss_topic_}")
        self.get_logger().info(f"  PX4:      {self.px4_topic_}")
        self.get_logger().info("Press Ctrl+C after bag playback finishes")

    # ============================================================
    # ROS CALLBACKS
    # ============================================================

    def kiss_callback(self, message: Odometry) -> None:
        """Store one KISS-ICP position with source and receipt timestamps."""
        position = message.pose.pose.position

        if not all(math.isfinite(value) for value in (position.x, position.y, position.z)):
            self.get_logger().warning("Skipped a KISS-ICP sample containing NaN or infinity")
            return

        source_time_s = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )

        self.kiss_samples_.append(
            (time.monotonic(), source_time_s, position.x, position.y, position.z)
        )

        if len(self.kiss_samples_) % 250 == 0:
            kiss_count = len(self.kiss_samples_)
            px4_count = len(self.px4_samples_)
            self.get_logger().info(
                f"Collected {kiss_count} KISS-ICP samples and {px4_count} PX4 samples"
            )

    def px4_callback(self, message: VehicleOdometry) -> None:
        """Convert one PX4 NED position to ENU and store it."""
        if message.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            if not self.unsupported_px4_frame_warning_printed_:
                self.get_logger().warning(
                    f"PX4 pose_frame is {message.pose_frame}, "
                    "but this first evaluator supports NED only"
                )
                self.unsupported_px4_frame_warning_printed_ = True
            return

        north, east, down = message.position

        if not all(math.isfinite(value) for value in (north, east, down)):
            return

        # PX4 NED (north, east, down) becomes ROS ENU (east, north, up).
        east_enu = float(east)
        north_enu = float(north)
        up_enu = -float(down)

        timestamp_us = message.timestamp_sample or message.timestamp
        source_time_s = float(timestamp_us) * 1.0e-6

        self.px4_samples_.append(
            (time.monotonic(), source_time_s, east_enu, north_enu, up_enu)
        )

    # ============================================================
    # RESULT GENERATION
    # ============================================================

    def save_results(self) -> Path | None:
        """Align the collected trajectories and write plots, CSV, and a summary."""
        if len(self.kiss_samples_) < 10 or len(self.px4_samples_) < 10:
            kiss_count = len(self.kiss_samples_)
            px4_count = len(self.px4_samples_)
            self.get_logger().error(
                f"Not enough data to compare: {kiss_count} KISS-ICP and "
                f"{px4_count} PX4 samples"
            )
            return None

        comparison = prepare_comparison(self.kiss_samples_, self.px4_samples_)
        if comparison is None:
            self.get_logger().error("The KISS-ICP and PX4 samples do not overlap in time")
            return None

        output_directory = self.output_root_ / datetime.now().strftime(
            "kiss_px4_%Y%m%d_%H%M%S"
        )
        output_directory.mkdir(parents=True, exist_ok=False)

        write_csv(output_directory / "trajectory_data.csv", comparison)
        write_summary(output_directory / "summary.txt", comparison, self)
        plot_xy_trajectory(output_directory / "trajectory_xy.png", comparison)
        plot_position_over_time(output_directory / "position_vs_time.png", comparison)
        plot_position_error(output_directory / "position_error.png", comparison)

        self.get_logger().info(f"Comparison written to {output_directory}")
        rmse_3d_m = comparison["rmse_3d_m"]
        self.get_logger().info(f"Position RMSE: {rmse_3d_m:.3f} m")
        return output_directory


# ============================================================
# TRAJECTORY ALIGNMENT
# ============================================================


def prepare_comparison(
    kiss_samples: list[tuple[float, float, float, float, float]],
    px4_samples: list[tuple[float, float, float, float, float]],
) -> dict[str, np.ndarray | float] | None:
    """Synchronize samples and align KISS XY heading to the PX4 trajectory."""
    kiss = np.asarray(sorted(kiss_samples), dtype=np.float64)
    px4 = np.asarray(sorted(px4_samples), dtype=np.float64)

    # Repeated receipt timestamps cannot be used as interpolation coordinates.
    _, unique_px4_indices = np.unique(px4[:, 0], return_index=True)
    px4 = px4[np.sort(unique_px4_indices)]

    overlap_start = max(kiss[0, 0], px4[0, 0])
    overlap_end = min(kiss[-1, 0], px4[-1, 0])

    if overlap_end <= overlap_start:
        return None

    kiss = kiss[(kiss[:, 0] >= overlap_start) & (kiss[:, 0] <= overlap_end)]
    if len(kiss) < 10:
        return None

    px4_interpolated = np.column_stack(
        [
            np.interp(kiss[:, 0], px4[:, 0], px4[:, axis])
            for axis in (2, 3, 4)
        ]
    )

    time_s = kiss[:, 0] - kiss[0, 0]
    kiss_relative = kiss[:, 2:5] - kiss[0, 2:5]
    px4_relative = px4_interpolated - px4_interpolated[0]

    aligned_xy, heading_rotation_rad = align_xy_without_scaling(
        kiss_relative[:, :2],
        px4_relative[:, :2],
    )

    kiss_aligned = kiss_relative.copy()
    kiss_aligned[:, :2] = aligned_xy

    error_xyz = kiss_aligned - px4_relative
    error_norm = np.linalg.norm(error_xyz, axis=1)

    return {
        "time_s": time_s,
        "kiss_source_time_s": kiss[:, 1],
        "kiss_relative": kiss_relative,
        "kiss_aligned": kiss_aligned,
        "px4_relative": px4_relative,
        "error_xyz": error_xyz,
        "error_norm": error_norm,
        "heading_rotation_rad": heading_rotation_rad,
        "rmse_xyz_m": np.sqrt(np.mean(np.square(error_xyz), axis=0)),
        "rmse_3d_m": float(np.sqrt(np.mean(np.square(error_norm)))),
        "maximum_error_m": float(np.max(error_norm)),
        "final_error_m": float(error_norm[-1]),
    }


def align_xy_without_scaling(
    source_xy: np.ndarray,
    target_xy: np.ndarray,
) -> tuple[np.ndarray, float]:
    """Find the rigid 2D rotation and translation that best align two paths."""
    source_center = np.mean(source_xy, axis=0)
    target_center = np.mean(target_xy, axis=0)

    source_centered = source_xy - source_center
    target_centered = target_xy - target_center

    if np.linalg.norm(source_centered) < 1.0e-9:
        return source_xy.copy(), 0.0

    covariance = source_centered.T @ target_centered
    left_vectors, _, right_vectors_transposed = np.linalg.svd(covariance)
    rotation = right_vectors_transposed.T @ left_vectors.T

    # Disallow a mirror reflection; only a physical yaw rotation is permitted.
    if np.linalg.det(rotation) < 0.0:
        right_vectors_transposed[-1, :] *= -1.0
        rotation = right_vectors_transposed.T @ left_vectors.T

    translation = target_center - rotation @ source_center
    aligned = (rotation @ source_xy.T).T + translation
    heading_rotation_rad = math.atan2(rotation[1, 0], rotation[0, 0])

    return aligned, heading_rotation_rad


# ============================================================
# FILE OUTPUT
# ============================================================


def write_csv(path: Path, comparison: dict[str, np.ndarray | float]) -> None:
    """Write synchronized trajectory and error samples for later inspection."""
    time_s = comparison["time_s"]
    kiss_relative = comparison["kiss_relative"]
    kiss_aligned = comparison["kiss_aligned"]
    px4_relative = comparison["px4_relative"]
    error_xyz = comparison["error_xyz"]
    error_norm = comparison["error_norm"]

    with path.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.writer(output_file)
        writer.writerow(
            [
                "time_s",
                "kiss_raw_x_m",
                "kiss_raw_y_m",
                "kiss_raw_z_m",
                "kiss_aligned_x_m",
                "kiss_aligned_y_m",
                "kiss_aligned_z_m",
                "px4_enu_x_m",
                "px4_enu_y_m",
                "px4_enu_z_m",
                "error_x_m",
                "error_y_m",
                "error_z_m",
                "error_norm_m",
            ]
        )

        for index in range(len(time_s)):
            writer.writerow(
                [
                    time_s[index],
                    *kiss_relative[index],
                    *kiss_aligned[index],
                    *px4_relative[index],
                    *error_xyz[index],
                    error_norm[index],
                ]
            )


def write_summary(
    path: Path,
    comparison: dict[str, np.ndarray | float],
    node: OdometryComparison,
) -> None:
    """Write the assumptions and headline position-error measurements."""
    rmse_xyz = comparison["rmse_xyz_m"]
    heading_deg = math.degrees(comparison["heading_rotation_rad"])

    summary = f"""KISS-ICP and PX4 Position Comparison
=====================================

KISS-ICP topic: {node.kiss_topic_}
PX4 topic:      {node.px4_topic_}

Collected KISS-ICP samples: {len(node.kiss_samples_)}
Collected PX4 samples:      {len(node.px4_samples_)}
Compared samples:           {len(comparison['time_s'])}
Compared duration:          {comparison['time_s'][-1]:.3f} s

Coordinate handling
-------------------
PX4 positions were converted from NED to ENU.
Both trajectories were translated to a common starting origin.
KISS-ICP XY was rigidly aligned to PX4 without changing scale.
Applied horizontal rotation: {heading_deg:.3f} deg

Position error
--------------
X RMSE:       {rmse_xyz[0]:.3f} m
Y RMSE:       {rmse_xyz[1]:.3f} m
Z RMSE:       {rmse_xyz[2]:.3f} m
3D RMSE:      {comparison['rmse_3d_m']:.3f} m
Maximum error:{comparison['maximum_error_m']: .3f} m
Final error:  {comparison['final_error_m']:.3f} m

Limitations
-----------
PX4 odometry is an estimator, not simulation ground truth.
This first comparison evaluates position only, not orientation or velocity.
KISS-ICP describes lidar_link while PX4 describes the vehicle body.
Callbacks are synchronized using their shared receipt timing during playback.
"""

    path.write_text(summary, encoding="utf-8")


# ============================================================
# PLOTS
# ============================================================


def plot_xy_trajectory(
    path: Path,
    comparison: dict[str, np.ndarray | float],
) -> None:
    """Plot the aligned top-down paths."""
    kiss = comparison["kiss_aligned"]
    px4 = comparison["px4_relative"]

    figure, axis = plt.subplots(figsize=(9, 7))
    axis.plot(px4[:, 0], px4[:, 1], color="#202124", linewidth=2.2, label="PX4")
    axis.plot(
        kiss[:, 0],
        kiss[:, 1],
        color="#00a6d6",
        linewidth=2.0,
        linestyle="--",
        label="KISS-ICP",
    )
    axis.scatter([0.0], [0.0], color="#2e7d32", marker="o", s=70, label="Start")
    axis.set_title("Aligned XY trajectory")
    axis.set_xlabel("East / aligned X (m)")
    axis.set_ylabel("North / aligned Y (m)")
    axis.axis("equal")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def plot_position_over_time(
    path: Path,
    comparison: dict[str, np.ndarray | float],
) -> None:
    """Plot corresponding X, Y, and Z positions over playback time."""
    time_s = comparison["time_s"]
    kiss = comparison["kiss_aligned"]
    px4 = comparison["px4_relative"]

    figure, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axis_names = ("X", "Y", "Z")

    for index, axis in enumerate(axes):
        axis.plot(time_s, px4[:, index], color="#202124", linewidth=2.0, label="PX4")
        axis.plot(
            time_s,
            kiss[:, index],
            color="#00a6d6",
            linewidth=1.8,
            linestyle="--",
            label="KISS-ICP",
        )
        axis.set_ylabel(f"{axis_names[index]} (m)")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right")

    axes[0].set_title("Position estimates over time")
    axes[-1].set_xlabel("Time since comparison start (s)")
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def plot_position_error(
    path: Path,
    comparison: dict[str, np.ndarray | float],
) -> None:
    """Plot component and total differences between both position estimates."""
    time_s = comparison["time_s"]
    error_xyz = comparison["error_xyz"]
    error_norm = comparison["error_norm"]

    figure, axis = plt.subplots(figsize=(10, 6))
    axis.plot(time_s, error_xyz[:, 0], color="#d32f2f", label="X error")
    axis.plot(time_s, error_xyz[:, 1], color="#388e3c", linestyle="--", label="Y error")
    axis.plot(time_s, error_xyz[:, 2], color="#1976d2", linestyle=":", label="Z error")
    axis.plot(
        time_s,
        error_norm,
        color="#6a1b9a",
        linewidth=2.2,
        label="3D error magnitude",
    )
    axis.set_title("KISS-ICP position difference from PX4")
    axis.set_xlabel("Time since comparison start (s)")
    axis.set_ylabel("Position difference (m)")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


# ============================================================
# PROGRAM ENTRY POINT
# ============================================================


def parse_arguments() -> argparse.Namespace:
    """Read project-specific topic and output options."""
    script_directory = Path(__file__).resolve().parent

    parser = argparse.ArgumentParser(
        description="Compare KISS-ICP position estimates with PX4 odometry",
    )
    parser.add_argument("--kiss-topic", default="/kiss/odometry")
    parser.add_argument("--px4-topic", default="/fmu/out/vehicle_odometry")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=script_directory / "generated",
    )
    return parser.parse_args()


def main() -> int:
    """Collect until interrupted, then save the comparison artifacts."""
    arguments = parse_arguments()

    rclpy.init()
    node = OdometryComparison(
        kiss_topic=arguments.kiss_topic,
        px4_topic=arguments.px4_topic,
        output_root=arguments.output_root.expanduser().resolve(),
    )

    result_path = None

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping collection and preparing results")
    finally:
        result_path = node.save_results()
        node.destroy_node()
        rclpy.shutdown()

    return 0 if result_path is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())
