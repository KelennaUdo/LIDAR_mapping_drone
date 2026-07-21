import math
import unittest

from lidar_mapping_drone_control.state_estimator import (
    EstimatorPhase,
    HybridEstimatorConfig,
    HybridEstimatorCore,
    quaternion_from_euler,
    quaternion_multiply,
    rotate_vector,
)


MODEL_FRAME = "x3_lidar"
IMU_FRAME = "x3_lidar/imu_link"
RANGE_FRAME = "x3_lidar/downward_range_link"
SENSOR_OFFSET = (0.0, 0.0, 0.013302)
RANGE_ROTATION = quaternion_from_euler(0.0, math.pi / 2.0, 0.0)


def make_core(alpha=0.25):
    core = HybridEstimatorCore(
        HybridEstimatorConfig(
            tracked_child_frame=MODEL_FRAME,
            imu_link_frame=IMU_FRAME,
            range_link_frame=RANGE_FRAME,
            ground_z_m=0.0,
            vertical_velocity_filter_alpha=alpha,
        )
    )
    core.update_mount_transform(
        MODEL_FRAME,
        IMU_FRAME,
        (0.0, 0.0, 0.053302),
        (0.0, 0.0, 0.0, 1.0),
    )
    core.update_mount_transform(
        MODEL_FRAME,
        RANGE_FRAME,
        SENSOR_OFFSET,
        RANGE_ROTATION,
    )
    return core


def update_pose_and_imu(core, timestamp, received, orientation):
    core.update_pose(
        timestamp_s=timestamp,
        received_time_s=received,
        parent_frame_id="lidar_robot_world",
        child_frame_id=MODEL_FRAME,
        x_m=1.0,
        y_m=-2.0,
    )
    core.update_imu(
        timestamp_s=timestamp,
        received_time_s=received,
        orientation=orientation,
        angular_velocity=(0.1, -0.2, 0.3),
    )


def range_for_model_altitude(model_z, model_orientation):
    offset_world = rotate_vector(model_orientation, SENSOR_OFFSET)
    sensor_orientation = quaternion_multiply(
        model_orientation,
        RANGE_ROTATION,
    )
    beam_world = rotate_vector(sensor_orientation, (1.0, 0.0, 0.0))
    return -(model_z + offset_world[2]) / beam_world[2]


class HybridEstimatorCoreTest(unittest.TestCase):
    def test_level_range_recovers_model_altitude(self):
        core = make_core()
        orientation = quaternion_from_euler(0.0, 0.0, 0.0)
        update_pose_and_imu(core, 1.0, 10.0, orientation)
        core.update_range(
            timestamp_s=1.0,
            received_time_s=10.0,
            range_m=range_for_model_altitude(0.75, orientation),
            min_range_m=0.05,
            max_range_m=10.0,
        )

        self.assertEqual(core.phase, EstimatorPhase.AIRBORNE)
        self.assertAlmostEqual(core.state.z_m, 0.75, places=9)
        self.assertAlmostEqual(core.state.roll_rate_radps, 0.1)
        self.assertAlmostEqual(core.state.pitch_rate_radps, -0.2)
        self.assertAlmostEqual(core.state.yaw_rate_radps, 0.3)

    def test_tilted_range_uses_beam_geometry(self):
        core = make_core()
        orientation = quaternion_from_euler(
            math.radians(20.0),
            math.radians(-15.0),
            math.radians(30.0),
        )
        update_pose_and_imu(core, 1.0, 10.0, orientation)
        core.update_range(
            timestamp_s=1.0,
            received_time_s=10.0,
            range_m=range_for_model_altitude(1.25, orientation),
            min_range_m=0.05,
            max_range_m=10.0,
        )

        self.assertAlmostEqual(core.state.z_m, 1.25, places=9)
        self.assertAlmostEqual(core.state.roll_rad, math.radians(20.0), places=9)
        self.assertAlmostEqual(core.state.pitch_rad, math.radians(-15.0), places=9)

    def test_invalid_startup_range_enters_landed_then_latches_airborne(self):
        core = make_core()
        orientation = quaternion_from_euler(0.0, 0.0, 0.0)
        update_pose_and_imu(core, 1.0, 10.0, orientation)
        core.update_range(1.0, 10.0, 11.0, 0.05, 10.0)

        self.assertEqual(core.phase, EstimatorPhase.LANDED)
        self.assertAlmostEqual(core.state.z_m, 0.0)
        self.assertAlmostEqual(core.state.vz_mps, 0.0)

        core.update_range(
            2.0,
            11.0,
            range_for_model_altitude(0.10, orientation),
            0.05,
            10.0,
        )
        self.assertEqual(core.phase, EstimatorPhase.AIRBORNE)
        self.assertAlmostEqual(core.state.z_m, 0.10)

        core.update_range(3.0, 12.0, 11.0, 0.05, 10.0)
        self.assertEqual(core.phase, EstimatorPhase.AIRBORNE)
        self.assertAlmostEqual(core.state.z_m, 0.10)

    def test_airborne_invalid_range_does_not_refresh_state_age(self):
        core = make_core()
        orientation = quaternion_from_euler(0.0, 0.0, 0.0)
        update_pose_and_imu(core, 1.0, 10.0, orientation)
        core.update_range(
            1.0,
            10.0,
            range_for_model_altitude(0.75, orientation),
            0.05,
            10.0,
        )
        update_pose_and_imu(core, 2.0, 10.4, orientation)
        core.update_range(2.0, 10.4, 11.0, 0.05, 10.0)

        self.assertAlmostEqual(core.state_age_s(10.6), 0.6)

    def test_vertical_velocity_is_low_pass_filtered(self):
        core = make_core(alpha=0.25)
        orientation = quaternion_from_euler(0.0, 0.0, 0.0)
        update_pose_and_imu(core, 1.0, 10.0, orientation)
        core.update_range(
            1.0,
            10.0,
            range_for_model_altitude(0.10, orientation),
            0.05,
            10.0,
        )
        core.update_range(
            2.0,
            11.0,
            range_for_model_altitude(0.50, orientation),
            0.05,
            10.0,
        )

        self.assertAlmostEqual(core.state.vz_mps, 0.10, places=9)


if __name__ == "__main__":
    unittest.main()
