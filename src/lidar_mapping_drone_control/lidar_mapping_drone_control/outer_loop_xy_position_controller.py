"""OuterLoopXYPositionController block.

This block represents the outer horizontal position loop. Its inputs are the
desired x/y position and the estimated state. Its output is the
reference_pitch_roll_angle signal consumed by the inner pitch/roll controller.
It uses yaw to convert world-frame position error into body-frame tilt
references, matching the cascaded hover-control architecture.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .common import clamp
from .state_estimator import EstimatedState


@dataclass
class XYPositionControllerConfig:
    kp: float
    kd: float
    max_accel_mps2: float
    max_tilt_rad: float
    gravity_mps2: float


@dataclass
class PitchRollReference:
    pitch_rad: float
    roll_rad: float


class OuterLoopXYPositionController:
    """Converts x/y position error into desired pitch and roll angles."""

    def __init__(self, config: XYPositionControllerConfig) -> None:
        self.config = config

    def update(
        self,
        target_x_m: float,
        target_y_m: float,
        state: EstimatedState,
    ) -> PitchRollReference:
        x_error = target_x_m - state.x_m
        y_error = target_y_m - state.y_m

        accel_x_world = self.config.kp * x_error - self.config.kd * state.vx_mps
        accel_y_world = self.config.kp * y_error - self.config.kd * state.vy_mps
        accel_x_world = clamp(
            accel_x_world,
            -self.config.max_accel_mps2,
            self.config.max_accel_mps2,
        )
        accel_y_world = clamp(
            accel_y_world,
            -self.config.max_accel_mps2,
            self.config.max_accel_mps2,
        )

        cos_yaw = math.cos(state.yaw_rad)
        sin_yaw = math.sin(state.yaw_rad)
        accel_x_body = cos_yaw * accel_x_world + sin_yaw * accel_y_world
        accel_y_body = -sin_yaw * accel_x_world + cos_yaw * accel_y_world

        # Small-angle hover approximation. Pitch moves along body x, roll moves
        # along body y with the sign convention used by the current Gazebo X3.
        pitch_ref = accel_x_body / self.config.gravity_mps2
        roll_ref = -accel_y_body / self.config.gravity_mps2

        return PitchRollReference(
            pitch_rad=clamp(
                pitch_ref,
                -self.config.max_tilt_rad,
                self.config.max_tilt_rad,
            ),
            roll_rad=clamp(
                roll_ref,
                -self.config.max_tilt_rad,
                self.config.max_tilt_rad,
            ),
        )
