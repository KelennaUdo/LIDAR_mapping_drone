"""InnerLoopPitchRollController block.

This block represents the inner attitude loop for roll and pitch. Its inputs
are reference_pitch_roll_angle and estimated_states. Its outputs are
roll_torque_command and pitch_torque_command, which feed the MotorMixer.
"""

from __future__ import annotations

from dataclasses import dataclass

from .common import PIDController, PIDGains, clamp, wrap_pi
from .state_estimator import EstimatedState


@dataclass
class PitchRollControllerConfig:
    roll_gains: PIDGains
    pitch_gains: PIDGains
    torque_limit_nm: float


@dataclass
class PitchRollTorqueCommand:
    roll_torque_nm: float
    pitch_torque_nm: float


class InnerLoopPitchRollController:
    """Tracks desired roll/pitch angles with conservative PID torque commands."""

    def __init__(self, config: PitchRollControllerConfig) -> None:
        self.config = config
        self._roll_pid = PIDController(config.roll_gains)
        self._pitch_pid = PIDController(config.pitch_gains)

    def reset(self) -> None:
        self._roll_pid.reset()
        self._pitch_pid.reset()

    def update(
        self,
        reference_roll_rad: float,
        reference_pitch_rad: float,
        state: EstimatedState,
        dt_s: float,
    ) -> PitchRollTorqueCommand:
        roll_error = wrap_pi(reference_roll_rad - state.roll_rad)
        pitch_error = wrap_pi(reference_pitch_rad - state.pitch_rad)

        roll_torque = self._roll_pid.update(
            roll_error,
            -state.roll_rate_radps,
            dt_s,
        )
        pitch_torque = self._pitch_pid.update(
            pitch_error,
            -state.pitch_rate_radps,
            dt_s,
        )

        return PitchRollTorqueCommand(
            roll_torque_nm=clamp(
                roll_torque,
                -self.config.torque_limit_nm,
                self.config.torque_limit_nm,
            ),
            pitch_torque_nm=clamp(
                pitch_torque,
                -self.config.torque_limit_nm,
                self.config.torque_limit_nm,
            ),
        )
