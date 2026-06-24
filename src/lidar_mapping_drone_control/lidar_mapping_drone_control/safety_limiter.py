"""SafetyLimiter block.

This block is the final safety gate before commands leave the controller. Its
inputs are estimated_states plus the MotorMixer output. Its output is a clamped
or zeroed set of rotor commands. It protects the first sandbox controller from
stale state feedback, excessive altitude, excessive tilt, and emergency stop.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from .common import clamp
from .motor_mixer import MotorCommand
from .state_estimator import EstimatedState


@dataclass
class SafetyLimiterConfig:
    min_motor_speed_rad_s: float
    max_motor_speed_rad_s: float
    max_altitude_m: float
    max_tilt_rad: float
    state_timeout_s: float


@dataclass
class SafetyResult:
    command: MotorCommand
    triggered: bool
    reason: str


class SafetyLimiter:
    """Clamps motor commands and zeros them when a safety limit is active."""

    def __init__(self, config: SafetyLimiterConfig) -> None:
        self.config = config

    def apply(
        self,
        command: MotorCommand,
        state: Optional[EstimatedState],
        state_age_s: Optional[float],
        emergency_stop: bool,
    ) -> SafetyResult:
        reason = self._stop_reason(state, state_age_s, emergency_stop)
        if reason:
            return SafetyResult(command=zero_motor_command(), triggered=True, reason=reason)

        clamped = [
            clamp(
                value,
                self.config.min_motor_speed_rad_s,
                self.config.max_motor_speed_rad_s,
            )
            for value in command.as_velocity_list()
        ]
        return SafetyResult(
            command=MotorCommand(*clamped),
            triggered=False,
            reason="",
        )

    def _stop_reason(
        self,
        state: Optional[EstimatedState],
        state_age_s: Optional[float],
        emergency_stop: bool,
    ) -> str:
        if emergency_stop:
            return "emergency_stop"
        if state is None or state_age_s is None:
            return "missing_state_feedback"
        if state_age_s > self.config.state_timeout_s:
            return f"stale_state_feedback age={state_age_s:.2f}s"
        if state.z_m > self.config.max_altitude_m:
            return (
                f"max_altitude_exceeded z={state.z_m:.2f}m "
                f"limit={self.config.max_altitude_m:.2f}m"
            )
        tilt_rad = max(abs(state.roll_rad), abs(state.pitch_rad))
        if tilt_rad > self.config.max_tilt_rad:
            return (
                f"max_tilt_exceeded tilt={math.degrees(tilt_rad):.1f}deg "
                f"limit={math.degrees(self.config.max_tilt_rad):.1f}deg"
            )
        return ""


def zero_motor_command() -> MotorCommand:
    return MotorCommand(0.0, 0.0, 0.0, 0.0)
