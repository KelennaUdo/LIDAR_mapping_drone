"""YawController block.

This block represents the yaw attitude loop. Its inputs are yaw_reference and
estimated_states. Its output is yaw_torque_command for the MotorMixer.
"""

from __future__ import annotations

from dataclasses import dataclass

from .common import PIDController, PIDGains, clamp, wrap_pi
from .state_estimator import EstimatedState


@dataclass
class YawControllerConfig:
    gains: PIDGains
    torque_limit_nm: float


class YawController:
    """Tracks a yaw angle reference and returns a yaw torque command."""

    def __init__(self, config: YawControllerConfig) -> None:
        self.config = config
        self._pid = PIDController(config.gains)

    def reset(self) -> None:
        self._pid.reset()

    def update(
        self,
        reference_yaw_rad: float,
        state: EstimatedState,
        dt_s: float,
    ) -> float:
        yaw_error = wrap_pi(reference_yaw_rad - state.yaw_rad)
        yaw_torque = self._pid.update(yaw_error, -state.yaw_rate_radps, dt_s)
        return clamp(yaw_torque, -self.config.torque_limit_nm, self.config.torque_limit_nm)
