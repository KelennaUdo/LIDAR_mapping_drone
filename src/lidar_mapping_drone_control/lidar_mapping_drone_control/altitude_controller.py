"""AltitudeController block.

This block represents the vertical loop. Its inputs are altitude_reference and
estimated_states. Its output is thrust_command, expressed as estimated total
upward thrust in newtons, which feeds the MotorMixer.
"""

from __future__ import annotations

from dataclasses import dataclass

from .common import PIDController, PIDGains, clamp
from .state_estimator import EstimatedState


@dataclass
class AltitudeControllerConfig:
    gains: PIDGains
    mass_kg: float
    gravity_mps2: float
    min_thrust_n: float
    max_thrust_n: float


class AltitudeController:
    """Conservative altitude PID that adds correction around hover thrust."""

    def __init__(self, config: AltitudeControllerConfig) -> None:
        self.config = config
        self._pid = PIDController(config.gains)

    def reset(self) -> None:
        self._pid.reset()

    def update(self, reference_altitude_m: float, state: EstimatedState, dt_s: float) -> float:
        altitude_error = reference_altitude_m - state.z_m
        acceleration_command = self._pid.update(altitude_error, -state.vz_mps, dt_s)
        thrust_n = self.config.mass_kg * (
            self.config.gravity_mps2 + acceleration_command
        )
        return clamp(thrust_n, self.config.min_thrust_n, self.config.max_thrust_n)
