"""Small shared math helpers for the block-structured flight controller."""

from __future__ import annotations

import math
from dataclasses import dataclass


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a scalar value between inclusive limits."""
    return max(lower, min(upper, value))


def wrap_pi(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class PIDGains:
    """PID gains plus an integral clamp."""

    kp: float
    ki: float
    kd: float
    integral_limit: float = 0.0


class PIDController:
    """Minimal PID helper used by the visible controller blocks."""

    def __init__(self, gains: PIDGains) -> None:
        self.gains = gains
        self.integral = 0.0

    def reset(self) -> None:
        self.integral = 0.0

    def update(self, error: float, error_rate: float, dt: float) -> float:
        if dt > 0.0 and self.gains.ki != 0.0:
            self.integral += error * dt
            if self.gains.integral_limit > 0.0:
                self.integral = clamp(
                    self.integral,
                    -self.gains.integral_limit,
                    self.gains.integral_limit,
                )

        return (
            self.gains.kp * error
            + self.gains.ki * self.integral
            + self.gains.kd * error_rate
        )
