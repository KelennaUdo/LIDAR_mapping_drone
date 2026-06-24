"""MotorMixer block.

This block represents the motor mixing algorithm. Its inputs are
thrust_command, roll_torque_command, pitch_torque_command, and
yaw_torque_command. Its output is four rotor angular velocity commands for the
Gazebo X3 topic `/X3/gazebo/command/motor_speed`.

Assumptions from the current SDF:
- actuator 0: rotor_0 at x=0.13, y=-0.22, turningDirection=ccw
- actuator 1: rotor_1 at x=-0.13, y=0.20, turningDirection=ccw
- actuator 2: rotor_2 at x=0.13, y=0.22, turningDirection=cw
- actuator 3: rotor_3 at x=-0.13, y=-0.20, turningDirection=cw
- motorConstant converts rotor speed squared to thrust.
- momentConstant converts rotor thrust to yaw moment with the configured spin
  signs. If yaw behavior is reversed in testing, flip the spin-direction signs
  in YAML before changing controller code.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .common import clamp


@dataclass
class MotorMixerConfig:
    motor_constant_n_per_radps2: float
    moment_constant_m: float
    rotor_x_m: list[float]
    rotor_y_m: list[float]
    rotor_spin_sign: list[float]
    min_motor_speed_rad_s: float
    max_motor_speed_rad_s: float


@dataclass
class MotorCommand:
    rotor_0_rad_s: float
    rotor_1_rad_s: float
    rotor_2_rad_s: float
    rotor_3_rad_s: float

    def as_velocity_list(self) -> list[float]:
        return [
            self.rotor_0_rad_s,
            self.rotor_1_rad_s,
            self.rotor_2_rad_s,
            self.rotor_3_rad_s,
        ]


class MotorMixer:
    """Allocates collective thrust and torques to four X3 rotor speeds."""

    def __init__(self, config: MotorMixerConfig) -> None:
        self.config = config
        if len(config.rotor_x_m) != 4 or len(config.rotor_y_m) != 4:
            raise ValueError("MotorMixer requires exactly four rotor positions.")
        if len(config.rotor_spin_sign) != 4:
            raise ValueError("MotorMixer requires exactly four spin signs.")
        if config.motor_constant_n_per_radps2 <= 0.0:
            raise ValueError("motor_constant_n_per_radps2 must be positive.")

    def mix(
        self,
        thrust_command_n: float,
        roll_torque_command_nm: float,
        pitch_torque_command_nm: float,
        yaw_torque_command_nm: float,
    ) -> MotorCommand:
        allocation = [
            [1.0, 1.0, 1.0, 1.0],
            list(self.config.rotor_y_m),
            [-x for x in self.config.rotor_x_m],
            [
                spin * self.config.moment_constant_m
                for spin in self.config.rotor_spin_sign
            ],
        ]
        requested = [
            thrust_command_n,
            roll_torque_command_nm,
            pitch_torque_command_nm,
            yaw_torque_command_nm,
        ]

        rotor_forces_n = solve_4x4(allocation, requested)
        rotor_speeds = []
        for force_n in rotor_forces_n:
            speed = math.sqrt(max(0.0, force_n) / self.config.motor_constant_n_per_radps2)
            rotor_speeds.append(
                clamp(
                    speed,
                    self.config.min_motor_speed_rad_s,
                    self.config.max_motor_speed_rad_s,
                )
            )

        return MotorCommand(
            rotor_0_rad_s=rotor_speeds[0],
            rotor_1_rad_s=rotor_speeds[1],
            rotor_2_rad_s=rotor_speeds[2],
            rotor_3_rad_s=rotor_speeds[3],
        )


def solve_4x4(matrix: list[list[float]], vector: list[float]) -> list[float]:
    """Solve a dense 4x4 linear system with Gaussian elimination."""
    augmented = [row[:] + [rhs] for row, rhs in zip(matrix, vector)]
    size = 4

    for pivot_index in range(size):
        pivot_row = max(
            range(pivot_index, size),
            key=lambda row_index: abs(augmented[row_index][pivot_index]),
        )
        if abs(augmented[pivot_row][pivot_index]) < 1.0e-9:
            raise ValueError("Motor allocation matrix is singular.")
        augmented[pivot_index], augmented[pivot_row] = (
            augmented[pivot_row],
            augmented[pivot_index],
        )

        pivot = augmented[pivot_index][pivot_index]
        for col in range(pivot_index, size + 1):
            augmented[pivot_index][col] /= pivot

        for row in range(size):
            if row == pivot_index:
                continue
            scale = augmented[row][pivot_index]
            for col in range(pivot_index, size + 1):
                augmented[row][col] -= scale * augmented[pivot_index][col]

    return [augmented[row][size] for row in range(size)]
