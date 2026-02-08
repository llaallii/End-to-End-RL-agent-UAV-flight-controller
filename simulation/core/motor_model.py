"""First-order motor dynamics model.

Models the lag between commanded and actual rotor speed using
first-order dynamics with asymmetric time constants for spin-up vs spin-down.

Requirement ref: SIM006 (50-100ms time constant)
"""

from __future__ import annotations

import math

import numpy as np

from simulation.core.types import MotorParams


class MotorModel:
    """First-order motor dynamics for N motors.

    Each motor follows: d(omega)/dt = (omega_cmd - omega) / tau
    where tau = tau_up if spinning up, tau_down if spinning down.

    Thrust per motor: F = kF * omega^2
    Reaction torque:  T = -kM * F * direction  (opposes spin)
    """

    def __init__(self, params: MotorParams) -> None:
        self.params = params
        self.velocities = np.zeros(params.num_motors, dtype=np.float64)
        self.thrusts = np.zeros(params.num_motors, dtype=np.float64)

    def reset(self, velocities: np.ndarray | None = None) -> None:
        """Reset motor velocities.

        Args:
            velocities: Initial velocities. If None, reset to zero.
        """
        if velocities is not None:
            self.velocities = np.array(velocities, dtype=np.float64)
        else:
            self.velocities = np.zeros(self.params.num_motors, dtype=np.float64)
        self._update_thrusts()

    def update(self, commands: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """Advance motor states by dt.

        Args:
            commands: (N,) commanded rotor velocities in rad/s.
            dt: Timestep in seconds.

        Returns:
            thrusts: (N,) thrust force per motor in N (along +Z body).
            torques: (N,) reaction torque per motor in N*m (about Z body).
        """
        commands = np.clip(commands, 0.0, self.params.max_rot_velocity)

        # Select time constant per motor based on spin-up vs spin-down
        spinning_up = commands > self.velocities
        tau = np.where(
            spinning_up,
            self.params.time_constant_up,
            self.params.time_constant_down,
        )

        # First-order response: omega += (cmd - omega) * (1 - exp(-dt/tau))
        alpha = 1.0 - np.exp(-dt / tau)
        self.velocities += (commands - self.velocities) * alpha
        self.velocities = np.clip(self.velocities, 0.0, self.params.max_rot_velocity)

        self._update_thrusts()

        # Reaction torque opposes spin: T = -kM * F * direction
        # CCW motor (dir=+1) produces CW reaction (-Z) on body.
        reaction_torques = -self.params.moment_constant * self.thrusts * self.params.directions

        return self.thrusts.copy(), reaction_torques

    def _update_thrusts(self) -> None:
        """Recompute thrusts from current velocities."""
        self.thrusts = self.params.motor_constant * self.velocities**2

    @property
    def hover_velocity(self) -> float:
        """Calculate the per-motor rotor velocity needed for hover.

        omega_hover = sqrt(m * g / (N * kF))
        Assumes equal distribution across all motors.
        """
        # This needs mass and gravity, which are QuadrotorParams-level.
        # Provide a static helper instead.
        raise NotImplementedError("Use MotorModel.compute_hover_velocity(mass, gravity) instead")

    @staticmethod
    def compute_hover_velocity(
        mass: float, gravity: float, motor_constant: float, num_motors: int = 4
    ) -> float:
        """Compute per-motor rotor velocity for hover.

        Args:
            mass: Vehicle mass in kg.
            gravity: Gravitational acceleration in m/s^2.
            motor_constant: kF in N/(rad/s)^2.
            num_motors: Number of motors.

        Returns:
            Hover rotor velocity in rad/s.
        """
        return math.sqrt(mass * gravity / (num_motors * motor_constant))
