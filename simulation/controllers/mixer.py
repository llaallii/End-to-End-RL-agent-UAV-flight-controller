"""Quad-X motor mixer: converts force/torque demands to per-motor commands.

Maps collective thrust and body-frame torque demands (roll, pitch, yaw) into
individual motor speed commands using the standard 4-motor X-configuration
allocation matrix.

Requirement refs: SIM005, SIM013
"""

from __future__ import annotations

import numpy as np

from simulation.core.types import QuadrotorParams


class QuadXMixer:
    """Allocate collective thrust + 3-axis torques to 4 motor speeds.

    The mixer builds a 4x4 *allocation matrix* **B** such that:

        [F1, F2, F3, F4]^T  =  B  @  [T, tau_x, tau_y, tau_z]^T

    where T is total thrust, and tau_{x,y,z} are body-frame roll/pitch/yaw
    torque demands.  Motor forces are then converted to rotor speed commands
    via  omega_i = sqrt(F_i / kF).

    The inverse (B^-1) is also stored so that the mixer can convert measured
    motor forces **back** to total thrust + torques (useful for logging and
    tests).
    """

    def __init__(self, params: QuadrotorParams) -> None:
        self.params = params
        kf = params.motor.motor_constant
        km = params.motor.moment_constant
        positions = params.motor.positions  # (4, 3) motor XYZ in body frame
        directions = params.motor.directions  # +1 CCW, -1 CW

        # Build the *effectiveness matrix* A (4x4):
        #   Row 0: thrust contribution   -> all 1.0 (each motor contributes F_i)
        #   Row 1: roll torque  (about x) -> y_i * F_i
        #   Row 2: pitch torque (about y) -> -x_i * F_i  (NED sign convention
        #          would flip, but we use FLU so positive x forward, positive
        #          pitch = nose-up requires negative torque from front motors)
        #   Row 3: yaw torque   (about z) -> -direction_i * km * F_i
        #          (reaction torque opposes spin: CCW motor creates -Z yaw torque)
        # A maps motor forces -> [T, tau_x, tau_y, tau_z]
        n = params.motor.num_motors
        a_matrix = np.zeros((4, n), dtype=np.float64)
        for i in range(n):
            a_matrix[0, i] = 1.0
            a_matrix[1, i] = positions[i, 1]  # y_i
            a_matrix[2, i] = -positions[i, 0]  # -x_i
            a_matrix[3, i] = -directions[i] * km  # reaction opposes spin

        # Store effectiveness matrix and its pseudo-inverse (allocation matrix)
        self._effectiveness = a_matrix  # (4, 4)
        self._allocation = np.linalg.inv(a_matrix)  # B = A^{-1}: demands -> forces
        self._kf = kf
        self._max_omega = params.motor.max_rot_velocity
        self._max_thrust_per_motor = kf * self._max_omega**2

    # -- public interface ----------------------------------------------------

    def mix(
        self,
        thrust: float,
        torque_cmd: np.ndarray,
    ) -> np.ndarray:
        """Convert thrust + torque demands into motor speed commands.

        Args:
            thrust: Desired collective thrust in Newtons (positive up, body Z).
            torque_cmd: (3,) desired body-frame torques [tau_x, tau_y, tau_z]
                in N*m.

        Returns:
            (4,) motor speed commands in rad/s, clipped to [0, max_omega].
        """
        demand = np.array(
            [thrust, torque_cmd[0], torque_cmd[1], torque_cmd[2]],
            dtype=np.float64,
        )

        # Allocation: demand -> per-motor force
        forces = self._allocation @ demand

        # Clamp forces to physically realisable range [0, F_max]
        forces = np.clip(forces, 0.0, self._max_thrust_per_motor)

        # Force -> rotor speed:  F = kF * omega^2  =>  omega = sqrt(F / kF)
        speeds = np.sqrt(forces / self._kf)
        return np.clip(speeds, 0.0, self._max_omega)

    def inverse_mix(self, motor_forces: np.ndarray) -> tuple[float, np.ndarray]:
        """Recover total thrust and torques from measured motor forces.

        Args:
            motor_forces: (4,) per-motor thrust forces in N.

        Returns:
            thrust: Total collective thrust in N.
            torques: (3,) body-frame torques [tau_x, tau_y, tau_z] in N*m.
        """
        result = self._effectiveness @ motor_forces
        return float(result[0]), result[1:4].copy()

    def forces_to_speeds(self, forces: np.ndarray) -> np.ndarray:
        """Convert per-motor forces (N) to rotor speeds (rad/s).

        Useful for initialising the motor model at a known thrust condition
        (e.g. hover).

        Args:
            forces: (4,) per-motor thrust in N.

        Returns:
            (4,) rotor speeds in rad/s.
        """
        forces = np.clip(forces, 0.0, self._max_thrust_per_motor)
        return np.sqrt(forces / self._kf)

    def hover_motor_forces(self, mass: float, gravity: float) -> np.ndarray:
        """Return per-motor forces needed for hover (equal distribution).

        Args:
            mass: Vehicle mass in kg.
            gravity: Gravitational acceleration in m/s^2.

        Returns:
            (4,) per-motor hover thrust in N.
        """
        total = mass * gravity
        return np.full(self.params.motor.num_motors, total / self.params.motor.num_motors)

    def hover_speeds(self, mass: float, gravity: float) -> np.ndarray:
        """Return per-motor rotor speeds for hover.

        Args:
            mass: Vehicle mass in kg.
            gravity: Gravitational acceleration in m/s^2.

        Returns:
            (4,) rotor speeds in rad/s.
        """
        forces = self.hover_motor_forces(mass, gravity)
        return self.forces_to_speeds(forces)

    # -- introspection -------------------------------------------------------

    @property
    def allocation_matrix(self) -> np.ndarray:
        """Return a *copy* of the 4x4 allocation matrix B."""
        return self._allocation.copy()

    @property
    def effectiveness_matrix(self) -> np.ndarray:
        """Return a *copy* of the 4x4 effectiveness matrix A."""
        return self._effectiveness.copy()
