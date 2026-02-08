"""Aerodynamic drag models.

Simple quadratic drag opposing vehicle motion.
Requirement ref: SIM008
"""

from __future__ import annotations

import numpy as np

from simulation.core.types import AeroParams


class AerodynamicsModel:
    """Quadratic aerodynamic drag for translational and rotational motion.

    Drag force:  F_drag = -c * |v| * v   (opposes motion, quadratic in speed)
    Drag torque: T_drag = -c_rot * |omega| * omega
    """

    def __init__(self, params: AeroParams) -> None:
        self.params = params

    def translational_drag(self, velocity: np.ndarray) -> np.ndarray:
        """Compute drag force in world frame.

        F_drag = -drag_coeff * |v| * v (quadratic drag, per-axis)

        Args:
            velocity: (3,) linear velocity in world frame [m/s].

        Returns:
            (3,) drag force in world frame [N].
        """
        speed = float(np.linalg.norm(velocity))
        if speed < 1e-10:
            return np.zeros(3, dtype=np.float64)
        result: np.ndarray = -self.params.translational_drag_coeff * speed * velocity
        return result

    def rotational_drag(self, angular_velocity: np.ndarray) -> np.ndarray:
        """Compute rotational drag torque in body frame.

        tau_drag = -rot_drag_coeff * |omega| * omega

        Args:
            angular_velocity: (3,) angular velocity in body frame [rad/s].

        Returns:
            (3,) drag torque in body frame [N*m].
        """
        rate = float(np.linalg.norm(angular_velocity))
        if rate < 1e-10:
            return np.zeros(3, dtype=np.float64)
        result: np.ndarray = -self.params.rotational_drag_coeff * rate * angular_velocity
        return result
