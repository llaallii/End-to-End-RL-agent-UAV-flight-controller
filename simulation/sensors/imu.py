"""IMU sensor model matching MPU-6050 MEMS specifications.

Simulates a 6-axis IMU (accelerometer + gyroscope) with Gaussian
measurement noise and slowly drifting bias (random walk).

Requirement refs: SIM009 (>=100 Hz), SIM010 (MPU-6050 noise)
"""

from __future__ import annotations

import numpy as np

from simulation.core.quaternion_utils import quaternion_to_rotation_matrix
from simulation.core.types import QuadrotorState


class IMUSensor:
    """Simulated 6-axis IMU (accelerometer + gyroscope).

    Noise model:
        measurement = true_value + white_noise + bias
        bias drifts via random walk: bias += sqrt(dt) * bias_stddev * N(0,1)

    Accelerometer measures specific force in body frame:
        a_meas = R' @ (a_world + [0,0,g]) + noise + bias
    where R' = R_body_to_world.T = R_world_to_body.

    Gyroscope measures angular velocity in body frame:
        omega_meas = omega_true + noise + bias
    """

    def __init__(
        self,
        gyro_noise_stddev: float = 0.00138,
        gyro_bias_stddev: float = 0.0003,
        accel_noise_stddev: float = 0.062,
        accel_bias_stddev: float = 0.001,
        update_rate: float = 250.0,
        seed: int | None = None,
    ) -> None:
        self.gyro_noise_stddev = gyro_noise_stddev
        self.gyro_bias_stddev = gyro_bias_stddev
        self.accel_noise_stddev = accel_noise_stddev
        self.accel_bias_stddev = accel_bias_stddev
        self.update_rate = update_rate

        self._rng = np.random.default_rng(seed)
        self._gyro_bias = np.zeros(3, dtype=np.float64)
        self._accel_bias = np.zeros(3, dtype=np.float64)

    def reset(self) -> None:
        """Reset biases to zero."""
        self._gyro_bias = np.zeros(3, dtype=np.float64)
        self._accel_bias = np.zeros(3, dtype=np.float64)

    def measure(
        self,
        state: QuadrotorState,
        gravity: float = 9.81,
        acceleration_world: np.ndarray | None = None,
        dt: float | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Generate noisy IMU measurement from true state.

        Args:
            state: Current quadrotor state.
            gravity: Gravitational acceleration [m/s^2].
            acceleration_world: (3,) true linear acceleration in world frame [m/s^2].
                If None, assumes hover (zero acceleration).
            dt: Timestep for bias random walk. If None, uses 1/update_rate.

        Returns:
            accel: (3,) accelerometer reading [m/s^2] in body frame.
            gyro: (3,) gyroscope reading [rad/s] in body frame.
        """
        if dt is None:
            dt = 1.0 / self.update_rate

        # Update biases (random walk)
        self._update_bias(dt)

        # Rotation matrix: body-to-world
        r_body_to_world = quaternion_to_rotation_matrix(state.quaternion)
        r_world_to_body = r_body_to_world.T

        # Specific force in world frame: a_world + [0, 0, g]
        # When hovering (a_world=0), specific force = [0, 0, g] in world
        gravity_vector = np.array([0.0, 0.0, gravity])
        if acceleration_world is not None:
            specific_force_world = acceleration_world + gravity_vector
        else:
            specific_force_world = gravity_vector

        # Rotate to body frame
        specific_force_body = r_world_to_body @ specific_force_world

        # Add noise and bias
        accel_noise = self._rng.normal(0.0, self.accel_noise_stddev, size=3)
        accel = specific_force_body + accel_noise + self._accel_bias

        # Gyroscope: true angular velocity in body frame + noise + bias
        gyro_noise = self._rng.normal(0.0, self.gyro_noise_stddev, size=3)
        gyro = state.angular_velocity + gyro_noise + self._gyro_bias

        return accel.astype(np.float64), gyro.astype(np.float64)

    def _update_bias(self, dt: float) -> None:
        """Random-walk bias update.

        bias += sqrt(dt) * bias_stddev * N(0, 1)
        """
        sqrt_dt = np.sqrt(dt)
        self._gyro_bias += sqrt_dt * self.gyro_bias_stddev * self._rng.standard_normal(3)
        self._accel_bias += sqrt_dt * self.accel_bias_stddev * self._rng.standard_normal(3)
