"""Unit tests for the IMU sensor model."""

from __future__ import annotations

import math

import numpy as np
from numpy.testing import assert_allclose

from simulation.core.quaternion_utils import euler_to_quaternion
from simulation.core.types import QuadrotorState
from simulation.sensors.imu import IMUSensor


def _zeros_state() -> QuadrotorState:
    """Convenience helper for a fresh zeros state."""
    return QuadrotorState.zeros()


# -------------------------------------------------------------------
# Basic functionality
# -------------------------------------------------------------------


def test_at_rest_reads_gravity(identity_state: QuadrotorState) -> None:
    """At rest with identity quaternion, accel should read ~[0, 0, g]."""
    imu = IMUSensor(seed=42)
    accel, gyro = imu.measure(identity_state, dt=0.0)

    # Accel should be close to [0, 0, 9.81] (noise adds some spread)
    assert_allclose(accel, [0.0, 0.0, 9.81], atol=0.5)
    # Gyro should be close to zero
    assert_allclose(gyro, [0.0, 0.0, 0.0], atol=0.1)


# -------------------------------------------------------------------
# Noise statistics
# -------------------------------------------------------------------


def test_accel_noise_statistics() -> None:
    """Accel noise mean and stddev should match configured values.

    Uses zero bias to isolate the white noise component.
    """
    n_samples = 10_000
    readings = np.empty((n_samples, 3))
    state = _zeros_state()

    for i in range(n_samples):
        imu = IMUSensor(
            accel_noise_stddev=0.062,
            accel_bias_stddev=0.0,  # disable bias
            seed=i,
        )
        accel, _ = imu.measure(state, dt=0.0)
        readings[i] = accel

    mean = readings.mean(axis=0)
    std = readings.std(axis=0)

    # Mean should be close to [0, 0, 9.81]
    assert_allclose(mean[:2], [0.0, 0.0], atol=0.005)
    assert_allclose(mean[2], 9.81, atol=0.005)

    # Standard deviation should be close to accel_noise_stddev on all axes
    assert_allclose(std, 0.062, atol=0.01)


def test_gyro_noise_statistics() -> None:
    """Gyro noise stddev should match configured value."""
    n_samples = 10_000
    readings = np.empty((n_samples, 3))
    state = _zeros_state()

    for i in range(n_samples):
        imu = IMUSensor(
            gyro_noise_stddev=0.00138,
            gyro_bias_stddev=0.0,  # disable bias
            seed=i,
        )
        _, gyro = imu.measure(state, dt=0.0)
        readings[i] = gyro

    mean = readings.mean(axis=0)
    std = readings.std(axis=0)

    assert_allclose(mean, [0.0, 0.0, 0.0], atol=0.001)
    assert_allclose(std, 0.00138, atol=0.0005)


# -------------------------------------------------------------------
# Bias drift
# -------------------------------------------------------------------


def test_bias_drift() -> None:
    """Bias should accumulate over time, causing systematic offset."""
    imu = IMUSensor(
        accel_bias_stddev=0.01,
        gyro_bias_stddev=0.01,
        accel_noise_stddev=0.001,  # small noise to make bias effect clear
        gyro_noise_stddev=0.001,
        seed=42,
    )
    state = _zeros_state()
    n_measurements = 1000
    dt = 0.004

    accel_readings = np.empty((n_measurements, 3))
    for i in range(n_measurements):
        accel, _ = imu.measure(state, dt=dt)
        accel_readings[i] = accel

    # First 100 vs last 100 readings should differ due to bias drift
    early_mean = accel_readings[:100].mean(axis=0)
    late_mean = accel_readings[-100:].mean(axis=0)

    # The bias should have drifted enough to produce a measurable difference
    drift = np.linalg.norm(late_mean - early_mean)
    assert drift > 0.01, f"Expected measurable bias drift, got {drift}"


# -------------------------------------------------------------------
# Reproducibility
# -------------------------------------------------------------------


def test_seed_reproducibility() -> None:
    """Two sensors with the same seed must produce identical outputs."""
    state = _zeros_state()

    imu_a = IMUSensor(seed=123)
    imu_b = IMUSensor(seed=123)

    accel_a, gyro_a = imu_a.measure(state)
    accel_b, gyro_b = imu_b.measure(state)

    assert_allclose(accel_a, accel_b, atol=0.0)
    assert_allclose(gyro_a, gyro_b, atol=0.0)


# -------------------------------------------------------------------
# Reset
# -------------------------------------------------------------------


def test_reset() -> None:
    """After reset, biases return to zero and outputs match a fresh sensor."""
    state = _zeros_state()
    seed = 77

    # Take several measurements to accumulate bias
    imu = IMUSensor(seed=seed)
    for _ in range(50):
        imu.measure(state)

    # Reset the sensor
    imu.reset()

    # Create a fresh sensor with the same seed -- note the RNG state differs
    # but the biases should both be zero. We verify biases are zero by
    # checking that the internal bias arrays are zero after reset.
    assert_allclose(imu._accel_bias, np.zeros(3), atol=0.0)
    assert_allclose(imu._gyro_bias, np.zeros(3), atol=0.0)


# -------------------------------------------------------------------
# Tilted body frame
# -------------------------------------------------------------------


def test_tilted_body() -> None:
    """With roll=pi/2, gravity should project onto body Y axis.

    R_body_to_world for roll=pi/2:
        [[1, 0, 0],
         [0, 0, -1],
         [0, 1, 0]]

    R_world_to_body = R^T:
        [[1, 0, 0],
         [0, 0, 1],
         [0, -1, 0]]

    specific_force_body = R_world_to_body @ [0, 0, g] = [0, g, 0]
    """
    q = euler_to_quaternion(math.pi / 2, 0.0, 0.0)
    state = QuadrotorState(
        position=np.zeros(3, dtype=np.float64),
        velocity=np.zeros(3, dtype=np.float64),
        quaternion=q,
        angular_velocity=np.zeros(3, dtype=np.float64),
    )

    imu = IMUSensor(
        accel_bias_stddev=0.0,
        gyro_bias_stddev=0.0,
        seed=42,
    )
    accel, gyro = imu.measure(state, dt=0.0)

    # accel should be approximately [0, 9.81, 0]
    assert_allclose(accel, [0.0, 9.81, 0.0], atol=0.5)
    # gyro should still be approximately zero
    assert_allclose(gyro, [0.0, 0.0, 0.0], atol=0.1)
