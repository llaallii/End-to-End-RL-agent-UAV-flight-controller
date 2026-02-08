"""Unit tests for the GPS sensor model."""

from __future__ import annotations

import numpy as np

from simulation.sensors.gps import GPS

# -------------------------------------------------------------------
# Noise statistics at origin
# -------------------------------------------------------------------


def test_mean_error_near_zero() -> None:
    """Mean GPS error over many samples should be approximately zero."""
    n_samples = 10_000
    position = np.array([0.0, 0.0, 0.0])
    readings = np.empty((n_samples, 3))

    for i in range(n_samples):
        gps = GPS(seed=i)
        readings[i] = gps.measure(position)

    mean = readings.mean(axis=0)
    np.testing.assert_allclose(mean, [0.0, 0.0, 0.0], atol=0.2)


def test_horizontal_noise_stddev() -> None:
    """Horizontal (X, Y) noise stddev should match configured value."""
    n_samples = 10_000
    position = np.array([0.0, 0.0, 0.0])
    readings = np.empty((n_samples, 3))

    for i in range(n_samples):
        gps = GPS(horizontal_noise_stddev=1.5, seed=i)
        readings[i] = gps.measure(position)

    std_x = readings[:, 0].std()
    std_y = readings[:, 1].std()

    np.testing.assert_allclose(std_x, 1.5, atol=0.2)
    np.testing.assert_allclose(std_y, 1.5, atol=0.2)


def test_vertical_noise_stddev() -> None:
    """Vertical (Z) noise stddev should match configured value."""
    n_samples = 10_000
    position = np.array([0.0, 0.0, 0.0])
    readings = np.empty((n_samples, 3))

    for i in range(n_samples):
        gps = GPS(vertical_noise_stddev=3.0, seed=i)
        readings[i] = gps.measure(position)

    std_z = readings[:, 2].std()
    np.testing.assert_allclose(std_z, 3.0, atol=0.3)


# -------------------------------------------------------------------
# Reproducibility
# -------------------------------------------------------------------


def test_seed_reproducibility() -> None:
    """Two GPS instances with the same seed must produce identical outputs."""
    position = np.array([5.0, 10.0, 15.0])

    gps_a = GPS(seed=123)
    gps_b = GPS(seed=123)

    reading_a = gps_a.measure(position)
    reading_b = gps_b.measure(position)

    np.testing.assert_allclose(reading_a, reading_b, atol=0.0)


# -------------------------------------------------------------------
# Non-zero position
# -------------------------------------------------------------------


def test_nonzero_position() -> None:
    """GPS readings at a nonzero position should average to that position."""
    n_samples = 10_000
    position = np.array([10.0, 20.0, 30.0])
    readings = np.empty((n_samples, 3))

    for i in range(n_samples):
        gps = GPS(seed=i)
        readings[i] = gps.measure(position)

    mean = readings.mean(axis=0)
    np.testing.assert_allclose(mean, position, atol=0.2)


# -------------------------------------------------------------------
# Custom noise levels
# -------------------------------------------------------------------


def test_different_noise() -> None:
    """GPS with custom noise levels should have matching stddev."""
    n_samples = 10_000
    position = np.array([0.0, 0.0, 0.0])
    readings = np.empty((n_samples, 3))

    for i in range(n_samples):
        gps = GPS(horizontal_noise_stddev=0.5, vertical_noise_stddev=1.0, seed=i)
        readings[i] = gps.measure(position)

    std_x = readings[:, 0].std()
    std_y = readings[:, 1].std()
    std_z = readings[:, 2].std()

    np.testing.assert_allclose(std_x, 0.5, atol=0.1)
    np.testing.assert_allclose(std_y, 0.5, atol=0.1)
    np.testing.assert_allclose(std_z, 1.0, atol=0.15)
