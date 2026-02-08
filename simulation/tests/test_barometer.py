"""Unit tests for the barometric pressure sensor model."""

from __future__ import annotations

import numpy as np

from simulation.sensors.barometer import Barometer

# -------------------------------------------------------------------
# Basic functionality
# -------------------------------------------------------------------


def test_sea_level_pressure() -> None:
    """At altitude 0 m, pressure should be close to 101325.0 Pa."""
    baro = Barometer(seed=42)
    pressure, _ = baro.measure(0.0)

    np.testing.assert_allclose(pressure, 101325.0, atol=2.0)


def test_pressure_decreases_with_altitude() -> None:
    """Pressure at 100 m should be less than pressure at 0 m."""
    baro = Barometer(pressure_noise_stddev=0.0, seed=42)
    pressure_0, _ = baro.measure(0.0)
    pressure_100, _ = baro.measure(100.0)

    assert pressure_100 < pressure_0


def test_pressure_increases_at_depth() -> None:
    """Pressure at -10 m (below reference) should be more than at 0 m."""
    baro = Barometer(pressure_noise_stddev=0.0, seed=42)
    pressure_0, _ = baro.measure(0.0)
    pressure_neg, _ = baro.measure(-10.0)

    assert pressure_neg > pressure_0


def test_altitude_roundtrip() -> None:
    """Altitude estimate from noisy pressure should be close to true altitude."""
    target_alt = 50.0

    # Take many samples and average to wash out noise
    n_samples = 5000
    alt_estimates = np.empty(n_samples)
    for i in range(n_samples):
        baro_i = Barometer(seed=i)
        _, alt_est = baro_i.measure(target_alt)
        alt_estimates[i] = alt_est

    mean_alt = alt_estimates.mean()
    np.testing.assert_allclose(mean_alt, target_alt, atol=1.0)


# -------------------------------------------------------------------
# Noise statistics
# -------------------------------------------------------------------


def test_noise_statistics() -> None:
    """Pressure noise should have the configured mean and stddev."""
    n_samples = 10_000
    pressures = np.empty(n_samples)

    for i in range(n_samples):
        baro = Barometer(pressure_noise_stddev=0.25, seed=i)
        pressure, _ = baro.measure(0.0)
        pressures[i] = pressure

    mean = pressures.mean()
    std = pressures.std()

    np.testing.assert_allclose(mean, 101325.0, atol=0.1)
    np.testing.assert_allclose(std, 0.25, atol=0.05)


# -------------------------------------------------------------------
# Reproducibility
# -------------------------------------------------------------------


def test_seed_reproducibility() -> None:
    """Two barometers with the same seed must produce identical outputs."""
    baro_a = Barometer(seed=123)
    baro_b = Barometer(seed=123)

    pressure_a, alt_a = baro_a.measure(50.0)
    pressure_b, alt_b = baro_b.measure(50.0)

    np.testing.assert_allclose(pressure_a, pressure_b, atol=0.0)
    np.testing.assert_allclose(alt_a, alt_b, atol=0.0)


# -------------------------------------------------------------------
# Different reference altitude
# -------------------------------------------------------------------


def test_different_reference() -> None:
    """Barometer with reference_altitude=100 should read ~101325 Pa at 100 m."""
    baro = Barometer(
        pressure_noise_stddev=0.0,
        reference_pressure=101325.0,
        reference_altitude=100.0,
        seed=42,
    )
    pressure, alt_est = baro.measure(100.0)

    # At the reference altitude, pressure equals reference pressure
    np.testing.assert_allclose(pressure, 101325.0, atol=0.01)
    np.testing.assert_allclose(alt_est, 100.0, atol=0.01)
