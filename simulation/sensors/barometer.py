"""Barometric pressure sensor model (BMP280-like).

Simulates altitude measurement via barometric pressure with Gaussian noise.
"""

from __future__ import annotations

import math

import numpy as np

# ISA (International Standard Atmosphere) constants
_LAPSE_RATE = 0.0065  # K/m (temperature lapse rate)
_SEA_LEVEL_TEMP = 288.15  # K
_MOLAR_MASS_AIR = 0.0289644  # kg/mol
_GAS_CONSTANT = 8.31447  # J/(mol*K)
_GRAVITY = 9.81  # m/s^2
_EXPONENT = _GRAVITY * _MOLAR_MASS_AIR / (_GAS_CONSTANT * _LAPSE_RATE)


class Barometer:
    """Simulated barometric altitude sensor.

    Uses the ISA barometric formula to convert between altitude and pressure.
    Adds Gaussian noise to pressure readings.
    """

    def __init__(
        self,
        pressure_noise_stddev: float = 0.25,
        reference_pressure: float = 101325.0,
        reference_altitude: float = 0.0,
        update_rate: float = 50.0,
        seed: int | None = None,
    ) -> None:
        self.pressure_noise_stddev = pressure_noise_stddev
        self.reference_pressure = reference_pressure
        self.reference_altitude = reference_altitude
        self.update_rate = update_rate
        self._rng = np.random.default_rng(seed)

    def reset(self) -> None:
        """Reset sensor state (no persistent state beyond RNG)."""

    def measure(self, altitude: float) -> tuple[float, float]:
        """Generate noisy pressure and altitude measurement.

        Args:
            altitude: True altitude above reference [m].

        Returns:
            pressure: Measured pressure with noise [Pa].
            altitude_estimate: Altitude derived from noisy pressure [m].
        """
        true_pressure = self._altitude_to_pressure(altitude)
        noise = self._rng.normal(0.0, self.pressure_noise_stddev)
        noisy_pressure = true_pressure + noise
        altitude_estimate = self._pressure_to_altitude(noisy_pressure)
        return float(noisy_pressure), float(altitude_estimate)

    def _altitude_to_pressure(self, altitude: float) -> float:
        """Convert altitude to pressure using ISA barometric formula.

        P = P0 * (1 - L * h / T0) ^ (g * M / (R * L))
        """
        h = altitude - self.reference_altitude
        ratio = 1.0 - _LAPSE_RATE * h / _SEA_LEVEL_TEMP
        ratio = max(ratio, 1e-6)  # prevent negative/zero
        return self.reference_pressure * math.pow(ratio, _EXPONENT)

    def _pressure_to_altitude(self, pressure: float) -> float:
        """Convert pressure to altitude (inverse barometric formula).

        h = (T0 / L) * (1 - (P / P0) ^ (1 / exponent))
        """
        pressure = max(pressure, 1e-6)
        ratio = pressure / self.reference_pressure
        h = (_SEA_LEVEL_TEMP / _LAPSE_RATE) * (1.0 - math.pow(ratio, 1.0 / _EXPONENT))
        return h + self.reference_altitude
