"""GPS sensor model with position noise."""

from __future__ import annotations

import numpy as np


class GPS:
    """Simulated GPS receiver with Gaussian position noise.

    Models a simple GPS with independent noise on each axis.
    Horizontal (XY) and vertical (Z) noise have separate standard deviations.
    """

    def __init__(
        self,
        horizontal_noise_stddev: float = 1.5,
        vertical_noise_stddev: float = 3.0,
        update_rate: float = 5.0,
        seed: int | None = None,
    ) -> None:
        self.horizontal_noise_stddev = horizontal_noise_stddev
        self.vertical_noise_stddev = vertical_noise_stddev
        self.update_rate = update_rate
        self._rng = np.random.default_rng(seed)

    def reset(self) -> None:
        """Reset sensor state (no persistent state beyond RNG)."""

    def measure(self, position: np.ndarray) -> np.ndarray:
        """Generate noisy GPS position measurement.

        Args:
            position: (3,) true position [x, y, z] in world frame [m].

        Returns:
            (3,) noisy position measurement [m].
        """
        noise = np.array(
            [
                self._rng.normal(0.0, self.horizontal_noise_stddev),
                self._rng.normal(0.0, self.horizontal_noise_stddev),
                self._rng.normal(0.0, self.vertical_noise_stddev),
            ],
            dtype=np.float64,
        )
        return position + noise
