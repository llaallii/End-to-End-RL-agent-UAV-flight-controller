"""Configuration dataclass for the QuadrotorHover Gymnasium environment.

Loads tunable parameters from a YAML file and provides typed access.
Requirement refs: RL001-RL006, SAF007-SAF010
"""

from __future__ import annotations

import dataclasses
import math
from pathlib import Path

import numpy as np
import yaml  # type: ignore[import-untyped]


@dataclasses.dataclass
class HoverEnvConfig:
    """All tunable parameters for QuadrotorHoverEnv."""

    # Physics timing
    sim_freq: int = 250
    control_freq: int = 50
    max_episode_time: float = 30.0

    # Observation normalization scales
    position_scale: float = 5.0
    velocity_scale: float = 5.0
    angular_velocity_scale: float = 10.0

    # Action mapping
    max_rot_velocity: float = 838.0

    # Goal
    goal_position: np.ndarray = dataclasses.field(
        default_factory=lambda: np.array([0.0, 0.0, 1.0])
    )

    # Reward weights
    position_weight: float = 1.0
    velocity_weight: float = 0.1
    attitude_weight: float = 0.5
    angular_rate_weight: float = 0.05
    action_smoothness_weight: float = 0.01
    alive_bonus: float = 0.5
    crash_penalty: float = 5.0

    # Termination limits
    max_tilt_rad: float = math.radians(45.0)
    max_altitude: float = 100.0
    min_altitude: float = -0.5
    max_horizontal_velocity: float = 15.0
    max_vertical_velocity: float = 5.0
    geofence_radius: float = 500.0

    # Initial state
    initial_mode: str = "hover"
    position_noise: float = 0.0
    velocity_noise: float = 0.0
    attitude_noise_rad: float = 0.0

    # Domain randomization
    domain_randomization: bool = False
    mass_range: tuple[float, float] = (0.8, 1.2)
    inertia_scale_range: tuple[float, float] = (0.85, 1.15)
    motor_constant_range: tuple[float, float] = (7.7e-6, 9.4e-6)

    @property
    def dt(self) -> float:
        """Physics timestep in seconds."""
        return 1.0 / self.sim_freq

    @property
    def substeps(self) -> int:
        """Number of physics substeps per control step."""
        return self.sim_freq // self.control_freq

    @classmethod
    def from_yaml(cls, path: str | Path) -> HoverEnvConfig:
        """Load configuration from a YAML file."""
        path = Path(path)
        with path.open() as f:
            data = yaml.safe_load(f)

        physics = data.get("physics", {})
        obs = data.get("observation", {})
        action = data.get("action", {})
        goal = data.get("goal", {})
        reward = data.get("reward", {})
        term = data.get("termination", {})
        init = data.get("initial_state", {})
        dr = data.get("domain_randomization", {})

        return cls(
            sim_freq=physics.get("sim_freq", 250),
            control_freq=physics.get("control_freq", 50),
            max_episode_time=physics.get("max_episode_time", 30.0),
            position_scale=obs.get("position_scale", 5.0),
            velocity_scale=obs.get("velocity_scale", 5.0),
            angular_velocity_scale=obs.get("angular_velocity_scale", 10.0),
            max_rot_velocity=action.get("max_rot_velocity", 838.0),
            goal_position=np.array(goal.get("position", [0.0, 0.0, 1.0])),
            position_weight=reward.get("position_weight", 1.0),
            velocity_weight=reward.get("velocity_weight", 0.1),
            attitude_weight=reward.get("attitude_weight", 0.5),
            angular_rate_weight=reward.get("angular_rate_weight", 0.05),
            action_smoothness_weight=reward.get("action_smoothness_weight", 0.01),
            alive_bonus=reward.get("alive_bonus", 0.5),
            crash_penalty=reward.get("crash_penalty", 5.0),
            max_tilt_rad=math.radians(term.get("max_tilt_deg", 45.0)),
            max_altitude=term.get("max_altitude", 100.0),
            min_altitude=term.get("min_altitude", -0.5),
            max_horizontal_velocity=term.get("max_horizontal_velocity", 15.0),
            max_vertical_velocity=term.get("max_vertical_velocity", 5.0),
            geofence_radius=term.get("geofence_radius", 500.0),
            initial_mode=init.get("mode", "hover"),
            position_noise=init.get("position_noise", 0.0),
            velocity_noise=init.get("velocity_noise", 0.0),
            attitude_noise_rad=math.radians(init.get("attitude_noise_deg", 0.0)),
            domain_randomization=dr.get("enabled", False),
            mass_range=tuple(dr.get("mass_range", [0.8, 1.2])),
            inertia_scale_range=tuple(dr.get("inertia_scale_range", [0.85, 1.15])),
            motor_constant_range=tuple(dr.get("motor_constant_range", [7.7e-6, 9.4e-6])),
        )

    @classmethod
    def default(cls) -> HoverEnvConfig:
        """Return default configuration (matches hover_env.yaml defaults)."""
        return cls()
