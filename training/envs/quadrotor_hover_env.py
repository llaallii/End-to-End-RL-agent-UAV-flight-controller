"""Gymnasium environment for quadrotor hover task.

Wraps the Python QuadrotorDynamics in a Gymnasium-compatible interface with
15-dim observation, 4-dim action, configurable reward, and safety termination.

Requirement refs: RL001-RL006, RL008, SAF007-SAF010
"""

from __future__ import annotations

import copy
from typing import Any, ClassVar

import gymnasium as gym
import numpy as np

from simulation.core.quadrotor_dynamics import QuadrotorDynamics
from simulation.core.quaternion_utils import quaternion_to_euler
from simulation.core.types import QuadrotorParams, QuadrotorState
from training.envs.config import HoverEnvConfig


class QuadrotorHoverEnv(gym.Env[np.ndarray, np.ndarray]):
    """Gymnasium environment for learning quadrotor hover.

    Observation (15-dim, normalized to ~[-1, 1]):
        [0:3]  position error (goal - position) / pos_scale
        [3:6]  velocity / vel_scale
        [6:9]  euler angles [roll, pitch, yaw] / pi
        [9:12] angular velocity [p, q, r] / ang_vel_scale
        [12:15] goal position / pos_scale

    Action (4-dim, [-1, 1]):
        Mapped to motor speeds [0, max_rot_velocity] via (a + 1) / 2 * max.

    RL008 verification:
        With a 2x128 MLP: (15+1)*128 + (128+1)*128 + (128+1)*4 = 19,076 params.
    """

    metadata: ClassVar[dict[str, Any]] = {  # type: ignore[misc]
        "render_modes": ["human"],
        "render_fps": 50,
    }

    def __init__(
        self,
        config: HoverEnvConfig | None = None,
        render_mode: str | None = None,
    ) -> None:
        super().__init__()
        self.config = config if config is not None else HoverEnvConfig.default()
        self.render_mode = render_mode

        # Spaces (RL001, RL002, RL003)
        self.observation_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(15,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(4,), dtype=np.float32
        )

        # Internal state — created on reset()
        self._base_params = QuadrotorParams.default()
        self._dynamics: QuadrotorDynamics | None = None
        self._prev_action = np.zeros(4, dtype=np.float64)
        self._episode_time: float = 0.0
        self._step_count: int = 0

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        """Reset environment to initial conditions.

        Args:
            seed: RNG seed for reproducibility.
            options: Optional overrides (e.g. {"goal": [x,y,z]}).

        Returns:
            observation, info dict.
        """
        super().reset(seed=seed)

        # Apply domain randomization (RL006)
        params = self._maybe_randomize_params()

        # Create fresh dynamics
        self._dynamics = QuadrotorDynamics(params)

        # Set initial state
        initial_state = self._make_initial_state()
        hover_speeds = self._compute_hover_speeds(params) if self.config.initial_mode == "hover" else None
        self._dynamics.reset(state=initial_state, motor_velocities=hover_speeds)

        # Reset tracking
        self._prev_action = np.zeros(4, dtype=np.float64)
        self._episode_time = 0.0
        self._step_count = 0

        # Allow goal override via options
        goal = self.config.goal_position.copy()
        if options is not None and "goal" in options:
            goal = np.array(options["goal"], dtype=np.float64)
        self._goal = goal

        obs = self._get_observation()
        info = self._get_info()
        return obs, info

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        """Execute one control step (multiple physics substeps).

        Args:
            action: (4,) motor commands in [-1, 1].

        Returns:
            observation, reward, terminated, truncated, info.
        """
        assert self._dynamics is not None, "Call reset() before step()"

        # Map action [-1, 1] -> motor speeds [0, max_rot_velocity] (RL003)
        action_clipped = np.clip(action, -1.0, 1.0).astype(np.float64)
        motor_speeds = (action_clipped + 1.0) / 2.0 * self.config.max_rot_velocity

        # Run physics substeps
        dt = self.config.dt
        for _ in range(self.config.substeps):
            self._dynamics.step(motor_speeds, dt)

        self._episode_time += self.config.substeps * dt
        self._step_count += 1

        # Compute outputs
        obs = self._get_observation()
        terminated = self._check_termination()
        truncated = self._check_truncation()
        reward = self._compute_reward(action_clipped, terminated)
        info = self._get_info()

        self._prev_action = action_clipped.copy()
        return obs, reward, terminated, truncated, info

    # -- Observation ---------------------------------------------------------------

    def _get_observation(self) -> np.ndarray:
        """Build 15-dim normalized observation (RL001, RL002)."""
        assert self._dynamics is not None
        state = self._dynamics.state
        cfg = self.config

        # Position error: goal - current position
        pos_error = self._goal - state.position

        # Euler angles from quaternion
        euler = quaternion_to_euler(state.quaternion)

        obs = np.concatenate([
            pos_error / cfg.position_scale,
            state.velocity / cfg.velocity_scale,
            euler / np.pi,
            state.angular_velocity / cfg.angular_velocity_scale,
            self._goal / cfg.position_scale,
        ])

        result: np.ndarray = np.clip(obs, -1.0, 1.0).astype(np.float32)
        return result

    # -- Reward --------------------------------------------------------------------

    def _compute_reward(self, action: np.ndarray, terminated: bool) -> float:
        """Compute shaped reward (RL004)."""
        assert self._dynamics is not None
        state = self._dynamics.state
        cfg = self.config

        pos_error = self._goal - state.position
        euler = quaternion_to_euler(state.quaternion)

        # Penalties (all non-negative, subtracted)
        pos_penalty = cfg.position_weight * float(np.linalg.norm(pos_error))
        vel_penalty = cfg.velocity_weight * float(np.linalg.norm(state.velocity))
        att_penalty = cfg.attitude_weight * (abs(euler[0]) + abs(euler[1]))
        rate_penalty = cfg.angular_rate_weight * float(np.linalg.norm(state.angular_velocity))
        smooth_penalty = cfg.action_smoothness_weight * float(
            np.linalg.norm(action - self._prev_action)
        )

        reward = (
            -pos_penalty
            - vel_penalty
            - att_penalty
            - rate_penalty
            - smooth_penalty
            + cfg.alive_bonus
        )

        if terminated:
            reward -= cfg.crash_penalty

        return float(reward)

    # -- Termination / Truncation --------------------------------------------------

    def _check_termination(self) -> bool:
        """Check safety-critical termination conditions (RL005, SAF007-SAF010)."""
        assert self._dynamics is not None
        state = self._dynamics.state
        cfg = self.config

        euler = quaternion_to_euler(state.quaternion)

        # SAF008: max tilt
        if abs(euler[0]) > cfg.max_tilt_rad or abs(euler[1]) > cfg.max_tilt_rad:
            return True

        # SAF007: altitude limits
        if state.position[2] > cfg.max_altitude or state.position[2] < cfg.min_altitude:
            return True

        # SAF009: velocity limits
        horiz_vel = float(np.linalg.norm(state.velocity[:2]))
        if horiz_vel > cfg.max_horizontal_velocity:
            return True
        if abs(state.velocity[2]) > cfg.max_vertical_velocity:
            return True

        # SAF010: geofence
        horiz_dist = float(np.linalg.norm(state.position[:2]))
        return horiz_dist > cfg.geofence_radius

    def _check_truncation(self) -> bool:
        """Check time-based truncation (RL005)."""
        return self._episode_time >= self.config.max_episode_time

    # -- Info ----------------------------------------------------------------------

    def _get_info(self) -> dict[str, Any]:
        """Build info dict with diagnostic data."""
        assert self._dynamics is not None
        state = self._dynamics.state
        return {
            "time": self._episode_time,
            "step_count": self._step_count,
            "position": state.position.copy(),
            "velocity": state.velocity.copy(),
            "euler": quaternion_to_euler(state.quaternion),
            "angular_velocity": state.angular_velocity.copy(),
            "motor_speeds": self._dynamics.motors.velocities.copy(),
            "goal": self._goal.copy(),
        }

    # -- Internal helpers ----------------------------------------------------------

    def _maybe_randomize_params(self) -> QuadrotorParams:
        """Optionally apply domain randomization (RL006)."""
        params = copy.deepcopy(self._base_params)
        cfg = self.config

        if not cfg.domain_randomization or self.np_random is None:
            return params

        rng = self.np_random

        # Randomize mass
        params.mass = rng.uniform(cfg.mass_range[0], cfg.mass_range[1])

        # Randomize inertia (scale diagonal)
        scale = rng.uniform(cfg.inertia_scale_range[0], cfg.inertia_scale_range[1])
        params.inertia = params.inertia * scale

        # Randomize motor constant
        params.motor.motor_constant = rng.uniform(
            cfg.motor_constant_range[0], cfg.motor_constant_range[1]
        )

        return params

    def _make_initial_state(self) -> QuadrotorState:
        """Create initial state based on config."""
        cfg = self.config

        if cfg.initial_mode == "hover":
            state = QuadrotorState.hover_at(
                x=cfg.goal_position[0],
                y=cfg.goal_position[1],
                z=cfg.goal_position[2],
            )
        else:
            state = QuadrotorState.zeros()

        # Add noise if configured
        if cfg.position_noise > 0.0 and self.np_random is not None:
            state.position += self.np_random.normal(0, cfg.position_noise, size=3)

        if cfg.velocity_noise > 0.0 and self.np_random is not None:
            state.velocity += self.np_random.normal(0, cfg.velocity_noise, size=3)

        if cfg.attitude_noise_rad > 0.0 and self.np_random is not None:
            from simulation.core.quaternion_utils import euler_to_quaternion

            euler = self.np_random.normal(0, cfg.attitude_noise_rad, size=3)
            state.quaternion = euler_to_quaternion(
                float(euler[0]), float(euler[1]), float(euler[2])
            )

        return state

    def _compute_hover_speeds(self, params: QuadrotorParams) -> np.ndarray:
        """Compute per-motor hover speeds for initial motor state."""
        from simulation.core.motor_model import MotorModel

        omega = MotorModel.compute_hover_velocity(
            mass=params.mass,
            gravity=params.gravity,
            motor_constant=params.motor.motor_constant,
            num_motors=params.motor.num_motors,
        )
        return np.full(params.motor.num_motors, omega)
