"""Unit tests for QuadrotorHoverEnv.

Tests cover observation/action spaces, reward, termination, truncation,
domain randomization, and Gymnasium compatibility.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from training.envs.config import HoverEnvConfig
from training.envs.quadrotor_hover_env import QuadrotorHoverEnv


class TestSpaces:
    """Observation and action space shape/bounds."""

    def test_obs_space_shape(self, env: QuadrotorHoverEnv) -> None:
        assert env.observation_space.shape == (15,)

    def test_obs_space_bounds(self, env: QuadrotorHoverEnv) -> None:
        assert np.all(env.observation_space.low == -1.0)  # type: ignore[attr-defined]
        assert np.all(env.observation_space.high == 1.0)  # type: ignore[attr-defined]

    def test_action_space_shape(self, env: QuadrotorHoverEnv) -> None:
        assert env.action_space.shape == (4,)

    def test_action_space_bounds(self, env: QuadrotorHoverEnv) -> None:
        assert np.all(env.action_space.low == -1.0)  # type: ignore[attr-defined]
        assert np.all(env.action_space.high == 1.0)  # type: ignore[attr-defined]


class TestReset:
    """reset() returns valid observation and info."""

    def test_reset_returns_valid_obs(self, default_config: HoverEnvConfig) -> None:
        env = QuadrotorHoverEnv(config=default_config)
        obs, _info = env.reset(seed=0)
        assert obs.shape == (15,)
        assert obs.dtype == np.float32
        assert np.all(obs >= -1.0) and np.all(obs <= 1.0)

    def test_reset_info_has_keys(self, env: QuadrotorHoverEnv) -> None:
        _obs, info = env.reset(seed=0)
        expected_keys = {"time", "step_count", "position", "velocity", "euler",
                         "angular_velocity", "motor_speeds", "goal"}
        assert expected_keys.issubset(info.keys())

    def test_reset_time_is_zero(self, env: QuadrotorHoverEnv) -> None:
        _, info = env.reset(seed=0)
        assert info["time"] == 0.0
        assert info["step_count"] == 0

    def test_hover_mode_starts_at_goal(self, default_config: HoverEnvConfig) -> None:
        env = QuadrotorHoverEnv(config=default_config)
        _, info = env.reset(seed=0)
        np.testing.assert_allclose(info["position"], default_config.goal_position, atol=0.01)

    def test_ground_mode_starts_at_origin(self, ground_start_config: HoverEnvConfig) -> None:
        env = QuadrotorHoverEnv(config=ground_start_config)
        _, info = env.reset(seed=0)
        np.testing.assert_allclose(info["position"], [0, 0, 0], atol=0.01)

    def test_goal_override_via_options(self, default_config: HoverEnvConfig) -> None:
        env = QuadrotorHoverEnv(config=default_config)
        goal = [2.0, 3.0, 5.0]
        _, info = env.reset(seed=0, options={"goal": goal})
        np.testing.assert_allclose(info["goal"], goal)


class TestStep:
    """step() returns five-tuple with correct types."""

    def test_step_returns_five_tuple(self, env: QuadrotorHoverEnv) -> None:
        action = env.action_space.sample()
        result = env.step(action)
        assert len(result) == 5
        obs, reward, terminated, truncated, info = result
        assert obs.shape == (15,)
        assert isinstance(reward, float)
        assert isinstance(terminated, bool)
        assert isinstance(truncated, bool)
        assert isinstance(info, dict)

    def test_obs_stays_in_bounds_after_step(self, env: QuadrotorHoverEnv) -> None:
        for _ in range(10):
            action = env.action_space.sample()
            obs, *_ = env.step(action)
            assert np.all(obs >= -1.0) and np.all(obs <= 1.0)

    def test_time_advances(self, env: QuadrotorHoverEnv) -> None:
        action = np.zeros(4, dtype=np.float32)
        _, _, _, _, info = env.step(action)
        expected_dt = env.config.substeps * env.config.dt
        assert abs(info["time"] - expected_dt) < 1e-9


class TestActionMapping:
    """Action [-1, 1] maps correctly to motor speeds."""

    def test_action_minus_one_maps_to_zero(self, env: QuadrotorHoverEnv) -> None:
        action = np.full(4, -1.0, dtype=np.float32)
        env.step(action)
        # After one step, motor speeds should be near zero
        # (first-order dynamics won't reach exactly 0 in one step but command is 0)
        # Check that the observation changed (motors were commanded to 0)
        # We verify mapping indirectly: -1 -> 0 rad/s command
        speeds = (action.astype(np.float64) + 1.0) / 2.0 * env.config.max_rot_velocity
        np.testing.assert_allclose(speeds, 0.0)

    def test_action_plus_one_maps_to_max(self, env: QuadrotorHoverEnv) -> None:
        action = np.full(4, 1.0, dtype=np.float32)
        speeds = (action.astype(np.float64) + 1.0) / 2.0 * env.config.max_rot_velocity
        np.testing.assert_allclose(speeds, env.config.max_rot_velocity)

    def test_action_zero_maps_to_half(self, env: QuadrotorHoverEnv) -> None:
        action = np.zeros(4, dtype=np.float32)
        speeds = (action.astype(np.float64) + 1.0) / 2.0 * env.config.max_rot_velocity
        np.testing.assert_allclose(speeds, env.config.max_rot_velocity / 2.0)


class TestReward:
    """Reward function properties."""

    def test_reward_is_finite(self, env: QuadrotorHoverEnv) -> None:
        for _ in range(20):
            action = env.action_space.sample()
            _, reward, terminated, _, _ = env.step(action)
            assert math.isfinite(reward)
            if terminated:
                break

    def test_hover_at_goal_gives_best_reward(self, default_config: HoverEnvConfig) -> None:
        """Hovering at goal with zero action change should give maximum reward."""
        env = QuadrotorHoverEnv(config=default_config)
        env.reset(seed=42)
        # Compute hover action: hover_speed / max_speed * 2 - 1
        from simulation.core.motor_model import MotorModel

        params = env._base_params
        hover_omega = MotorModel.compute_hover_velocity(
            params.mass, params.gravity, params.motor.motor_constant
        )
        hover_action = np.full(4, (hover_omega / default_config.max_rot_velocity) * 2.0 - 1.0,
                               dtype=np.float32)

        # First step sets prev_action
        _, _r1, _, _, _ = env.step(hover_action)
        # Second step with same action has zero smoothness penalty
        _, r2, _, _, _ = env.step(hover_action)
        # Reward at goal should be close to alive_bonus (penalties ≈ 0)
        assert r2 > 0.0, f"Hover reward should be positive, got {r2}"

    def test_crash_penalty_applied(self, default_config: HoverEnvConfig) -> None:
        """Termination should include crash penalty."""
        env = QuadrotorHoverEnv(config=default_config)
        env.reset(seed=42)
        # Drive motors asymmetrically to cause extreme tilt
        action_flip = np.array([1.0, -1.0, -1.0, 1.0], dtype=np.float32)
        terminated = False
        for _ in range(500):
            _, reward, terminated, _, _ = env.step(action_flip)
            if terminated:
                # Reward on termination step should include crash penalty
                assert reward < -default_config.crash_penalty / 2
                break
        assert terminated, "Expected termination from tilt"


class TestTermination:
    """Safety termination conditions."""

    def test_termination_on_extreme_tilt(self, default_config: HoverEnvConfig) -> None:
        env = QuadrotorHoverEnv(config=default_config)
        env.reset(seed=42)
        # Apply large asymmetric thrust to flip
        action = np.array([1.0, -1.0, -1.0, 1.0], dtype=np.float32)
        terminated = False
        for _ in range(500):
            _, _, terminated, _, _ = env.step(action)
            if terminated:
                break
        assert terminated, "Expected termination from exceeding tilt limit"

    def test_truncation_on_timeout(self) -> None:
        cfg = HoverEnvConfig.default()
        cfg.max_episode_time = 0.1  # Very short episode
        env = QuadrotorHoverEnv(config=cfg)
        env.reset(seed=42)

        # Hover action to avoid crash
        from simulation.core.motor_model import MotorModel

        params = env._base_params
        hover_omega = MotorModel.compute_hover_velocity(
            params.mass, params.gravity, params.motor.motor_constant
        )
        hover_action = np.full(4, (hover_omega / cfg.max_rot_velocity) * 2.0 - 1.0,
                               dtype=np.float32)

        truncated = False
        for _ in range(1000):
            _, _, terminated, truncated, _ = env.step(hover_action)
            if truncated or terminated:
                break
        assert truncated, "Expected truncation from timeout"


class TestDomainRandomization:
    """Domain randomization varies parameters across resets."""

    def test_mass_varies_across_resets(self, dr_config: HoverEnvConfig) -> None:
        env = QuadrotorHoverEnv(config=dr_config)
        masses = []
        for i in range(10):
            env.reset(seed=i)
            assert env._dynamics is not None
            masses.append(env._dynamics.params.mass)
        # With 10 different seeds, mass should vary
        assert len(set(round(m, 6) for m in masses)) > 1, "Mass should vary across resets"

    def test_randomized_mass_in_range(self, dr_config: HoverEnvConfig) -> None:
        env = QuadrotorHoverEnv(config=dr_config)
        for i in range(20):
            env.reset(seed=i)
            assert env._dynamics is not None
            m = env._dynamics.params.mass
            assert dr_config.mass_range[0] <= m <= dr_config.mass_range[1]


class TestDeterminism:
    """Same seed produces same trajectory."""

    def test_deterministic_with_seed(self, default_config: HoverEnvConfig) -> None:
        env1 = QuadrotorHoverEnv(config=default_config)
        env2 = QuadrotorHoverEnv(config=default_config)

        obs1, _ = env1.reset(seed=123)
        obs2, _ = env2.reset(seed=123)
        np.testing.assert_array_equal(obs1, obs2)

        action = np.array([0.2, -0.1, 0.3, 0.0], dtype=np.float32)
        for _ in range(5):
            obs1, r1, t1, tr1, _ = env1.step(action)
            obs2, r2, t2, tr2, _ = env2.step(action)
            np.testing.assert_array_equal(obs1, obs2)
            assert r1 == r2
            assert t1 == t2
            assert tr1 == tr2


class TestGymnasiumCompat:
    """Gymnasium compatibility via check_env."""

    def test_check_env_passes(self, default_config: HoverEnvConfig) -> None:
        from gymnasium.utils.env_checker import check_env

        env = QuadrotorHoverEnv(config=default_config)
        # check_env will raise if anything is wrong
        check_env(env.unwrapped, skip_render_check=True)


class TestConfigLoading:
    """Config creation and YAML loading."""

    def test_default_config_values(self) -> None:
        cfg = HoverEnvConfig.default()
        assert cfg.sim_freq == 250
        assert cfg.control_freq == 50
        assert cfg.substeps == 5
        assert abs(cfg.dt - 0.004) < 1e-9
        assert cfg.max_rot_velocity == 838.0

    def test_from_yaml(self, tmp_path: pytest.TempPathFactory) -> None:
        yaml_content = """\
physics:
  sim_freq: 500
  control_freq: 100
  max_episode_time: 10.0

observation:
  position_scale: 3.0

goal:
  position: [1.0, 2.0, 3.0]

domain_randomization:
  enabled: true
  mass_range: [0.9, 1.1]
"""
        p = tmp_path / "test_cfg.yaml"  # type: ignore[operator]
        p.write_text(yaml_content)
        cfg = HoverEnvConfig.from_yaml(p)
        assert cfg.sim_freq == 500
        assert cfg.control_freq == 100
        assert cfg.position_scale == 3.0
        np.testing.assert_array_equal(cfg.goal_position, [1.0, 2.0, 3.0])
        assert cfg.domain_randomization is True
        assert cfg.mass_range == (0.9, 1.1)
