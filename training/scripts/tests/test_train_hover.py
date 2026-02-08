"""Unit tests for the training pipeline.

Tests cover config loading, policy parameter count (RL008),
short training runs, model save/load, and callback behavior.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor

from training.callbacks.eval_and_checkpoint import MetricLoggerCallback, make_callbacks
from training.configs.train_config import TrainConfig
from training.envs.config import HoverEnvConfig
from training.scripts.train_hover import count_actor_params, make_env


@pytest.fixture
def train_config() -> TrainConfig:
    """Default training configuration."""
    return TrainConfig.default()


@pytest.fixture
def env_config() -> HoverEnvConfig:
    """Default environment configuration."""
    return HoverEnvConfig.default()


@pytest.fixture
def sac_model(env_config: HoverEnvConfig) -> SAC:
    """Create a SAC model with the standard 2x128 architecture."""
    env: Monitor = Monitor(make_env(env_config, seed=42))
    model = SAC(
        "MlpPolicy",
        env,
        policy_kwargs={"net_arch": [128, 128]},
        seed=42,
        verbose=0,
    )
    return model


class TestTrainConfig:
    """Training configuration loading."""

    def test_default_config_values(self, train_config: TrainConfig) -> None:
        assert train_config.algorithm == "SAC"
        assert train_config.net_arch == [128, 128]
        assert train_config.learning_rate == 3e-4
        assert train_config.batch_size == 256
        assert train_config.total_timesteps == 500_000
        assert train_config.seed == 42

    def test_from_yaml(self, tmp_path: Path) -> None:
        yaml_content = """\
algorithm: PPO
policy_kwargs:
  net_arch: [64, 64]
learning_rate: 1.0e-3
total_timesteps: 100000
seed: 99
"""
        cfg_path = tmp_path / "test_train.yaml"
        cfg_path.write_text(yaml_content)
        cfg = TrainConfig.from_yaml(cfg_path)
        assert cfg.algorithm == "PPO"
        assert cfg.net_arch == [64, 64]
        assert cfg.learning_rate == 1e-3
        assert cfg.total_timesteps == 100_000
        assert cfg.seed == 99


class TestPolicyParamCount:
    """RL008: Actor network must have <= 19,076 parameters."""

    def test_actor_param_count_exact(self, sac_model: SAC) -> None:
        """2x128 MLP with 15-dim input and 4-dim output = 19,076 params.

        Breakdown:
        - Layer 1: 15*128 + 128 = 2,048
        - Layer 2: 128*128 + 128 = 16,512
        - Mean head: 128*4 + 4 = 516  (SAC outputs mean of Gaussian)
        Total: 19,076

        Note: SAC also has a log_std head (+516) but those are NOT deployed.
        We count only the latent_pi + mu (action_net) layers.
        """
        actor_params = count_actor_params(sac_model)
        # SAC actor includes both mu (mean) and log_std networks
        # The actual deployment only needs mu, but count_actor_params counts all actor params.
        # With SB3's SAC, actor has: latent (2x128) + mu (128->4) + log_std (128->4)
        # = 2048 + 16512 + 516 + 516 = 19,592
        # For RL008, the DEPLOYED actor is 19,076 (without log_std).
        # We verify it's within budget by checking the architecture matches.
        assert actor_params <= 20_000, f"Actor params {actor_params} too large"
        # Verify net_arch matches RL007
        assert sac_model.policy.net_arch == [128, 128]

    def test_deployed_param_count(self, sac_model: SAC) -> None:
        """Count only the parameters that would be deployed (no log_std)."""
        deployed = 0
        for name, p in sac_model.actor.named_parameters():
            if "log_std" not in name:
                deployed += p.numel()
        assert deployed == 19_076, f"Deployed actor params: {deployed}, expected 19,076"


class TestShortTraining:
    """Short training runs complete without errors."""

    def test_sac_100_steps(self, env_config: HoverEnvConfig) -> None:
        train_env = make_vec_env(
            lambda: Monitor(make_env(env_config, seed=0)),
            n_envs=1,
            seed=0,
        )
        model = SAC(
            "MlpPolicy",
            train_env,
            policy_kwargs={"net_arch": [128, 128]},
            learning_starts=10,
            seed=0,
            verbose=0,
        )
        model.learn(total_timesteps=100)
        train_env.close()


class TestModelSaveLoad:
    """Model save and reload produces same predictions."""

    def test_save_load_deterministic(
        self, sac_model: SAC, env_config: HoverEnvConfig, tmp_path: Path
    ) -> None:
        # Get a prediction before save
        env = make_env(env_config, seed=42)
        obs, _ = env.reset(seed=42)
        action_before, _ = sac_model.predict(obs, deterministic=True)

        # Save and reload
        save_path = tmp_path / "test_model"
        sac_model.save(str(save_path))
        loaded_model = SAC.load(str(save_path))
        action_after, _ = loaded_model.predict(obs, deterministic=True)

        np.testing.assert_array_equal(action_before, action_after)


class TestCallbacks:
    """Callback behavior during training."""

    def test_eval_callback_creates_best_model(
        self, env_config: HoverEnvConfig, tmp_path: Path
    ) -> None:
        train_env = make_vec_env(
            lambda: Monitor(make_env(env_config, seed=0)),
            n_envs=1,
            seed=0,
        )
        eval_env = make_vec_env(
            lambda: Monitor(make_env(env_config, seed=1, eval_mode=True)),
            n_envs=1,
            seed=1,
        )
        model = SAC(
            "MlpPolicy",
            train_env,
            policy_kwargs={"net_arch": [128, 128]},
            learning_starts=10,
            seed=0,
            verbose=0,
        )
        callbacks = make_callbacks(
            eval_env=eval_env,
            log_dir=tmp_path / "logs",
            model_dir=tmp_path / "models",
            eval_freq=50,
            n_eval_episodes=1,
            checkpoint_freq=200,
            verbose=0,
        )
        model.learn(total_timesteps=200, callback=callbacks)

        # EvalCallback should have created best_model
        assert (tmp_path / "models" / "best" / "best_model.zip").exists()
        train_env.close()
        eval_env.close()

    def test_metric_logger_callback_runs(self, env_config: HoverEnvConfig) -> None:
        train_env = make_vec_env(
            lambda: Monitor(make_env(env_config, seed=0)),
            n_envs=1,
            seed=0,
        )
        model = SAC(
            "MlpPolicy",
            train_env,
            policy_kwargs={"net_arch": [128, 128]},
            learning_starts=10,
            seed=0,
            verbose=0,
        )
        callback = MetricLoggerCallback(verbose=0)
        # Should not raise
        model.learn(total_timesteps=100, callback=callback)
        train_env.close()
