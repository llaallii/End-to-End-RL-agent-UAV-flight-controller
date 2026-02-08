"""Configuration dataclass for RL training hyperparameters.

Loads algorithm, policy, and training schedule settings from a YAML file.
Requirement refs: RL007, RL008, RL011
"""

from __future__ import annotations

import dataclasses
from pathlib import Path

import yaml  # type: ignore[import-untyped]


@dataclasses.dataclass
class WandbConfig:
    """Weights & Biases logging configuration."""

    enabled: bool = False
    project: str = "uav-hover-rl"
    entity: str | None = None


@dataclasses.dataclass
class TrainConfig:
    """All tunable parameters for the training pipeline."""

    # Algorithm
    algorithm: str = "SAC"

    # Environment config path
    env_config_path: str = "training/configs/hover_env.yaml"

    # Policy network
    net_arch: list[int] = dataclasses.field(default_factory=lambda: [128, 128])

    # Algorithm hyperparameters
    learning_rate: float = 3e-4
    batch_size: int = 256
    buffer_size: int = 100_000
    learning_starts: int = 1000
    gamma: float = 0.99
    tau: float = 0.005
    ent_coef: str = "auto"
    train_freq: int = 1
    gradient_steps: int = 1

    # Training schedule
    total_timesteps: int = 500_000
    seed: int = 42

    # Evaluation
    eval_freq: int = 10_000
    n_eval_episodes: int = 5

    # Checkpointing
    checkpoint_freq: int = 50_000

    # Logging
    log_dir: str = "training/logs"
    model_dir: str = "training/models"
    tensorboard: bool = True
    wandb: WandbConfig = dataclasses.field(default_factory=WandbConfig)

    @classmethod
    def from_yaml(cls, path: str | Path) -> TrainConfig:
        """Load training configuration from a YAML file."""
        path = Path(path)
        with path.open() as f:
            data = yaml.safe_load(f)

        policy_kwargs = data.get("policy_kwargs", {})
        wandb_data = data.get("wandb", {})

        return cls(
            algorithm=data.get("algorithm", "SAC"),
            env_config_path=data.get("env_config_path", "training/configs/hover_env.yaml"),
            net_arch=policy_kwargs.get("net_arch", [128, 128]),
            learning_rate=data.get("learning_rate", 3e-4),
            batch_size=data.get("batch_size", 256),
            buffer_size=data.get("buffer_size", 100_000),
            learning_starts=data.get("learning_starts", 1000),
            gamma=data.get("gamma", 0.99),
            tau=data.get("tau", 0.005),
            ent_coef=str(data.get("ent_coef", "auto")),
            train_freq=data.get("train_freq", 1),
            gradient_steps=data.get("gradient_steps", 1),
            total_timesteps=data.get("total_timesteps", 500_000),
            seed=data.get("seed", 42),
            eval_freq=data.get("eval_freq", 10_000),
            n_eval_episodes=data.get("n_eval_episodes", 5),
            checkpoint_freq=data.get("checkpoint_freq", 50_000),
            log_dir=data.get("log_dir", "training/logs"),
            model_dir=data.get("model_dir", "training/models"),
            tensorboard=data.get("tensorboard", True),
            wandb=WandbConfig(
                enabled=wandb_data.get("enabled", False),
                project=wandb_data.get("project", "uav-hover-rl"),
                entity=wandb_data.get("entity"),
            ),
        )

    @classmethod
    def default(cls) -> TrainConfig:
        """Return default training configuration."""
        return cls()
