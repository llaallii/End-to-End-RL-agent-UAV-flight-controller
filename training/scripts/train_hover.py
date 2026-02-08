#!/usr/bin/env python3
"""Train an RL policy for quadrotor hover using Stable-Baselines3.

Usage:
    python -m training.scripts.train_hover
    python -m training.scripts.train_hover --config training/configs/train_hover.yaml
    python -m training.scripts.train_hover --timesteps 100000

Requirement refs: RL007, RL008, RL009, RL011, RL012
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from stable_baselines3 import PPO, SAC, TD3
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor

from training.callbacks.eval_and_checkpoint import make_callbacks
from training.configs.train_config import TrainConfig
from training.envs.config import HoverEnvConfig
from training.envs.quadrotor_hover_env import QuadrotorHoverEnv

ALGORITHMS = {"SAC": SAC, "PPO": PPO, "TD3": TD3}


def count_actor_params(model: SAC | PPO | TD3) -> int:
    """Count trainable parameters in the actor (policy) network only.

    RL008: must be <= 19,076 for deployment on MCU.
    RL009: critic is training-only, not counted.
    """
    if hasattr(model, "actor"):
        # SAC / TD3: separate actor network
        return sum(p.numel() for p in model.actor.parameters())
    # PPO: actor is part of the shared policy
    return sum(
        p.numel()
        for name, p in model.policy.named_parameters()
        if "value" not in name and "critic" not in name
    )


def make_env(
    env_config: HoverEnvConfig, seed: int, eval_mode: bool = False
) -> QuadrotorHoverEnv:
    """Create a single environment instance.

    Args:
        env_config: Environment configuration.
        seed: RNG seed.
        eval_mode: If True, disable domain randomization for consistent eval.
    """
    config = HoverEnvConfig(
        sim_freq=env_config.sim_freq,
        control_freq=env_config.control_freq,
        max_episode_time=env_config.max_episode_time,
        position_scale=env_config.position_scale,
        velocity_scale=env_config.velocity_scale,
        angular_velocity_scale=env_config.angular_velocity_scale,
        max_rot_velocity=env_config.max_rot_velocity,
        goal_position=env_config.goal_position.copy(),
        position_weight=env_config.position_weight,
        velocity_weight=env_config.velocity_weight,
        attitude_weight=env_config.attitude_weight,
        angular_rate_weight=env_config.angular_rate_weight,
        action_smoothness_weight=env_config.action_smoothness_weight,
        alive_bonus=env_config.alive_bonus,
        crash_penalty=env_config.crash_penalty,
        max_tilt_rad=env_config.max_tilt_rad,
        max_altitude=env_config.max_altitude,
        min_altitude=env_config.min_altitude,
        max_horizontal_velocity=env_config.max_horizontal_velocity,
        max_vertical_velocity=env_config.max_vertical_velocity,
        geofence_radius=env_config.geofence_radius,
        initial_mode=env_config.initial_mode,
        position_noise=env_config.position_noise,
        velocity_noise=env_config.velocity_noise,
        attitude_noise_rad=env_config.attitude_noise_rad,
        domain_randomization=False if eval_mode else env_config.domain_randomization,
        mass_range=env_config.mass_range,
        inertia_scale_range=env_config.inertia_scale_range,
        motor_constant_range=env_config.motor_constant_range,
    )
    env = QuadrotorHoverEnv(config=config)
    return env


def train(train_config: TrainConfig) -> Path:
    """Run the full training pipeline.

    Args:
        train_config: Training configuration.

    Returns:
        Path to the saved final model.
    """
    # Load environment config
    env_config = HoverEnvConfig.from_yaml(train_config.env_config_path)

    # Create directories
    log_dir = Path(train_config.log_dir)
    model_dir = Path(train_config.model_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    model_dir.mkdir(parents=True, exist_ok=True)

    # Create training env (vectorized for SB3 compatibility)
    train_env = make_vec_env(
        lambda: Monitor(make_env(env_config, seed=train_config.seed)),
        n_envs=1,
        seed=train_config.seed,
    )

    # Create evaluation env (no domain randomization)
    eval_env = make_vec_env(
        lambda: Monitor(make_env(env_config, seed=train_config.seed + 1000, eval_mode=True)),
        n_envs=1,
        seed=train_config.seed + 1000,
    )

    # Select algorithm
    algo_name = train_config.algorithm.upper()
    if algo_name not in ALGORITHMS:
        print(f"Unknown algorithm '{algo_name}'. Options: {list(ALGORITHMS.keys())}")
        sys.exit(1)
    algo_cls = ALGORITHMS[algo_name]

    # Build policy kwargs (RL007: 2x128 MLP ReLU)
    policy_kwargs = {"net_arch": train_config.net_arch}

    # Common kwargs shared by all algorithms
    common_kwargs: dict = {
        "policy": "MlpPolicy",
        "env": train_env,
        "learning_rate": train_config.learning_rate,
        "batch_size": train_config.batch_size,
        "gamma": train_config.gamma,
        "seed": train_config.seed,
        "policy_kwargs": policy_kwargs,
        "verbose": 1,
        "tensorboard_log": str(log_dir) if train_config.tensorboard else None,
    }

    # Algorithm-specific kwargs
    if algo_name == "SAC":
        common_kwargs.update({
            "buffer_size": train_config.buffer_size,
            "learning_starts": train_config.learning_starts,
            "tau": train_config.tau,
            "ent_coef": train_config.ent_coef,
            "train_freq": train_config.train_freq,
            "gradient_steps": train_config.gradient_steps,
        })
    elif algo_name == "TD3":
        common_kwargs.update({
            "buffer_size": train_config.buffer_size,
            "learning_starts": train_config.learning_starts,
            "tau": train_config.tau,
            "train_freq": train_config.train_freq,
        })
    # PPO uses defaults for the rest

    model = algo_cls(**common_kwargs)

    # Verify parameter count (RL008)
    actor_params = count_actor_params(model)
    print(f"Actor parameters: {actor_params} (limit: 19,076)")
    if actor_params > 19_076:
        print(f"WARNING: Actor exceeds RL008 limit ({actor_params} > 19,076)")

    # Create callbacks
    callbacks = make_callbacks(
        eval_env=eval_env,
        log_dir=log_dir,
        model_dir=model_dir,
        eval_freq=train_config.eval_freq,
        n_eval_episodes=train_config.n_eval_episodes,
        checkpoint_freq=train_config.checkpoint_freq,
    )

    # Train
    print(f"Training {algo_name} for {train_config.total_timesteps} timesteps...")
    model.learn(
        total_timesteps=train_config.total_timesteps,
        callback=callbacks,
        progress_bar=True,
    )

    # Save final model
    final_path = model_dir / "hover_final"
    model.save(str(final_path))
    print(f"Final model saved to {final_path}")

    # Cleanup
    train_env.close()
    eval_env.close()

    return final_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Train RL hover policy")
    parser.add_argument(
        "--config",
        type=str,
        default="training/configs/train_hover.yaml",
        help="Path to training config YAML",
    )
    parser.add_argument(
        "--timesteps",
        type=int,
        default=None,
        help="Override total_timesteps from config",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Override seed from config",
    )
    args = parser.parse_args()

    config = TrainConfig.from_yaml(args.config)
    if args.timesteps is not None:
        config.total_timesteps = args.timesteps
    if args.seed is not None:
        config.seed = args.seed

    train(config)


if __name__ == "__main__":
    main()
