#!/usr/bin/env python3
"""Smoke test for QuadrotorHoverEnv.

Runs a short episode with random actions and prints observation/reward stats.
Usage: python -m training.scripts.smoke_test_env
"""

from __future__ import annotations

import numpy as np

from training.envs.config import HoverEnvConfig
from training.envs.quadrotor_hover_env import QuadrotorHoverEnv


def main() -> None:
    config = HoverEnvConfig.default()
    env = QuadrotorHoverEnv(config=config)

    obs, info = env.reset(seed=42)
    print(f"Initial position: {info['position']}")
    print(f"Goal:             {info['goal']}")
    print(f"Obs shape: {obs.shape}, dtype: {obs.dtype}")
    print(f"Action space: {env.action_space}")
    print()

    rewards = []
    n_steps = 100
    for step in range(n_steps):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        rewards.append(reward)

        if step % 20 == 0:
            print(
                f"Step {step:3d}  pos={info['position']}  "
                f"reward={reward:+.3f}  "
                f"motor_speeds={info['motor_speeds']}"
            )

        if terminated or truncated:
            reason = "terminated" if terminated else "truncated"
            print(f"\nEpisode ended ({reason}) at step {step}")
            break

    print(f"\n--- Stats over {len(rewards)} steps ---")
    print(f"Reward  mean={np.mean(rewards):.3f}  std={np.std(rewards):.3f}")
    print(f"        min={np.min(rewards):.3f}   max={np.max(rewards):.3f}")
    print(f"Final position: {info['position']}")
    print(f"Final euler:    {info['euler']}")
    print("Smoke test passed.")


if __name__ == "__main__":
    main()
