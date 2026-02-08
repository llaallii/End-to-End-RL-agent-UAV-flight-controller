"""Custom Gymnasium environments for RL training."""

import gymnasium

gymnasium.register(
    id="QuadrotorHover-v0",
    entry_point="training.envs.quadrotor_hover_env:QuadrotorHoverEnv",
)
