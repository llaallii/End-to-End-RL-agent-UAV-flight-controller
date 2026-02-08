"""Shared fixtures for QuadrotorHoverEnv tests."""

from __future__ import annotations

import pytest

from training.envs.config import HoverEnvConfig
from training.envs.quadrotor_hover_env import QuadrotorHoverEnv


@pytest.fixture
def default_config() -> HoverEnvConfig:
    """Default environment configuration."""
    return HoverEnvConfig.default()


@pytest.fixture
def env(default_config: HoverEnvConfig) -> QuadrotorHoverEnv:
    """Fresh environment instance, reset with seed=42."""
    e = QuadrotorHoverEnv(config=default_config)
    e.reset(seed=42)
    return e


@pytest.fixture
def ground_start_config() -> HoverEnvConfig:
    """Config that starts on the ground instead of at hover."""
    cfg = HoverEnvConfig.default()
    cfg.initial_mode = "ground"
    return cfg


@pytest.fixture
def dr_config() -> HoverEnvConfig:
    """Config with domain randomization enabled."""
    cfg = HoverEnvConfig.default()
    cfg.domain_randomization = True
    return cfg
