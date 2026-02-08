"""Shared fixtures for simulation unit tests."""

from __future__ import annotations

import numpy as np
import pytest

from simulation.core.types import QuadrotorParams, QuadrotorState


@pytest.fixture()
def default_params() -> QuadrotorParams:
    """Default quadrotor parameters matching quadrotor_params.yaml."""
    return QuadrotorParams.default()


@pytest.fixture()
def identity_state() -> QuadrotorState:
    """State at origin with identity orientation and zero velocity."""
    return QuadrotorState.zeros()


@pytest.fixture()
def hover_state() -> QuadrotorState:
    """State hovering at z=1m with identity orientation."""
    return QuadrotorState.hover_at(z=1.0)


@pytest.fixture()
def rng() -> np.random.Generator:
    """Seeded random number generator for reproducible tests."""
    return np.random.default_rng(42)
