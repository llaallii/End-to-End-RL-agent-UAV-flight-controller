"""Unit tests for simulation.core.quadrotor_dynamics."""

from __future__ import annotations

import numpy as np
import pytest
from numpy.testing import assert_allclose

from simulation.core.motor_model import MotorModel
from simulation.core.quadrotor_dynamics import QuadrotorDynamics
from simulation.core.types import QuadrotorParams, QuadrotorState


@pytest.fixture()
def dynamics():
    params = QuadrotorParams.default()
    return QuadrotorDynamics(params)


def test_free_fall_one_step(dynamics):
    """With zero motors, first step should produce acceleration ≈ -g."""
    state = QuadrotorState.hover_at(z=10.0)
    dynamics.reset(state)
    dt = 0.004
    s = dynamics.step(np.zeros(4), dt)
    # After one step: vz ≈ -g * dt
    expected_vz = -dynamics.params.gravity * dt
    assert s.velocity[2] == pytest.approx(expected_vz, rel=0.05)


def test_free_fall_z_decreases(dynamics):
    """In free fall, z should decrease monotonically."""
    state = QuadrotorState.hover_at(z=10.0)
    dynamics.reset(state)
    prev_z = 10.0
    for _ in range(100):
        s = dynamics.step(np.zeros(4), 0.004)
        assert s.position[2] < prev_z
        prev_z = s.position[2]


def test_ground_collision_clamp(dynamics):
    """When hitting the ground, z should be clamped to 0 and vz should not be negative."""
    state = QuadrotorState.hover_at(z=0.1)
    dynamics.reset(state)
    # Run until ground contact
    for _ in range(500):
        s = dynamics.step(np.zeros(4), 0.004)
    assert s.position[2] >= 0.0
    assert s.velocity[2] >= 0.0


def test_hover_equilibrium(dynamics):
    """Commanding hover speed should maintain altitude ≈ 1.0m."""
    params = dynamics.params
    hover_omega = MotorModel.compute_hover_velocity(
        params.mass, params.gravity, params.motor.motor_constant, 4
    )
    hover_cmds = np.full(4, hover_omega)
    state = QuadrotorState.hover_at(z=1.0)
    dynamics.reset(state, hover_cmds)
    dt = 0.004
    for _ in range(1000):  # 4 seconds
        s = dynamics.step(hover_cmds, dt)
    assert s.position[2] == pytest.approx(1.0, abs=0.15)
    assert np.linalg.norm(s.velocity) < 0.5


def test_quaternion_normalization(dynamics):
    """Quaternion should stay normalized after many steps."""
    state = QuadrotorState.hover_at(z=5.0)
    params = dynamics.params
    hover_omega = MotorModel.compute_hover_velocity(
        params.mass, params.gravity, params.motor.motor_constant, 4
    )
    dynamics.reset(state, np.full(4, hover_omega))
    for _ in range(1000):
        s = dynamics.step(np.full(4, hover_omega), 0.004)
    assert np.linalg.norm(s.quaternion) == pytest.approx(1.0, abs=1e-6)


def test_reset(dynamics):
    """Reset should return to initial state."""
    dynamics.reset(QuadrotorState.hover_at(z=5.0))
    for _ in range(10):
        dynamics.step(np.zeros(4), 0.004)
    s = dynamics.reset()
    assert_allclose(s.position, [0, 0, 0])
    assert_allclose(s.quaternion, [1, 0, 0, 0])
    assert dynamics.time == pytest.approx(0.0)


def test_asymmetric_motors_produce_rotation(dynamics):
    """Asymmetric motor commands should produce angular velocity (6-DOF coupling)."""
    state = QuadrotorState.hover_at(z=5.0)
    params = dynamics.params
    hover_omega = MotorModel.compute_hover_velocity(
        params.mass, params.gravity, params.motor.motor_constant, 4
    )
    dynamics.reset(state, np.full(4, hover_omega))
    # Run motors 0,2 faster (CCW pair), motors 1,3 slower
    cmds = np.array([hover_omega * 1.1, hover_omega * 0.9, hover_omega * 1.1, hover_omega * 0.9])
    for _ in range(100):
        s = dynamics.step(cmds, 0.004)
    # Angular velocity should be non-zero
    assert np.linalg.norm(s.angular_velocity) > 0.01
