"""Unit tests for simulation.core.motor_model."""

from __future__ import annotations

import math

import numpy as np
import pytest
from numpy.testing import assert_allclose

from simulation.core.motor_model import MotorModel
from simulation.core.types import QuadrotorParams


@pytest.fixture()
def motor_model():
    params = QuadrotorParams.default()
    return MotorModel(params.motor)


@pytest.fixture()
def motor_params():
    return QuadrotorParams.default().motor


def test_reset_to_zero(motor_model):
    motor_model.reset()
    assert_allclose(motor_model.velocities, 0.0)
    assert_allclose(motor_model.thrusts, 0.0)


def test_reset_to_values(motor_model):
    init_vel = np.array([100.0, 200.0, 300.0, 400.0])
    motor_model.reset(init_vel)
    assert_allclose(motor_model.velocities, init_vel)


def test_zero_command_zero_thrust(motor_model):
    motor_model.reset()
    commands = np.zeros(4)
    thrusts, _torques = motor_model.update(commands, 0.004)
    assert_allclose(thrusts, 0.0, atol=1e-12)


def test_steady_state_thrust(motor_model, motor_params):
    """After many steps at constant command, thrust should equal kF * omega^2."""
    motor_model.reset()
    cmd = np.full(4, 500.0)
    for _ in range(1000):
        motor_model.update(cmd, 0.004)
    expected_thrust = motor_params.motor_constant * 500.0**2
    assert_allclose(motor_model.thrusts, expected_thrust, rtol=0.01)
    assert_allclose(motor_model.velocities, 500.0, rtol=0.01)


def test_time_constant_sim006(motor_params):
    """SIM006: Motor time constant must be 50-100 ms. Configured at 62.5 ms."""
    assert 0.050 <= motor_params.time_constant_up <= 0.100
    # Verify by simulation: step from 0 to target, measure ~63.2% rise time
    model = MotorModel(motor_params)
    model.reset()
    target = 500.0
    dt = 0.001
    for step in range(200):
        model.update(np.full(4, target), dt)
        if model.velocities[0] >= target * 0.632:
            rise_time = (step + 1) * dt
            assert 0.040 <= rise_time <= 0.120
            return
    pytest.fail("Motor never reached 63.2% of target")


def test_spindown_faster_than_spinup(motor_params):
    """Spin-down (tau=25ms) should be faster than spin-up (tau=62.5ms)."""
    target = 500.0
    dt = 0.001

    # Spin up
    model = MotorModel(motor_params)
    model.reset()
    spinup_steps = 0
    for _ in range(500):
        model.update(np.full(4, target), dt)
        spinup_steps += 1
        if model.velocities[0] >= target * 0.632:
            break

    # Spin down
    model.reset(np.full(4, target))
    spindown_steps = 0
    for _ in range(500):
        model.update(np.zeros(4), dt)
        spindown_steps += 1
        if model.velocities[0] <= target * 0.368:
            break

    assert spindown_steps < spinup_steps


def test_clamp_at_max(motor_model, motor_params):
    """Commanding above max should be clamped."""
    motor_model.reset()
    cmd = np.full(4, 2000.0)  # way above 838
    for _ in range(1000):
        motor_model.update(cmd, 0.004)
    assert motor_model.velocities[0] <= motor_params.max_rot_velocity + 1e-6


def test_compute_hover_velocity(motor_params):
    expected = math.sqrt(1.0 * 9.81 / (4 * 8.55e-6))
    actual = MotorModel.compute_hover_velocity(1.0, 9.81, motor_params.motor_constant, 4)
    assert actual == pytest.approx(expected, rel=1e-6)


def test_reaction_torque_directions(motor_model, motor_params):
    """CCW motors (+1 direction) produce negative Z reaction torque (opposes spin)."""
    motor_model.reset()
    cmd = np.full(4, 500.0)
    for _ in range(1000):
        _thrusts, torques = motor_model.update(cmd, 0.004)
    for i in range(4):
        if motor_params.directions[i] > 0:
            assert torques[i] < 0, f"Motor {i} (CCW) reaction should be negative (CW)"
        else:
            assert torques[i] > 0, f"Motor {i} (CW) reaction should be positive (CCW)"
