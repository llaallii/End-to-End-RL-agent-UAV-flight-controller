"""Unit tests for simulation.controllers.mixer."""

from __future__ import annotations

import math

import numpy as np
import pytest
from numpy.testing import assert_allclose

from simulation.controllers.mixer import QuadXMixer
from simulation.core.types import QuadrotorParams


@pytest.fixture()
def mixer():
    return QuadXMixer(QuadrotorParams.default())


@pytest.fixture()
def params():
    return QuadrotorParams.default()


def test_hover_thrust_equal_speeds(mixer, params):
    """Hover thrust with zero torque should produce 4 equal motor speeds."""
    total_thrust = params.mass * params.gravity  # 9.81 N
    speeds = mixer.mix(total_thrust, np.zeros(3))
    # All 4 should be approximately equal
    assert_allclose(speeds, speeds[0], atol=0.1)
    # Expected hover speed: sqrt(9.81/4 / 8.55e-6) ≈ 535.6
    expected = math.sqrt(total_thrust / 4 / params.motor.motor_constant)
    assert_allclose(speeds, expected, atol=1.0)


def test_zero_thrust_zero_speeds(mixer):
    speeds = mixer.mix(0.0, np.zeros(3))
    assert_allclose(speeds, 0.0, atol=1e-10)


def test_roll_torque_differential(mixer, params):
    """Positive roll torque should create differential between left and right motors."""
    total_thrust = params.mass * params.gravity
    torque = np.array([0.01, 0.0, 0.0])  # positive roll
    speeds = mixer.mix(total_thrust, torque)
    # Motors are: 0=FR(-y), 1=FL(+y), 2=RL(+y), 3=RR(-y)
    # Positive roll torque increases thrust on +y side (needs more from -y motors)
    # Right motors (0, 3) at -y should differ from left motors (1, 2) at +y
    right_avg = (speeds[0] + speeds[3]) / 2
    left_avg = (speeds[1] + speeds[2]) / 2
    assert right_avg != pytest.approx(left_avg, abs=0.1)


def test_inverse_mix_roundtrip(mixer, params):
    """mix → compute forces → inverse_mix should recover original demands."""
    total_thrust = params.mass * params.gravity
    torque_cmd = np.array([0.005, -0.003, 0.001])
    speeds = mixer.mix(total_thrust, torque_cmd)
    # Compute forces: F = kF * omega^2
    forces = params.motor.motor_constant * speeds**2
    recovered_thrust, recovered_torque = mixer.inverse_mix(forces)
    assert recovered_thrust == pytest.approx(total_thrust, rel=0.05)
    assert_allclose(recovered_torque, torque_cmd, atol=0.002)


def test_hover_speeds(mixer, params):
    speeds = mixer.hover_speeds(params.mass, params.gravity)
    expected = math.sqrt(params.mass * params.gravity / 4 / params.motor.motor_constant)
    assert_allclose(speeds, expected, atol=1.0)


def test_hover_motor_forces(mixer, params):
    forces = mixer.hover_motor_forces(params.mass, params.gravity)
    expected = params.mass * params.gravity / 4
    assert_allclose(forces, expected, atol=0.001)


def test_allocation_matrix_invertible(mixer):
    a = mixer.allocation_matrix
    assert a.shape == (4, 4)
    assert abs(np.linalg.det(a)) > 1e-10


def test_max_clamp(mixer, params):
    """Excessive thrust should clamp speeds to max."""
    speeds = mixer.mix(100.0, np.zeros(3))
    assert np.all(speeds <= params.motor.max_rot_velocity + 1e-6)


def test_effectiveness_times_allocation_is_identity(mixer):
    a = mixer.effectiveness_matrix
    b = mixer.allocation_matrix
    assert_allclose(a @ b, np.eye(4), atol=1e-10)
