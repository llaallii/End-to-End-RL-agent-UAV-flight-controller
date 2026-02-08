"""Unit tests for simulation.core.types."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from numpy.testing import assert_allclose

from simulation.core.types import QuadrotorParams, QuadrotorState

# ---------------------------------------------------------------------------
# QuadrotorState
# ---------------------------------------------------------------------------


def test_zeros_identity_quaternion():
    s = QuadrotorState.zeros()
    assert_allclose(s.quaternion, [1, 0, 0, 0])
    assert_allclose(s.position, [0, 0, 0])
    assert_allclose(s.velocity, [0, 0, 0])
    assert_allclose(s.angular_velocity, [0, 0, 0])


def test_hover_at_position():
    s = QuadrotorState.hover_at(x=1.0, y=2.0, z=3.0)
    assert_allclose(s.position, [1, 2, 3])
    assert_allclose(s.velocity, [0, 0, 0])
    assert_allclose(s.angular_velocity, [0, 0, 0])


def test_hover_at_yaw():
    s = QuadrotorState.hover_at(yaw=0.5)
    assert s.quaternion[0] != 1.0  # non-identity when yaw nonzero
    assert_allclose(np.linalg.norm(s.quaternion), 1.0)


def test_to_array_length():
    s = QuadrotorState.zeros()
    arr = s.to_array()
    assert arr.shape == (13,)


def test_from_array_roundtrip():
    s = QuadrotorState.hover_at(x=1.0, y=-2.0, z=3.0, yaw=0.7)
    arr = s.to_array()
    s2 = QuadrotorState.from_array(arr)
    assert_allclose(s2.position, s.position)
    assert_allclose(s2.velocity, s.velocity)
    assert_allclose(s2.quaternion, s.quaternion)
    assert_allclose(s2.angular_velocity, s.angular_velocity)


def test_copy_independence():
    s = QuadrotorState.hover_at(z=5.0)
    s2 = s.copy()
    s2.position[2] = 99.0
    assert s.position[2] == pytest.approx(5.0)  # original unchanged


# ---------------------------------------------------------------------------
# QuadrotorParams
# ---------------------------------------------------------------------------


def test_default_mass_gravity():
    p = QuadrotorParams.default()
    assert p.mass == pytest.approx(1.0)
    assert p.gravity == pytest.approx(9.81)
    assert p.arm_length == pytest.approx(0.175)


def test_default_motor_params():
    p = QuadrotorParams.default()
    assert p.motor.num_motors == 4
    assert p.motor.motor_constant == pytest.approx(8.55e-6)
    assert p.motor.max_rot_velocity == pytest.approx(838.0)
    assert p.motor.positions.shape == (4, 3)


def test_default_inertia():
    p = QuadrotorParams.default()
    assert p.inertia.shape == (3, 3)
    assert_allclose(np.diag(p.inertia), [0.0049, 0.0049, 0.0069])
    # Off-diagonal should be zero
    assert_allclose(p.inertia - np.diag(np.diag(p.inertia)), 0.0)


def test_from_yaml_matches_default():
    yaml_path = Path(__file__).resolve().parent.parent / "config" / "quadrotor_params.yaml"
    if not yaml_path.exists():
        pytest.skip("quadrotor_params.yaml not found")
    p_yaml = QuadrotorParams.from_yaml(yaml_path)
    p_default = QuadrotorParams.default()
    assert p_yaml.mass == pytest.approx(p_default.mass)
    assert p_yaml.gravity == pytest.approx(p_default.gravity)
    assert p_yaml.motor.motor_constant == pytest.approx(p_default.motor.motor_constant)
    assert p_yaml.motor.max_rot_velocity == pytest.approx(p_default.motor.max_rot_velocity)
    assert_allclose(np.diag(p_yaml.inertia), np.diag(p_default.inertia))
