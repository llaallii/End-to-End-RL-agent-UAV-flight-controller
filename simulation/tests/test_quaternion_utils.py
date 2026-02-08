"""Unit tests for simulation.core.quaternion_utils."""

from __future__ import annotations

import math

import numpy as np
import pytest
from numpy.testing import assert_allclose

from simulation.core.quaternion_utils import (
    angle_wrap,
    euler_to_quaternion,
    normalize_quaternion,
    quaternion_conjugate,
    quaternion_derivative,
    quaternion_multiply,
    quaternion_to_euler,
    quaternion_to_rotation_matrix,
    rotate_vector_by_quaternion,
    rotation_matrix_to_quaternion,
)

# ---------------------------------------------------------------------------
# Rotation matrix
# ---------------------------------------------------------------------------


def test_identity_quaternion_to_rotation_matrix():
    q = np.array([1, 0, 0, 0], dtype=np.float64)
    r = quaternion_to_rotation_matrix(q)
    assert_allclose(r, np.eye(3), atol=1e-12)


# ---------------------------------------------------------------------------
# Multiply
# ---------------------------------------------------------------------------


def test_multiply_identity_left():
    identity = np.array([1, 0, 0, 0], dtype=np.float64)
    q = np.array([0.5, 0.5, 0.5, 0.5], dtype=np.float64)
    assert_allclose(quaternion_multiply(identity, q), q, atol=1e-12)


def test_multiply_identity_right():
    identity = np.array([1, 0, 0, 0], dtype=np.float64)
    q = np.array([0.5, 0.5, 0.5, 0.5], dtype=np.float64)
    assert_allclose(quaternion_multiply(q, identity), q, atol=1e-12)


def test_multiply_q_times_conjugate_is_identity():
    q = normalize_quaternion(np.array([1, 2, 3, 4], dtype=np.float64))
    result = quaternion_multiply(q, quaternion_conjugate(q))
    assert_allclose(result, [1, 0, 0, 0], atol=1e-12)


# ---------------------------------------------------------------------------
# Normalize
# ---------------------------------------------------------------------------


def test_normalize_near_zero():
    q = np.array([1e-20, 0, 0, 0], dtype=np.float64)
    result = normalize_quaternion(q)
    assert_allclose(result, [1, 0, 0, 0])


def test_normalize_preserves_unit():
    q = normalize_quaternion(np.array([1, 1, 1, 1], dtype=np.float64))
    assert_allclose(np.linalg.norm(q), 1.0, atol=1e-12)


# ---------------------------------------------------------------------------
# Euler roundtrips
# ---------------------------------------------------------------------------


def test_euler_zero_to_identity():
    q = euler_to_quaternion(0.0, 0.0, 0.0)
    assert_allclose(q, [1, 0, 0, 0], atol=1e-12)


@pytest.mark.parametrize(
    "roll, pitch, yaw",
    [
        (0.1, 0.2, 0.3),
        (0.0, 0.0, math.pi / 2),
        (-0.3, 0.0, 1.0),
        (0.0, 0.4, -0.5),
    ],
)
def test_euler_roundtrip(roll, pitch, yaw):
    q = euler_to_quaternion(roll, pitch, yaw)
    euler = quaternion_to_euler(q)
    assert_allclose(euler, [roll, pitch, yaw], atol=1e-10)


# ---------------------------------------------------------------------------
# Vector rotation
# ---------------------------------------------------------------------------


def test_rotate_identity():
    v = np.array([3.0, -2.0, 1.0])
    q = np.array([1, 0, 0, 0], dtype=np.float64)
    assert_allclose(rotate_vector_by_quaternion(v, q), v, atol=1e-12)


def test_rotate_90_about_z():
    """Rotate [1,0,0] by 90° about Z should give [0,1,0]."""
    q = euler_to_quaternion(0.0, 0.0, math.pi / 2)
    v = np.array([1.0, 0.0, 0.0])
    result = rotate_vector_by_quaternion(v, q)
    assert_allclose(result, [0, 1, 0], atol=1e-10)


def test_rotate_z_by_identity():
    v = np.array([0.0, 0.0, 1.0])
    q = np.array([1, 0, 0, 0], dtype=np.float64)
    assert_allclose(rotate_vector_by_quaternion(v, q), [0, 0, 1], atol=1e-12)


# ---------------------------------------------------------------------------
# Quaternion derivative
# ---------------------------------------------------------------------------


def test_derivative_zero_omega():
    q = np.array([1, 0, 0, 0], dtype=np.float64)
    omega = np.zeros(3)
    dq = quaternion_derivative(q, omega)
    assert_allclose(dq, [0, 0, 0, 0], atol=1e-12)


# ---------------------------------------------------------------------------
# Rotation matrix roundtrip
# ---------------------------------------------------------------------------


def test_rotation_matrix_roundtrip():
    q_orig = euler_to_quaternion(0.3, -0.2, 1.1)
    r = quaternion_to_rotation_matrix(q_orig)
    q_back = rotation_matrix_to_quaternion(r)
    # Quaternions q and -q represent the same rotation
    if q_orig[0] * q_back[0] < 0:
        q_back = -q_back
    assert_allclose(q_back, q_orig, atol=1e-10)


# ---------------------------------------------------------------------------
# angle_wrap
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "input_angle, expected",
    [
        (0.0, 0.0),
        (math.pi, -math.pi),  # pi wraps to -pi with modular arithmetic
        (2 * math.pi, 0.0),
        (-math.pi, -math.pi),
        (3 * math.pi, -math.pi),  # 3*pi wraps to -pi
    ],
)
def test_angle_wrap(input_angle, expected):
    result = angle_wrap(input_angle)
    assert result == pytest.approx(expected, abs=1e-10)
