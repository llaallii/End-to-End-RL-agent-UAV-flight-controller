"""Unit tests for simulation.core.aerodynamics."""

from __future__ import annotations

import numpy as np
from numpy.testing import assert_allclose

from simulation.core.aerodynamics import AerodynamicsModel
from simulation.core.types import AeroParams


def _make_model():
    return AerodynamicsModel(AeroParams(translational_drag_coeff=0.1, rotational_drag_coeff=0.01))


def test_zero_velocity_zero_drag():
    model = _make_model()
    drag = model.translational_drag(np.zeros(3))
    assert_allclose(drag, [0, 0, 0], atol=1e-15)


def test_zero_angular_velocity_zero_rotational_drag():
    model = _make_model()
    drag = model.rotational_drag(np.zeros(3))
    assert_allclose(drag, [0, 0, 0], atol=1e-15)


def test_drag_opposes_motion():
    model = _make_model()
    v = np.array([1.0, 0.0, 0.0])
    drag = model.translational_drag(v)
    assert drag[0] < 0, "Drag should oppose positive X velocity"


def test_rotational_drag_opposes_rotation():
    model = _make_model()
    omega = np.array([0.0, 0.0, 1.0])
    drag = model.rotational_drag(omega)
    assert drag[2] < 0, "Rotational drag should oppose positive Z rotation"


def test_drag_quadratic_scaling():
    """Drag at 2v should be 4x drag at v (quadratic)."""
    model = _make_model()
    v = np.array([2.0, 0.0, 0.0])
    drag_v = model.translational_drag(v)
    drag_2v = model.translational_drag(2.0 * v)
    assert_allclose(np.abs(drag_2v), 4.0 * np.abs(drag_v), rtol=1e-10)


def test_rotational_drag_quadratic_scaling():
    model = _make_model()
    omega = np.array([0.0, 3.0, 0.0])
    drag_1 = model.rotational_drag(omega)
    drag_2 = model.rotational_drag(2.0 * omega)
    assert_allclose(np.abs(drag_2), 4.0 * np.abs(drag_1), rtol=1e-10)


def test_drag_antiparallel_to_velocity():
    """Drag vector should be anti-parallel to velocity vector."""
    model = _make_model()
    v = np.array([1.0, 2.0, 3.0])
    drag = model.translational_drag(v)
    # Dot product should be negative (anti-parallel)
    assert np.dot(v, drag) < 0
    # Cross product should be ~zero (parallel/anti-parallel)
    cross = np.cross(v, drag)
    assert_allclose(cross, [0, 0, 0], atol=1e-10)
