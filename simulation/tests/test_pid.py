"""Unit tests for simulation.controllers.pid."""

from __future__ import annotations

import pytest

from simulation.controllers.pid import PIDController, PIDGains


def test_p_only():
    pid = PIDController(PIDGains(kp=1.0))
    output = pid.update(2.0, 0.01)
    assert output == pytest.approx(2.0)


def test_pi_output_grows():
    pid = PIDController(PIDGains(kp=1.0, ki=0.5))
    outputs = []
    for _ in range(50):
        outputs.append(pid.update(1.0, 0.01))
    # Output should grow due to integral accumulation
    assert outputs[-1] > outputs[0]


def test_integral_clamp():
    pid = PIDController(PIDGains(kp=0.0, ki=1.0, integral_limit=5.0))
    for _ in range(1000):
        output = pid.update(10.0, 0.01)
    # Integral clamped to 5.0, so output = ki * integral = 1.0 * 5.0 = 5.0
    assert output == pytest.approx(5.0)
    assert pid.integral == pytest.approx(5.0)


def test_output_clamp():
    pid = PIDController(PIDGains(kp=100.0, output_limit=10.0))
    output = pid.update(1.0, 0.01)
    assert output == pytest.approx(10.0)


def test_anti_windup():
    """After saturating with large positive error, switching to negative error
    should cause output to go negative promptly (not delayed by wound-up integral)."""
    pid = PIDController(PIDGains(kp=1.0, ki=1.0, output_limit=2.0))
    # Saturate with large positive error
    for _ in range(100):
        pid.update(10.0, 0.01)
    # Switch to negative error
    went_negative = False
    for _i in range(20):
        output = pid.update(-10.0, 0.01)
        if output < 0:
            went_negative = True
            break
    assert went_negative, "Anti-windup should allow fast recovery to negative output"


def test_derivative_term():
    pid = PIDController(PIDGains(kp=0.0, kd=1.0))
    # First call: no previous error, derivative is 0
    pid.update(0.0, 0.01)
    # Step change in error: d(error)/dt = 1.0 / 0.01 = 100
    output = pid.update(1.0, 0.01)
    assert output == pytest.approx(100.0)


def test_reset():
    pid = PIDController(PIDGains(kp=1.0, ki=1.0))
    for _ in range(10):
        pid.update(1.0, 0.01)
    pid.reset()
    assert pid.integral == pytest.approx(0.0)
    # After reset, should behave like a fresh controller
    fresh = PIDController(PIDGains(kp=1.0, ki=1.0))
    assert pid.update(1.0, 0.01) == pytest.approx(fresh.update(1.0, 0.01))


def test_from_dict():
    gains = PIDGains.from_dict({"kp": 1.5, "ki": 0.3})
    assert gains.kp == pytest.approx(1.5)
    assert gains.ki == pytest.approx(0.3)
    assert gains.kd == pytest.approx(0.0)
    assert gains.integral_limit == pytest.approx(0.0)


def test_zero_dt():
    pid = PIDController(PIDGains(kp=1.0))
    output = pid.update(5.0, 0.0)
    assert output == pytest.approx(0.0)


def test_set_integral():
    pid = PIDController(PIDGains(kp=1.0, ki=1.0))
    pid.set_integral(5.0)
    assert pid.integral == pytest.approx(5.0)
