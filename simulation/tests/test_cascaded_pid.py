"""Unit tests for simulation.controllers.cascaded_pid_controller.

Includes closed-loop simulation tests verifying hover stability (SIM013-SIM017).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from simulation.controllers.cascaded_pid_controller import CascadedPIDController
from simulation.core.motor_model import MotorModel
from simulation.core.quadrotor_dynamics import QuadrotorDynamics
from simulation.core.quaternion_utils import quaternion_to_euler
from simulation.core.types import QuadrotorParams, QuadrotorState

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _run_closed_loop(
    params: QuadrotorParams,
    controller: CascadedPIDController,
    initial_state: QuadrotorState,
    duration_s: float,
    dt: float = 0.004,
) -> list[QuadrotorState]:
    """Run closed-loop dynamics + controller, return state history."""
    dynamics = QuadrotorDynamics(params)
    hover_omega = MotorModel.compute_hover_velocity(
        params.mass, params.gravity, params.motor.motor_constant, 4
    )
    dynamics.reset(initial_state, np.full(4, hover_omega))

    states = [initial_state.copy()]
    n_steps = int(duration_s / dt)
    for _ in range(n_steps):
        motor_cmds = controller.update(dynamics.state, dt)
        state = dynamics.step(motor_cmds, dt)
        states.append(state)
    return states


# ---------------------------------------------------------------------------
# Basic tests
# ---------------------------------------------------------------------------


def test_from_yaml():
    """from_yaml should load without error and have 12 PID instances."""
    params = QuadrotorParams.default()
    ctrl = CascadedPIDController.from_yaml(params)
    assert len(ctrl._all_pids()) == 12


def test_reset():
    params = QuadrotorParams.default()
    ctrl = CascadedPIDController.from_yaml(params)
    state = QuadrotorState.hover_at(z=1.0)
    # Run a few updates to populate state
    for _ in range(10):
        ctrl.update(state, 0.004)
    ctrl.reset()
    info = ctrl.get_debug_info()
    # After reset, thrust_sp should be approximately mass * gravity
    assert info["thrust_sp"] == pytest.approx(params.mass * params.gravity, rel=0.01)


# ---------------------------------------------------------------------------
# Closed-loop hover stability (SIM013, SIM014, SIM015)
# ---------------------------------------------------------------------------


@pytest.mark.slow
def test_hover_stability():
    """SIM013-SIM015: Hover for 60s at z=1.0, check position and attitude bounds.

    Start at z=0.2, setpoint z=1.0, yaw=0.
    After 10s transient:
      - Position error < 0.5m per axis (SIM014)
      - |roll|, |pitch| < 5 degrees (SIM015)
      - z > 0.5 at all times (SIM013: still flying)
    At t=60s: z within 0.2m of setpoint.
    """
    params = QuadrotorParams.default()
    ctrl = CascadedPIDController.from_yaml(params)
    ctrl.set_position_setpoint(np.array([0.0, 0.0, 1.0]), yaw=0.0)
    ctrl.reset()

    initial = QuadrotorState.hover_at(z=0.2)
    dt = 0.004
    duration = 60.0
    states = _run_closed_loop(params, ctrl, initial, duration, dt)

    transient_steps = int(10.0 / dt)  # skip first 10s
    deg5 = math.radians(5.0)

    for i in range(transient_steps, len(states)):
        s = states[i]
        # Position bounds (SIM014)
        assert abs(s.position[0]) < 0.5, f"X position out of bounds at step {i}"
        assert abs(s.position[1]) < 0.5, f"Y position out of bounds at step {i}"
        assert abs(s.position[2] - 1.0) < 0.5, f"Z position out of bounds at step {i}"
        # Attitude bounds (SIM015)
        euler = quaternion_to_euler(s.quaternion)
        assert abs(euler[0]) < deg5, f"Roll {math.degrees(euler[0]):.1f}° exceeds ±5° at step {i}"
        assert abs(euler[1]) < deg5, f"Pitch {math.degrees(euler[1]):.1f}° exceeds ±5° at step {i}"
        # Still flying (SIM013)
        assert s.position[2] > 0.5, f"Altitude dropped below 0.5m at step {i}"

    # Final position should be close to setpoint
    final = states[-1]
    assert abs(final.position[2] - 1.0) < 0.2, f"Final altitude {final.position[2]:.3f} not at 1.0m"


# ---------------------------------------------------------------------------
# Altitude step response (SIM016, SIM017)
# ---------------------------------------------------------------------------


@pytest.mark.slow
def test_altitude_step_response():
    """SIM016/SIM017: Step from z=1.0 to z=2.0, check settling and overshoot.

    1. Hover at z=1.0 for 10s to reach steady state.
    2. Change setpoint to z=2.0.
    3. Check: reaches z=1.8 within 5s (settling).
    4. Check: never exceeds z=2.6 (30% overshoot of 1.0m step).
    """
    params = QuadrotorParams.default()
    ctrl = CascadedPIDController.from_yaml(params)
    ctrl.set_position_setpoint(np.array([0.0, 0.0, 1.0]), yaw=0.0)
    ctrl.reset()

    dynamics = QuadrotorDynamics(params)
    hover_omega = MotorModel.compute_hover_velocity(
        params.mass, params.gravity, params.motor.motor_constant, 4
    )
    initial = QuadrotorState.hover_at(z=0.5)
    dynamics.reset(initial, np.full(4, hover_omega))

    dt = 0.004

    # Phase 1: hover at z=1.0 for 10s
    for _ in range(int(10.0 / dt)):
        cmds = ctrl.update(dynamics.state, dt)
        dynamics.step(cmds, dt)

    # Phase 2: step to z=2.0
    ctrl.set_position_setpoint(np.array([0.0, 0.0, 2.0]), yaw=0.0)
    reached_target = False
    max_z = 0.0
    settle_steps = int(5.0 / dt)

    for step in range(int(10.0 / dt)):
        cmds = ctrl.update(dynamics.state, dt)
        s = dynamics.step(cmds, dt)
        max_z = max(max_z, s.position[2])
        if step <= settle_steps and s.position[2] >= 1.8:
            reached_target = True

    assert reached_target, "Did not reach z=1.8 within 5 seconds"
    assert max_z < 2.6, f"Overshoot too large: max_z={max_z:.3f} (limit 2.6)"


# ---------------------------------------------------------------------------
# Motor command sanity
# ---------------------------------------------------------------------------


def test_motor_commands_positive():
    """During hover, all motor commands should be positive."""
    params = QuadrotorParams.default()
    ctrl = CascadedPIDController.from_yaml(params)
    ctrl.set_position_setpoint(np.array([0.0, 0.0, 1.0]), yaw=0.0)
    ctrl.reset()

    dynamics = QuadrotorDynamics(params)
    hover_omega = MotorModel.compute_hover_velocity(
        params.mass, params.gravity, params.motor.motor_constant, 4
    )
    dynamics.reset(QuadrotorState.hover_at(z=1.0), np.full(4, hover_omega))

    for _ in range(250):  # 1 second
        cmds = ctrl.update(dynamics.state, 0.004)
        assert np.all(cmds >= 0), f"Negative motor command: {cmds}"
        dynamics.step(cmds, 0.004)


def test_motor_commands_bounded():
    """During hover, all motor commands should not exceed max."""
    params = QuadrotorParams.default()
    ctrl = CascadedPIDController.from_yaml(params)
    ctrl.set_position_setpoint(np.array([0.0, 0.0, 1.0]), yaw=0.0)
    ctrl.reset()

    dynamics = QuadrotorDynamics(params)
    hover_omega = MotorModel.compute_hover_velocity(
        params.mass, params.gravity, params.motor.motor_constant, 4
    )
    dynamics.reset(QuadrotorState.hover_at(z=1.0), np.full(4, hover_omega))

    for _ in range(250):  # 1 second
        cmds = ctrl.update(dynamics.state, 0.004)
        assert np.all(cmds <= params.motor.max_rot_velocity + 1e-6), f"Motor cmd exceeds max: {cmds}"
        dynamics.step(cmds, 0.004)
