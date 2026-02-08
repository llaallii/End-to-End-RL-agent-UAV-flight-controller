"""Cascaded PID flight controller for a quadrotor.

Implements the classic four-loop cascade:

    Position (P) -> Velocity (PI) -> Attitude (P) -> Rate (PID) -> Mixer

The outer loops run at a lower rate (50 Hz default) while the inner loops run
at a higher rate (250 Hz default).  This module manages all PID instances,
loads gains from ``pid_gains.yaml``, and coordinates the cascade.

Requirement refs: SIM013-SIM018
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

from simulation.controllers.mixer import QuadXMixer
from simulation.controllers.pid import PIDController, PIDGains
from simulation.core.quaternion_utils import angle_wrap, quaternion_to_euler
from simulation.core.types import QuadrotorParams, QuadrotorState

# ---------------------------------------------------------------------------
# Default config paths (relative to *this* file)
# ---------------------------------------------------------------------------
_CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
_DEFAULT_GAINS_PATH = _CONFIG_DIR / "pid_gains.yaml"
_DEFAULT_PARAMS_PATH = _CONFIG_DIR / "quadrotor_params.yaml"


class CascadedPIDController:
    """Full cascaded PID controller for position-controlled quadrotor flight.

    The controller is split into an *outer loop* (position + velocity) and an
    *inner loop* (attitude + rate + mixer).  They can run at different rates;
    call :meth:`update` every inner-loop tick and the outer loop will execute
    only when enough time has elapsed.

    Typical usage::

        params = QuadrotorParams.from_yaml("quadrotor_params.yaml")
        ctrl = CascadedPIDController.from_yaml(params)
        ctrl.set_position_setpoint(np.array([0.0, 0.0, 1.0]))

        for step in range(sim_steps):
            motor_cmds = ctrl.update(state, dt=0.004)
            state = dynamics.step(motor_cmds, dt=0.004)
    """

    # --------------------------------------------------------------------- #
    # Construction
    # --------------------------------------------------------------------- #

    def __init__(
        self,
        params: QuadrotorParams,
        gains: dict,
        position_hz: float = 50.0,
        attitude_hz: float = 250.0,
    ) -> None:
        """Initialise the cascaded controller.

        Args:
            params: Quadrotor physical parameters.
            gains: Parsed YAML gains dict (matches ``pid_gains.yaml`` schema).
            position_hz: Outer-loop execution rate in Hz.
            attitude_hz: Inner-loop execution rate in Hz.
        """
        self.params = params
        self.mixer = QuadXMixer(params)

        # Loop timing
        self._outer_dt = 1.0 / position_hz
        self._inner_dt = 1.0 / attitude_hz
        self._outer_accumulator: float = 0.0

        # Build PID instances from gains dict
        self._build_pids(gains)

        # Setpoints (public, can be set directly or via helpers)
        self.position_sp = np.zeros(3, dtype=np.float64)
        self.yaw_sp: float = 0.0

        # Inter-loop signals (stored so outer loop output persists between
        # inner-loop ticks)
        self._velocity_sp = np.zeros(3, dtype=np.float64)
        self._accel_sp = np.zeros(3, dtype=np.float64)
        self._attitude_sp = np.zeros(3, dtype=np.float64)  # [roll, pitch, yaw]
        self._rate_sp = np.zeros(3, dtype=np.float64)
        self._thrust_sp: float = 0.0

    @classmethod
    def from_yaml(
        cls,
        params: QuadrotorParams,
        gains_path: str | Path | None = None,
    ) -> CascadedPIDController:
        """Create controller from YAML gains file.

        Args:
            params: Quadrotor physical parameters.
            gains_path: Path to ``pid_gains.yaml``.  Uses the default shipped
                with the package if *None*.

        Returns:
            Fully initialised controller.
        """
        path = Path(gains_path) if gains_path is not None else _DEFAULT_GAINS_PATH
        with path.open() as f:
            gains = yaml.safe_load(f)

        return cls(
            params=params,
            gains=gains,
            position_hz=float(gains.get("control_rates", {}).get("position_hz", 50)),
            attitude_hz=float(gains.get("control_rates", {}).get("attitude_hz", 250)),
        )

    # --------------------------------------------------------------------- #
    # PID construction helpers
    # --------------------------------------------------------------------- #

    def _build_pids(self, gains: dict) -> None:
        """Instantiate all PID sub-controllers from the gains dictionary."""
        # -- Position loop (P-only) --
        pos = gains.get("position", {})
        self._pid_pos_x = PIDController(PIDGains.from_dict(pos.get("xy", {})))
        self._pid_pos_y = PIDController(PIDGains.from_dict(pos.get("xy", {})))
        self._pid_pos_z = PIDController(PIDGains.from_dict(pos.get("z", {})))

        # -- Velocity loop (PI) --
        vel = gains.get("velocity", {})
        self._pid_vel_x = PIDController(PIDGains.from_dict(vel.get("xy", {})))
        self._pid_vel_y = PIDController(PIDGains.from_dict(vel.get("xy", {})))
        self._pid_vel_z = PIDController(PIDGains.from_dict(vel.get("z", {})))

        # -- Attitude loop (P-only) --
        att = gains.get("attitude", {})
        self._pid_att_roll = PIDController(PIDGains.from_dict(att.get("roll", {})))
        self._pid_att_pitch = PIDController(PIDGains.from_dict(att.get("pitch", {})))
        self._pid_att_yaw = PIDController(PIDGains.from_dict(att.get("yaw", {})))

        # -- Rate loop (PID) --
        rate = gains.get("rate", {})
        self._pid_rate_roll = PIDController(PIDGains.from_dict(rate.get("roll", {})))
        self._pid_rate_pitch = PIDController(PIDGains.from_dict(rate.get("pitch", {})))
        self._pid_rate_yaw = PIDController(PIDGains.from_dict(rate.get("yaw", {})))

    # --------------------------------------------------------------------- #
    # Setpoint helpers
    # --------------------------------------------------------------------- #

    def set_position_setpoint(
        self,
        position: np.ndarray,
        yaw: float = 0.0,
    ) -> None:
        """Set the target position and yaw.

        Args:
            position: (3,) desired [x, y, z] in world frame (m).
            yaw: Desired yaw angle in radians.
        """
        self.position_sp = np.asarray(position, dtype=np.float64)
        self.yaw_sp = yaw

    # --------------------------------------------------------------------- #
    # Main update
    # --------------------------------------------------------------------- #

    def update(self, state: QuadrotorState, dt: float) -> np.ndarray:
        """Run one controller tick and return motor speed commands.

        The outer loop (position + velocity) runs at *position_hz*.  The inner
        loop (attitude + rate + mixer) runs every call.

        Args:
            state: Current quadrotor state.
            dt: Timestep since last call in seconds.

        Returns:
            (4,) motor speed commands in rad/s.
        """
        # Accumulate time for outer loop decimation
        self._outer_accumulator += dt

        if self._outer_accumulator >= self._outer_dt:
            outer_dt = self._outer_accumulator
            self._outer_accumulator = 0.0
            self._run_outer_loop(state, outer_dt)

        # Inner loop always runs
        return self._run_inner_loop(state, dt)

    # --------------------------------------------------------------------- #
    # Outer loop: position -> velocity -> desired attitude + thrust
    # --------------------------------------------------------------------- #

    def _run_outer_loop(self, state: QuadrotorState, dt: float) -> None:
        """Position and velocity loops producing attitude + thrust setpoints."""
        euler = quaternion_to_euler(state.quaternion)
        yaw = euler[2]

        # --- Position -> velocity setpoint ---
        pos_error = self.position_sp - state.position
        self._velocity_sp[0] = self._pid_pos_x.update(pos_error[0], dt)
        self._velocity_sp[1] = self._pid_pos_y.update(pos_error[1], dt)
        self._velocity_sp[2] = self._pid_pos_z.update(pos_error[2], dt)

        # --- Velocity -> acceleration setpoint (world frame) ---
        vel_error = self._velocity_sp - state.velocity
        self._accel_sp[0] = self._pid_vel_x.update(vel_error[0], dt)
        self._accel_sp[1] = self._pid_vel_y.update(vel_error[1], dt)
        self._accel_sp[2] = self._pid_vel_z.update(vel_error[2], dt)

        # --- Convert world-frame XY acceleration to roll/pitch angles ---
        # The desired acceleration in the horizontal plane must be rotated from
        # world frame into a frame aligned with the current yaw so that the
        # tilt angles map correctly to body roll and pitch.
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # Rotate world accel into yaw-aligned frame
        accel_forward = self._accel_sp[0] * cos_yaw + self._accel_sp[1] * sin_yaw
        accel_left = -self._accel_sp[0] * sin_yaw + self._accel_sp[1] * cos_yaw

        # Desired pitch and roll (small-angle linearisation):
        #   pitch = arctan(accel_forward / (g + az))  (nose-down for forward accel)
        #   roll  = arctan(-accel_left / (g + az))    (right-wing-down for left accel)
        g_plus_az = self.params.gravity + self._accel_sp[2]
        g_plus_az = max(g_plus_az, 0.1)  # guard against division by ~zero

        desired_pitch = math.atan2(accel_forward, g_plus_az)
        desired_roll = math.atan2(-accel_left, g_plus_az)

        self._attitude_sp[0] = desired_roll
        self._attitude_sp[1] = desired_pitch
        self._attitude_sp[2] = self.yaw_sp

        # Thrust magnitude: T = m * sqrt(ax^2 + ay^2 + (g + az)^2)
        self._thrust_sp = self.params.mass * math.sqrt(
            self._accel_sp[0] ** 2 + self._accel_sp[1] ** 2 + g_plus_az**2
        )

    # --------------------------------------------------------------------- #
    # Inner loop: attitude -> rate -> mixer -> motor commands
    # --------------------------------------------------------------------- #

    def _run_inner_loop(self, state: QuadrotorState, dt: float) -> np.ndarray:
        """Attitude and rate loops producing motor speed commands."""
        euler = quaternion_to_euler(state.quaternion)

        # --- Attitude -> rate setpoint ---
        att_error_roll = angle_wrap(self._attitude_sp[0] - euler[0])
        att_error_pitch = angle_wrap(self._attitude_sp[1] - euler[1])
        att_error_yaw = angle_wrap(self._attitude_sp[2] - euler[2])

        self._rate_sp[0] = self._pid_att_roll.update(att_error_roll, dt)
        self._rate_sp[1] = self._pid_att_pitch.update(att_error_pitch, dt)
        self._rate_sp[2] = self._pid_att_yaw.update(att_error_yaw, dt)

        # --- Rate -> torque demands ---
        omega = state.angular_velocity  # body frame [p, q, r]
        rate_error = self._rate_sp - omega

        torque_x = self._pid_rate_roll.update(rate_error[0], dt)
        torque_y = self._pid_rate_pitch.update(rate_error[1], dt)
        torque_z = self._pid_rate_yaw.update(rate_error[2], dt)

        torque_cmd = np.array([torque_x, torque_y, torque_z], dtype=np.float64)

        # --- Mixer -> motor speeds ---
        return self.mixer.mix(self._thrust_sp, torque_cmd)

    # --------------------------------------------------------------------- #
    # Reset
    # --------------------------------------------------------------------- #

    def reset(self) -> None:
        """Reset all PID states and internal setpoints."""
        for pid in self._all_pids():
            pid.reset()
        self._velocity_sp[:] = 0.0
        self._accel_sp[:] = 0.0
        self._attitude_sp[:] = 0.0
        self._rate_sp[:] = 0.0
        self._thrust_sp = self.params.mass * self.params.gravity
        self._outer_accumulator = 0.0

    def _all_pids(self) -> list[PIDController]:
        """Return a flat list of every PID instance (convenience)."""
        return [
            self._pid_pos_x,
            self._pid_pos_y,
            self._pid_pos_z,
            self._pid_vel_x,
            self._pid_vel_y,
            self._pid_vel_z,
            self._pid_att_roll,
            self._pid_att_pitch,
            self._pid_att_yaw,
            self._pid_rate_roll,
            self._pid_rate_pitch,
            self._pid_rate_yaw,
        ]

    # --------------------------------------------------------------------- #
    # Diagnostics
    # --------------------------------------------------------------------- #

    def get_debug_info(self) -> dict[str, float | np.ndarray]:
        """Return a snapshot of internal signals for logging / telemetry.

        Returns:
            Dictionary with keys for setpoints, errors, and PID integrator
            values.
        """
        return {
            "velocity_sp": self._velocity_sp.copy(),
            "accel_sp": self._accel_sp.copy(),
            "attitude_sp": self._attitude_sp.copy(),
            "rate_sp": self._rate_sp.copy(),
            "thrust_sp": self._thrust_sp,
            "integral_vel_x": self._pid_vel_x.integral,
            "integral_vel_y": self._pid_vel_y.integral,
            "integral_vel_z": self._pid_vel_z.integral,
            "integral_rate_roll": self._pid_rate_roll.integral,
            "integral_rate_pitch": self._pid_rate_pitch.integral,
            "integral_rate_yaw": self._pid_rate_yaw.integral,
        }
