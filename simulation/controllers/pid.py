"""Generic single-axis PID controller with anti-windup and derivative filtering.

Provides a reusable PID building block for the cascaded flight controller.
Each axis (e.g., roll rate, x-velocity) gets its own PID instance so that
integrator state is tracked independently.

Requirement refs: SIM013-SIM018
"""

from __future__ import annotations

import dataclasses


@dataclasses.dataclass
class PIDGains:
    """Gains and limits for a single PID axis.

    Attributes:
        kp: Proportional gain.
        ki: Integral gain.
        kd: Derivative gain.
        integral_limit: Maximum absolute value of the integrator state.
            Set to 0 to disable integral clamping.
        output_limit: Maximum absolute value of the PID output.
            Set to 0 to disable output clamping.
        d_filter_coeff: First-order low-pass coefficient for the derivative
            term.  alpha=1 means no filtering; lower values increase smoothing.
            Typical range: 0.1 -- 1.0.
    """

    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    integral_limit: float = 0.0
    output_limit: float = 0.0
    d_filter_coeff: float = 1.0

    @classmethod
    def from_dict(cls, data: dict[str, float]) -> PIDGains:
        """Create gains from a dictionary (e.g. loaded from YAML).

        Missing keys are filled with defaults so the same dict format works for
        P-only, PI, and full PID configurations.
        """
        return cls(
            kp=float(data.get("kp", 0.0)),
            ki=float(data.get("ki", 0.0)),
            kd=float(data.get("kd", 0.0)),
            integral_limit=float(data.get("integral_limit", 0.0)),
            output_limit=float(data.get("output_limit", 0.0)),
            d_filter_coeff=float(data.get("d_filter_coeff", 1.0)),
        )


class PIDController:
    """Discrete-time PID controller for a single axis.

    Features:
        - Anti-windup via integrator clamping (rolls back accumulation when
          output saturates).
        - First-order low-pass filter on the derivative term to reduce
          high-frequency noise.
        - Configurable output saturation.

    Typical usage::

        pid = PIDController(PIDGains(kp=1.0, ki=0.1, kd=0.01, output_limit=5.0))
        for dt in timesteps:
            output = pid.update(setpoint - measurement, dt)
    """

    def __init__(self, gains: PIDGains) -> None:
        self.gains = gains
        self._integral: float = 0.0
        self._prev_error: float | None = None
        self._prev_derivative: float = 0.0

    # -- public interface ----------------------------------------------------

    def update(self, error: float, dt: float) -> float:
        """Compute PID output for the current timestep.

        Args:
            error: Signed error (setpoint - measurement).  Positive error means
                the measurement is below the setpoint.
            dt: Timestep in seconds.  Must be > 0.

        Returns:
            Clamped controller output.
        """
        if dt <= 0.0:
            return 0.0

        # -- Proportional --
        p_term = self.gains.kp * error

        # -- Integral (trapezoidal accumulation) --
        self._integral += error * dt
        if self.gains.integral_limit > 0.0:
            self._integral = _clamp(self._integral, self.gains.integral_limit)
        i_term = self.gains.ki * self._integral

        # -- Derivative (backward difference + low-pass filter) --
        raw_derivative = 0.0 if self._prev_error is None else (error - self._prev_error) / dt

        alpha = self.gains.d_filter_coeff
        filtered_derivative = alpha * raw_derivative + (1.0 - alpha) * self._prev_derivative
        self._prev_derivative = filtered_derivative
        self._prev_error = error
        d_term = self.gains.kd * filtered_derivative

        # -- Sum and clamp output --
        output = p_term + i_term + d_term
        if self.gains.output_limit > 0.0:
            clamped = _clamp(output, self.gains.output_limit)
            # Anti-windup: if output was saturated, roll back integral
            if clamped != output and self.gains.ki != 0.0:
                self._integral -= error * dt
            output = clamped

        return output

    def reset(self) -> None:
        """Zero all internal state (integrator, previous error, derivative)."""
        self._integral = 0.0
        self._prev_error = None
        self._prev_derivative = 0.0

    @property
    def integral(self) -> float:
        """Current integrator accumulation (read-only)."""
        return self._integral

    def set_integral(self, value: float) -> None:
        """Manually set the integrator value.

        Useful for bumpless transfer when switching controllers.
        """
        self._integral = value


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _clamp(value: float, limit: float) -> float:
    """Symmetric clamp: return value clamped to [-limit, +limit]."""
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value
