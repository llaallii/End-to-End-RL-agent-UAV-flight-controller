"""Cascaded PID flight controller and motor mixer for quadrotors.

This package provides the complete control pipeline:

    Position (P) -> Velocity (PI) -> Attitude (P) -> Rate (PID) -> Mixer

Typical usage::

    from simulation.controllers import CascadedPIDController, QuadXMixer
    from simulation.core.types import QuadrotorParams

    params = QuadrotorParams.from_yaml("quadrotor_params.yaml")
    controller = CascadedPIDController.from_yaml(params)
"""

from simulation.controllers.cascaded_pid_controller import CascadedPIDController
from simulation.controllers.mixer import QuadXMixer
from simulation.controllers.pid import PIDController, PIDGains

__all__ = [
    "CascadedPIDController",
    "PIDController",
    "PIDGains",
    "QuadXMixer",
]
