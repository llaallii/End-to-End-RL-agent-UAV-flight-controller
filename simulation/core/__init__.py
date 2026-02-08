"""Core quadrotor dynamics simulation library.

Standalone 6-DOF rigid body dynamics, motor model, and aerodynamics.
No ROS or Gazebo dependencies -- suitable for Gymnasium environments.
"""

from simulation.core.aerodynamics import AerodynamicsModel
from simulation.core.motor_model import MotorModel
from simulation.core.quadrotor_dynamics import QuadrotorDynamics
from simulation.core.types import AeroParams, MotorParams, QuadrotorParams, QuadrotorState

__all__ = [
    "AeroParams",
    "AerodynamicsModel",
    "MotorModel",
    "MotorParams",
    "QuadrotorDynamics",
    "QuadrotorParams",
    "QuadrotorState",
]
