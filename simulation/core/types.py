"""Dataclasses for quadrotor simulation state and parameters."""

from __future__ import annotations

import dataclasses
from pathlib import Path

import numpy as np
import yaml  # type: ignore[import-untyped]


@dataclasses.dataclass
class MotorParams:
    """Motor and propeller parameters."""

    motor_constant: float  # kF: N/(rad/s)^2
    moment_constant: float  # kM: dimensionless (torque = kM * thrust)
    max_rot_velocity: float  # rad/s
    time_constant_up: float  # s (spin-up time constant)
    time_constant_down: float  # s (spin-down time constant)
    num_motors: int = 4
    positions: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros((4, 3)))
    directions: np.ndarray = dataclasses.field(
        default_factory=lambda: np.array([1.0, -1.0, 1.0, -1.0])
    )  # +1 CCW, -1 CW
    rotor_drag_coefficient: float = 8.06e-5
    rolling_moment_coefficient: float = 1.0e-6


@dataclasses.dataclass
class AeroParams:
    """Aerodynamic drag parameters."""

    translational_drag_coeff: float  # N*s^2/m^2
    rotational_drag_coeff: float  # N*m*s^2/rad^2


@dataclasses.dataclass
class QuadrotorParams:
    """Complete quadrotor physical parameters."""

    mass: float  # kg
    inertia: np.ndarray  # (3, 3) inertia tensor
    gravity: float  # m/s^2
    arm_length: float  # m
    motor: MotorParams
    aero: AeroParams

    @classmethod
    def from_yaml(cls, path: str | Path) -> QuadrotorParams:
        """Load parameters from YAML config file."""
        path = Path(path)
        with path.open() as f:
            data = yaml.safe_load(f)

        q = data["quadrotor"]

        # Build inertia tensor
        inertia_data = q["inertia"]
        inertia = np.array(
            [
                [inertia_data["ixx"], inertia_data["ixy"], inertia_data["ixz"]],
                [inertia_data["ixy"], inertia_data["iyy"], inertia_data["iyz"]],
                [inertia_data["ixz"], inertia_data["iyz"], inertia_data["izz"]],
            ]
        )

        # Build motor positions and directions
        motor_cfg = q["motors"]
        positions = np.array([[m["x"], m["y"], m["z"]] for m in motor_cfg["positions"]])
        directions = np.array(
            [1.0 if m["direction"] == "ccw" else -1.0 for m in motor_cfg["positions"]]
        )

        motor = MotorParams(
            motor_constant=motor_cfg["motor_constant"],
            moment_constant=motor_cfg["moment_constant"],
            max_rot_velocity=motor_cfg["max_rot_velocity"],
            time_constant_up=motor_cfg["time_constant_up"],
            time_constant_down=motor_cfg["time_constant_down"],
            num_motors=motor_cfg["count"],
            positions=positions,
            directions=directions,
            rotor_drag_coefficient=motor_cfg["rotor_drag_coefficient"],
            rolling_moment_coefficient=motor_cfg["rolling_moment_coefficient"],
        )

        aero_cfg = q["aerodynamics"]
        aero = AeroParams(
            translational_drag_coeff=aero_cfg["translational_drag_coeff"],
            rotational_drag_coeff=aero_cfg["rotational_drag_coeff"],
        )

        return cls(
            mass=q["mass"],
            inertia=inertia,
            gravity=q["gravity"],
            arm_length=q["geometry"]["arm_length"],
            motor=motor,
            aero=aero,
        )

    @classmethod
    def default(cls) -> QuadrotorParams:
        """Create default parameters matching quadrotor_params.yaml."""
        d = 0.1237  # arm_length / sqrt(2)
        positions = np.array(
            [
                [d, -d, 0.02],  # M0: front-right, CCW
                [d, d, 0.02],  # M1: front-left, CW
                [-d, d, 0.02],  # M2: rear-left, CCW
                [-d, -d, 0.02],  # M3: rear-right, CW
            ]
        )
        directions = np.array([1.0, -1.0, 1.0, -1.0])

        motor = MotorParams(
            motor_constant=8.55e-6,
            moment_constant=0.016,
            max_rot_velocity=838.0,
            time_constant_up=0.0625,
            time_constant_down=0.025,
            num_motors=4,
            positions=positions,
            directions=directions,
        )
        aero = AeroParams(
            translational_drag_coeff=0.1,
            rotational_drag_coeff=0.01,
        )
        inertia = np.diag([0.0049, 0.0049, 0.0069])

        return cls(
            mass=1.0,
            inertia=inertia,
            gravity=9.81,
            arm_length=0.175,
            motor=motor,
            aero=aero,
        )


@dataclasses.dataclass
class QuadrotorState:
    """Full quadrotor state using quaternion orientation.

    Conventions:
        - World frame: ENU (East-North-Up)
        - Body frame: FLU (Forward-Left-Up)
        - Quaternion: [w, x, y, z] (scalar-first, Hamilton convention)
    """

    position: np.ndarray  # (3,) [x, y, z] world frame, m
    velocity: np.ndarray  # (3,) [vx, vy, vz] world frame, m/s
    quaternion: np.ndarray  # (4,) [w, x, y, z] body-to-world rotation
    angular_velocity: np.ndarray  # (3,) [p, q, r] body frame, rad/s

    @classmethod
    def hover_at(
        cls,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 1.0,
        yaw: float = 0.0,
    ) -> QuadrotorState:
        """Create a hover state at given position with optional yaw."""
        from simulation.core.quaternion_utils import euler_to_quaternion

        return cls(
            position=np.array([x, y, z], dtype=np.float64),
            velocity=np.zeros(3, dtype=np.float64),
            quaternion=euler_to_quaternion(0.0, 0.0, yaw),
            angular_velocity=np.zeros(3, dtype=np.float64),
        )

    @classmethod
    def zeros(cls) -> QuadrotorState:
        """Create a state at origin with identity orientation."""
        return cls(
            position=np.zeros(3, dtype=np.float64),
            velocity=np.zeros(3, dtype=np.float64),
            quaternion=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            angular_velocity=np.zeros(3, dtype=np.float64),
        )

    def to_array(self) -> np.ndarray:
        """Flatten state to a 13-element array."""
        return np.concatenate(
            [self.position, self.velocity, self.quaternion, self.angular_velocity]
        )

    @classmethod
    def from_array(cls, arr: np.ndarray) -> QuadrotorState:
        """Reconstruct state from a 13-element array."""
        return cls(
            position=arr[0:3].copy(),
            velocity=arr[3:6].copy(),
            quaternion=arr[6:10].copy(),
            angular_velocity=arr[10:13].copy(),
        )

    def copy(self) -> QuadrotorState:
        """Return a deep copy of this state."""
        return QuadrotorState(
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            quaternion=self.quaternion.copy(),
            angular_velocity=self.angular_velocity.copy(),
        )
