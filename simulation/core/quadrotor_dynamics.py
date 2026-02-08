"""Full 6-DOF quadrotor rigid body dynamics.

Implements Newton-Euler equations with RK4 integration.
Requirement refs: SIM003 (timestep <=0.004s), SIM004 (stable 10+ min), SIM005 (6-DOF)
"""

from __future__ import annotations

import numpy as np

from simulation.core.aerodynamics import AerodynamicsModel
from simulation.core.motor_model import MotorModel
from simulation.core.quaternion_utils import (
    normalize_quaternion,
    quaternion_derivative,
    rotate_vector_by_quaternion,
)
from simulation.core.types import QuadrotorParams, QuadrotorState


class QuadrotorDynamics:
    """Full 6-DOF quadrotor simulation.

    State vector (13 elements):
        position:         [x, y, z]           world frame ENU
        velocity:         [vx, vy, vz]        world frame
        quaternion:       [qw, qx, qy, qz]   body-to-world
        angular_velocity: [p, q, r]           body frame

    Equations of motion:
        Translation: m * dv/dt = F_gravity + R @ F_thrust + F_drag
        Rotation:    I * d(omega)/dt = tau - omega x (I @ omega)
        Quaternion:  dq/dt = 0.5 * q * [0, omega]
    """

    def __init__(self, params: QuadrotorParams) -> None:
        self.params = params
        self.state = QuadrotorState.zeros()
        self.motors = MotorModel(params.motor)
        self.aero = AerodynamicsModel(params.aero)
        self.time: float = 0.0

        # Precompute inverse inertia
        self._inertia_inv = np.linalg.inv(params.inertia)

    def reset(
        self,
        state: QuadrotorState | None = None,
        motor_velocities: np.ndarray | None = None,
    ) -> QuadrotorState:
        """Reset simulation to given initial conditions.

        Args:
            state: Initial state. If None, resets to origin with identity orientation.
            motor_velocities: Initial motor velocities. If None, resets to zero.

        Returns:
            The initial state.
        """
        if state is not None:
            self.state = state.copy()
        else:
            self.state = QuadrotorState.zeros()

        self.motors.reset(motor_velocities)
        self.time = 0.0
        return self.state.copy()

    def step(self, motor_commands: np.ndarray, dt: float) -> QuadrotorState:
        """Advance simulation by one timestep.

        Args:
            motor_commands: (4,) commanded rotor velocities [rad/s].
            dt: Timestep [s], should be <= 0.004 per SIM003.

        Returns:
            Updated state after integration.
        """
        # 1. Motor model: commands -> actual thrusts & reaction torques
        thrusts, reaction_torques = self.motors.update(motor_commands, dt)

        # 2. Compute forces (world) and torques (body) from motor thrusts
        force_world, torque_body = self._compute_forces_torques(thrusts, reaction_torques)

        # 3. Integrate state using RK4
        self.state = self._rk4_step(force_world, torque_body, dt)

        # 4. Normalize quaternion to prevent numerical drift
        self.state.quaternion = normalize_quaternion(self.state.quaternion)

        # 5. Ground collision: clamp position and zero velocity if below ground
        if self.state.position[2] <= 0.0:
            self.state.position[2] = 0.0
            if self.state.velocity[2] < 0.0:
                self.state.velocity[2] = 0.0

        self.time += dt
        return self.state.copy()

    def _compute_forces_torques(
        self,
        thrusts: np.ndarray,
        reaction_torques: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Compute total forces (world frame) and torques (body frame).

        Forces (world frame):
          - Gravity: [0, 0, -m*g]
          - Motor thrust: R @ [0, 0, sum(thrusts)]  (thrust along body +Z)
          - Aerodynamic drag: translational_drag(velocity)

        Torques (body frame):
          - Motor torques: sum(r_i x [0, 0, F_i]) for each motor
          - Motor reaction torques: sum([0, 0, tau_i]) about Z
          - Aerodynamic rotational drag
        """
        p = self.params

        # --- Forces in world frame ---
        gravity_world = np.array([0.0, 0.0, -p.mass * p.gravity])

        # Total thrust in body frame is along +Z
        total_thrust_body = np.array([0.0, 0.0, np.sum(thrusts)])
        thrust_world = rotate_vector_by_quaternion(total_thrust_body, self.state.quaternion)

        drag_world = self.aero.translational_drag(self.state.velocity)

        force_world = gravity_world + thrust_world + drag_world

        # --- Torques in body frame ---
        torque_body = np.zeros(3, dtype=np.float64)

        # Motor moment arms: r_i x [0, 0, F_i]
        for i in range(p.motor.num_motors):
            r_i = p.motor.positions[i]
            f_i = np.array([0.0, 0.0, thrusts[i]])
            torque_body += np.cross(r_i, f_i)

        # Reaction torques about Z axis
        torque_body[2] += np.sum(reaction_torques)

        # Rotational drag
        torque_body += self.aero.rotational_drag(self.state.angular_velocity)

        return force_world, torque_body

    def _state_derivative(
        self,
        state: QuadrotorState,
        force_world: np.ndarray,
        torque_body: np.ndarray,
    ) -> np.ndarray:
        """Compute state derivative for RK4 integration.

        Returns 13-element derivative array:
            [d_position, d_velocity, d_quaternion, d_angular_velocity]

        Translation:
            d(position)/dt = velocity
            d(velocity)/dt = force_world / mass

        Rotation:
            d(quaternion)/dt = 0.5 * q * [0, omega]
            d(omega)/dt = I^-1 * (torque - omega x (I * omega))
        """
        # Position derivative = velocity
        d_position = state.velocity

        # Velocity derivative = force / mass
        d_velocity = force_world / self.params.mass

        # Quaternion derivative
        d_quaternion = quaternion_derivative(state.quaternion, state.angular_velocity)

        # Angular velocity derivative (Euler's equation for rigid bodies)
        omega = state.angular_velocity
        i_omega = self.params.inertia @ omega
        gyroscopic = np.cross(omega, i_omega)
        d_omega = self._inertia_inv @ (torque_body - gyroscopic)

        return np.concatenate([d_position, d_velocity, d_quaternion, d_omega])

    def _rk4_step(
        self,
        force_world: np.ndarray,
        torque_body: np.ndarray,
        dt: float,
    ) -> QuadrotorState:
        """4th-order Runge-Kutta integration step.

        Note: Forces and torques are held constant during the RK4 substeps.
        This is valid when dt is small relative to the dynamics timescales.
        """
        y = self.state.to_array()

        k1 = self._state_derivative(QuadrotorState.from_array(y), force_world, torque_body)
        k2 = self._state_derivative(
            QuadrotorState.from_array(y + 0.5 * dt * k1), force_world, torque_body
        )
        k3 = self._state_derivative(
            QuadrotorState.from_array(y + 0.5 * dt * k2), force_world, torque_body
        )
        k4 = self._state_derivative(
            QuadrotorState.from_array(y + dt * k3), force_world, torque_body
        )

        y_new = y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        return QuadrotorState.from_array(y_new)
