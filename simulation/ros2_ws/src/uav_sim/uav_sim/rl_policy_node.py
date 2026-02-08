"""ROS 2 node that runs a trained RL policy in Gazebo SITL.

Replaces the PID controller node — subscribes to the same state topics,
runs the SAC policy to compute motor commands, and publishes to Gazebo.

Topics (use same namespace as PID node for bridge compatibility)
------
Subscriptions:
    ~/odometry      (nav_msgs/Odometry)       - pose + twist from Gazebo
    ~/imu           (sensor_msgs/Imu)         - angular velocity (body-frame)

Publishers:
    ~/command/motor_speed  (actuator_msgs/Actuators) - 4 motor velocities (rad/s)

Parameters:
    model_path     (str)   - path to trained SB3 model (.zip)
    control_rate   (float, default 50.0) - policy inference Hz (must match training)
    goal_x/y/z    (float)  - hover goal position

Requirement refs: RL003, RL012
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import rclpy
from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu

# ---------------------------------------------------------------------------
# Ensure the project root is on sys.path so that simulation/training imports work.
# ---------------------------------------------------------------------------
_THIS_DIR = Path(__file__).resolve().parent
_PROJECT_ROOT = _THIS_DIR.parents[4]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from stable_baselines3 import SAC  # noqa: E402

from simulation.core.quaternion_utils import quaternion_to_euler  # noqa: E402


class RLPolicyNode(Node):
    """ROS 2 node running a trained RL policy at a fixed rate."""

    def __init__(self) -> None:
        super().__init__("rl_policy_node")

        # ------------------------------------------------------------- #
        # Parameters
        # ------------------------------------------------------------- #
        self.declare_parameter("model_path", "")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 1.0)
        self.declare_parameter("max_rot_velocity", 838.0)
        self.declare_parameter("position_scale", 5.0)
        self.declare_parameter("velocity_scale", 5.0)
        self.declare_parameter("angular_velocity_scale", 10.0)

        model_path = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        control_rate = (
            self.get_parameter("control_rate").get_parameter_value().double_value
        )

        self._goal = np.array([
            self.get_parameter("goal_x").get_parameter_value().double_value,
            self.get_parameter("goal_y").get_parameter_value().double_value,
            self.get_parameter("goal_z").get_parameter_value().double_value,
        ])
        self._max_rot_velocity = (
            self.get_parameter("max_rot_velocity").get_parameter_value().double_value
        )
        self._pos_scale = (
            self.get_parameter("position_scale").get_parameter_value().double_value
        )
        self._vel_scale = (
            self.get_parameter("velocity_scale").get_parameter_value().double_value
        )
        self._ang_vel_scale = (
            self.get_parameter("angular_velocity_scale").get_parameter_value().double_value
        )

        # ------------------------------------------------------------- #
        # Load trained policy
        # ------------------------------------------------------------- #
        if not model_path:
            self.get_logger().error("No model_path provided! Set via --ros-args -p model_path:=...")
            raise ValueError("model_path parameter is required")

        self.get_logger().info(f"Loading RL model from {model_path}")
        self._model = SAC.load(model_path)
        self.get_logger().info(
            f"Model loaded — goal [{self._goal[0]:.1f}, {self._goal[1]:.1f}, {self._goal[2]:.1f}], "
            f"control rate {control_rate} Hz"
        )

        # ------------------------------------------------------------- #
        # State cache
        # ------------------------------------------------------------- #
        self._last_odom: Odometry | None = None
        self._last_imu: Imu | None = None
        self._prev_action = np.zeros(4, dtype=np.float64)
        self._step_count = 0

        # ------------------------------------------------------------- #
        # Subscriptions
        # ------------------------------------------------------------- #
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        self._sub_odom = self.create_subscription(
            Odometry, "~/odometry", self._odom_callback, sensor_qos,
        )
        self._sub_imu = self.create_subscription(
            Imu, "~/imu", self._imu_callback, sensor_qos,
        )

        # ------------------------------------------------------------- #
        # Publisher
        # ------------------------------------------------------------- #
        self._pub_motors = self.create_publisher(
            Actuators, "~/command/motor_speed", 10,
        )

        # ------------------------------------------------------------- #
        # Control timer
        # ------------------------------------------------------------- #
        timer_period = 1.0 / control_rate
        self._timer = self.create_timer(timer_period, self._control_loop)

        self.get_logger().info("RL policy node started")

    # ----------------------------------------------------------------- #
    # Subscriber callbacks
    # ----------------------------------------------------------------- #

    def _odom_callback(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _imu_callback(self, msg: Imu) -> None:
        self._last_imu = msg

    # ----------------------------------------------------------------- #
    # Control loop
    # ----------------------------------------------------------------- #

    def _control_loop(self) -> None:
        """Timer callback: build obs from state, run policy, publish motors."""
        odom = self._last_odom
        if odom is None:
            return

        # -- Extract state from ROS messages ----------------------------- #
        position = np.array([
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
        ], dtype=np.float64)

        velocity = np.array([
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
        ], dtype=np.float64)

        # Quaternion: ROS (x,y,z,w) -> our convention (w,x,y,z)
        q = odom.pose.pose.orientation
        quaternion = np.array([q.w, q.x, q.y, q.z], dtype=np.float64)

        imu = self._last_imu
        if imu is not None:
            angular_velocity = np.array([
                imu.angular_velocity.x,
                imu.angular_velocity.y,
                imu.angular_velocity.z,
            ], dtype=np.float64)
        else:
            angular_velocity = np.array([
                odom.twist.twist.angular.x,
                odom.twist.twist.angular.y,
                odom.twist.twist.angular.z,
            ], dtype=np.float64)

        # -- Build 15-dim observation (must match training env) ---------- #
        euler = quaternion_to_euler(quaternion)
        pos_error = self._goal - position

        obs = np.concatenate([
            pos_error / self._pos_scale,
            velocity / self._vel_scale,
            euler / np.pi,
            angular_velocity / self._ang_vel_scale,
            self._goal / self._pos_scale,
        ])
        obs = np.clip(obs, -1.0, 1.0).astype(np.float32)

        # -- Run policy -------------------------------------------------- #
        action, _ = self._model.predict(obs, deterministic=True)

        # -- Map action [-1, 1] -> motor speeds [0, max_rot_velocity] --- #
        action_clipped = np.clip(action, -1.0, 1.0).astype(np.float64)
        motor_speeds = (action_clipped + 1.0) / 2.0 * self._max_rot_velocity

        # -- Publish ----------------------------------------------------- #
        msg = Actuators()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.velocity = motor_speeds.tolist()
        self._pub_motors.publish(msg)

        # -- Log periodically -------------------------------------------- #
        self._step_count += 1
        if self._step_count % 250 == 0:  # every ~5 seconds at 50Hz
            pos_err = float(np.linalg.norm(pos_error))
            self.get_logger().info(
                f"Step {self._step_count}: pos={position}, "
                f"pos_err={pos_err:.3f}m, motors={motor_speeds}"
            )

        self._prev_action = action_clipped


def main(args: list[str] | None = None) -> None:
    """Entry point for ``ros2 run uav_sim rl_policy_node``."""
    rclpy.init(args=args)
    node = RLPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
