"""ROS 2 node wrapping the CascadedPIDController for Gazebo SITL.

Subscribes to odometry and IMU from the Gazebo bridge, runs the cascaded
PID control law, and publishes motor speed commands back to Gazebo.

Topics
------
Subscriptions:
    ~/odometry      (nav_msgs/Odometry)       - pose + twist from Gazebo odometry plugin
    ~/imu           (sensor_msgs/Imu)         - angular velocity + linear acceleration
    ~/setpoint      (geometry_msgs/PoseStamped) - target position (optional yaw)

Publishers:
    ~/command/motor_speed  (actuator_msgs/Actuators) - 4 motor velocities (rad/s)

Parameters:
    hover_height   (float, default 1.0)  - initial Z setpoint in metres
    control_rate   (float, default 250.0) - timer frequency in Hz

Requirement refs: SIM013-SIM018
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import rclpy
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu

# ---------------------------------------------------------------------------
# Ensure the project root is on sys.path so that ``simulation.*`` imports
# work regardless of whether the outer project is pip-installed.
# ---------------------------------------------------------------------------
_THIS_DIR = Path(__file__).resolve().parent
_PROJECT_ROOT = _THIS_DIR.parents[4]  # .../uav_sim/uav_sim -> 5 levels up to project root
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from simulation.controllers.cascaded_pid_controller import CascadedPIDController  # noqa: E402
from simulation.core.types import QuadrotorParams, QuadrotorState  # noqa: E402


class PIDControllerNode(Node):
    """ROS 2 lifecycle-free node running the cascaded PID at a fixed rate."""

    def __init__(self) -> None:
        super().__init__("pid_controller_node")

        # ------------------------------------------------------------- #
        # Parameters
        # ------------------------------------------------------------- #
        self.declare_parameter("hover_height", 1.0)
        self.declare_parameter("control_rate", 250.0)

        hover_height: float = (
            self.get_parameter("hover_height").get_parameter_value().double_value
        )
        control_rate: float = (
            self.get_parameter("control_rate").get_parameter_value().double_value
        )

        # ------------------------------------------------------------- #
        # Controller
        # ------------------------------------------------------------- #
        params = QuadrotorParams.default()
        self._controller = CascadedPIDController.from_yaml(params)
        self._controller.set_position_setpoint(
            np.array([0.0, 0.0, hover_height]),
            yaw=0.0,
        )
        self._controller.reset()

        self.get_logger().info(
            f"Controller initialised — hover setpoint [0, 0, {hover_height}] m, "
            f"control rate {control_rate} Hz"
        )

        # ------------------------------------------------------------- #
        # State cache (populated by subscribers)
        # ------------------------------------------------------------- #
        self._last_odom: Odometry | None = None
        self._last_imu: Imu | None = None

        # ------------------------------------------------------------- #
        # Subscriptions
        # ------------------------------------------------------------- #
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        self._sub_odom = self.create_subscription(
            Odometry,
            "~/odometry",
            self._odom_callback,
            sensor_qos,
        )
        self._sub_imu = self.create_subscription(
            Imu,
            "~/imu",
            self._imu_callback,
            sensor_qos,
        )
        self._sub_setpoint = self.create_subscription(
            PoseStamped,
            "~/setpoint",
            self._setpoint_callback,
            10,
        )

        # ------------------------------------------------------------- #
        # Publisher
        # ------------------------------------------------------------- #
        self._pub_motors = self.create_publisher(
            Actuators,
            "~/command/motor_speed",
            10,
        )

        # ------------------------------------------------------------- #
        # Control-loop timer
        # ------------------------------------------------------------- #
        timer_period = 1.0 / control_rate
        self._timer = self.create_timer(timer_period, self._control_loop)

        self.get_logger().info("PID controller node started")

    # ----------------------------------------------------------------- #
    # Subscriber callbacks
    # ----------------------------------------------------------------- #

    def _odom_callback(self, msg: Odometry) -> None:
        """Cache the latest odometry message."""
        self._last_odom = msg

    def _imu_callback(self, msg: Imu) -> None:
        """Cache the latest IMU message."""
        self._last_imu = msg

    def _setpoint_callback(self, msg: PoseStamped) -> None:
        """Update the position setpoint from an external planner / CLI.

        Extracts position (x, y, z) and yaw from the incoming PoseStamped.
        Yaw is derived from the quaternion orientation in the message.
        """
        pos = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=np.float64,
        )

        # Extract yaw from the quaternion (ROS order: x, y, z, w)
        q = msg.pose.orientation
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = float(np.arctan2(siny_cosp, cosy_cosp))

        self._controller.set_position_setpoint(pos, yaw=yaw)
        self.get_logger().info(
            f"Setpoint updated: pos=[{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}], "
            f"yaw={np.degrees(yaw):.1f} deg"
        )

    # ----------------------------------------------------------------- #
    # Control loop
    # ----------------------------------------------------------------- #

    def _control_loop(self) -> None:
        """Timer callback: build state from cached messages, run PID, publish."""
        odom = self._last_odom
        if odom is None:
            return  # no data yet — skip silently

        # -- Build QuadrotorState from ROS messages -------------------- #
        position = np.array(
            [
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z,
            ],
            dtype=np.float64,
        )
        velocity = np.array(
            [
                odom.twist.twist.linear.x,
                odom.twist.twist.linear.y,
                odom.twist.twist.linear.z,
            ],
            dtype=np.float64,
        )

        # Quaternion: ROS uses (x, y, z, w), our QuadrotorState uses (w, x, y, z)
        q = odom.pose.pose.orientation
        quaternion = np.array([q.w, q.x, q.y, q.z], dtype=np.float64)

        # Angular velocity: prefer IMU (body-frame) if available,
        # otherwise fall back to odometry twist (which may be world-frame).
        imu = self._last_imu
        if imu is not None:
            angular_velocity = np.array(
                [
                    imu.angular_velocity.x,
                    imu.angular_velocity.y,
                    imu.angular_velocity.z,
                ],
                dtype=np.float64,
            )
        else:
            angular_velocity = np.array(
                [
                    odom.twist.twist.angular.x,
                    odom.twist.twist.angular.y,
                    odom.twist.twist.angular.z,
                ],
                dtype=np.float64,
            )

        state = QuadrotorState(
            position=position,
            velocity=velocity,
            quaternion=quaternion,
            angular_velocity=angular_velocity,
        )

        # -- Run controller -------------------------------------------- #
        # dt matches the physics timestep (0.004 s @ 250 Hz)
        dt = 1.0 / 250.0
        motor_speeds = self._controller.update(state, dt)

        # -- Publish motor commands ------------------------------------ #
        msg = Actuators()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.velocity = motor_speeds.tolist()
        self._pub_motors.publish(msg)


def main(args: list[str] | None = None) -> None:
    """Entry point for ``ros2 run uav_sim pid_controller_node``."""
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
