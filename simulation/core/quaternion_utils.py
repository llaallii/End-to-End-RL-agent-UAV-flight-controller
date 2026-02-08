"""Quaternion math utilities for rigid body dynamics.

Convention: q = [w, x, y, z] where w is the scalar part (Hamilton convention).
Body frame: FLU (Forward-Left-Up). World frame: ENU (East-North-Up).
"""

from __future__ import annotations

import math

import numpy as np


def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product of two quaternions.

    Args:
        q1: First quaternion [w, x, y, z].
        q2: Second quaternion [w, x, y, z].

    Returns:
        Product quaternion [w, x, y, z].
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    """Return conjugate (inverse for unit quaternions).

    Args:
        q: Quaternion [w, x, y, z].

    Returns:
        Conjugate quaternion [w, -x, -y, -z].
    """
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion to unit length.

    Args:
        q: Quaternion [w, x, y, z].

    Returns:
        Unit quaternion.
    """
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    result: np.ndarray = q / norm
    return result


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Convert unit quaternion to 3x3 rotation matrix (body to world).

    Args:
        q: Unit quaternion [w, x, y, z].

    Returns:
        3x3 rotation matrix R such that v_world = R @ v_body.
    """
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rotation_matrix_to_quaternion(r: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to unit quaternion.

    Uses Shepperd's method for numerical stability.

    Args:
        r: 3x3 rotation matrix.

    Returns:
        Unit quaternion [w, x, y, z].
    """
    trace = r[0, 0] + r[1, 1] + r[2, 2]

    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r[2, 1] - r[1, 2]) * s
        y = (r[0, 2] - r[2, 0]) * s
        z = (r[1, 0] - r[0, 1]) * s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s

    return normalize_quaternion(np.array([w, x, y, z], dtype=np.float64))


def quaternion_to_euler(q: np.ndarray) -> np.ndarray:
    """Convert quaternion to Euler angles [roll, pitch, yaw] in radians.

    Uses ZYX (yaw-pitch-roll) convention.

    Args:
        q: Quaternion [w, x, y, z].

    Returns:
        Euler angles [roll, pitch, yaw] in radians.
    """
    w, x, y, z = q

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation) -- clamp for numerical stability
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw], dtype=np.float64)


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert Euler angles to quaternion.

    Uses ZYX (yaw-pitch-roll) convention.

    Args:
        roll: Roll angle in radians.
        pitch: Pitch angle in radians.
        yaw: Yaw angle in radians.

    Returns:
        Quaternion [w, x, y, z].
    """
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([w, x, y, z], dtype=np.float64)


def rotate_vector_by_quaternion(v: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Rotate vector v from body frame to world frame using quaternion q.

    Computes q * [0, v] * q_conj.

    Args:
        v: 3D vector in body frame.
        q: Unit quaternion [w, x, y, z] (body-to-world).

    Returns:
        Rotated 3D vector in world frame.
    """
    v_quat = np.array([0.0, v[0], v[1], v[2]], dtype=np.float64)
    q_conj = quaternion_conjugate(q)
    rotated = quaternion_multiply(quaternion_multiply(q, v_quat), q_conj)
    return rotated[1:4]


def quaternion_derivative(q: np.ndarray, omega: np.ndarray) -> np.ndarray:
    """Compute dq/dt given angular velocity omega in body frame.

    dq/dt = 0.5 * q * [0, omega_x, omega_y, omega_z]

    Args:
        q: Current quaternion [w, x, y, z].
        omega: Angular velocity [p, q, r] in body frame, rad/s.

    Returns:
        Quaternion derivative [dw, dx, dy, dz].
    """
    omega_quat = np.array([0.0, omega[0], omega[1], omega[2]], dtype=np.float64)
    return 0.5 * quaternion_multiply(q, omega_quat)


def angle_wrap(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi
