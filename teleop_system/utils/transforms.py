"""Coordinate frame transformations and quaternion utilities.

All coordinate conversions between SteamVR, ROS2, and URDF frames are
centralized here. No other module should perform frame transformations directly.

Coordinate conventions:
    - ROS2 / URDF: X-forward, Y-left, Z-up, quaternion (x, y, z, w)
    - SteamVR (OpenVR): X-right, Y-up, Z-backward, quaternion (w, x, y, z)
    - Units: meters, radians, seconds
"""

import numpy as np


# --- Pure numpy quaternion / rotation helpers (no transforms3d dependency) ---

def _qmult_wxyz(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product of two wxyz quaternions."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def _quat2mat_wxyz(q: np.ndarray) -> np.ndarray:
    """Convert wxyz quaternion to 3x3 rotation matrix."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def _mat2quat_wxyz(M: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to wxyz quaternion (Shepperd's method)."""
    tr = np.trace(M)
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (M[2, 1] - M[1, 2]) * s
        y = (M[0, 2] - M[2, 0]) * s
        z = (M[1, 0] - M[0, 1]) * s
    elif M[0, 0] > M[1, 1] and M[0, 0] > M[2, 2]:
        s = 2.0 * np.sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2])
        w = (M[2, 1] - M[1, 2]) / s
        x = 0.25 * s
        y = (M[0, 1] + M[1, 0]) / s
        z = (M[0, 2] + M[2, 0]) / s
    elif M[1, 1] > M[2, 2]:
        s = 2.0 * np.sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2])
        w = (M[0, 2] - M[2, 0]) / s
        x = (M[0, 1] + M[1, 0]) / s
        y = 0.25 * s
        z = (M[1, 2] + M[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1])
        w = (M[1, 0] - M[0, 1]) / s
        x = (M[0, 2] + M[2, 0]) / s
        y = (M[1, 2] + M[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    if w < 0:
        q = -q
    return q


def _euler_to_mat_sxyz(ai: float, aj: float, ak: float) -> np.ndarray:
    """Euler angles (sxyz convention: roll, pitch, yaw) to 3x3 rotation matrix.

    R = Rz(ak) @ Ry(aj) @ Rx(ai)  (extrinsic X-Y-Z = intrinsic z-y-x).
    """
    ci, si = np.cos(ai), np.sin(ai)
    cj, sj = np.cos(aj), np.sin(aj)
    ck, sk = np.cos(ak), np.sin(ak)
    return np.array([
        [cj*ck,              -cj*sk,               sj     ],
        [ci*sk + si*sj*ck,    ci*ck - si*sj*sk,   -si*cj  ],
        [si*sk - ci*sj*ck,    si*ck + ci*sj*sk,    ci*cj  ],
    ])


def _mat_to_euler_sxyz(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix to Euler angles (sxyz convention)."""
    sy = R[0, 2]
    sy = np.clip(sy, -1.0, 1.0)
    aj = np.arcsin(sy)
    if np.abs(sy) < 1.0 - 1e-8:
        ai = np.arctan2(-R[1, 2], R[2, 2])
        ak = np.arctan2(-R[0, 1], R[0, 0])
    else:
        ai = np.arctan2(R[2, 1], R[1, 1])
        ak = 0.0
    return np.array([ai, aj, ak])


# --- Quaternion Convention Conversions ---

def quat_wxyz_to_xyzw(q_wxyz: np.ndarray) -> np.ndarray:
    """Convert quaternion from (w, x, y, z) to (x, y, z, w).

    Args:
        q_wxyz: (4,) quaternion in wxyz order (OpenVR convention).

    Returns:
        (4,) quaternion in xyzw order (ROS2 convention).
    """
    return np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]])


def quat_xyzw_to_wxyz(q_xyzw: np.ndarray) -> np.ndarray:
    """Convert quaternion from (x, y, z, w) to (w, x, y, z).

    Args:
        q_xyzw: (4,) quaternion in xyzw order (ROS2 convention).

    Returns:
        (4,) quaternion in wxyz order (OpenVR convention).
    """
    return np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])


# --- Quaternion Operations ---

def quat_multiply(q1_xyzw: np.ndarray, q2_xyzw: np.ndarray) -> np.ndarray:
    """Multiply two quaternions (Hamilton product), both in xyzw convention.

    Args:
        q1_xyzw: (4,) first quaternion in xyzw.
        q2_xyzw: (4,) second quaternion in xyzw.

    Returns:
        (4,) product quaternion in xyzw.
    """
    q1w = quat_xyzw_to_wxyz(q1_xyzw)
    q2w = quat_xyzw_to_wxyz(q2_xyzw)
    result_wxyz = _qmult_wxyz(q1w, q2w)
    return quat_wxyz_to_xyzw(result_wxyz)


def quat_inverse(q_xyzw: np.ndarray) -> np.ndarray:
    """Compute the inverse (conjugate) of a unit quaternion in xyzw convention.

    Args:
        q_xyzw: (4,) unit quaternion in xyzw.

    Returns:
        (4,) inverse quaternion in xyzw.
    """
    return np.array([-q_xyzw[0], -q_xyzw[1], -q_xyzw[2], q_xyzw[3]])


def quat_to_rotation_matrix(q_xyzw: np.ndarray) -> np.ndarray:
    """Convert xyzw quaternion to 3x3 rotation matrix.

    Args:
        q_xyzw: (4,) quaternion in xyzw.

    Returns:
        (3, 3) rotation matrix.
    """
    q_wxyz = quat_xyzw_to_wxyz(q_xyzw)
    return _quat2mat_wxyz(q_wxyz)


def rotation_matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to xyzw quaternion.

    Args:
        R: (3, 3) rotation matrix.

    Returns:
        (4,) quaternion in xyzw.
    """
    q_wxyz = _mat2quat_wxyz(R)
    return quat_wxyz_to_xyzw(q_wxyz)


def quat_to_euler(q_xyzw: np.ndarray, axes: str = "sxyz") -> np.ndarray:
    """Convert xyzw quaternion to Euler angles.

    Args:
        q_xyzw: (4,) quaternion in xyzw.
        axes: Euler angle convention string (default: 'sxyz' = roll, pitch, yaw).

    Returns:
        (3,) Euler angles in radians.
    """
    if axes != "sxyz":
        raise ValueError(f"Only 'sxyz' Euler convention is supported, got '{axes}'")
    R = quat_to_rotation_matrix(q_xyzw)
    return _mat_to_euler_sxyz(R)


def euler_to_quat(euler: np.ndarray, axes: str = "sxyz") -> np.ndarray:
    """Convert Euler angles to xyzw quaternion.

    Args:
        euler: (3,) Euler angles in radians.
        axes: Euler angle convention string (default: 'sxyz').

    Returns:
        (4,) quaternion in xyzw.
    """
    if axes != "sxyz":
        raise ValueError(f"Only 'sxyz' Euler convention is supported, got '{axes}'")
    R = _euler_to_mat_sxyz(float(euler[0]), float(euler[1]), float(euler[2]))
    return rotation_matrix_to_quat(R)


# --- Frame Conversions ---

# SteamVR to ROS2 rotation matrix.
# SteamVR: X-right, Y-up, Z-backward  ->  ROS2: X-forward, Y-left, Z-up
# ROS2_X = -SteamVR_Z, ROS2_Y = -SteamVR_X, ROS2_Z = SteamVR_Y
_STEAMVR_TO_ROS2_MATRIX = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])


def steamvr_position_to_ros2(pos_steamvr: np.ndarray) -> np.ndarray:
    """Convert position from SteamVR frame to ROS2 frame.

    Args:
        pos_steamvr: (3,) position in SteamVR coordinates.

    Returns:
        (3,) position in ROS2 coordinates (meters).
    """
    return _STEAMVR_TO_ROS2_MATRIX @ pos_steamvr


def steamvr_orientation_to_ros2(q_steamvr_wxyz: np.ndarray) -> np.ndarray:
    """Convert orientation from SteamVR (wxyz) to ROS2 (xyzw).

    Args:
        q_steamvr_wxyz: (4,) quaternion in SteamVR frame, wxyz order.

    Returns:
        (4,) quaternion in ROS2 frame, xyzw order.
    """
    R_steamvr = _quat2mat_wxyz(q_steamvr_wxyz)
    R_ros2 = _STEAMVR_TO_ROS2_MATRIX @ R_steamvr @ _STEAMVR_TO_ROS2_MATRIX.T
    q_ros2_wxyz = _mat2quat_wxyz(R_ros2)
    return quat_wxyz_to_xyzw(q_ros2_wxyz)


def steamvr_pose_to_ros2(
    pos_steamvr: np.ndarray,
    quat_steamvr_wxyz: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Convert full 6DoF pose from SteamVR to ROS2 frame.

    Args:
        pos_steamvr: (3,) position in SteamVR frame.
        quat_steamvr_wxyz: (4,) quaternion in SteamVR wxyz.

    Returns:
        Tuple of (position_ros2 (3,), quaternion_ros2_xyzw (4,)).
    """
    pos_ros2 = steamvr_position_to_ros2(pos_steamvr)
    quat_ros2 = steamvr_orientation_to_ros2(quat_steamvr_wxyz)
    return pos_ros2, quat_ros2


def ros2_position_to_steamvr(pos_ros2: np.ndarray) -> np.ndarray:
    """Convert position from ROS2 frame to SteamVR frame.

    Args:
        pos_ros2: (3,) position in ROS2 coordinates.

    Returns:
        (3,) position in SteamVR coordinates.
    """
    return _STEAMVR_TO_ROS2_MATRIX.T @ pos_ros2


# --- Homogeneous Transform Utilities ---

def pose_to_homogeneous(position: np.ndarray, quat_xyzw: np.ndarray) -> np.ndarray:
    """Create 4x4 homogeneous transform from position and xyzw quaternion.

    Args:
        position: (3,) translation.
        quat_xyzw: (4,) orientation quaternion in xyzw.

    Returns:
        (4, 4) homogeneous transformation matrix.
    """
    T = np.eye(4)
    T[:3, :3] = quat_to_rotation_matrix(quat_xyzw)
    T[:3, 3] = position
    return T


def homogeneous_to_pose(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Extract position and xyzw quaternion from 4x4 homogeneous transform.

    Args:
        T: (4, 4) homogeneous transformation matrix.

    Returns:
        Tuple of (position (3,), quaternion_xyzw (4,)).
    """
    position = T[:3, 3].copy()
    quat_xyzw = rotation_matrix_to_quat(T[:3, :3])
    return position, quat_xyzw


def invert_homogeneous(T: np.ndarray) -> np.ndarray:
    """Efficiently invert a 4x4 homogeneous transform.

    Args:
        T: (4, 4) homogeneous transformation matrix.

    Returns:
        (4, 4) inverse transformation matrix.
    """
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


# --- Angular Utilities ---

def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi].

    Args:
        angle: Angle in radians.

    Returns:
        Normalized angle in [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


def angular_distance(q1_xyzw: np.ndarray, q2_xyzw: np.ndarray) -> float:
    """Compute the angular distance between two quaternions.

    Args:
        q1_xyzw: (4,) first quaternion in xyzw.
        q2_xyzw: (4,) second quaternion in xyzw.

    Returns:
        Angular distance in radians [0, pi].
    """
    q1w = quat_xyzw_to_wxyz(q1_xyzw)
    q2w = quat_xyzw_to_wxyz(q2_xyzw)
    dot = np.clip(np.abs(np.dot(q1w, q2w)), 0.0, 1.0)
    return 2.0 * np.arccos(dot)
