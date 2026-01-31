"""Unit tests for coordinate transform utilities."""

import numpy as np
import pytest

from teleop_system.utils.transforms import (
    angular_distance,
    euler_to_quat,
    homogeneous_to_pose,
    invert_homogeneous,
    normalize_angle,
    pose_to_homogeneous,
    quat_inverse,
    quat_multiply,
    quat_to_euler,
    quat_to_rotation_matrix,
    quat_wxyz_to_xyzw,
    quat_xyzw_to_wxyz,
    rotation_matrix_to_quat,
    steamvr_position_to_ros2,
    steamvr_pose_to_ros2,
)


class TestQuaternionConventions:
    def test_wxyz_to_xyzw(self):
        q_wxyz = np.array([1.0, 0.0, 0.0, 0.0])  # identity
        q_xyzw = quat_wxyz_to_xyzw(q_wxyz)
        np.testing.assert_array_equal(q_xyzw, [0.0, 0.0, 0.0, 1.0])

    def test_xyzw_to_wxyz(self):
        q_xyzw = np.array([0.0, 0.0, 0.0, 1.0])  # identity
        q_wxyz = quat_xyzw_to_wxyz(q_xyzw)
        np.testing.assert_array_equal(q_wxyz, [1.0, 0.0, 0.0, 0.0])

    def test_roundtrip(self):
        q = np.array([0.1, 0.2, 0.3, 0.9])
        q = q / np.linalg.norm(q)
        result = quat_wxyz_to_xyzw(quat_xyzw_to_wxyz(q))
        np.testing.assert_array_almost_equal(result, q)


class TestQuaternionOperations:
    def test_multiply_identity(self):
        identity = np.array([0.0, 0.0, 0.0, 1.0])
        q = np.array([0.1, 0.2, 0.3, 0.9])
        q = q / np.linalg.norm(q)
        result = quat_multiply(identity, q)
        np.testing.assert_array_almost_equal(result, q)

    def test_inverse(self):
        q = np.array([0.1, 0.2, 0.3, 0.9])
        q = q / np.linalg.norm(q)
        q_inv = quat_inverse(q)
        result = quat_multiply(q, q_inv)
        identity = np.array([0.0, 0.0, 0.0, 1.0])
        np.testing.assert_array_almost_equal(np.abs(result), np.abs(identity), decimal=5)

    def test_to_rotation_matrix_identity(self):
        identity_q = np.array([0.0, 0.0, 0.0, 1.0])
        R = quat_to_rotation_matrix(identity_q)
        np.testing.assert_array_almost_equal(R, np.eye(3))

    def test_rotation_matrix_roundtrip(self):
        q = np.array([0.1, 0.2, 0.3, 0.9])
        q = q / np.linalg.norm(q)
        R = quat_to_rotation_matrix(q)
        q_back = rotation_matrix_to_quat(R)
        # Quaternion may be negated but represents same rotation
        assert np.allclose(q, q_back) or np.allclose(q, -q_back)


class TestEulerConversions:
    def test_identity(self):
        euler = np.array([0.0, 0.0, 0.0])
        q = euler_to_quat(euler)
        np.testing.assert_array_almost_equal(q, [0, 0, 0, 1], decimal=5)

    def test_roundtrip(self):
        euler_in = np.array([0.1, 0.2, 0.3])
        q = euler_to_quat(euler_in)
        euler_out = quat_to_euler(q)
        np.testing.assert_array_almost_equal(euler_out, euler_in, decimal=5)


class TestSteamVRConversion:
    def test_position_axes(self):
        # SteamVR: X-right=1 -> ROS2: should map to Y=-1
        pos_svr = np.array([1.0, 0.0, 0.0])
        pos_ros2 = steamvr_position_to_ros2(pos_svr)
        np.testing.assert_array_almost_equal(pos_ros2, [0.0, -1.0, 0.0])

        # SteamVR: Y-up=1 -> ROS2: should map to Z=1
        pos_svr = np.array([0.0, 1.0, 0.0])
        pos_ros2 = steamvr_position_to_ros2(pos_svr)
        np.testing.assert_array_almost_equal(pos_ros2, [0.0, 0.0, 1.0])

        # SteamVR: Z-backward=1 -> ROS2: should map to X=-1
        pos_svr = np.array([0.0, 0.0, 1.0])
        pos_ros2 = steamvr_position_to_ros2(pos_svr)
        np.testing.assert_array_almost_equal(pos_ros2, [-1.0, 0.0, 0.0])


class TestHomogeneousTransform:
    def test_identity(self):
        pos = np.zeros(3)
        quat = np.array([0.0, 0.0, 0.0, 1.0])
        T = pose_to_homogeneous(pos, quat)
        np.testing.assert_array_almost_equal(T, np.eye(4))

    def test_roundtrip(self):
        pos = np.array([1.0, 2.0, 3.0])
        quat = np.array([0.1, 0.2, 0.3, 0.9])
        quat = quat / np.linalg.norm(quat)
        T = pose_to_homogeneous(pos, quat)
        pos_out, quat_out = homogeneous_to_pose(T)
        np.testing.assert_array_almost_equal(pos_out, pos)
        assert np.allclose(quat_out, quat) or np.allclose(quat_out, -quat)

    def test_inverse(self):
        pos = np.array([1.0, 2.0, 3.0])
        quat = np.array([0.1, 0.2, 0.3, 0.9])
        quat = quat / np.linalg.norm(quat)
        T = pose_to_homogeneous(pos, quat)
        T_inv = invert_homogeneous(T)
        result = T @ T_inv
        np.testing.assert_array_almost_equal(result, np.eye(4), decimal=10)


class TestAngularUtilities:
    def test_normalize_angle(self):
        assert abs(normalize_angle(0.0)) < 1e-10
        assert abs(normalize_angle(2 * np.pi)) < 1e-10
        assert abs(normalize_angle(-2 * np.pi)) < 1e-10
        assert abs(normalize_angle(np.pi) - np.pi) < 1e-10 or abs(normalize_angle(np.pi) + np.pi) < 1e-10

    def test_angular_distance_zero(self):
        q = np.array([0.0, 0.0, 0.0, 1.0])
        assert angular_distance(q, q) < 1e-10

    def test_angular_distance_90deg(self):
        q1 = np.array([0.0, 0.0, 0.0, 1.0])
        q2 = euler_to_quat(np.array([np.pi / 2, 0.0, 0.0]))
        dist = angular_distance(q1, q2)
        np.testing.assert_almost_equal(dist, np.pi / 2, decimal=5)
