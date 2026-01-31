"""Unit tests for interface definitions and data classes."""

import numpy as np
import pytest

from teleop_system.interfaces import (
    ArmSide,
    HandJointState,
    IKResult,
    JointState,
    Pose6D,
    RGBDFrame,
    SimState,
    TrackerRole,
    VelocityCommand,
)


class TestPose6D:
    def test_default_construction(self):
        pose = Pose6D()
        np.testing.assert_array_equal(pose.position, np.zeros(3))
        np.testing.assert_array_equal(pose.orientation, [0, 0, 0, 1])
        assert pose.timestamp == 0.0
        assert pose.valid is True

    def test_custom_construction(self):
        pos = np.array([1.0, 2.0, 3.0])
        ori = np.array([0.0, 0.707, 0.0, 0.707])
        pose = Pose6D(position=pos, orientation=ori, timestamp=1.5, valid=False)
        np.testing.assert_array_equal(pose.position, pos)
        np.testing.assert_array_almost_equal(pose.orientation, ori)
        assert pose.timestamp == 1.5
        assert pose.valid is False


class TestHandJointState:
    def test_default_construction(self):
        state = HandJointState()
        assert state.joint_angles.shape == (20,)
        assert state.side == "right"
        assert state.valid is True

    def test_custom_construction(self):
        angles = np.random.randn(20)
        state = HandJointState(joint_angles=angles, side="left")
        np.testing.assert_array_equal(state.joint_angles, angles)
        assert state.side == "left"


class TestJointState:
    def test_default_construction(self):
        state = JointState()
        assert state.positions.shape == (0,)
        assert state.velocities is None
        assert state.efforts is None

    def test_with_data(self):
        pos = np.array([0.1, 0.2, 0.3])
        vel = np.array([1.0, 2.0, 3.0])
        state = JointState(
            positions=pos,
            velocities=vel,
            names=["j1", "j2", "j3"],
        )
        assert len(state.positions) == 3
        assert len(state.names) == 3


class TestIKResult:
    def test_default_success(self):
        result = IKResult()
        assert result.success is True
        assert result.error_position == 0.0


class TestRGBDFrame:
    def test_default_dimensions(self):
        frame = RGBDFrame()
        assert frame.rgb.shape == (480, 640, 3)
        assert frame.depth.shape == (480, 640)
        assert frame.intrinsics.shape == (3, 3)
        assert frame.width == 640
        assert frame.height == 480


class TestSimState:
    def test_default_base_orientation(self):
        state = SimState()
        np.testing.assert_array_equal(state.base_orientation, [0, 0, 0, 1])


class TestVelocityCommand:
    def test_default_zero(self):
        cmd = VelocityCommand()
        assert cmd.linear_x == 0.0
        assert cmd.linear_y == 0.0
        assert cmd.angular_z == 0.0


class TestEnums:
    def test_tracker_roles(self):
        assert TrackerRole.RIGHT_HAND.name == "RIGHT_HAND"
        assert len(TrackerRole) == 6

    def test_arm_sides(self):
        assert ArmSide.LEFT.name == "LEFT"
        assert ArmSide.TORSO.name == "TORSO"
        assert len(ArmSide) == 3
