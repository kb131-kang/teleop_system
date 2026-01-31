"""Unit tests for simulation components (SimulatedTracker, SimulatedHand, MuJoCo)."""

import time

import numpy as np
import pytest

from teleop_system.interfaces.master_device import TrackerRole
from teleop_system.simulators.simulated_tracker import SimulatedTracker
from teleop_system.simulators.simulated_hand import SimulatedHand


class TestSimulatedTracker:
    def test_initialization(self):
        tracker = SimulatedTracker(role=TrackerRole.RIGHT_HAND)
        assert tracker.initialize() is True
        assert tracker.is_connected() is True
        assert tracker.get_role() == TrackerRole.RIGHT_HAND

    def test_get_pose_returns_valid(self):
        tracker = SimulatedTracker(role=TrackerRole.RIGHT_HAND)
        tracker.initialize()
        pose = tracker.get_pose()
        assert pose.valid is True
        assert pose.position.shape == (3,)
        assert pose.orientation.shape == (4,)
        # Quaternion should be unit length
        assert abs(np.linalg.norm(pose.orientation) - 1.0) < 1e-5

    def test_pose_changes_over_time(self):
        tracker = SimulatedTracker(
            role=TrackerRole.RIGHT_HAND,
            frequency=10.0,  # Fast for testing
        )
        tracker.initialize()
        pose1 = tracker.get_pose()
        time.sleep(0.05)
        pose2 = tracker.get_pose()
        # Position should have changed
        assert not np.allclose(pose1.position, pose2.position, atol=1e-6)

    def test_not_connected_before_init(self):
        tracker = SimulatedTracker(role=TrackerRole.LEFT_HAND)
        assert tracker.is_connected() is False
        pose = tracker.get_pose()
        assert pose.valid is False

    def test_shutdown(self):
        tracker = SimulatedTracker(role=TrackerRole.WAIST)
        tracker.initialize()
        tracker.shutdown()
        assert tracker.is_connected() is False

    def test_all_roles_have_defaults(self):
        for role in TrackerRole:
            tracker = SimulatedTracker(role=role)
            tracker.initialize()
            pose = tracker.get_pose()
            assert pose.valid is True
            tracker.shutdown()

    def test_custom_center_position(self):
        center = np.array([1.0, 2.0, 3.0])
        tracker = SimulatedTracker(
            role=TrackerRole.RIGHT_HAND,
            center_position=center,
            amplitude=0.0,  # No motion -> stays at center
        )
        tracker.initialize()
        pose = tracker.get_pose()
        np.testing.assert_array_almost_equal(pose.position, center, decimal=2)


class TestSimulatedHand:
    def test_initialization(self):
        hand = SimulatedHand(side="right")
        assert hand.initialize() is True
        assert hand.is_connected() is True
        assert hand.get_side() == "right"

    def test_joint_state_shape(self):
        hand = SimulatedHand(side="left")
        hand.initialize()
        state = hand.get_joint_state()
        assert state.valid is True
        assert state.joint_angles.shape == (20,)
        assert state.side == "left"

    def test_angles_within_range(self):
        hand = SimulatedHand(side="right", max_angle=1.4)
        hand.initialize()
        state = hand.get_joint_state()
        assert np.all(state.joint_angles >= 0.0)
        assert np.all(state.joint_angles <= 1.5)

    def test_not_connected_before_init(self):
        hand = SimulatedHand()
        state = hand.get_joint_state()
        assert state.valid is False
