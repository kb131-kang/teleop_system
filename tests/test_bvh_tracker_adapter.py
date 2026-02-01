"""Tests for BVH tracker and hand adapters."""

import time
import tempfile
from pathlib import Path

import numpy as np
import pytest

from tests.test_bvh_loader import SAMPLE_BVH


@pytest.fixture
def mapped_motion(tmp_path):
    """Create mapped motion from sample BVH."""
    bvh_file = tmp_path / "test.bvh"
    bvh_file.write_text(SAMPLE_BVH)

    from teleop_system.mocap.bvh_loader import load_bvh
    from teleop_system.mocap.skeleton_mapper import SkeletonMapper

    data = load_bvh(str(bvh_file), scale=1.0)
    mapper = SkeletonMapper(normalize_mode="relative")
    return mapper.map(data)


class TestBVHTrackerAdapter:
    """Tests for BVHTrackerAdapter."""

    def test_module_imports(self):
        from teleop_system.mocap.bvh_tracker_adapter import BVHTrackerAdapter
        assert BVHTrackerAdapter is not None

    def test_implements_interface(self, mapped_motion):
        from teleop_system.mocap.bvh_tracker_adapter import BVHTrackerAdapter
        from teleop_system.interfaces.master_device import IMasterTracker, TrackerRole

        adapter = BVHTrackerAdapter(
            role=TrackerRole.RIGHT_HAND,
            mapped_motion=mapped_motion,
        )

        assert isinstance(adapter, IMasterTracker)

    def test_initialize_and_connect(self, mapped_motion):
        from teleop_system.mocap.bvh_tracker_adapter import BVHTrackerAdapter
        from teleop_system.interfaces.master_device import TrackerRole

        adapter = BVHTrackerAdapter(
            role=TrackerRole.RIGHT_HAND,
            mapped_motion=mapped_motion,
        )

        assert not adapter.is_connected()
        result = adapter.initialize()
        assert result is True
        assert adapter.is_connected()

    def test_get_role(self, mapped_motion):
        from teleop_system.mocap.bvh_tracker_adapter import BVHTrackerAdapter
        from teleop_system.interfaces.master_device import TrackerRole

        adapter = BVHTrackerAdapter(
            role=TrackerRole.LEFT_HAND,
            mapped_motion=mapped_motion,
        )

        assert adapter.get_role() == TrackerRole.LEFT_HAND

    def test_get_pose_returns_valid(self, mapped_motion):
        from teleop_system.mocap.bvh_tracker_adapter import BVHTrackerAdapter
        from teleop_system.interfaces.master_device import Pose6D, TrackerRole

        adapter = BVHTrackerAdapter(
            role=TrackerRole.RIGHT_HAND,
            mapped_motion=mapped_motion,
        )
        adapter.initialize()

        pose = adapter.get_pose()
        assert isinstance(pose, Pose6D)
        assert pose.valid is True
        assert pose.position.shape == (3,)
        assert pose.orientation.shape == (4,)

    def test_get_pose_invalid_when_not_connected(self, mapped_motion):
        from teleop_system.mocap.bvh_tracker_adapter import BVHTrackerAdapter
        from teleop_system.interfaces.master_device import TrackerRole

        adapter = BVHTrackerAdapter(
            role=TrackerRole.RIGHT_HAND,
            mapped_motion=mapped_motion,
        )
        # Not initialized
        pose = adapter.get_pose()
        assert pose.valid is False

    def test_shutdown(self, mapped_motion):
        from teleop_system.mocap.bvh_tracker_adapter import BVHTrackerAdapter
        from teleop_system.interfaces.master_device import TrackerRole

        adapter = BVHTrackerAdapter(
            role=TrackerRole.RIGHT_HAND,
            mapped_motion=mapped_motion,
        )
        adapter.initialize()
        assert adapter.is_connected()

        adapter.shutdown()
        assert not adapter.is_connected()
        pose = adapter.get_pose()
        assert pose.valid is False

    def test_loop_mode(self, mapped_motion):
        from teleop_system.mocap.bvh_tracker_adapter import BVHTrackerAdapter
        from teleop_system.interfaces.master_device import TrackerRole

        adapter = BVHTrackerAdapter(
            role=TrackerRole.RIGHT_HAND,
            mapped_motion=mapped_motion,
            loop=True,
            playback_speed=100000.0,  # Very fast to go past end
        )
        adapter.initialize()

        # Even at very high speed, loop mode should always return valid
        time.sleep(0.01)
        pose = adapter.get_pose()
        assert pose.valid is True

    def test_no_loop_ends(self, mapped_motion):
        from teleop_system.mocap.bvh_tracker_adapter import BVHTrackerAdapter
        from teleop_system.interfaces.master_device import TrackerRole

        adapter = BVHTrackerAdapter(
            role=TrackerRole.RIGHT_HAND,
            mapped_motion=mapped_motion,
            loop=False,
            playback_speed=1000000.0,  # Very fast
        )
        adapter.initialize()

        # Give time to reach past the end
        time.sleep(0.01)
        pose = adapter.get_pose()
        assert pose.valid is False

    def test_pause_resume(self, mapped_motion):
        from teleop_system.mocap.bvh_tracker_adapter import BVHTrackerAdapter
        from teleop_system.interfaces.master_device import TrackerRole

        adapter = BVHTrackerAdapter(
            role=TrackerRole.RIGHT_HAND,
            mapped_motion=mapped_motion,
        )
        adapter.initialize()

        # Get initial pose
        pose1 = adapter.get_pose()
        assert pose1.valid is True

        # Pause
        adapter.pause()
        frame_paused = adapter.get_frame_index()

        time.sleep(0.05)

        # Frame should not advance while paused
        assert adapter.get_frame_index() == frame_paused

        # Resume
        adapter.resume()

    def test_missing_role_returns_invalid(self, mapped_motion):
        """Adapter with a role not in mapped data should return invalid poses."""
        from teleop_system.mocap.bvh_tracker_adapter import BVHTrackerAdapter
        from teleop_system.mocap.skeleton_mapper import MappedMotion
        from teleop_system.interfaces.master_device import TrackerRole

        # Create empty mapped motion with no roles
        empty_motion = MappedMotion(
            frame_count=10,
            frame_time=0.01,
            fps=100,
            frames=[],
            roles=[],
        )

        adapter = BVHTrackerAdapter(
            role=TrackerRole.RIGHT_HAND,
            mapped_motion=empty_motion,
        )
        adapter.initialize()
        pose = adapter.get_pose()
        assert pose.valid is False


class TestBVHHandAdapter:
    """Tests for BVHHandAdapter."""

    def test_module_imports(self):
        from teleop_system.mocap.bvh_hand_adapter import BVHHandAdapter
        assert BVHHandAdapter is not None

    def test_implements_interface(self, mapped_motion):
        from teleop_system.mocap.bvh_hand_adapter import BVHHandAdapter
        from teleop_system.interfaces.master_device import IHandInput

        adapter = BVHHandAdapter(
            side="right",
            mapped_motion=mapped_motion,
        )

        assert isinstance(adapter, IHandInput)

    def test_get_joint_state(self, mapped_motion):
        from teleop_system.mocap.bvh_hand_adapter import BVHHandAdapter
        from teleop_system.interfaces.master_device import HandJointState

        adapter = BVHHandAdapter(
            side="right",
            mapped_motion=mapped_motion,
        )
        adapter.initialize()

        state = adapter.get_joint_state()
        assert isinstance(state, HandJointState)
        assert state.valid is True
        assert state.side == "right"
        assert state.joint_angles.shape == (20,)

    def test_joint_angles_in_range(self, mapped_motion):
        from teleop_system.mocap.bvh_hand_adapter import BVHHandAdapter

        adapter = BVHHandAdapter(
            side="left",
            mapped_motion=mapped_motion,
            max_angle=1.4,
        )
        adapter.initialize()

        state = adapter.get_joint_state()
        assert np.all(state.joint_angles >= 0.0)
        assert np.all(state.joint_angles <= 1.4 + 0.01)

    def test_get_side(self, mapped_motion):
        from teleop_system.mocap.bvh_hand_adapter import BVHHandAdapter

        for side in ["left", "right"]:
            adapter = BVHHandAdapter(side=side, mapped_motion=mapped_motion)
            assert adapter.get_side() == side

    def test_shutdown(self, mapped_motion):
        from teleop_system.mocap.bvh_hand_adapter import BVHHandAdapter

        adapter = BVHHandAdapter(side="right", mapped_motion=mapped_motion)
        adapter.initialize()
        assert adapter.is_connected()

        adapter.shutdown()
        assert not adapter.is_connected()
        state = adapter.get_joint_state()
        assert state.valid is False
