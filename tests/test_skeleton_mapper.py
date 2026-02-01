"""Tests for BVH skeleton to TrackerRole mapping."""

import tempfile
from pathlib import Path

import numpy as np
import pytest

from tests.test_bvh_loader import SAMPLE_BVH


@pytest.fixture
def sample_bvh_data(tmp_path):
    """Load BVH data from sample file."""
    bvh_file = tmp_path / "test.bvh"
    bvh_file.write_text(SAMPLE_BVH)

    from teleop_system.mocap.bvh_loader import load_bvh
    return load_bvh(str(bvh_file), scale=1.0)


class TestSkeletonMapper:
    """Tests for SkeletonMapper."""

    def test_module_imports(self):
        from teleop_system.mocap.skeleton_mapper import (
            SkeletonMapper, MappedMotion, TrackerFrameData,
        )
        assert SkeletonMapper is not None
        assert MappedMotion is not None
        assert TrackerFrameData is not None

    def test_map_basic(self, sample_bvh_data):
        from teleop_system.mocap.skeleton_mapper import SkeletonMapper
        from teleop_system.interfaces.master_device import TrackerRole

        mapper = SkeletonMapper(normalize_mode="absolute")
        result = mapper.map(sample_bvh_data)

        assert result.frame_count == sample_bvh_data.frame_count
        assert result.frame_time == sample_bvh_data.frame_time
        assert len(result.frames) == sample_bvh_data.frame_count

    def test_mapped_roles_present(self, sample_bvh_data):
        from teleop_system.mocap.skeleton_mapper import SkeletonMapper
        from teleop_system.interfaces.master_device import TrackerRole

        mapper = SkeletonMapper(normalize_mode="absolute")
        result = mapper.map(sample_bvh_data)

        # Sample BVH has: Hips, Head, LeftHand, RightHand, LeftFoot, RightFoot
        assert TrackerRole.WAIST in result.roles
        assert TrackerRole.HEAD in result.roles
        assert TrackerRole.LEFT_HAND in result.roles
        assert TrackerRole.RIGHT_HAND in result.roles
        assert TrackerRole.LEFT_FOOT in result.roles
        assert TrackerRole.RIGHT_FOOT in result.roles

    def test_pose6d_format(self, sample_bvh_data):
        from teleop_system.mocap.skeleton_mapper import SkeletonMapper
        from teleop_system.interfaces.master_device import Pose6D, TrackerRole

        mapper = SkeletonMapper(normalize_mode="absolute")
        result = mapper.map(sample_bvh_data)

        for frame in result.frames:
            for role, pose in frame.poses.items():
                assert isinstance(pose, Pose6D)
                assert pose.position.shape == (3,)
                assert pose.orientation.shape == (4,)
                assert pose.valid is True

    def test_relative_mode_first_frame_at_reference(self, sample_bvh_data):
        """In relative mode, first frame should be at robot reference position."""
        from teleop_system.mocap.skeleton_mapper import (
            SkeletonMapper, DEFAULT_ROBOT_POSITIONS,
        )
        from teleop_system.interfaces.master_device import TrackerRole

        mapper = SkeletonMapper(normalize_mode="relative")
        result = mapper.map(sample_bvh_data)

        # First frame: delta=0, so position should equal robot reference
        frame0 = result.frames[0]
        for role in result.roles:
            expected = DEFAULT_ROBOT_POSITIONS.get(role, np.zeros(3))
            np.testing.assert_allclose(
                frame0.poses[role].position, expected, atol=1e-5,
                err_msg=f"First frame {role.name} not at reference position",
            )

    def test_relative_mode_preserves_motion(self, sample_bvh_data):
        """In relative mode, motion deltas should be preserved."""
        from teleop_system.mocap.skeleton_mapper import SkeletonMapper
        from teleop_system.interfaces.master_device import TrackerRole

        mapper_abs = SkeletonMapper(normalize_mode="absolute")
        mapper_rel = SkeletonMapper(normalize_mode="relative")

        result_abs = mapper_abs.map(sample_bvh_data)
        result_rel = mapper_rel.map(sample_bvh_data)

        # Check that deltas between frames are the same
        for role in result_abs.roles:
            for i in range(1, len(result_abs.frames)):
                delta_abs = (
                    result_abs.frames[i].poses[role].position -
                    result_abs.frames[0].poses[role].position
                )
                delta_rel = (
                    result_rel.frames[i].poses[role].position -
                    result_rel.frames[0].poses[role].position
                )
                np.testing.assert_allclose(
                    delta_abs, delta_rel, atol=1e-5,
                    err_msg=f"Motion delta mismatch for {role.name} frame {i}",
                )

    def test_timestamp_progression(self, sample_bvh_data):
        from teleop_system.mocap.skeleton_mapper import SkeletonMapper
        from teleop_system.interfaces.master_device import TrackerRole

        mapper = SkeletonMapper(normalize_mode="absolute")
        result = mapper.map(sample_bvh_data)

        timestamps = []
        for frame in result.frames:
            # All poses in a frame should have the same timestamp
            ts_set = set()
            for role, pose in frame.poses.items():
                ts_set.add(pose.timestamp)
            assert len(ts_set) == 1, "All roles should have same timestamp per frame"
            timestamps.append(ts_set.pop())

        # Timestamps should be monotonically increasing
        for i in range(1, len(timestamps)):
            assert timestamps[i] > timestamps[i - 1]

    def test_custom_joint_mapping(self, sample_bvh_data):
        from teleop_system.mocap.skeleton_mapper import SkeletonMapper
        from teleop_system.interfaces.master_device import TrackerRole

        # Map waist to Spine instead of Hips
        custom_mapping = {
            TrackerRole.WAIST: "Spine",
            TrackerRole.HEAD: "Head",
        }

        mapper = SkeletonMapper(
            joint_mapping=custom_mapping,
            normalize_mode="absolute",
        )
        result = mapper.map(sample_bvh_data)

        assert TrackerRole.WAIST in result.roles
        assert TrackerRole.HEAD in result.roles
        # Only 2 roles mapped
        assert len(result.roles) == 2
