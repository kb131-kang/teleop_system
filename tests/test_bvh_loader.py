"""Tests for BVH motion capture file loader."""

import tempfile
from pathlib import Path

import numpy as np
import pytest

# Minimal BVH file for testing (3 joints, 3 frames)
SAMPLE_BVH = """\
HIERARCHY
ROOT Hips
{
	OFFSET 0.00000 0.00000 0.00000
	CHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation
	JOINT Spine
	{
		OFFSET 0.00000 5.00000 0.00000
		CHANNELS 3 Zrotation Yrotation Xrotation
		JOINT Head
		{
			OFFSET 0.00000 5.00000 0.00000
			CHANNELS 3 Zrotation Yrotation Xrotation
			End Site
			{
				OFFSET 0.00000 2.00000 0.00000
			}
		}
	}
	JOINT LeftHand
	{
		OFFSET 3.00000 4.00000 0.00000
		CHANNELS 3 Zrotation Yrotation Xrotation
		End Site
		{
			OFFSET 2.00000 0.00000 0.00000
		}
	}
	JOINT RightHand
	{
		OFFSET -3.00000 4.00000 0.00000
		CHANNELS 3 Zrotation Yrotation Xrotation
		End Site
		{
			OFFSET -2.00000 0.00000 0.00000
		}
	}
	JOINT LeftFoot
	{
		OFFSET 1.00000 -5.00000 0.00000
		CHANNELS 3 Zrotation Yrotation Xrotation
		End Site
		{
			OFFSET 0.00000 0.00000 1.00000
		}
	}
	JOINT RightFoot
	{
		OFFSET -1.00000 -5.00000 0.00000
		CHANNELS 3 Zrotation Yrotation Xrotation
		End Site
		{
			OFFSET 0.00000 0.00000 1.00000
		}
	}
}
MOTION
Frames: 3
Frame Time: 0.0083333
0.0 10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
1.0 10.0 1.0 5.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
2.0 10.0 2.0 10.0 0.0 0.0 0.0 5.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
"""


@pytest.fixture
def sample_bvh_path(tmp_path):
    """Create a temporary BVH file for testing."""
    bvh_file = tmp_path / "test.bvh"
    bvh_file.write_text(SAMPLE_BVH)
    return str(bvh_file)


class TestBVHLoader:
    """Tests for BVH file loading and parsing."""

    def test_module_imports(self):
        from teleop_system.mocap.bvh_loader import load_bvh, load_bvh_lazy, BVHData
        assert load_bvh is not None
        assert load_bvh_lazy is not None
        assert BVHData is not None

    def test_load_bvh_basic(self, sample_bvh_path):
        from teleop_system.mocap.bvh_loader import load_bvh

        data = load_bvh(sample_bvh_path, scale=1.0)

        assert data.frame_count == 3
        assert abs(data.frame_time - 0.0083333) < 1e-5
        assert abs(data.fps - 120.0) < 1.0
        assert len(data.joint_names) > 0
        assert "Hips" in data.joint_names
        assert "Head" in data.joint_names
        assert "LeftHand" in data.joint_names
        assert "RightHand" in data.joint_names

    def test_frame_count_matches(self, sample_bvh_path):
        from teleop_system.mocap.bvh_loader import load_bvh

        data = load_bvh(sample_bvh_path, scale=1.0)

        for jname in data.joint_names:
            if jname in data.frames:
                assert len(data.frames[jname]) == data.frame_count, \
                    f"Joint {jname} has {len(data.frames[jname])} frames, expected {data.frame_count}"

    def test_joint_frame_shapes(self, sample_bvh_path):
        from teleop_system.mocap.bvh_loader import load_bvh

        data = load_bvh(sample_bvh_path, scale=1.0)

        for jname in data.joint_names:
            if jname in data.frames:
                for frame in data.frames[jname]:
                    assert frame.position.shape == (3,), \
                        f"Position shape mismatch for {jname}"
                    assert frame.orientation.shape == (4,), \
                        f"Orientation shape mismatch for {jname}"

    def test_quaternion_is_unit(self, sample_bvh_path):
        from teleop_system.mocap.bvh_loader import load_bvh

        data = load_bvh(sample_bvh_path, scale=1.0)

        for jname in data.joint_names:
            if jname in data.frames:
                for i, frame in enumerate(data.frames[jname]):
                    norm = np.linalg.norm(frame.orientation)
                    assert abs(norm - 1.0) < 0.01, \
                        f"Non-unit quaternion for {jname} frame {i}: norm={norm}"

    def test_coordinate_conversion_applied(self, sample_bvh_path):
        """Verify BVH Y-up is converted to ROS2 Z-up."""
        from teleop_system.mocap.bvh_loader import load_bvh

        # Load with scale=1.0 (no unit scaling, only coordinate transform)
        data = load_bvh(sample_bvh_path, scale=1.0)

        # In the BVH file, Hips is at Y=10 (height).
        # After conversion: BVH Y → ROS2 Z, so Hips Z should be ~10
        hips_frame0 = data.frames["Hips"][0]
        assert hips_frame0.position[2] > 5.0, \
            f"Expected Hips Z > 5.0 (BVH Y-up → ROS2 Z-up), got {hips_frame0.position[2]}"

    def test_scale_factor(self, sample_bvh_path):
        from teleop_system.mocap.bvh_loader import load_bvh

        data_1 = load_bvh(sample_bvh_path, scale=1.0)
        data_2 = load_bvh(sample_bvh_path, scale=0.5)

        pos_1 = data_1.frames["Hips"][0].position
        pos_2 = data_2.frames["Hips"][0].position

        np.testing.assert_allclose(pos_2, pos_1 * 0.5, atol=1e-5)

    def test_skeleton_parents(self, sample_bvh_path):
        from teleop_system.mocap.bvh_loader import load_bvh

        data = load_bvh(sample_bvh_path, scale=1.0)

        assert data.skeleton_parents["Hips"] is None  # root
        assert data.skeleton_parents["Spine"] == "Hips"
        assert data.skeleton_parents["Head"] == "Spine"

    def test_file_not_found(self):
        from teleop_system.mocap.bvh_loader import load_bvh

        with pytest.raises(FileNotFoundError):
            load_bvh("/nonexistent/file.bvh")

    def test_load_lazy_specific_joints(self, sample_bvh_path):
        from teleop_system.mocap.bvh_loader import load_bvh_lazy

        data = load_bvh_lazy(sample_bvh_path, scale=1.0, joints=["Hips", "Head"])

        assert "Hips" in data.frames
        assert "Head" in data.frames
        # Lazy load only extracts requested joints
        assert "LeftHand" not in data.frames
        # But all joint names are still listed
        assert "LeftHand" in data.joint_names

    def test_skeleton_offsets_exist(self, sample_bvh_path):
        from teleop_system.mocap.bvh_loader import load_bvh

        data = load_bvh(sample_bvh_path, scale=1.0)

        assert len(data.skeleton_offsets) > 0
        for jname in data.joint_names:
            assert jname in data.skeleton_offsets
            assert data.skeleton_offsets[jname].shape == (3,)


class TestBVHLoaderCMU:
    """Tests with real CMU BVH data (skipped if not available)."""

    CMU_FILE = Path(__file__).parent.parent / "data" / "bvh" / "cmu" / "002" / "02_01.bvh"

    @pytest.fixture(autouse=True)
    def check_cmu_data(self):
        if not self.CMU_FILE.exists():
            pytest.skip(f"CMU BVH data not available: {self.CMU_FILE}")

    def test_load_cmu_walking(self):
        from teleop_system.mocap.bvh_loader import load_bvh

        data = load_bvh(str(self.CMU_FILE), scale=0.056)

        assert data.frame_count > 100
        assert abs(data.fps - 120.0) < 1.0
        assert "Hips" in data.joint_names
        assert "Head" in data.joint_names
        assert "LeftHand" in data.joint_names
        assert "RightHand" in data.joint_names
        assert "LeftFoot" in data.joint_names
        assert "RightFoot" in data.joint_names

    def test_cmu_positions_reasonable(self):
        """Verify converted positions are in reasonable human-scale range."""
        from teleop_system.mocap.bvh_loader import load_bvh

        data = load_bvh(str(self.CMU_FILE), scale=0.056)

        hips = data.frames["Hips"][0]
        head = data.frames["Head"][0]

        # Hips height (Z in ROS2) should be roughly 0.8-1.2m
        assert 0.5 < hips.position[2] < 1.5, \
            f"Hips Z={hips.position[2]} outside expected range"

        # Head should be above hips
        assert head.position[2] > hips.position[2], \
            "Head should be above hips"

    def test_cmu_lazy_load(self):
        from teleop_system.mocap.bvh_loader import load_bvh_lazy

        data = load_bvh_lazy(
            str(self.CMU_FILE), scale=0.056,
            joints=["Hips", "Head", "LeftHand", "RightHand", "LeftFoot", "RightFoot"],
        )

        assert len(data.frames) == 6
        assert data.frame_count > 100
