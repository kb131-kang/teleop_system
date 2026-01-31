"""Phase 5 tests: PointCloudGenerator and GUI ControlPanel.

PointCloudGenerator tests use the numpy fallback (no Open3D requirement).
GUI tests verify the ControlPanel data model without requiring a display.
"""

import numpy as np
import pytest

from teleop_system.interfaces.camera_stream import RGBDFrame
from teleop_system.modules.camera.pointcloud_generator import PointCloudGenerator


# ---------------------------------------------------------------------------
# Helper: create a synthetic RGB-D frame with known geometry
# ---------------------------------------------------------------------------


def _make_test_frame(
    width: int = 64,
    height: int = 48,
    depth_value: float = 2.0,
) -> RGBDFrame:
    """Create a synthetic RGB-D frame.

    All pixels have the same depth, with a centered pinhole intrinsic.
    """
    rgb = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)
    depth = np.full((height, width), depth_value, dtype=np.float32)

    fx = fy = 50.0
    cx, cy = width / 2.0, height / 2.0
    intrinsics = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0,  0,  1],
    ], dtype=np.float64)

    return RGBDFrame(
        rgb=rgb,
        depth=depth,
        intrinsics=intrinsics,
        width=width,
        height=height,
    )


# ===========================================================================
# PointCloudGenerator (numpy fallback)
# ===========================================================================


class TestPointCloudGeneratorNumpy:
    """Test point cloud generation using the numpy backend."""

    def test_basic_generation(self):
        gen = PointCloudGenerator(use_open3d=False)
        frame = _make_test_frame()
        result = gen.generate(frame)

        assert "points" in result
        assert "colors" in result
        assert "count" in result
        assert result["count"] > 0
        assert result["points"].shape[1] == 3
        assert result["colors"].shape[1] == 3

    def test_point_count_matches(self):
        gen = PointCloudGenerator(use_open3d=False)
        frame = _make_test_frame(width=32, height=24)
        result = gen.generate(frame)

        # All pixels have valid depth -> all should be in the cloud
        assert result["count"] == 32 * 24

    def test_depth_filtering(self):
        gen = PointCloudGenerator(use_open3d=False, min_depth=1.0, max_depth=3.0)

        # All at depth 2.0 -> all valid
        frame = _make_test_frame(depth_value=2.0)
        result = gen.generate(frame)
        assert result["count"] == frame.width * frame.height

        # All at depth 0.5 -> all filtered out (below min)
        frame = _make_test_frame(depth_value=0.5)
        result = gen.generate(frame)
        assert result["count"] == 0

        # All at depth 10.0 -> all filtered out (above max)
        frame = _make_test_frame(depth_value=10.0)
        result = gen.generate(frame)
        assert result["count"] == 0

    def test_z_values_match_depth(self):
        gen = PointCloudGenerator(use_open3d=False)
        frame = _make_test_frame(depth_value=3.0)
        result = gen.generate(frame)

        # All z values should equal the input depth
        z_values = result["points"][:, 2]
        np.testing.assert_array_almost_equal(z_values, 3.0)

    def test_colors_normalized(self):
        gen = PointCloudGenerator(use_open3d=False)
        frame = _make_test_frame()
        result = gen.generate(frame)

        # Colors should be in [0, 1] range
        assert result["colors"].min() >= 0.0
        assert result["colors"].max() <= 1.0

    def test_center_pixel_at_origin_xy(self):
        """The center pixel should project to approximately (0, 0, depth)."""
        gen = PointCloudGenerator(use_open3d=False)
        frame = _make_test_frame(width=64, height=48, depth_value=5.0)
        result = gen.generate(frame)

        # Find the point closest to the image center (x, y near 0)
        points = result["points"]
        # With fx=50 and depth=5.0, pixel spacing is 5.0/50=0.1 m/pixel
        center_mask = (np.abs(points[:, 0]) < 0.2) & (np.abs(points[:, 1]) < 0.2)
        assert np.sum(center_mask) > 0, "No points near image center"

    def test_o3d_pcd_is_none_for_numpy_backend(self):
        gen = PointCloudGenerator(use_open3d=False)
        frame = _make_test_frame()
        result = gen.generate(frame)
        assert result["o3d_pcd"] is None

    def test_zero_depth_filtered(self):
        """Pixels with zero depth should be excluded."""
        gen = PointCloudGenerator(use_open3d=False, min_depth=0.1)
        frame = _make_test_frame(depth_value=0.0)
        result = gen.generate(frame)
        assert result["count"] == 0


# ===========================================================================
# GUI ControlPanel data model (no display needed)
# ===========================================================================


class TestControlPanelModel:
    """Test the ControlPanel data model without creating a GUI window."""

    def test_module_registration(self):
        from teleop_system.gui.control_panel import ControlPanel, ModuleStatus

        panel = ControlPanel()
        status = panel.add_module("Arm Teleop")
        assert isinstance(status, ModuleStatus)
        assert status.name == "Arm Teleop"
        assert status.connected is False
        assert status.enabled is False

    def test_multiple_modules(self):
        from teleop_system.gui.control_panel import ControlPanel

        panel = ControlPanel()
        s1 = panel.add_module("Arm Teleop")
        s2 = panel.add_module("Locomotion")
        s3 = panel.add_module("Hand Teleop")

        assert s1.name == "Arm Teleop"
        assert s2.name == "Locomotion"
        assert s3.name == "Hand Teleop"

    def test_module_status_update(self):
        from teleop_system.gui.control_panel import ControlPanel

        panel = ControlPanel()
        status = panel.add_module("IK Solver")
        status.connected = True
        status.enabled = True
        status.metrics["pos_error"] = 0.005
        status.metrics["ori_error"] = 0.01

        assert status.connected is True
        assert status.metrics["pos_error"] == 0.005

    def test_parameter_access(self):
        from teleop_system.gui.control_panel import ControlPanel

        panel = ControlPanel()
        # Check default parameters exist
        assert panel.get_parameter("position_scale") == 1.0
        assert panel.get_parameter("max_joint_velocity") == 2.0
        assert panel.get_parameter("nonexistent") == 0.0

    def test_parameter_callback(self):
        from teleop_system.gui.control_panel import ControlPanel

        panel = ControlPanel()
        received = {}

        def on_change(value):
            received["value"] = value

        panel.set_parameter_callback("position_scale", on_change)
        # Simulate parameter change via internal call
        panel._parameter_change_callback(None, 2.5, "position_scale")
        assert received["value"] == 2.5
        assert panel.get_parameter("position_scale") == 2.5

    def test_toggle_callback(self):
        from teleop_system.gui.control_panel import ControlPanel

        toggled = {}

        def on_toggle(enabled):
            toggled["state"] = enabled

        panel = ControlPanel()
        panel.add_module("TestModule", on_toggle=on_toggle)
        # Simulate toggle via internal call
        panel._module_toggle_callback(None, True, "TestModule")
        assert toggled["state"] is True
