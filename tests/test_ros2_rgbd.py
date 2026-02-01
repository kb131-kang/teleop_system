"""Tests for ROS2 RGB-D subscriber and head sync modules.

Tests the non-ROS2 aspects (imports, default state, graceful degradation)
since ROS2 may not be available in the test environment.
"""

import numpy as np
import pytest


# ── ROS2RGBDSubscriber tests ──


class TestROS2RGBDSubscriberImport:
    """Test that the module imports and degrades gracefully without ROS2."""

    def test_import_without_ros2(self):
        """Module imports cleanly even without ROS2."""
        from teleop_system.modules.camera.ros2_rgbd_subscriber import (
            ROS2RGBDSubscriber,
        )

    def test_init_fails_gracefully_without_ros2(self):
        """initialize() returns False when ROS2 is unavailable."""
        from teleop_system.modules.camera import ros2_rgbd_subscriber as mod

        if not mod._ROS2_AVAILABLE:
            sub = mod.ROS2RGBDSubscriber()
            assert sub.initialize() is False

    def test_default_state(self):
        """Default state returns empty frame and zero orientation."""
        from teleop_system.modules.camera.ros2_rgbd_subscriber import (
            ROS2RGBDSubscriber,
        )

        sub = ROS2RGBDSubscriber()

        frame = sub.get_rgbd()
        assert frame.width == 640
        assert frame.height == 480

        pan, tilt = sub.get_orientation()
        assert pan == 0.0
        assert tilt == 0.0

    def test_is_connected_false_before_init(self):
        """is_connected() returns False before initialization."""
        from teleop_system.modules.camera.ros2_rgbd_subscriber import (
            ROS2RGBDSubscriber,
        )

        sub = ROS2RGBDSubscriber()
        assert not sub.is_connected()

    def test_get_intrinsics_default(self):
        """get_intrinsics() returns identity before receiving CameraInfo."""
        from teleop_system.modules.camera.ros2_rgbd_subscriber import (
            ROS2RGBDSubscriber,
        )

        sub = ROS2RGBDSubscriber()
        intrinsics = sub.get_intrinsics()
        np.testing.assert_array_equal(intrinsics, np.eye(3))

    def test_set_orientation_updates_local_state(self):
        """set_orientation() updates local pan/tilt even without ROS2."""
        from teleop_system.modules.camera.ros2_rgbd_subscriber import (
            ROS2RGBDSubscriber,
        )

        sub = ROS2RGBDSubscriber()
        sub.set_orientation(0.3, -0.2)

        pan, tilt = sub.get_orientation()
        assert pan == 0.3
        assert tilt == -0.2

    def test_shutdown_is_safe(self):
        """shutdown() can be called without initialization."""
        from teleop_system.modules.camera.ros2_rgbd_subscriber import (
            ROS2RGBDSubscriber,
        )

        sub = ROS2RGBDSubscriber()
        sub.shutdown()
        assert not sub.is_connected()

    def test_custom_topic_names(self):
        """Constructor accepts custom topic names."""
        from teleop_system.modules.camera.ros2_rgbd_subscriber import (
            ROS2RGBDSubscriber,
        )

        sub = ROS2RGBDSubscriber(
            color_topic="/custom/color",
            depth_topic="/custom/depth",
            info_topic="/custom/info",
            joint_states_topic="/custom/joints",
            pan_tilt_topic="/custom/pan_tilt",
        )
        assert sub._color_topic == "/custom/color"
        assert sub._depth_topic == "/custom/depth"


# ── ROS2HeadSync tests ──


class TestROS2HeadSyncImport:
    """Test that the head sync module imports and degrades gracefully."""

    def test_import_without_ros2(self):
        """Module imports cleanly even without ROS2."""
        from teleop_system.modules.camera.ros2_head_sync import ROS2HeadSync

    def test_start_fails_without_ros2(self):
        """start() returns False when ROS2 is unavailable."""
        from teleop_system.modules.camera import ros2_head_sync as mod

        if not mod._ROS2_AVAILABLE:
            from teleop_system.interfaces.camera_stream import ICameraStream, RGBDFrame

            class DummyCam(ICameraStream):
                def initialize(self) -> bool:
                    return True

                def get_rgbd(self) -> RGBDFrame:
                    return RGBDFrame()

                def set_orientation(self, pan, tilt):
                    pass

                def get_orientation(self):
                    return (0.0, 0.0)

                def get_intrinsics(self):
                    return np.eye(3)

                def is_connected(self) -> bool:
                    return True

                def shutdown(self):
                    pass

            sync = mod.ROS2HeadSync(camera=DummyCam())
            assert sync.start() is False

    def test_stop_is_safe_before_start(self):
        """stop() can be called without start()."""
        from teleop_system.modules.camera.ros2_head_sync import ROS2HeadSync
        from teleop_system.interfaces.camera_stream import ICameraStream, RGBDFrame

        class DummyCam(ICameraStream):
            def initialize(self) -> bool:
                return True

            def get_rgbd(self) -> RGBDFrame:
                return RGBDFrame()

            def set_orientation(self, pan, tilt):
                pass

            def get_orientation(self):
                return (0.0, 0.0)

            def get_intrinsics(self):
                return np.eye(3)

            def is_connected(self) -> bool:
                return True

            def shutdown(self):
                pass

        sync = ROS2HeadSync(camera=DummyCam())
        sync.stop()  # should not raise


# ── MuJoCo bridge camera publishing tests ──


class TestBridgeCameraPublishing:
    """Test that the bridge's camera parameters are declared correctly."""

    def test_bridge_import(self):
        """Bridge module imports (ROS2 may or may not be available)."""
        from teleop_system.simulators import mujoco_ros2_bridge as mod

        # The module should always import, but the class only exists with ROS2
        assert hasattr(mod, "_ROS2_AVAILABLE")

    def test_topic_names_exist(self):
        """New camera topic name constants exist in TopicNames."""
        from teleop_system.utils.ros2_helpers import TopicNames

        assert hasattr(TopicNames, "CAMERA_COLOR_IMAGE")
        assert hasattr(TopicNames, "CAMERA_DEPTH_IMAGE")
        assert hasattr(TopicNames, "CAMERA_INFO")
        assert TopicNames.CAMERA_COLOR_IMAGE == "/slave/camera/color/image_raw"
        assert TopicNames.CAMERA_DEPTH_IMAGE == "/slave/camera/depth/image_raw"
        assert TopicNames.CAMERA_INFO == "/slave/camera/camera_info"
