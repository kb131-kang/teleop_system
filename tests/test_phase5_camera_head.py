"""Phase 5 tests: Camera/head teleoperation module.

Tests SimCameraStream, CameraController, and head command pipeline.
All tests run without ROS2 or display (MuJoCo EGL rendering).
"""

import os

import numpy as np
import pytest

# Ensure headless rendering
if "MUJOCO_GL" not in os.environ:
    os.environ["MUJOCO_GL"] = "egl"

from pathlib import Path

from teleop_system.interfaces.camera_stream import ICameraStream, RGBDFrame
from teleop_system.interfaces.master_device import IMasterTracker, Pose6D, TrackerRole
from teleop_system.modules.camera.camera_controller import CameraController
from teleop_system.modules.camera.pointcloud_generator import PointCloudGenerator
from teleop_system.simulators.mujoco_sim import MuJoCoSimulator
from teleop_system.simulators.sim_camera_stream import SimCameraStream
from teleop_system.simulators.simulated_tracker import SimulatedTracker
from teleop_system.utils.transforms import euler_to_quat

PROJECT_ROOT = Path(__file__).resolve().parent.parent


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def mujoco_sim():
    """Create a MuJoCoSimulator with rendering enabled."""
    sim = MuJoCoSimulator()
    mjcf_path = str(PROJECT_ROOT / "models" / "rby1" / "model_teleop.xml")
    success = sim.initialize({
        "mjcf_path": mjcf_path,
        "timestep": 0.002,
        "render": True,
        "render_width": 64,
        "render_height": 48,
    })
    assert success, "MuJoCo initialization failed"
    yield sim
    sim.shutdown()


@pytest.fixture
def sim_camera(mujoco_sim):
    """Create a SimCameraStream wrapping MuJoCoSimulator."""
    cam = SimCameraStream(simulator=mujoco_sim)
    assert cam.initialize()
    yield cam
    cam.shutdown()


@pytest.fixture
def head_tracker():
    """Create a SimulatedTracker for HEAD role."""
    tracker = SimulatedTracker(
        role=TrackerRole.HEAD,
        amplitude=0.08,
        frequency=0.5,
    )
    tracker.initialize()
    yield tracker
    tracker.shutdown()


# ===========================================================================
# SimCameraStream tests
# ===========================================================================


class TestSimCameraStream:
    """Test SimCameraStream (ICameraStream implementation wrapping MuJoCo)."""

    def test_implements_interface(self, sim_camera):
        assert isinstance(sim_camera, ICameraStream)

    def test_initialize(self, sim_camera):
        assert sim_camera.is_connected()

    def test_get_rgbd_returns_valid_frame(self, sim_camera, mujoco_sim):
        # Step simulation forward to get a valid scene
        import mujoco
        mujoco.mj_forward(mujoco_sim._model, mujoco_sim._data)

        frame = sim_camera.get_rgbd()
        assert isinstance(frame, RGBDFrame)
        assert frame.rgb is not None
        assert frame.rgb.shape[2] == 3  # RGB channels
        assert frame.depth is not None
        assert frame.depth.ndim == 2

    def test_intrinsics_computed_from_fovy(self, sim_camera):
        intrinsics = sim_camera.get_intrinsics()
        assert intrinsics.shape == (3, 3)
        # Should NOT be identity (default)
        assert not np.allclose(intrinsics, np.eye(3))
        # fx, fy should be positive and reasonable
        fx = intrinsics[0, 0]
        fy = intrinsics[1, 1]
        assert fx > 0
        assert fy > 0
        # For fovy=60, small render: fy = height / (2 * tan(30deg))
        # 48 / (2 * 0.5774) = 41.6
        assert abs(fy - 41.6) < 1.0

    def test_set_orientation_clamps_to_range(self, sim_camera, mujoco_sim):
        # Set pan beyond range (+/- 0.523 rad)
        sim_camera.set_orientation(pan=2.0, tilt=5.0)
        pan, tilt = sim_camera.get_orientation()
        assert pan <= 0.523 + 1e-6
        assert tilt <= 1.57 + 1e-6

        sim_camera.set_orientation(pan=-2.0, tilt=-5.0)
        pan, tilt = sim_camera.get_orientation()
        assert pan >= -0.523 - 1e-6
        assert tilt >= -0.35 - 1e-6

    def test_get_orientation_reflects_set(self, sim_camera):
        sim_camera.set_orientation(0.1, 0.2)
        pan, tilt = sim_camera.get_orientation()
        assert abs(pan - 0.1) < 1e-6
        assert abs(tilt - 0.2) < 1e-6

    def test_orientation_affects_mujoco_ctrl(self, sim_camera, mujoco_sim):
        sim_camera.set_orientation(0.3, 0.5)
        assert abs(mujoco_sim._data.ctrl[22] - 0.3) < 1e-6
        assert abs(mujoco_sim._data.ctrl[23] - 0.5) < 1e-6

    def test_is_connected_after_init(self, sim_camera):
        assert sim_camera.is_connected()

    def test_not_connected_before_init(self, mujoco_sim):
        cam = SimCameraStream(simulator=mujoco_sim)
        assert not cam.is_connected()

    def test_shutdown(self, sim_camera):
        sim_camera.shutdown()
        assert not sim_camera.is_connected()


# ===========================================================================
# CameraController tests
# ===========================================================================


class TestCameraController:
    """Test CameraController (pure control logic, no ROS2)."""

    def test_disabled_returns_none(self, head_tracker, sim_camera):
        ctrl = CameraController(
            tracker=head_tracker,
            camera=sim_camera,
            dt=0.01,
        )
        # Controller starts disabled
        result = ctrl.update()
        assert result is None

    def test_enable_and_update(self, head_tracker, sim_camera):
        ctrl = CameraController(
            tracker=head_tracker,
            camera=sim_camera,
            smoothing_alpha=1.0,  # No smoothing for test
            max_angular_velocity=100.0,  # No velocity limit
            dt=0.01,
        )
        ctrl.enable()
        assert ctrl.is_enabled

        result = ctrl.update()
        assert result is not None
        pan, tilt = result
        assert isinstance(pan, float)
        assert isinstance(tilt, float)

    def test_hmd_orientation_to_pan_tilt(self, sim_camera):
        """Feed specific quaternion and verify correct pan/tilt extraction."""
        # Create a tracker-like mock that returns a known orientation
        class FixedTracker(IMasterTracker):
            def __init__(self, yaw, pitch):
                euler = np.array([0.0, pitch, yaw])
                self._orientation = euler_to_quat(euler)
                self._connected = True

            def initialize(self): return True
            def get_pose(self):
                return Pose6D(
                    position=np.array([0.0, 0.0, 1.6]),
                    orientation=self._orientation,
                    timestamp=0.0,
                    valid=True,
                )
            def is_connected(self): return self._connected
            def get_role(self): return TrackerRole.HEAD
            def shutdown(self): self._connected = False

        # Test: yaw=0.3, pitch=0.2
        tracker = FixedTracker(yaw=0.3, pitch=0.2)
        ctrl = CameraController(
            tracker=tracker,
            camera=sim_camera,
            smoothing_alpha=1.0,  # No smoothing
            max_angular_velocity=100.0,
            dt=0.01,
        )
        ctrl.enable()

        result = ctrl.update()
        assert result is not None
        pan, tilt = result
        # pan should be close to yaw (0.3), tilt close to pitch (0.2)
        assert abs(pan - 0.3) < 0.05
        assert abs(tilt - 0.2) < 0.05

    def test_smoothing(self, sim_camera):
        """Verify that smoothing reduces rapid orientation changes."""
        class StepTracker(IMasterTracker):
            def __init__(self):
                self._step = 0
            def initialize(self): return True
            def get_pose(self):
                self._step += 1
                # Jump from 0 to 0.5 at step 2
                yaw = 0.5 if self._step >= 2 else 0.0
                euler = np.array([0.0, 0.0, yaw])
                return Pose6D(
                    position=np.zeros(3),
                    orientation=euler_to_quat(euler),
                    timestamp=float(self._step),
                    valid=True,
                )
            def is_connected(self): return True
            def get_role(self): return TrackerRole.HEAD
            def shutdown(self): pass

        tracker = StepTracker()
        ctrl = CameraController(
            tracker=tracker,
            camera=sim_camera,
            smoothing_alpha=0.3,
            max_angular_velocity=100.0,  # No velocity limit
            dt=0.01,
        )
        ctrl.enable()

        # First update: pan ~= 0
        result1 = ctrl.update()
        assert result1 is not None
        pan1 = result1[0]

        # Second update: raw pan ~= 0.5, but smoothed from 0
        result2 = ctrl.update()
        assert result2 is not None
        pan2 = result2[0]

        # Smoothed value should be between 0 and 0.5
        assert pan2 > pan1
        assert pan2 < 0.5

    def test_velocity_limiting(self, sim_camera):
        """Verify large instantaneous change is limited."""
        class JumpTracker(IMasterTracker):
            def __init__(self):
                self._step = 0
            def initialize(self): return True
            def get_pose(self):
                self._step += 1
                yaw = 1.0 if self._step >= 2 else 0.0
                euler = np.array([0.0, 0.0, yaw])
                return Pose6D(
                    position=np.zeros(3),
                    orientation=euler_to_quat(euler),
                    timestamp=float(self._step),
                    valid=True,
                )
            def is_connected(self): return True
            def get_role(self): return TrackerRole.HEAD
            def shutdown(self): pass

        tracker = JumpTracker()
        ctrl = CameraController(
            tracker=tracker,
            camera=sim_camera,
            smoothing_alpha=1.0,  # No smoothing
            max_angular_velocity=2.0,  # 2 rad/s limit
            dt=0.01,  # -> max delta = 0.02 rad
        )
        ctrl.enable()

        # First update: pan ~= 0
        ctrl.update()

        # Second update: raw pan ~= 1.0, but limited to max_delta=0.02
        result = ctrl.update()
        assert result is not None
        pan = result[0]
        assert pan < 0.05  # Much less than the 1.0 jump


# ===========================================================================
# Integration tests: full pipeline
# ===========================================================================


class TestCameraHeadPipeline:
    """Integration test: full pipeline without ROS2.

    SimulatedTracker(HEAD) -> CameraController -> SimCameraStream
        -> MuJoCo physics -> get_camera_rgbd -> PointCloudGenerator
    """

    def test_end_to_end_pipeline(self, mujoco_sim, sim_camera, head_tracker):
        import mujoco

        ctrl = CameraController(
            tracker=head_tracker,
            camera=sim_camera,
            smoothing_alpha=0.5,
            max_angular_velocity=5.0,
            dt=0.01,
        )
        ctrl.enable()

        # Record initial ctrl values
        initial_pan = mujoco_sim._data.ctrl[22]
        initial_tilt = mujoco_sim._data.ctrl[23]

        # Run 50 update cycles with physics steps
        for _ in range(50):
            ctrl.update()
            for _ in range(5):
                mujoco_sim.step()

        # Verify ctrl values changed
        final_pan = mujoco_sim._data.ctrl[22]
        final_tilt = mujoco_sim._data.ctrl[23]
        assert final_pan != initial_pan or final_tilt != initial_tilt

        # Verify get_rgbd returns non-zero images
        frame = sim_camera.get_rgbd()
        assert frame.rgb.size > 0
        assert frame.depth.size > 0

    def test_rgbd_to_pointcloud(self, sim_camera, mujoco_sim):
        import mujoco
        mujoco.mj_forward(mujoco_sim._model, mujoco_sim._data)

        frame = sim_camera.get_rgbd()
        pcg = PointCloudGenerator(use_open3d=False, max_depth=100.0)
        result = pcg.generate(frame)

        assert result["count"] > 0
        assert result["points"].shape[1] == 3
        # Intrinsics should be properly passed through
        assert not np.allclose(frame.intrinsics, np.eye(3))

    def test_head_orientation_produces_different_images(self, sim_camera, mujoco_sim):
        import mujoco

        # Render at pan=0
        sim_camera.set_orientation(0.0, 0.5)
        for _ in range(20):
            mujoco.mj_step(mujoco_sim._model, mujoco_sim._data)
        frame1 = sim_camera.get_rgbd()

        # Render at pan=0.5 (30 degrees)
        sim_camera.set_orientation(0.5, 0.5)
        for _ in range(20):
            mujoco.mj_step(mujoco_sim._model, mujoco_sim._data)
        frame2 = sim_camera.get_rgbd()

        # RGB frames should differ
        diff = np.abs(frame1.rgb.astype(float) - frame2.rgb.astype(float))
        assert diff.sum() > 0, "Head orientation change should produce different images"


# ===========================================================================
# DummyHMDPub unit-level tests (no ROS2)
# ===========================================================================


class TestDummyHMDPubUnit:
    """Test DummyHMDPub components without ROS2."""

    def test_simulated_tracker_head_role(self):
        tracker = SimulatedTracker(role=TrackerRole.HEAD, frequency=0.5)
        tracker.initialize()
        assert tracker.get_role() == TrackerRole.HEAD
        assert tracker.is_connected()

        pose = tracker.get_pose()
        assert pose.valid
        assert pose.orientation.shape == (4,)
        # Quaternion should be normalized
        assert abs(np.linalg.norm(pose.orientation) - 1.0) < 1e-6
        tracker.shutdown()

    def test_orientation_extraction(self):
        """From a Pose6D with known orientation, extract the quaternion correctly."""
        from teleop_system.utils.transforms import quat_to_euler

        tracker = SimulatedTracker(role=TrackerRole.HEAD, frequency=0.5)
        tracker.initialize()

        pose = tracker.get_pose()
        euler = quat_to_euler(pose.orientation)
        # Euler angles should be reasonable (small oscillations)
        assert all(abs(e) < 1.0 for e in euler), f"Euler angles too large: {euler}"
        tracker.shutdown()

    def test_head_tracker_orientation_varies(self):
        """HEAD tracker orientation should vary over time."""
        import time

        tracker = SimulatedTracker(
            role=TrackerRole.HEAD,
            frequency=10.0,  # Fast for testing
        )
        tracker.initialize()

        poses = []
        for _ in range(5):
            poses.append(tracker.get_pose().orientation.copy())
            time.sleep(0.01)

        # At least some orientations should differ
        diffs = [np.linalg.norm(poses[i] - poses[0]) for i in range(1, len(poses))]
        assert max(diffs) > 1e-6, "Orientation should vary over time"
        tracker.shutdown()
