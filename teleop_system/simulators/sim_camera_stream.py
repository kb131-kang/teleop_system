"""Simulated camera stream wrapping MuJoCo head camera.

Implements ICameraStream by rendering RGB-D from MuJoCo's head_camera
and controlling the head pan-tilt joints via MuJoCo ctrl.
"""

import numpy as np

from teleop_system.interfaces.camera_stream import ICameraStream, RGBDFrame
from teleop_system.utils.logger import get_logger

logger = get_logger("sim_camera_stream")


class SimCameraStream(ICameraStream):
    """MuJoCo adapter for head camera RGB-D and pan-tilt control.

    Wraps MuJoCoSimulator to:
    - Render RGB-D from the named camera via get_camera_rgbd()
    - Control head pan/tilt via ctrl[22:24]
    - Compute proper intrinsics from MuJoCo camera fovy + render dimensions
    """

    def __init__(
        self,
        simulator,
        camera_name: str = "head_camera",
        pan_ctrl_index: int = 22,
        tilt_ctrl_index: int = 23,
        pan_range: tuple[float, float] = (-0.523, 0.523),
        tilt_range: tuple[float, float] = (-0.35, 1.57),
    ):
        """Initialize simulated camera stream.

        Args:
            simulator: MuJoCoSimulator instance (must be initialized with render=True).
            camera_name: Name of the camera in the MuJoCo model.
            pan_ctrl_index: ctrl index for head pan joint (head_0).
            tilt_ctrl_index: ctrl index for head tilt joint (head_1).
            pan_range: (min, max) pan angle in radians.
            tilt_range: (min, max) tilt angle in radians.
        """
        self._sim = simulator
        self._camera_name = camera_name
        self._pan_ctrl = pan_ctrl_index
        self._tilt_ctrl = tilt_ctrl_index
        self._pan_range = pan_range
        self._tilt_range = tilt_range
        self._initialized = False
        self._intrinsics: np.ndarray | None = None

        # Double buffering for non-blocking reads
        self._front_frame: RGBDFrame = RGBDFrame()
        self._back_frame: RGBDFrame = RGBDFrame()

    def initialize(self) -> bool:
        if not self._sim._initialized:
            logger.error("MuJoCoSimulator not initialized")
            return False

        if self._sim._renderer is None:
            logger.error("MuJoCoSimulator renderer not available (initialize with render=True)")
            return False

        self._intrinsics = self._compute_intrinsics()
        self._initialized = True
        logger.info(
            f"SimCameraStream initialized: camera='{self._camera_name}', "
            f"pan=[{self._pan_range[0]:.2f}, {self._pan_range[1]:.2f}], "
            f"tilt=[{self._tilt_range[0]:.2f}, {self._tilt_range[1]:.2f}]"
        )
        return True

    def get_rgbd(self) -> RGBDFrame:
        if not self._initialized:
            return RGBDFrame()

        # Render into back buffer
        self._back_frame = self._sim.get_camera_rgbd(self._camera_name)

        # Swap front/back (atomic reference swap under GIL)
        self._front_frame, self._back_frame = self._back_frame, self._front_frame
        return self._front_frame

    def set_orientation(self, pan: float, tilt: float) -> None:
        if not self._initialized:
            return

        # Clamp to joint ranges
        pan = float(np.clip(pan, self._pan_range[0], self._pan_range[1]))
        tilt = float(np.clip(tilt, self._tilt_range[0], self._tilt_range[1]))

        self._sim._data.ctrl[self._pan_ctrl] = pan
        self._sim._data.ctrl[self._tilt_ctrl] = tilt

    def get_orientation(self) -> tuple[float, float]:
        if not self._initialized:
            return (0.0, 0.0)
        pan = float(self._sim._data.ctrl[self._pan_ctrl])
        tilt = float(self._sim._data.ctrl[self._tilt_ctrl])
        return (pan, tilt)

    def get_intrinsics(self) -> np.ndarray:
        if self._intrinsics is not None:
            return self._intrinsics.copy()
        return np.eye(3)

    def is_connected(self) -> bool:
        return self._initialized and self._sim._initialized

    def shutdown(self) -> None:
        self._initialized = False
        logger.info("SimCameraStream shutdown")

    def _compute_intrinsics(self) -> np.ndarray:
        """Compute camera intrinsic matrix from MuJoCo fovy.

        MuJoCo uses vertical field of view. Given:
          fovy_rad = fovy * pi / 180
          fy = height / (2 * tan(fovy_rad / 2))
          fx = fy  (square pixels)
          cx = width / 2
          cy = height / 2
        """
        import mujoco

        cam_id = mujoco.mj_name2id(
            self._sim._model, mujoco.mjtObj.mjOBJ_CAMERA, self._camera_name
        )
        if cam_id < 0:
            logger.warning(f"Camera '{self._camera_name}' not found, using identity intrinsics")
            return np.eye(3)

        fovy_deg = self._sim._model.cam_fovy[cam_id]
        fovy_rad = np.deg2rad(fovy_deg)

        width = self._sim._renderer._width
        height = self._sim._renderer._height

        fy = height / (2.0 * np.tan(fovy_rad / 2.0))
        fx = fy
        cx = width / 2.0
        cy = height / 2.0

        return np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0],
        ])
