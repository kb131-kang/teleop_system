"""MuJoCo simulation backend.

Implements ISimulator and provides ISlaveArm/IMobileBase adapters
for controlling the robot in MuJoCo physics simulation.
"""

from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from teleop_system.interfaces.camera_stream import RGBDFrame
from teleop_system.interfaces.simulator import ISimulator, SimState
from teleop_system.interfaces.slave_robot import (
    ArmSide,
    IMobileBase,
    ISlaveArm,
    ISlaveHand,
    JointState,
)
from teleop_system.utils.logger import get_logger

logger = get_logger("mujoco_sim")


class MuJoCoSimulator(ISimulator):
    """MuJoCo physics simulation backend.

    Loads a robot model (URDF/MJCF) and provides physics stepping,
    joint control, and rendering.
    """

    def __init__(self):
        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None
        self._renderer: mujoco.Renderer | None = None
        self._viewer = None
        self._initialized = False
        self._render_enabled = False
        self._joint_names: list[str] = []
        self._viewer_handle = None

    def initialize(self, config: dict) -> bool:
        """Initialize MuJoCo simulation.

        Args:
            config: Dict with keys:
                - urdf_path or mjcf_path: Model file path.
                - timestep: Physics timestep (default: 0.002).
                - render: Whether to enable rendering (default: True).
                - render_width, render_height: Offscreen render dimensions.
        """
        model_path = config.get("mjcf_path") or config.get("urdf_path")
        if model_path is None:
            logger.error("No model path provided (urdf_path or mjcf_path)")
            return False

        model_path = str(Path(model_path).resolve())
        try:
            self._model = mujoco.MjModel.from_xml_path(model_path)
            self._model.opt.timestep = config.get("timestep", 0.002)
            self._data = mujoco.MjData(self._model)

            # Collect joint names
            self._joint_names = []
            for i in range(self._model.njnt):
                name = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, i)
                if name:
                    self._joint_names.append(name)

            # Setup rendering
            self._render_enabled = config.get("render", True)
            if self._render_enabled:
                width = config.get("render_width", 1280)
                height = config.get("render_height", 720)
                self._renderer = mujoco.Renderer(self._model, width=width, height=height)

            self._initialized = True
            logger.info(
                f"MuJoCo initialized: {len(self._joint_names)} joints, "
                f"timestep={self._model.opt.timestep}"
            )
            return True

        except Exception as e:
            logger.error(f"MuJoCo initialization failed: {e}")
            return False

    def step(self, dt: float | None = None) -> None:
        if not self._initialized:
            return
        if dt is not None:
            self._model.opt.timestep = dt
        mujoco.mj_step(self._model, self._data)

    def get_state(self) -> SimState:
        if not self._initialized:
            return SimState()
        return SimState(
            joint_positions=self._data.qpos.copy(),
            joint_velocities=self._data.qvel.copy(),
            base_position=self._data.qpos[:3].copy() if self._data.qpos.size >= 3 else np.zeros(3),
            base_orientation=(
                self._data.qpos[3:7].copy()
                if self._data.qpos.size >= 7
                else np.array([0.0, 0.0, 0.0, 1.0])
            ),
            timestamp=self._data.time,
        )

    def set_joint_positions(self, positions: np.ndarray) -> None:
        if not self._initialized:
            return
        n = min(len(positions), self._model.nq)
        self._data.qpos[:n] = positions[:n]
        mujoco.mj_forward(self._model, self._data)

    def set_joint_controls(self, controls: np.ndarray) -> None:
        """Set actuator control values directly.

        Args:
            controls: (nu,) actuator control values.
        """
        if not self._initialized:
            return
        n = min(len(controls), self._model.nu)
        self._data.ctrl[:n] = controls[:n]

    def set_base_velocity(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        if not self._initialized:
            return
        # For a floating-base robot, set root velocities
        if self._data.qvel.size >= 6:
            self._data.qvel[0] = linear_x
            self._data.qvel[1] = linear_y
            self._data.qvel[5] = angular_z

    def render(self) -> np.ndarray | None:
        if not self._initialized or not self._render_enabled or self._renderer is None:
            return None
        self._renderer.update_scene(self._data)
        return self._renderer.render()

    def get_camera_rgbd(self, camera_name: str) -> RGBDFrame:
        if not self._initialized or self._renderer is None:
            return RGBDFrame()

        cam_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if cam_id < 0:
            logger.warning(f"Camera '{camera_name}' not found")
            return RGBDFrame()

        self._renderer.update_scene(self._data, camera=camera_name)
        rgb = self._renderer.render()

        # Depth rendering
        self._renderer.enable_depth_rendering()
        depth = self._renderer.render()
        self._renderer.disable_depth_rendering()

        # Compute intrinsics from MuJoCo camera fovy
        height, width = rgb.shape[0], rgb.shape[1]
        fovy_deg = self._model.cam_fovy[cam_id]
        fovy_rad = np.deg2rad(fovy_deg)
        fy = height / (2.0 * np.tan(fovy_rad / 2.0))
        fx = fy  # square pixels
        cx = width / 2.0
        cy = height / 2.0
        intrinsics = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0],
        ])

        # Compute camera-to-world extrinsics (in Y-up visualization frame).
        #
        # Chain: OpenCV camera → OpenGL camera → MuJoCo world (Z-up) → viewer world (Y-up)
        #
        # 1. cam_xmat: MuJoCo camera (OpenGL: Y-up, Z-backward) → MuJoCo world (Z-up)
        # 2. cv2gl: OpenCV (Y-down, Z-forward) → OpenGL (Y-up, Z-backward)
        # 3. zup2yup: MuJoCo Z-up → viewer Y-up
        cam_pos_mj = self._data.cam_xpos[cam_id].copy()       # (3,) Z-up world
        cam_rot_mj = self._data.cam_xmat[cam_id].reshape(3, 3).copy()  # GL-cam → Z-up world

        # OpenCV to OpenGL: flip Y and Z
        cv2gl = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=np.float64)

        # MuJoCo Z-up to viewer Y-up: X→X, Z→Y, Y→-Z
        zup2yup = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=np.float64)

        R_cv_to_viewer = zup2yup @ cam_rot_mj @ cv2gl
        t_viewer = zup2yup @ cam_pos_mj

        extrinsics = np.eye(4, dtype=np.float64)
        extrinsics[:3, :3] = R_cv_to_viewer
        extrinsics[:3, 3] = t_viewer

        return RGBDFrame(
            rgb=rgb,
            depth=depth,
            intrinsics=intrinsics,
            extrinsics=extrinsics,
            timestamp=self._data.time,
            width=width,
            height=height,
        )

    def get_joint_names(self) -> list[str]:
        return self._joint_names.copy()

    def get_time(self) -> float:
        return self._data.time if self._initialized else 0.0

    def reset(self) -> None:
        if self._initialized:
            mujoco.mj_resetData(self._model, self._data)
            mujoco.mj_forward(self._model, self._data)

    def launch_viewer(self) -> None:
        """Launch the interactive MuJoCo viewer (blocking)."""
        if self._initialized:
            mujoco.viewer.launch(self._model, self._data)

    def launch_passive_viewer(self):
        """Launch a passive viewer that can be updated manually."""
        if self._initialized:
            self._viewer_handle = mujoco.viewer.launch_passive(self._model, self._data)
            return self._viewer_handle
        return None

    def sync_viewer(self) -> None:
        """Sync passive viewer with current simulation state."""
        if self._viewer_handle is not None:
            self._viewer_handle.sync()

    def shutdown(self) -> None:
        if self._viewer_handle is not None:
            self._viewer_handle.close()
            self._viewer_handle = None
        self._renderer = None
        self._initialized = False
        logger.info("MuJoCo simulator shutdown")


class MuJoCoArm(ISlaveArm):
    """MuJoCo adapter for robot arm control via the simulator.

    Reads/writes joint positions through the MuJoCoSimulator.
    """

    def __init__(
        self,
        simulator: MuJoCoSimulator,
        side: ArmSide,
        joint_indices: list[int],
        joint_names: list[str] | None = None,
    ):
        """Initialize MuJoCo arm adapter.

        Args:
            simulator: Shared MuJoCoSimulator instance.
            side: Which arm chain (LEFT, RIGHT, TORSO).
            joint_indices: Indices into qpos/qvel for this arm's joints.
            joint_names: Optional joint names for this arm.
        """
        self._sim = simulator
        self._side = side
        self._joint_indices = joint_indices
        self._joint_names = joint_names or [f"joint_{i}" for i in joint_indices]

    def initialize(self) -> bool:
        return self._sim._initialized

    def send_joint_command(self, joint_positions: np.ndarray) -> None:
        if not self._sim._initialized:
            return
        for i, idx in enumerate(self._joint_indices):
            if i < len(joint_positions) and idx < self._sim._data.ctrl.size:
                self._sim._data.ctrl[idx] = joint_positions[i]

    def get_joint_state(self) -> JointState:
        if not self._sim._initialized:
            return JointState()
        positions = np.array([self._sim._data.qpos[i] for i in self._joint_indices])
        velocities = np.array([self._sim._data.qvel[i] for i in self._joint_indices])
        return JointState(
            positions=positions,
            velocities=velocities,
            names=self._joint_names,
            timestamp=self._sim._data.time,
        )

    def get_joint_count(self) -> int:
        return len(self._joint_indices)

    def get_side(self) -> ArmSide:
        return self._side

    def is_connected(self) -> bool:
        return self._sim._initialized

    def shutdown(self) -> None:
        pass  # Lifecycle managed by the simulator


class MuJoCoBase(IMobileBase):
    """MuJoCo adapter for mobile base control via the simulator."""

    def __init__(self, simulator: MuJoCoSimulator):
        self._sim = simulator

    def initialize(self) -> bool:
        return self._sim._initialized

    def send_velocity(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        self._sim.set_base_velocity(linear_x, linear_y, angular_z)

    def get_odometry(self) -> tuple[np.ndarray, np.ndarray]:
        if not self._sim._initialized:
            return np.zeros(3), np.array([0, 0, 0, 1.0])
        state = self._sim.get_state()
        return state.base_position, state.base_orientation

    def stop(self) -> None:
        self.send_velocity(0.0, 0.0, 0.0)

    def is_connected(self) -> bool:
        return self._sim._initialized

    def shutdown(self) -> None:
        self.stop()


class MuJoCoHand(ISlaveHand):
    """MuJoCo adapter for robot hand/gripper control via the simulator.

    Maps 20-DOF hand joint commands to a single MuJoCo actuator ctrl index
    by taking the mean of all joint positions (matching the MuJoCo ROS2 bridge
    gripper_cmd_callback behavior). Reads gripper finger qpos for state feedback.
    """

    def __init__(
        self,
        simulator: MuJoCoSimulator,
        side: str,
        ctrl_index: int,
        qpos_indices: list[int] | None = None,
        ctrl_scale: float = 20.0,
    ):
        """Initialize MuJoCo hand adapter.

        Args:
            simulator: Shared MuJoCoSimulator instance.
            side: 'left' or 'right'.
            ctrl_index: Index into ctrl array for this gripper actuator
                (right=24, left=25).
            qpos_indices: Optional qpos indices for gripper finger joints
                (for state readback). If None, get_joint_state returns zeros.
            ctrl_scale: Scale factor applied to mean joint angle before
                writing to ctrl (motor force). Default 20.0 matches
                demo_teleop_sim.py behavior.
        """
        self._sim = simulator
        self._side = side
        self._ctrl_index = ctrl_index
        self._qpos_indices = qpos_indices or []
        self._ctrl_scale = ctrl_scale
        self._last_command: np.ndarray | None = None

    def initialize(self) -> bool:
        return self._sim._initialized

    def send_joint_command(self, joint_positions: np.ndarray) -> None:
        if not self._sim._initialized:
            return
        scalar = float(np.mean(joint_positions)) * self._ctrl_scale
        self._sim._data.ctrl[self._ctrl_index] = scalar
        self._last_command = joint_positions.copy()

    def get_joint_state(self) -> JointState:
        if not self._sim._initialized:
            return JointState()
        if self._qpos_indices:
            positions = np.array([self._sim._data.qpos[i] for i in self._qpos_indices])
        else:
            positions = np.zeros(0)
        return JointState(
            positions=positions,
            timestamp=self._sim._data.time,
        )

    def get_joint_count(self) -> int:
        return 20

    def get_side(self) -> str:
        return self._side

    def is_connected(self) -> bool:
        return self._sim._initialized

    def shutdown(self) -> None:
        pass
