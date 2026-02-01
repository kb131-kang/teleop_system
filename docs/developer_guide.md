# RB-Y1 Teleoperation System — Developer Guide

## Table of Contents

1. [Architecture](#1-architecture)
2. [Directory Structure](#2-directory-structure)
3. [Interface Contracts (ABCs)](#3-interface-contracts-abcs)
4. [Adding a New Module](#4-adding-a-new-module)
5. [Coordinate Conventions](#5-coordinate-conventions)
6. [IK Solver (Pink/Pinocchio)](#6-ik-solver-pinkpinocchio)
7. [MuJoCo Integration](#7-mujoco-integration)
8. [ROS2 Patterns](#8-ros2-patterns)
9. [Testing](#9-testing)
10. [Hardware Integration](#10-hardware-integration)
11. [Motion Capture Replay Pipeline](#11-motion-capture-replay-pipeline)

---

## 1. Architecture

### Design Principles

The codebase follows an **interface-driven architecture** using Python ABCs (Abstract Base Classes). Every hardware-dependent component has:

1. An abstract interface in `teleop_system/interfaces/`
2. One or more concrete implementations (real hardware in `devices/`, simulation in `simulators/`)

This lets the full test suite run without any hardware, SDK, or ROS2 installed.

### Layered Structure

```
┌───────────────────────────────────────────────────────────────────┐
│                        ROS2 Launch Layer                          │
│  launch/*.launch.py — orchestrates nodes, passes parameters      │
├───────────────────────────────────────────────────────────────────┤
│                        ROS2 Node Layer                            │
│  modules/*/ros2_adapters.py — topic subscriptions, publishing     │
│  modules/*/*_node.py — lifecycle nodes wrapping controllers       │
├───────────────────────────────────────────────────────────────────┤
│                     Controller Layer (pure Python)                 │
│  modules/*/arm_controller.py, hand_controller.py, etc.            │
│  No ROS2 dependency — testable standalone                         │
├───────────────────────────────────────────────────────────────────┤
│                      Interface Layer (ABCs)                        │
│  interfaces/master_device.py — IMasterTracker, IHandInput         │
│  interfaces/slave_robot.py — ISlaveArm, ISlaveHand, IMobileBase   │
│  interfaces/ik_solver.py — IIKSolver                              │
│  interfaces/simulator.py — ISimulator                             │
│  interfaces/camera_stream.py — ICameraStream                      │
├───────────────────────────────────────────────────────────────────┤
│                    Implementation Layer                            │
│  devices/ — ViveTracker, ManusGlove, RBY1Arm, DG5FHand, ...      │
│  simulators/ — MuJoCoSimulator, SimulatedTracker, SimulatedHand   │
│  solvers/ — PinkIKSolver, ProportionalMapper                     │
├───────────────────────────────────────────────────────────────────┤
│                      Utility Layer                                │
│  utils/transforms.py — coordinate transforms (pure numpy)         │
│  utils/ros2_helpers.py — topic names, QoS profiles                │
│  utils/config_loader.py — YAML loading via Hydra/OmegaConf       │
│  utils/logger.py — logging                                        │
└───────────────────────────────────────────────────────────────────┘
```

### Data Flow (Arm Teleop Example)

```
ViveTracker.get_pose()                    # IMasterTracker → Pose6D
    │
    ▼
ROS2 Adapter publishes PoseStamped        # ros2_adapters.py
    │
    ▼
ArmTeleopNode receives PoseStamped        # arm_teleop_node.py
    │
    ▼
ArmController.update(tracker_poses)       # arm_controller.py (no ROS2 dep)
    │
    ├── transforms.steamvr_to_ros2()      # Coordinate conversion
    │
    ├── PinkIKSolver.solve_multi()        # Multi-chain IK → joint angles
    │
    ▼
ROS2 Adapter publishes JointState         # ros2_adapters.py
    │
    ▼
MuJoCoROS2Bridge receives JointState      # mujoco_ros2_bridge.py
    │
    ▼
MuJoCoSimulator.set_joint_positions()     # mujoco_sim.py → ctrl[indices]
    │
    ▼
mujoco.mj_step()                          # Physics simulation
```

---

## 2. Directory Structure

```
teleop_system/
├── config/                        # YAML configuration files
│   ├── default.yaml               #   System-wide defaults
│   ├── teleop/                    #   Per-module teleop parameters
│   │   ├── arm.yaml               #     IK solver, joint limits, scaling
│   │   ├── hand.yaml              #     Retargeting, finger mapping
│   │   └── locomotion.yaml        #     Gait detection, velocity mapping
│   ├── hardware/                  #   Device-specific configs
│   │   ├── vive_tracker.yaml      #     Tracker serial numbers, auto-detect
│   │   ├── manus_glove.yaml       #     Glove SDK settings
│   │   ├── rby1.yaml              #     Robot connection, URDF paths
│   │   └── dg5f.yaml              #     Hand connection, joint limits
│   └── simulation/                #   Simulator configs
│       ├── mujoco.yaml            #     MuJoCo timestep, render, solver
│       └── isaac_lab.yaml         #     Isaac Lab version, scene setup
│
├── docs/                          # Documentation
│   ├── user_guide.md              #   End-user guide
│   ├── developer_guide.md         #   Developer/architecture guide
│   ├── PRD.md                     #   Product Requirements Document
│   ├── TRD.md                     #   Technical Requirements Document
│   └── Tasks.md                   #   Task tracking / development log
│
├── launch/                        # ROS2 launch files
│   ├── slave_mujoco.launch.py     #   MuJoCo bridge only
│   ├── master_sim.launch.py       #   Dummy publishers + teleop nodes
│   ├── teleop_sim_full.launch.py  #   Combined (includes both above)
│   ├── teleop_mujoco_bridge.launch.py  # Legacy combined launcher
│   ├── teleop_sim.launch.py       #   Basic simulation
│   ├── teleop_full.launch.py      #   Full hardware
│   ├── arm_only.launch.py         #   Arm module only
│   └── hand_only.launch.py        #   Hand module only
│
├── models/                        # Robot models
│   ├── rby1/
│   │   ├── model_teleop.xml       #   MuJoCo MJCF (26 actuators)
│   │   └── rby1.urdf              #   URDF for Pinocchio
│   └── dg5f/
│       └── dg5f.urdf              #   DG-5F hand URDF
│
├── scripts/                       # Runnable scripts (no colcon install needed)
│   ├── run_teleop.py              #   Main entry point
│   ├── launch_all.py              #   Launch all nodes (Python, no ros2 launch)
│   ├── launch_slave.py            #   Launch slave only
│   ├── launch_master.py           #   Launch master only
│   ├── demo_teleop_sim.py         #   Step-by-step MuJoCo demo
│   ├── demo_rgbd_streaming.py     #   RGB-D streaming (TCP + ROS2)
│   ├── demo_pointcloud_viewer.py  #   Point cloud visualization
│   └── test_*_standalone.py       #   Per-module standalone tests
│
├── teleop_system/                 # Main Python package
│   ├── interfaces/                #   Abstract base classes
│   │   ├── master_device.py       #     IMasterTracker, IHandInput
│   │   ├── slave_robot.py         #     ISlaveArm, ISlaveHand, IMobileBase
│   │   ├── ik_solver.py           #     IIKSolver
│   │   ├── simulator.py           #     ISimulator
│   │   └── camera_stream.py       #     ICameraStream, RGBDFrame
│   │
│   ├── devices/                   #   Real hardware implementations
│   │   ├── vive_tracker.py        #     HTC Vive Tracker (OpenVR)
│   │   ├── vive_tracker_pub.py    #     Tracker → ROS2 publisher node
│   │   ├── manus_glove.py         #     Manus VR Glove (MANUS SDK)
│   │   ├── rby1_arm.py            #     RB-Y1 robot arm (SDK)
│   │   ├── dg5f_hand.py           #     DG-5F dexterous hand
│   │   └── realsense_camera.py    #     Intel RealSense (ROS2)
│   │
│   ├── simulators/                #   Simulation implementations
│   │   ├── mujoco_sim.py          #     MuJoCoSimulator (ISimulator)
│   │   ├── mujoco_ros2_bridge.py  #     ROS2 bridge node for MuJoCo
│   │   ├── sim_camera_stream.py   #     SimCameraStream (ICameraStream)
│   │   ├── simulated_tracker.py   #     SimulatedTracker (IMasterTracker)
│   │   ├── simulated_hand.py      #     SimulatedHand (IHandInput)
│   │   ├── dummy_tracker_pub.py   #     Dummy tracker → ROS2 publisher
│   │   ├── dummy_glove_pub.py     #     Dummy glove → ROS2 publisher
│   │   └── dummy_hmd_pub.py       #     Dummy HMD → ROS2 publisher
│   │
│   ├── modules/                   #   Teleop processing modules
│   │   ├── arm_teleop/
│   │   │   ├── arm_teleop_node.py    # ROS2 lifecycle node
│   │   │   ├── arm_controller.py     # Core logic (no ROS2 dep)
│   │   │   └── ros2_adapters.py      # ROS2 subscription/publishing
│   │   ├── locomotion/
│   │   │   ├── locomotion_node.py
│   │   │   ├── locomotion_controller.py
│   │   │   ├── gait_detector.py
│   │   │   └── ros2_adapters.py
│   │   ├── hand_teleop/
│   │   │   ├── hand_teleop_node.py
│   │   │   ├── hand_controller.py
│   │   │   ├── retargeting.py
│   │   │   └── ros2_adapters.py
│   │   └── camera/
│   │       ├── camera_node.py          # ROS2 lifecycle node
│   │       ├── camera_controller.py    # Core logic
│   │       ├── ros2_adapters.py        # ROS2 adapters
│   │       ├── ros2_rgbd_subscriber.py # ICameraStream via ROS2 topics
│   │       ├── ros2_head_sync.py       # Head orientation sync (ROS2 → TCP)
│   │       ├── rgbd_streaming.py       # TCP RGB-D streaming (codec, server, client)
│   │       ├── pointcloud_generator.py # Depth → 3D points conversion
│   │       └── pointcloud_viewer.py    # Open3D visualization
│   │
│   ├── solvers/                   #   IK solvers
│   │   ├── pink_ik_solver.py      #     Multi-chain IK (Pink + Pinocchio)
│   │   └── proportional_mapper.py #     Simple proportional fallback
│   │
│   ├── utils/                     #   Shared utilities
│   │   ├── transforms.py          #     Coordinate transforms (pure numpy)
│   │   ├── ros2_helpers.py        #     TopicNames, QoS profiles, utilities
│   │   ├── config_loader.py       #     Hydra/OmegaConf config loading
│   │   └── logger.py              #     Centralized logging
│   │
│   └── gui/                       #   Optional GUI
│       └── control_panel.py       #     DearPyGui control panel
│
├── tests/                         # Unit and integration tests
│   ├── test_interfaces.py         #   Data class validation
│   ├── test_transforms.py         #   Coordinate math
│   ├── test_config.py             #   Config loading
│   ├── test_simulation.py         #   Simulated devices
│   ├── test_arm_teleop.py         #   Arm controller logic
│   ├── test_phase3_multichain.py  #   Multi-chain IK
│   ├── test_phase4_locomotion_hand.py  # Gait + retargeting
│   ├── test_phase5_camera_gui.py  #   Camera + GUI
│   ├── test_phase5_camera_head.py #   Head tracking
│   ├── test_phase6_devices.py     #   Hardware drivers
│   ├── test_ros2_rgbd.py          #   ROS2 RGB-D subscriber
│   └── test_rgbd_streaming.py     #   Streaming codec
│
├── package.xml                    # ROS2 package metadata
├── setup.py                       # Python package setup
├── setup.cfg                      # Pytest + flake8 config
└── resource/teleop_system         # Ament index marker (empty)
```

---

## 3. Interface Contracts (ABCs)

All interfaces are in `teleop_system/interfaces/`. Each defines data classes and abstract methods that implementations must satisfy.

### IMasterTracker (`interfaces/master_device.py`)

Represents a 6DoF tracking device (VR tracker).

```python
class TrackerRole(Enum):
    RIGHT_HAND, LEFT_HAND, WAIST, RIGHT_FOOT, LEFT_FOOT, HEAD

@dataclass
class Pose6D:
    position: np.ndarray    # (3,) xyz in meters
    orientation: np.ndarray # (4,) quaternion (x, y, z, w)
    timestamp: float
    valid: bool

class IMasterTracker(ABC):
    def initialize(self) -> bool: ...
    def get_pose(self) -> Pose6D: ...
    def is_connected(self) -> bool: ...
    def get_role(self) -> TrackerRole: ...
    def shutdown(self) -> None: ...
```

**Implementations:** `ViveTracker` (OpenVR SDK), `SimulatedTracker` (sinusoidal oscillation)

### IHandInput (`interfaces/master_device.py`)

Represents a hand joint input device (glove).

```python
@dataclass
class HandJointState:
    joint_angles: np.ndarray      # (20,) radians
    finger_tip_positions: np.ndarray | None  # (5, 3) optional
    timestamp: float
    side: str  # "left" or "right"
    valid: bool

class IHandInput(ABC):
    def initialize(self) -> bool: ...
    def get_joint_state(self) -> HandJointState: ...
    def is_connected(self) -> bool: ...
    def get_side(self) -> str: ...
    def shutdown(self) -> None: ...
```

**Implementations:** `ManusGlove` (MANUS SDK), `SimulatedHand` (random angles)

### ISlaveArm (`interfaces/slave_robot.py`)

Controls a robot arm.

```python
class ArmSide(Enum):
    LEFT, RIGHT, TORSO

@dataclass
class JointState:
    positions: np.ndarray     # (N,) radians
    velocities: np.ndarray | None
    efforts: np.ndarray | None
    names: list[str]
    timestamp: float

class ISlaveArm(ABC):
    def initialize(self) -> bool: ...
    def send_joint_command(self, joint_positions: np.ndarray) -> None: ...
    def get_joint_state(self) -> JointState: ...
    def get_joint_count(self) -> int: ...
    def get_side(self) -> ArmSide: ...
    def is_connected(self) -> bool: ...
    def shutdown(self) -> None: ...
```

### ISlaveHand (`interfaces/slave_robot.py`)

Controls a robot hand/gripper.

```python
class ISlaveHand(ABC):
    def initialize(self) -> bool: ...
    def send_joint_command(self, joint_positions: np.ndarray) -> None: ...  # (20,) for DG-5F
    def get_joint_state(self) -> JointState: ...
    def get_joint_count(self) -> int: ...
    def get_side(self) -> str: ...
    def is_connected(self) -> bool: ...
    def shutdown(self) -> None: ...
```

### IMobileBase (`interfaces/slave_robot.py`)

Controls a wheeled mobile base.

```python
@dataclass
class VelocityCommand:
    linear_x: float   # forward (m/s)
    linear_y: float   # lateral (m/s)
    angular_z: float   # rotation (rad/s)
    timestamp: float

class IMobileBase(ABC):
    def initialize(self) -> bool: ...
    def send_velocity(self, linear_x, linear_y, angular_z) -> None: ...
    def get_odometry(self) -> tuple[np.ndarray, np.ndarray]: ...  # (pos, quat)
    def stop(self) -> None: ...
    def is_connected(self) -> bool: ...
    def shutdown(self) -> None: ...
```

### IIKSolver (`interfaces/ik_solver.py`)

Inverse kinematics solver.

```python
@dataclass
class IKResult:
    joint_positions: np.ndarray  # (N,) radians
    success: bool
    error_position: float   # meters
    error_orientation: float  # radians

class IIKSolver(ABC):
    def initialize(self, urdf_path: str, **kwargs) -> bool: ...
    def solve(self, target_pose: Pose6D, current_joints: np.ndarray, dt=0.01) -> IKResult: ...
    def solve_multi(self, target_poses: dict[str, Pose6D], current_joints: np.ndarray, dt=0.01) -> IKResult: ...
    def get_joint_names(self) -> list[str]: ...
    def get_joint_count(self) -> int: ...
    def set_posture_target(self, posture: np.ndarray) -> None: ...
```

**Implementations:** `PinkIKSolver` (Pink/Pinocchio QP-based), `ProportionalMapper` (direct mapping)

### ISimulator (`interfaces/simulator.py`)

Physics simulation backend.

```python
@dataclass
class SimState:
    joint_positions: np.ndarray   # (N,)
    joint_velocities: np.ndarray  # (N,)
    base_position: np.ndarray     # (3,)
    base_orientation: np.ndarray  # (4,) xyzw
    timestamp: float

class ISimulator(ABC):
    def initialize(self, config: dict) -> bool: ...
    def step(self, dt=None) -> None: ...
    def get_state(self) -> SimState: ...
    def set_joint_positions(self, positions: np.ndarray) -> None: ...
    def set_base_velocity(self, linear_x, linear_y, angular_z) -> None: ...
    def render(self) -> np.ndarray | None: ...  # (H,W,3) RGB
    def get_camera_rgbd(self, camera_name: str) -> RGBDFrame: ...
    def get_joint_names(self) -> list[str]: ...
    def get_time(self) -> float: ...
    def reset(self) -> None: ...
    def shutdown(self) -> None: ...
```

**Implementations:** `MuJoCoSimulator` (MuJoCo 3.4), Isaac Lab wrapper

### ICameraStream (`interfaces/camera_stream.py`)

RGB-D camera with pan-tilt control.

```python
@dataclass
class RGBDFrame:
    rgb: np.ndarray        # (H,W,3) uint8
    depth: np.ndarray      # (H,W) float32, meters
    intrinsics: np.ndarray # (3,3) camera K matrix
    extrinsics: np.ndarray # (4,4) camera-to-world
    timestamp: float
    width: int = 640
    height: int = 480

class ICameraStream(ABC):
    def initialize(self) -> bool: ...
    def get_rgbd(self) -> RGBDFrame: ...
    def set_orientation(self, pan: float, tilt: float) -> None: ...
    def get_orientation(self) -> tuple[float, float]: ...
    def get_intrinsics(self) -> np.ndarray: ...
    def is_connected(self) -> bool: ...
    def shutdown(self) -> None: ...
```

**Implementations:** `RealSenseCamera` (ROS2 topics), `SimCameraStream` (MuJoCo render), `ROS2RGBDSubscriber` (ROS2 Image topics)

---

## 4. Adding a New Module

### Example: Adding a "Gaze Teleop" Module

**Step 1: Define the interface (if new)**

If your module needs a new device type, add an ABC to `interfaces/`. For gaze tracking, `IMasterTracker` with `TrackerRole.HEAD` already covers this, so no new interface is needed.

**Step 2: Create the module directory**

```
teleop_system/modules/gaze_teleop/
├── __init__.py
├── gaze_controller.py      # Core logic (no ROS2 dependency)
├── gaze_teleop_node.py     # ROS2 lifecycle node
└── ros2_adapters.py        # ROS2 topic adapters
```

**Step 3: Implement the controller (pure Python)**

```python
# gaze_controller.py
import numpy as np

class GazeController:
    def __init__(self, smoothing_alpha: float = 0.3):
        self._alpha = smoothing_alpha
        self._current_gaze = np.zeros(2)

    def update(self, target_pan: float, target_tilt: float, dt: float) -> tuple[float, float]:
        target = np.array([target_pan, target_tilt])
        self._current_gaze += self._alpha * (target - self._current_gaze)
        return float(self._current_gaze[0]), float(self._current_gaze[1])
```

**Step 4: Create the ROS2 node**

```python
# gaze_teleop_node.py
from teleop_system.utils.logger import get_logger
logger = get_logger("gaze_teleop")

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import QuaternionStamped
    from sensor_msgs.msg import JointState
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False

# ... node implementation using lazy ROS2 imports
```

**Step 5: Add entry point in setup.py**

```python
"gaze_teleop_node=teleop_system.modules.gaze_teleop.gaze_teleop_node:main",
```

**Step 6: Add to launch files and write tests**

Key rules:
- Controller must be testable without ROS2
- Use lazy imports for all ROS2 dependencies
- Follow the `try/except ImportError` + `_ROS2_AVAILABLE` pattern
- Use QoS profiles from `utils/ros2_helpers.py`
- Add topic names to `TopicNames` class

---

## 5. Coordinate Conventions

All coordinate transforms are centralized in `teleop_system/utils/transforms.py`. **No other module should perform frame conversions directly.**

### Two Coordinate Systems

| Property | ROS2 / URDF | SteamVR (OpenVR) |
|----------|-------------|------------------|
| Forward | +X | -Z |
| Left | +Y | -X |
| Up | +Z | +Y |
| Quaternion order | (x, y, z, w) | (w, x, y, z) |
| Handedness | Right-handed | Right-handed |

### Conversion Functions

```python
from teleop_system.utils.transforms import (
    steamvr_to_ros2_position,    # SteamVR xyz → ROS2 xyz
    steamvr_to_ros2_quaternion,  # SteamVR wxyz → ROS2 xyzw
    ros2_to_steamvr_position,    # ROS2 xyz → SteamVR xyz
    ros2_to_steamvr_quaternion,  # ROS2 xyzw → SteamVR wxyz
    quat_multiply,               # Hamilton product
    quat_inverse,                # Conjugate / inverse
    quat_to_euler,               # Quaternion → Euler (sxyz only)
    euler_to_quat,               # Euler → Quaternion (sxyz only)
    pose_to_homogeneous,         # Pose6D → 4x4 matrix
    homogeneous_to_pose,         # 4x4 matrix → position + quaternion
)
```

### Important Notes

- `transforms3d` is listed in `setup.py` but **not used at runtime** — `transforms.py` implements everything with pure numpy.
- The `axes` parameter in `quat_to_euler` / `euler_to_quat` only supports the `"sxyz"` convention.
- Quaternions are always `(x, y, z, w)` internally (ROS2 convention). Convert at the boundary (VR input, visualization output).

---

## 6. IK Solver (Pink/Pinocchio)

### Multi-Chain IK Architecture

`PinkIKSolver` in `solvers/pink_ik_solver.py` handles simultaneous IK for multiple kinematic chains using Pink's weighted `FrameTask` approach.

```python
from teleop_system.solvers.pink_ik_solver import PinkIKSolver, ChainConfig

solver = PinkIKSolver()
solver.initialize(
    urdf_path="models/rby1/rby1.urdf",
    chains={
        "left_arm": ChainConfig(
            ee_frame="left_hand_link",
            position_weight=1.0,
            orientation_weight=0.5,
        ),
        "right_arm": ChainConfig(
            ee_frame="right_hand_link",
            position_weight=1.0,
            orientation_weight=0.5,
        ),
        "torso": ChainConfig(
            ee_frame="torso_link",
            position_weight=0.3,
            orientation_weight=0.2,
        ),
    },
)

# Solve for all chains simultaneously
result = solver.solve_multi(
    target_poses={
        "left_arm": left_hand_pose,
        "right_arm": right_hand_pose,
        "torso": waist_pose,
    },
    current_joints=current_q,
    dt=0.01,
)
```

### QP Solver Auto-Detection

The solver auto-detects available QP backends at import time, preferring the fastest:

1. `proxqp` (ProxSuite) — recommended, fastest
2. `osqp` — good alternative
3. `daqp` — fallback

### Null-Space Posture Regularization

When the robot is redundant (more joints than task DoF), the solver uses null-space projection to keep joints near a default posture without affecting task accuracy:

```python
solver.set_posture_target(default_joint_angles)
```

---

## 7. MuJoCo Integration

### Model Structure (model_teleop.xml)

The MuJoCo MJCF model has 26 actuators:

| ctrl index | Joint(s) | Description |
|-----------|----------|-------------|
| 0 | left_wheel | Left wheel velocity |
| 1 | right_wheel | Right wheel velocity |
| 2-7 | torso_0..5 | 6-DOF torso |
| 8-14 | right_arm_0..6 | 7-DOF right arm |
| 15-21 | left_arm_0..6 | 7-DOF left arm |
| 22-23 | head_0, head_1 | Head pan, tilt |
| 24 | right_gripper | Right gripper |
| 25 | left_gripper | Left gripper |

### MuJoCoSimulator (`simulators/mujoco_sim.py`)

Wraps `mujoco.MjModel` and `mujoco.MjData`:

```python
from teleop_system.simulators.mujoco_sim import MuJoCoSimulator

sim = MuJoCoSimulator()
sim.initialize({
    "mjcf_path": "models/rby1/model_teleop.xml",
    "timestep": 0.002,
    "render": True,  # Enable EGL rendering for camera
})

sim.step()
state = sim.get_state()        # SimState with all joint positions
frame = sim.get_camera_rgbd("head_camera")  # RGBDFrame
```

### MuJoCo ROS2 Bridge (`simulators/mujoco_ros2_bridge.py`)

This ROS2 node wraps the simulator and exposes it via ROS2 topics:

- **Subscribes** to joint command topics → writes to `data.ctrl`
- **Publishes** joint states at `publish_rate_hz`
- **Optionally publishes** camera images at `camera_fps`
- Runs physics in a timer callback at `physics_rate_hz`

### ROS2-Server Mode (Cross-Machine Camera Streaming)

For cross-machine setups, a `ros2-server` mode bridges ROS2 camera topics to TCP. This is critical because the standalone `--mode server` creates its own MuJoCo instance that does **not** participate in the ROS2 teleop pipeline — meaning the robot doesn't move in response to master commands.

The `ros2-server` mode solves this by subscribing to the bridge's camera topics (which reflect the real simulation state) and forwarding frames over TCP:

```
MuJoCo Bridge (ROS2 node)
    │ publishes /slave/camera/* topics
    ▼
ros2-server (ROS2RGBDSubscriber → RGBDStreamServer)
    │ TCP streaming (lz4 compressed)
    ▼
TCP client (point cloud viewer)
    │ sends head orientation via reverse channel
    ▼
ros2-server publishes /slave/camera/pan_tilt_cmd
    │
    ▼
MuJoCo Bridge moves head actuators (ctrl[22:23])
```

The `ROS2RGBDSubscriber` implements `ICameraStream`, making it a drop-in replacement for `SimCameraStream` in the streaming server. See `teleop_system/modules/camera/ros2_rgbd_subscriber.py`.

### Why ROS2 Internally + TCP Externally (Transport Layer Design)

The camera streaming uses a **two-layer transport** pattern: ROS2 (DDS) for intra-machine communication, TCP for cross-machine streaming. This is a standard robotics pattern (used by `rosbridge_suite`, Foxglove Bridge, etc.) driven by the limitations of each transport:

**Why not ROS2-only for cross-machine?**

| Problem | Impact |
|---------|--------|
| No built-in compression | `sensor_msgs/Image` transmits raw pixels. 640×480 RGB @ 30fps = **~27 MB/s** uncompressed. WiFi saturates quickly |
| DDS discovery overhead | Multicast-based discovery doesn't traverse subnets without explicit DDS configuration (XML profiles, discovery servers) |
| NAT / firewall issues | DDS uses dynamic UDP ports. Corporate/cloud firewalls block this by default |

The TCP streaming layer solves these: JPEG + lz4 compression reduces bandwidth to **~250 KB/frame** (~8× reduction), and a single TCP socket simplifies firewall/NAT traversal.

**Why not TCP-only for everything?**

| Benefit of keeping ROS2 internally | Details |
|-------------------------------------|---------|
| Standard tooling | `rviz2`, `rqt_image_view`, `ros2 topic echo` can inspect camera data without custom code |
| Multiple subscribers | The same `/slave/camera/*` topics can be consumed by `ros2-viewer`, `ros2-server`, and `rviz2` simultaneously |
| Shared-memory transport | DDS on the same machine can use zero-copy shared memory, faster than TCP loopback |
| Ecosystem compatibility | Any ROS2 camera processing node (`image_proc`, `depth_image_proc`, SLAM) works out of the box |

**Summary of the layered approach:**

```
Same machine:   Bridge ──ROS2 topics──→ ros2-viewer      (direct, no bridge needed)
Cross machine:  Bridge ──ROS2 topics──→ ros2-server ──TCP──→ client
                         (internal bus)    (external transport, compressed)
```

This pattern is common in production robotics: Boston Dynamics Spot uses gRPC externally, surgical robots use dedicated video protocols externally, and autonomous vehicles use MQTT/gRPC for cloud connectivity — all while using ROS2 or similar middleware internally.

### EGL Thread Safety

MuJoCo's EGL rendering contexts are thread-local. The bridge uses a single-threaded executor, so all timer callbacks (physics, publishing, camera) run on the same thread. This is safe because the EGL context is created on the main thread and all rendering happens there.

If you need multi-threaded rendering, each thread must create its own `mujoco.Renderer` instance.

---

## 8. ROS2 Patterns

### Lazy Imports

All ROS2 code uses the lazy import pattern so the package works without ROS2:

```python
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
```

Every class checks `_ROS2_AVAILABLE` before using ROS2 APIs and returns `False` / logs a warning if unavailable.

### QoS Profiles

Defined in `utils/ros2_helpers.py`:

```python
from teleop_system.utils.ros2_helpers import QoSPreset, get_qos_profile

sensor_qos = get_qos_profile(QoSPreset.SENSOR_DATA)  # Best effort, volatile, depth=1
cmd_qos = get_qos_profile(QoSPreset.COMMAND)          # Reliable, volatile, depth=1
status_qos = get_qos_profile(QoSPreset.STATUS)        # Reliable, transient local, depth=10
```

Use `SENSOR_DATA` for high-frequency streams (trackers, cameras). Use `COMMAND` for joint commands and velocity.

### Topic Names

Centralized in `utils/ros2_helpers.py` `TopicNames` class:

```python
from teleop_system.utils.ros2_helpers import TopicNames

topic = TopicNames.RIGHT_TRACKER      # "/master/tracker/right"
topic = TopicNames.LEFT_ARM_CMD       # "/slave/arm/left/joint_cmd"
topic = TopicNames.CAMERA_COLOR_IMAGE # "/slave/camera/color/image_raw"
```

Always use `TopicNames` constants rather than hardcoding strings.

### Node Structure Pattern

Each module follows the same pattern:

```
module/
├── *_controller.py    # Pure Python logic (no ROS2)
├── *_node.py          # ROS2 node wrapping the controller
└── ros2_adapters.py   # ROS2 message conversion helpers
```

The controller is independently testable. The node only handles ROS2 lifecycle, subscriptions, and publishing.

---

## 9. Testing

### Test Organization

Tests are organized by development phase:

| Phase | Test Files | Requirements |
|-------|-----------|-------------|
| 1-2 | `test_interfaces.py`, `test_transforms.py`, `test_config.py` | numpy only |
| 3 | `test_phase3_multichain.py` | `pin`, `pink` |
| 4 | `test_phase4_locomotion_hand.py` | numpy, scipy |
| 5 | `test_phase5_camera_gui.py`, `test_phase5_camera_head.py` | mujoco (optional) |
| 6 | `test_phase6_devices.py` | Hardware SDKs (optional) |
| Streaming | `test_rgbd_streaming.py`, `test_ros2_rgbd.py` | lz4, pillow |

### Running Tests

```bash
# Full suite
python3 -m pytest tests/ -v

# With coverage
python3 -m pytest tests/ -v --cov=teleop_system --cov-report=html

# Skip tests that need unavailable packages
python3 -m pytest tests/ -v -k "not phase3"

# Single test
python3 -m pytest tests/test_transforms.py::TestQuaternionOperations -v
```

### Writing Tests

Follow these patterns:

1. **No ROS2 in tests** — Test controllers directly, not via ROS2 nodes.
2. **Mock hardware** — Use `SimulatedTracker`, `SimulatedHand`, or custom mocks.
3. **Graceful degradation** — Test that classes handle missing imports:

```python
def test_init_fails_gracefully_without_ros2(self):
    from teleop_system.modules.camera import ros2_rgbd_subscriber as mod
    if not mod._ROS2_AVAILABLE:
        sub = mod.ROS2RGBDSubscriber()
        assert sub.initialize() is False
```

4. **Data class tests** — Verify default construction and field access for all dataclasses.
5. **Transform tests** — Every new coordinate conversion needs roundtrip tests.

---

## 10. Hardware Integration

### Adding a New Device Driver

Follow the existing pattern in `devices/`:

1. **Implement the relevant interface** (e.g., `ICameraStream` for a new camera)
2. **Use lazy SDK imports:**

```python
try:
    import my_hardware_sdk
    _SDK_AVAILABLE = True
except ImportError:
    _SDK_AVAILABLE = False
    logger.warning("my_hardware SDK not available")
```

3. **Handle graceful degradation:**
   - `initialize()` returns `False` if SDK is missing
   - `is_connected()` returns `False`
   - Other methods return safe defaults (empty frames, zero poses)

4. **Add configuration** in `config/hardware/my_device.yaml`
5. **Add tests** that work without the SDK installed

### Current Hardware Stack

| Device | SDK | Interface | Config |
|--------|-----|-----------|--------|
| HTC Vive Tracker (5x) | PyOpenVR 2.12 | IMasterTracker | `hardware/vive_tracker.yaml` |
| Manus VR Glove (2x) | MANUS SDK 3.0 | IHandInput | `hardware/manus_glove.yaml` |
| RB-Y1 Robot | Rainbow SDK | ISlaveArm, IMobileBase | `hardware/rby1.yaml` |
| DG-5F Hand (2x) | DELTO_M_ROS2 | ISlaveHand | `hardware/dg5f.yaml` |
| Intel RealSense | ROS2 driver | ICameraStream | Topics in `ros2_helpers.py` |

### VR Tracker Mapping

The system uses 5 Vive Trackers mapped by role:

| Role | Body Part | Topic |
|------|-----------|-------|
| RIGHT_HAND | Right wrist | `/master/tracker/right` |
| LEFT_HAND | Left wrist | `/master/tracker/left` |
| WAIST | Torso/pelvis | `/master/tracker/waist` |
| RIGHT_FOOT | Right ankle | `/master/tracker/right_foot` |
| LEFT_FOOT | Left ankle | `/master/tracker/left_foot` |

Auto-detection assigns trackers based on their height at startup (configurable in `vive_tracker.yaml`).

### Manus Glove Joint Mapping

Each Manus glove provides 20 joint angles (4 joints x 5 fingers):

```
Finger order: thumb, index, middle, ring, pinky
Per finger:   [MCP_flex, MCP_abd, PIP_flex, DIP_flex]
```

The `retargeting.py` module maps these 20 angles to the DG-5F hand's 20 DOF via configurable scale factors and offsets (see `config/teleop/hand.yaml`).

---

## 11. Motion Capture Replay Pipeline

### Overview

The `teleop_system/mocap/` package provides a BVH motion capture replay pipeline for testing the teleop system with real human motion data instead of synthetic sinusoidal inputs.

### Architecture

```
BVH File (CMU, Y-up, custom units)
    |  bvh_loader.py (bvhio parse + coordinate convert)
    v
BVHData (ROS2 Z-up, meters, xyzw quaternion)
    |  skeleton_mapper.py (joint → TrackerRole mapping)
    v
MappedMotion (per-frame Pose6D for 6 tracker roles)
    |  bvh_tracker_adapter.py / bvh_hand_adapter.py
    v
IMasterTracker / IHandInput interfaces
    |  bvh_replay_publisher.py (ROS2 topics)
    v
Standard teleop nodes (arm_teleop, locomotion, hand_teleop)
```

### Coordinate Conversion

CMU BVH files use Y-up coordinates with custom units. Conversion to ROS2:

| BVH Axis | Direction | ROS2 Axis | Direction |
|----------|-----------|-----------|-----------|
| X | Lateral (left+) | Y | Left+ |
| Y | Up | Z | Up |
| Z | Forward | X | Forward |

Position: `ros2 = (bvh_z, bvh_x, bvh_y) * scale`

Quaternion (glm w,x,y,z → ROS2 x,y,z,w): `ros2 = (bvh_z, bvh_x, bvh_y, bvh_w)`

### Key Classes

| Class | File | Interface | Purpose |
|-------|------|-----------|---------|
| `BVHData` | `bvh_loader.py` | Dataclass | Parsed BVH frames in ROS2 coordinates |
| `SkeletonMapper` | `skeleton_mapper.py` | — | Maps BVH joints to TrackerRole Pose6D |
| `BVHTrackerAdapter` | `bvh_tracker_adapter.py` | `IMasterTracker` | Time-indexed BVH playback |
| `BVHHandAdapter` | `bvh_hand_adapter.py` | `IHandInput` | Finger motion from wrist angular velocity |
| `BVHReplayPub` | `bvh_replay_publisher.py` | ROS2 `Node` | Publishes BVH data on standard topics |
| `DataRecorder` | `data_recorder.py` | — | Records input/output to `.npz` |

### Normalization Modes

- **`relative`** (default): First frame aligns to robot reference positions (`SimulatedTracker` defaults). Motion deltas are preserved. Best for testing with the robot's workspace.
- **`absolute`**: Raw converted positions. Useful for analyzing BVH data directly.

### Adding Support for New BVH Datasets

1. Create a joint mapping YAML in `config/mocap/` (like `cmu_joint_mapping.yaml`)
2. Map your skeleton's joint names to the 6 TrackerRole joints
3. Set the appropriate `scale` factor (BVH units → meters)
4. If the BVH uses a different up-axis, modify the coordinate conversion in `bvh_loader.py`

### Metrics

The `metrics.py` module provides:
- **Tracking error**: Position/orientation RMSE between input and output
- **Velocity saturation**: % of frames where velocity hits limits
- **Workspace utilization**: Motion range coverage
- **Smoothness**: Dimensionless jerk metric (lower = smoother)
