# RB-Y1 Teleoperation System — User Guide

## Table of Contents

1. [Overview](#1-overview)
2. [Prerequisites](#2-prerequisites)
3. [Installation](#3-installation)
4. [Quick Start](#4-quick-start)
5. [Running Modes](#5-running-modes)
6. [Camera & RGB-D Streaming](#6-camera--rgb-d-streaming)
7. [ROS2 Topics Reference](#7-ros2-topics-reference)
8. [Configuration](#8-configuration)
9. [Standalone Testing (No ROS2)](#9-standalone-testing-no-ros2)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. Overview

This system maps VR input devices to a Rainbow Robotics RB-Y1 humanoid robot for teleoperation. The operator wears VR trackers and data gloves; the system translates their motions into robot commands in real time.

### System Architecture

```
┌─────────────────────────────────┐       ROS2 Topics       ┌──────────────────────────────────┐
│         MASTER (Operator)       │ ──────────────────────── │         SLAVE (Robot)             │
│                                 │                          │                                  │
│  5x Vive Trackers               │   /master/tracker/*      │  MuJoCo Simulation               │
│    ├─ Right/Left Hand (2x)     ├──────────────────────────►│    OR                            │
│    ├─ Waist (1x)               │   /master/hand/*/joints   │  Real RB-Y1 Robot                │
│    └─ Right/Left Foot (2x)    │                          │    ├─ 2x 7-DOF Arms             │
│                                 │   /master/hmd/orientation │    ├─ 6-DOF Torso               │
│  2x Manus Gloves               │                          │    ├─ 2x DG-5F Hands (20 DOF)   │
│    ├─ Left Hand (20 joints)    │                          │    ├─ Mobile Base (2 wheels)     │
│    └─ Right Hand (20 joints)   │                          │    └─ Pan-Tilt Head Camera       │
│                                 │   /slave/arm/*/joint_cmd │                                  │
│  1x HMD (head orientation)     │◄──────────────────────────│  Joint State Feedback            │
│                                 │   /slave/base/cmd_vel    │  Camera RGB-D                    │
└─────────────────────────────────┘   /slave/hand/*/joint_cmd└──────────────────────────────────┘
```

### Four Independent Teleop Modules

| Module | Input | Output | Description |
|--------|-------|--------|-------------|
| **Arm Teleop** | 3 VR trackers (hands + waist) | Arm + torso joint commands | Multi-chain IK (Pink/Pinocchio) maps tracker poses to 20 joints |
| **Locomotion** | 2 VR foot trackers | Base velocity (cmd_vel) | Gait detection converts foot movement to wheel velocity |
| **Hand Teleop** | 2 Manus gloves (20 joints each) | Gripper/hand commands | Retargeting maps human hand angles to DG-5F joints |
| **Camera Teleop** | HMD orientation | Head pan/tilt commands | Maps head orientation to camera mount servos |

---

## 2. Prerequisites

### Required

| Software | Version | Purpose |
|----------|---------|---------|
| Python | >= 3.10 | Runtime |
| NumPy | >= 1.26.0 | Numerical computation |
| SciPy | >= 1.12.0 | Scientific utilities |
| PyYAML | >= 6.0 | Configuration loading |
| Hydra / OmegaConf | >= 1.3.0 / 2.3.0 | Hierarchical config |

### Optional — by feature

| Feature | Packages | Install group |
|---------|----------|---------------|
| MuJoCo simulation | `mujoco >= 3.4.0` | `pip install -e ".[sim]"` |
| IK solver | `pin >= 2.7.0`, `pin-pink >= 3.4.0` | `pip install -e ".[ik]"` |
| VR tracker input | `openvr >= 2.12.1401` | `pip install -e ".[vr]"` |
| Point cloud viewer | `open3d >= 0.19.0` | `pip install -e ".[viz]"` |
| GUI control panel | `dearpygui >= 2.1.1` | `pip install -e ".[gui]"` |
| RGB-D streaming | `lz4 >= 4.0.0`, `pillow >= 10.0.0` | `pip install -e ".[streaming]"` |
| ROS2 integration | ROS2 Jazzy | System install |

### ROS2 (for launch files and inter-node communication)

```bash
# Install ROS2 Jazzy (Ubuntu 24.04)
# See: https://docs.ros.org/en/jazzy/Installation.html
sudo apt install ros-jazzy-desktop

# Required ROS2 packages (already in package.xml)
# rclpy, std_msgs, geometry_msgs, sensor_msgs, lifecycle_msgs
```

---

## 3. Installation

### Option A: pip install (standalone, no ROS2 launch)

```bash
cd teleop_system

# Core only
pip install -e .

# With simulation + IK + visualization
pip install -e ".[sim,ik,viz,streaming]"

# Everything
pip install -e ".[all]"
```

### Option B: colcon build (ROS2 workspace)

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# From your ROS2 workspace (e.g., ~/ros2_ws/)
cd ~/ros2_ws/src
ln -s /path/to/teleop_system .

# Build
cd ~/ros2_ws
colcon build --packages-select teleop_system --symlink-install

# Source the workspace
source install/setup.bash

# Verify launch files are discoverable
ros2 launch teleop_system --list
```

### Verify Installation

```bash
# Run the test suite (no ROS2 or hardware needed)
python3 -m pytest tests/ -v

# Check that MuJoCo model loads
python3 scripts/demo_teleop_sim.py --step 1
```

---

## 4. Quick Start

### Full Simulation in One Command

**With ROS2 launch:**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Start everything (MuJoCo + all dummy inputs + all teleop nodes)
MUJOCO_GL=glfw ros2 launch teleop_system teleop_sim_full.launch.py

# With MuJoCo viewer window
MUJOCO_GL=glfw ros2 launch teleop_system teleop_sim_full.launch.py launch_viewer:=true

# With camera publishing (RGB-D available on ROS2 topics)
MUJOCO_GL=glfw ros2 launch teleop_system teleop_sim_full.launch.py publish_camera:=true

# With camera publishing + point cloud viewer on the master side
MUJOCO_GL=glfw ros2 launch teleop_system teleop_sim_full.launch.py publish_camera:=true launch_camera_viewer:=true
```

**Without ROS2 launch (Python scripts):**

```bash
# Start everything
python3 scripts/launch_all.py

# With viewer
MUJOCO_GL=glfw python3 scripts/launch_all.py --launch-viewer

# With camera publishing
python3 scripts/launch_all.py --publish-camera

# With camera publishing + point cloud viewer
python3 scripts/launch_all.py --publish-camera --launch-camera-viewer
```

### Monitor System

```bash
# In another terminal, check active topics
ros2 topic list

# Monitor joint state feedback
ros2 topic echo /mujoco/joint_states

# Monitor arm commands
ros2 topic echo /slave/arm/right/joint_cmd

# Monitor base velocity
ros2 topic echo /slave/base/cmd_vel
```

---

## 5. Running Modes

### 5.1 Full Simulation (slave + master)

Runs MuJoCo physics with simulated VR inputs. No hardware needed.

```bash
# ROS2 launch
ros2 launch teleop_system teleop_sim_full.launch.py

# Python script
python3 scripts/launch_all.py
```

### 5.2 Slave Only (MuJoCo robot)

Runs only the MuJoCo simulation. Use this when connecting a real master system or testing the slave independently.

```bash
# ROS2 launch
ros2 launch teleop_system slave_mujoco.launch.py
ros2 launch teleop_system slave_mujoco.launch.py launch_viewer:=true publish_camera:=true

# With TCP streaming server (bridges ROS2 camera → TCP for remote viewers)
ros2 launch teleop_system slave_mujoco.launch.py publish_camera:=true launch_streaming:=true
ros2 launch teleop_system slave_mujoco.launch.py publish_camera:=true launch_streaming:=true streaming_port:=9876

# Python script
python3 scripts/launch_slave.py --launch-viewer
python3 scripts/launch_slave.py --publish-camera --launch-streaming --streaming-port 9876
```

The slave subscribes to command topics and publishes joint state feedback:
- Subscribes: `/slave/arm/*/joint_cmd`, `/slave/base/cmd_vel`, `/slave/hand/*/joint_cmd`, `/slave/camera/pan_tilt_cmd`
- Publishes: `/mujoco/joint_states`
- Optionally publishes: `/slave/camera/color/image_raw`, `/slave/camera/depth/image_raw`, `/slave/camera/camera_info`

When `launch_streaming` is enabled, a TCP streaming server starts alongside the MuJoCo bridge. This server subscribes to the ROS2 camera topics published by the bridge and streams them over TCP. Remote clients can connect to view the robot's camera in real time (see Section 6.1).

### 5.3 Master Only (simulated inputs)

Runs simulated VR inputs and teleop processing. Use this when connecting to a real robot or an external simulation.

```bash
# ROS2 launch
ros2 launch teleop_system master_sim.launch.py

# With custom tracker behavior
ros2 launch teleop_system master_sim.launch.py tracker_amplitude:=0.10 tracker_frequency:=0.5

# With auto-launched point cloud viewer (subscribes to ROS2 camera topics)
ros2 launch teleop_system master_sim.launch.py launch_viewer:=true

# Python script
python3 scripts/launch_master.py

# With point cloud viewer
python3 scripts/launch_master.py --launch-viewer
```

When `launch_viewer` is enabled, a point cloud viewer (`ros2-viewer` mode) launches alongside the master nodes. It subscribes to the slave's camera topics and displays the robot's first-person camera view. The view updates in real time as the robot's head moves.

### 5.4 Individual Modules

```bash
# Arm teleop only (with dummy tracker)
ros2 launch teleop_system arm_only.launch.py

# Hand teleop only (with dummy glove)
ros2 launch teleop_system hand_only.launch.py
```

### 5.5 Hardware Mode (real robot + real VR)

```bash
# Full hardware system
ros2 launch teleop_system teleop_full.launch.py robot_ip:=192.168.0.100

# Or via main entry point
python3 scripts/run_teleop.py --mode hardware
```

### 5.6 Mixed Mode (selective modules)

```bash
# Only arm and locomotion, no hand or camera
python3 scripts/run_teleop.py --modules arm locomotion --no-gui

# Simulation mode with specific backend
python3 scripts/run_teleop.py --mode simulation --sim-backend mujoco
```

---

## 6. Camera & RGB-D Streaming

The system supports four camera modes:

### 6.1 TCP Streaming (cross-machine, low-latency)

For streaming RGB-D from a slave machine to a remote master/viewer machine over the network.

**Mode A: Standalone server (creates its own MuJoCo instance, no ROS2 needed)**

Use this for quick standalone testing. Note that this MuJoCo instance does **not** receive teleop commands, so the robot won't move in response to master input.

```bash
# Server (standalone MuJoCo)
python3 scripts/demo_rgbd_streaming.py --mode server --host 0.0.0.0 --port 9999

# Client (viewer)
python3 scripts/demo_rgbd_streaming.py --mode client --host <server-ip> --port 9999
```

**Mode B: ROS2-backed server (bridges ROS2 camera topics → TCP)**

Use this for production cross-machine setups. The server subscribes to the MuJoCo bridge's ROS2 camera topics and forwards them over TCP. Since there is only one MuJoCo instance (the bridge), the camera view correctly reflects robot motion from teleop commands.

```bash
# On the slave machine: start bridge + streaming server
ros2 launch teleop_system slave_mujoco.launch.py publish_camera:=true launch_streaming:=true

# Or manually:
python3 scripts/demo_rgbd_streaming.py --mode ros2-server --host 0.0.0.0 --port 9876

# On the master machine: connect client viewer
python3 scripts/demo_rgbd_streaming.py --mode client --host <slave-ip> --port 9876

# Client with ROS2 head tracking sync
python3 scripts/demo_rgbd_streaming.py --mode client --host <slave-ip> --port 9876 --ros2-sync
```

The `--ros2-sync` flag subscribes to `/mujoco/joint_states` to read `head_0`/`head_1` values and sends them back to the server via the TCP reverse channel. This makes the streamed point cloud update as the robot's head moves.

**Cross-Machine Workflow Summary:**

```
Slave Machine                            Master Machine
┌──────────────────────────┐             ┌──────────────────────────┐
│ MuJoCo Bridge            │    TCP      │ TCP Client               │
│   (physics + camera)     │────────────►│   (point cloud viewer)   │
│        │                 │   port 9876 │        │                 │
│        ▼ ROS2 topics     │             │        │ --ros2-sync     │
│ ros2-server              │◄────────────│   head orientation       │
│   (ROS2 → TCP bridge)   │  reverse ch │   from /mujoco/          │
└──────────────────────────┘             │   joint_states           │
                                         └──────────────────────────┘
```

### 6.2 ROS2 Camera Publishing (same machine)

For environments where all nodes run on the same machine and ROS2 transport is preferred.

**Enable camera publishing on the MuJoCo bridge:**
```bash
ros2 launch teleop_system slave_mujoco.launch.py publish_camera:=true camera_fps:=30
```

**View the camera output:**
```bash
# Point cloud viewer via ROS2 topics
python3 scripts/demo_rgbd_streaming.py --mode ros2-viewer

# Or use standard ROS2 tools
ros2 topic echo /slave/camera/color/image_raw --no-arr
ros2 topic echo /slave/camera/camera_info
```

**Published topics:**
| Topic | Message Type | Content |
|-------|-------------|---------|
| `/slave/camera/color/image_raw` | `sensor_msgs/Image` (rgb8) | RGB color image |
| `/slave/camera/depth/image_raw` | `sensor_msgs/Image` (32FC1) | Depth in meters |
| `/slave/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics (K matrix) |

### 6.3 Direct Simulation Viewer

For local testing without networking or ROS2:

```bash
# Point cloud viewer with first-person camera
python3 scripts/demo_pointcloud_viewer.py

# Verify first-person view tracks head movement
python3 scripts/verify_first_person_view.py
```

### 6.4 Choosing the Right Camera Mode

| Scenario | Mode | Command |
|----------|------|---------|
| Local development, single machine | `ros2-viewer` | `--mode ros2-viewer` |
| Cross-machine with teleop sync | `ros2-server` + `client` | `--mode ros2-server` on slave, `--mode client` on master |
| Quick standalone camera test | `server` + `client` | `--mode server` (no teleop sync) |
| No ROS2, no networking | Direct viewer | `demo_pointcloud_viewer.py` |

---

## 7. ROS2 Topics Reference

### Master (Input) Topics

| Topic | Message Type | QoS | Rate | Description |
|-------|-------------|-----|------|-------------|
| `/master/tracker/right` | `geometry_msgs/PoseStamped` | SENSOR_DATA | 100 Hz | Right hand 6DoF pose |
| `/master/tracker/left` | `geometry_msgs/PoseStamped` | SENSOR_DATA | 100 Hz | Left hand 6DoF pose |
| `/master/tracker/waist` | `geometry_msgs/PoseStamped` | SENSOR_DATA | 100 Hz | Waist 6DoF pose |
| `/master/tracker/right_foot` | `geometry_msgs/PoseStamped` | SENSOR_DATA | 100 Hz | Right foot 6DoF pose |
| `/master/tracker/left_foot` | `geometry_msgs/PoseStamped` | SENSOR_DATA | 100 Hz | Left foot 6DoF pose |
| `/master/hmd/orientation` | `geometry_msgs/QuaternionStamped` | SENSOR_DATA | 90 Hz | HMD head quaternion |
| `/master/hand/left/joints` | `sensor_msgs/JointState` | SENSOR_DATA | 100 Hz | Left glove (20 joints) |
| `/master/hand/right/joints` | `sensor_msgs/JointState` | SENSOR_DATA | 100 Hz | Right glove (20 joints) |

### Slave (Output) Topics

| Topic | Message Type | QoS | Rate | Description |
|-------|-------------|-----|------|-------------|
| `/slave/arm/left/joint_cmd` | `sensor_msgs/JointState` | COMMAND | 100 Hz | Left arm (7 joints) |
| `/slave/arm/right/joint_cmd` | `sensor_msgs/JointState` | COMMAND | 100 Hz | Right arm (7 joints) |
| `/slave/torso/joint_cmd` | `sensor_msgs/JointState` | COMMAND | 100 Hz | Torso (6 joints) |
| `/slave/hand/left/joint_cmd` | `sensor_msgs/JointState` | COMMAND | 100 Hz | Left hand (20 joints) |
| `/slave/hand/right/joint_cmd` | `sensor_msgs/JointState` | COMMAND | 100 Hz | Right hand (20 joints) |
| `/slave/base/cmd_vel` | `geometry_msgs/Twist` | COMMAND | 50 Hz | Base velocity |
| `/slave/camera/pan_tilt_cmd` | `sensor_msgs/JointState` | COMMAND | 30 Hz | Head pan/tilt |

### System Topics

| Topic | Message Type | QoS | Description |
|-------|-------------|-----|-------------|
| `/mujoco/joint_states` | `sensor_msgs/JointState` | SENSOR_DATA | Simulation feedback (all joints) |
| `/slave/camera/color/image_raw` | `sensor_msgs/Image` | SENSOR_DATA | RGB camera (optional) |
| `/slave/camera/depth/image_raw` | `sensor_msgs/Image` | SENSOR_DATA | Depth camera (optional) |
| `/slave/camera/camera_info` | `sensor_msgs/CameraInfo` | SENSOR_DATA | Camera intrinsics (optional) |

### QoS Profiles

| Profile | Reliability | Durability | Depth | Use Case |
|---------|------------|------------|-------|----------|
| SENSOR_DATA | Best Effort | Volatile | 1 | Real-time sensor streams |
| COMMAND | Reliable | Volatile | 1 | Joint/velocity commands |
| STATUS | Reliable | Transient Local | 10 | System status, diagnostics |
| PARAMETER | Reliable | Transient Local | 1 | Parameter updates |

---

## 8. Configuration

Configuration files are in `config/` and loaded via Hydra/OmegaConf.

### Directory Structure

```
config/
├── default.yaml              # Top-level: mode, log_level, module enable/disable
├── teleop/
│   ├── arm.yaml              # IK solver params, joint limits, scaling
│   ├── hand.yaml             # Retargeting method, finger scale factors
│   └── locomotion.yaml       # Gait detection, velocity mapping, deadzone
├── hardware/
│   ├── vive_tracker.yaml     # Tracker serial mapping, auto-detect
│   ├── manus_glove.yaml      # Glove SDK config, data source
│   ├── rby1.yaml             # Robot IP, URDF, joint counts, control mode
│   └── dg5f.yaml             # Hand connection, joint limits
└── simulation/
    ├── mujoco.yaml           # Timestep, render, solver, model paths
    └── isaac_lab.yaml        # Isaac Lab version, device type
```

### Key Parameters

**Arm IK (`config/teleop/arm.yaml`):**
- `ik.dt`: IK solver timestep (default: 0.01)
- `ik.position_weight` / `ik.orientation_weight`: Task priority
- `limits.max_joint_velocity`: Safety limit (rad/s)

**Locomotion (`config/teleop/locomotion.yaml`):**
- `gait_detection.step_threshold`: Minimum foot displacement to detect a step (meters)
- `gait_detection.deadzone`: Ignore movements below this threshold
- `velocity_mapping.max_linear_velocity`: Maximum base speed (m/s)

**MuJoCo (`config/simulation/mujoco.yaml`):**
- `timestep`: Physics integration step (default: 0.002 = 500 Hz)
- `render.camera_name`: Camera to render from (default: `head_camera`)

---

## 9. Standalone Testing (No ROS2)

These scripts run without ROS2, using direct Python imports.

### Step-by-Step MuJoCo Demo

```bash
# All 5 steps: model load, arm, hand, mobility, vision
python3 scripts/demo_teleop_sim.py

# Single step
python3 scripts/demo_teleop_sim.py --step 1   # Model info
python3 scripts/demo_teleop_sim.py --step 2   # Arm teleop
python3 scripts/demo_teleop_sim.py --step 3   # Hand teleop
python3 scripts/demo_teleop_sim.py --step 4   # Mobility (wheels)
python3 scripts/demo_teleop_sim.py --step 5   # Vision (camera)

# More simulation steps for longer test
python3 scripts/demo_teleop_sim.py --steps 500
```

### Per-Module Standalone Tests

```bash
# Arm teleop (SimulatedTracker + MuJoCo, no ROS2)
python3 scripts/test_arm_teleop_standalone.py

# Locomotion (SimulatedTracker + gait detection, no ROS2)
python3 scripts/test_locomotion_standalone.py

# Hand teleop (SimulatedHand + retargeting, no ROS2)
python3 scripts/test_hand_teleop_standalone.py

# Camera (head camera rendering, no ROS2)
python3 scripts/test_camera_teleop_standalone.py
```

### Point Cloud & Streaming Tests

```bash
# Point cloud generation pipeline
python3 scripts/verify_pointcloud_pipeline.py

# First-person view (head tracking)
python3 scripts/verify_first_person_view.py

# RGB-D streaming (TCP, launches both server and client locally)
python3 scripts/test_camera_streaming.py
```

### Unit Tests

```bash
# Full test suite
python3 -m pytest tests/ -v

# By phase
python3 -m pytest tests/test_interfaces.py tests/test_transforms.py tests/test_config.py -v   # Phase 1-2
python3 -m pytest tests/test_phase3_multichain.py -v                                           # Phase 3 (IK)
python3 -m pytest tests/test_phase4_locomotion_hand.py -v                                      # Phase 4
python3 -m pytest tests/test_phase5_camera_gui.py tests/test_phase5_camera_head.py -v          # Phase 5
python3 -m pytest tests/test_ros2_rgbd.py tests/test_rgbd_streaming.py -v                      # Camera/streaming

# Single test
python3 -m pytest tests/test_transforms.py::TestQuaternionOperations::test_multiply_identity -v
```

---

## 10. Troubleshooting

### EGL / MuJoCo Rendering Errors

**Symptom:** `RuntimeError: Failed to initialize EGL` or blank camera renders.

```bash
# Use software rendering
MUJOCO_GL=egl python3 scripts/demo_teleop_sim.py --step 5

# Or use GLFW for windowed rendering
MUJOCO_GL=glfw python3 scripts/demo_teleop_sim.py --step 5

# Check GPU drivers
nvidia-smi
eglinfo
```

**Headless servers (no display):** Use `MUJOCO_GL=egl`. Ensure `libegl1-mesa-dev` is installed.

**With MuJoCo viewer:** Must use `MUJOCO_GL=glfw` (requires a display or Xvfb).

### ROS2 Not Found

**Symptom:** `ModuleNotFoundError: No module named 'rclpy'`

This is expected when running without ROS2. All ROS2-dependent code uses lazy imports and gracefully degrades:

```python
try:
    import rclpy
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
```

Non-ROS2 scripts (demo_teleop_sim.py, standalone tests) work without ROS2. Only `ros2 launch` and ROS2 node scripts require it.

### Launch Files Not Found

**Symptom:** `ros2 launch teleop_system slave_mujoco.launch.py` fails with "Package not found".

```bash
# Rebuild and re-source
cd ~/ros2_ws
colcon build --packages-select teleop_system --symlink-install
source install/setup.bash

# Verify
ros2 pkg prefix teleop_system
ros2 launch teleop_system --list
```

### IK Solver Not Available

**Symptom:** `ImportError: No module named 'pink'` or `ModuleNotFoundError: No module named 'pinocchio'`

```bash
pip install pin pin-pink

# Verify
python3 -c "import pinocchio; import pink; print('IK OK')"
```

The IK solver auto-detects available QP backends: `proxqp` > `osqp` > `daqp`. Install at least one:

```bash
pip install proxsuite   # Recommended (fastest)
pip install osqp         # Alternative
```

### Missing MuJoCo Model

**Symptom:** `FileNotFoundError: models/rby1/model_teleop.xml`

Ensure you are running from the project root directory:

```bash
cd /path/to/teleop_system
python3 scripts/demo_teleop_sim.py
```

### Point Cloud Viewer Crashes

**Symptom:** Open3D window doesn't open or crashes immediately.

```bash
# Verify Open3D
pip install open3d
python3 -c "import open3d; print(open3d.__version__)"

# If using headless server, Open3D visualization requires a display
# Use Xvfb for headless:
Xvfb :1 -screen 0 1024x768x24 &
export DISPLAY=:1
python3 scripts/demo_pointcloud_viewer.py
```

### Camera Publishing Shows No Data

**Symptom:** Topics are publishing but images are empty.

```bash
# Check if bridge is configured with camera
ros2 param get /mujoco_bridge publish_camera  # Should be True

# Verify rendering works
python3 scripts/demo_teleop_sim.py --step 5

# Check topic rates
ros2 topic hz /slave/camera/color/image_raw
```

### Robot Not Responding to Commands

```bash
# Verify topics are being published
ros2 topic hz /slave/arm/right/joint_cmd

# Check MuJoCo is receiving commands
ros2 topic echo /mujoco/joint_states --once

# Verify node connections
ros2 node list
ros2 node info /mujoco_bridge
```
