# Development Summary

This document tracks development progress for the RB-Y1 teleoperation system. Each session's changes are recorded in reverse chronological order (newest first).

---

## 2026-02-01 (Session 6) — GUI Enhancements: E-Stop, Module Activity, Hand Viewer, 3-View Tracker

### Summary

Major GUI control panel improvements: fixed non-functional Emergency Stop (now toggle with continuous 50Hz zero publishing), added live module activity tracking, added Hand Data tab with finger joint bar charts, expanded Tracker View to 3-view layout (XY/XZ/YZ), added joint states diagnostic messages, and doubled default window/font size for readability.

### Changes

| Change | Description |
|--------|-------------|
| Emergency Stop toggle | Changed from single-shot to toggle mode with 50Hz continuous zero publishing on all arm/hand/base topics. Visual feedback: button turns red when active, label changes to "RELEASE E-STOP". |
| Module Status activity | Replaced static connected/enabled flags with `last_activity` timestamp tracking. Shows green (<2s), yellow (<10s), orange (>10s), gray (never). Added base_cmd_vel and hand joint subscriptions for Locomotion and Hand Teleop monitoring. |
| Hand Data tab | New tab with left/right hand bar charts showing 20-DOF finger joint angles in real time. Subscribes to `/master/hand/{left,right}/joints`. |
| 3-view tracker | Added Front (Y-Z) view alongside existing Top-Down (X-Y) and Side (X-Z) for pseudo-3D visualization. |
| Window/font 2x | Base window: 1800×1000 (was 1000×700). Auto-scale minimum: 2.0x (was 1.0x). 4K→3.0x, 1440p→2.5x, 1080p→2.0x. |
| Joint States diagnostics | Shows "Waiting for joint data..." message when no data sources are active. |

### Modified Files

| File | Changes |
|------|---------|
| `teleop_system/gui/control_panel.py` | HandData dataclass, 3-view tracker, hand tab, E-stop toggle with themes, module activity display, joint/hand status text, doubled window size |
| `teleop_system/gui/gui_node.py` | E-stop timer (50Hz), hand joint subscriptions, base_cmd_vel subscription, module activity tracking via `_mark_module_active()`, hand/E-stop publishers |
| `docs/user_guide.md` | Font scaling section, updated tab descriptions (hand data, 3-view, module status colors), E-stop toggle docs |
| `docs/debugging_log.md` | 3 new entries: E-stop, module status, window size |

### Test Results

- **270/270 tests pass** (no regressions)

---

## 2026-02-01 (Session 5) — Debugging, Staged Playback, TCP Viewer, GUI Scaling

### Summary

Fixed GUI crash (exit code -6), implemented staged BVH playback with Start Playback button, added cross-machine TCP viewer mode to launchers and GUI, and added auto-scaling font/window sizing based on screen resolution.

### Bug Fixes

| Issue | Root Cause | Fix |
|-------|-----------|-----|
| GUI exit code -6 (SIGABRT) even after error-message improvement | ROS2 executor spin thread still joinable when `rclpy.shutdown()` called; C++ std::thread destructor calls terminate() | Added `spin_thread.join(timeout=3.0)` after `executor.shutdown()` in both error and normal paths |
| `camera_fps:=30` INTEGER vs DOUBLE type error | `declare_parameter("camera_fps", 15.0)` locks type to DOUBLE | Added `ParameterDescriptor(dynamic_typing=True)` to all numeric/bool params |
| MuJoCo model not found in colcon install tree | `__file__` path inside `ros2_ws/install/` doesn't reach source root | Added CWD fallback path resolution |
| matplotlib < 3.9.0 crashes with NumPy 2.x | Incompatible C extension linking | Updated requirement to `matplotlib>=3.9.0` |

### New Features

| Feature | Description |
|---------|-------------|
| Staged BVH playback | BVH replay starts in READY state (frame 0 on loop). Press Start Playback or call `/teleop/start_playback` to begin motion. `auto_start` param to skip. |
| TCP viewer mode | Launchers support `viewer_mode:=client viewer_host:=<ip>` for cross-machine first-person camera. GUI has "Viewer (TCP)" button with host/port input. |
| GUI auto-scaling | Font and window size auto-detect from screen resolution via `xrandr`. Manual override: `font_scale:=2.0`. |

### Modified Files

| File | Changes |
|------|---------|
| `teleop_system/gui/gui_node.py` | Fixed exit -6 (spin thread join), added playback/TCP viewer/font_scale params |
| `teleop_system/gui/control_panel.py` | Added Start Playback button, TCP viewer button, auto-scaling, playback state display |
| `teleop_system/mocap/bvh_replay_publisher.py` | Staged start: auto_start param, READY/PLAYING states, /teleop/start_playback service |
| `teleop_system/mocap/bvh_hand_adapter.py` | Added `pause()`/`resume()` methods |
| `teleop_system/utils/ros2_helpers.py` | Added `PLAYBACK_STATE`, `START_PLAYBACK` names |
| `launch/master_sim.launch.py` | Added viewer_mode/host/port, font_scale args, GUI params |
| `launch/master_mocap.launch.py` | Same + auto_start arg |
| `scripts/launch_master.py` | Added --viewer-mode/host/port flags |
| `scripts/launch_master_mocap.py` | Same + --auto-start flag |
| `setup.py` | Updated matplotlib>=3.9.0 |
| `teleop_system/simulators/mujoco_ros2_bridge.py` | ParameterDescriptor(dynamic_typing=True), CWD fallback |

### Test Results

- **270/270 tests pass** (no regressions)

---

## 2026-02-01 (Session 4) — GUI Control Panel + Tracker Calibration

### Summary

Implemented a full-featured ROS2 Node-based GUI control panel with Dear PyGui (4 tabs: Status, Tracker 3D View, Joint State Plots, Parameters), an A-Pose tracker calibration system, and updated the TRD.md architecture diagram to reflect all current modules.

### New Files

| File | Description |
|------|-------------|
| `teleop_system/calibration/__init__.py` | Calibration package init |
| `teleop_system/calibration/pose_calibrator.py` | A-Pose calibration state machine with offset computation |
| `teleop_system/calibration/calibration_node.py` | ROS2 node: `/teleop/calibrate` service + state/offset publishers |
| `config/calibration/a_pose_reference.yaml` | Robot A-Pose reference positions (6 trackers) |
| `teleop_system/gui/gui_node.py` | ROS2 node wrapping ControlPanel with topic subscriptions |
| `tests/test_calibration.py` | 21 unit tests for calibration state machine |

### Modified Files

| File | Changes |
|------|---------|
| `teleop_system/gui/control_panel.py` | Rewritten: 4-tab layout (Status/Tracker/Joints/Params), calibration UI, action buttons, scatter plots, time series |
| `teleop_system/utils/ros2_helpers.py` | Added `CALIBRATION_STATE`, `CALIBRATION_OFFSETS` topic names |
| `teleop_system/simulators/dummy_tracker_pub.py` | Added `/calibration/offsets` subscription + offset application |
| `teleop_system/mocap/bvh_replay_publisher.py` | Added `/calibration/offsets` subscription + offset application |
| `teleop_system/devices/vive_tracker_pub.py` | Added `/calibration/offsets` subscription + offset application |
| `launch/master_sim.launch.py` | Added `launch_gui` arg, calibration_node, gui_control_panel |
| `launch/master_mocap.launch.py` | Added `launch_gui` arg, calibration_node, gui_control_panel |
| `launch/teleop_sim_full.launch.py` | Added `launch_gui` arg pass-through |
| `scripts/launch_master.py` | Added `--no-gui` flag, calibration_node, GUI subprocess |
| `scripts/launch_master_mocap.py` | Added `--no-gui` flag, calibration_node, GUI subprocess |
| `setup.py` | Added `gui_control_panel` and `calibration_node` entry points + calibration config data_files |
| `docs/TRD.md` | Updated Section 3.1 architecture diagram + Section 4 directory structure |
| `tests/test_phase5_camera_gui.py` | Fixed renamed callback method references |

### Architecture

```
Calibration Flow:
  GUI "Calibrate" button
    → /teleop/calibrate (std_srvs/Trigger)
    → CalibrationNode starts PoseCalibrator state machine
    → WAITING (3s countdown) → CAPTURING (1s) → COMPUTING → CALIBRATED
    → Publishes offsets as JSON on /calibration/offsets (transient local)
    → Tracker publishers apply offsets before publishing
```

```
GUI Threading Model:
  Main Thread:  DearPyGui render loop (setup → render_frame loop → shutdown)
  Background:   ROS2 MultiThreadedExecutor.spin()
  Sync:         threading.Lock on shared data buffers (TrackerData, JointStateData)
```

### Test Results

- 21 new calibration tests: **all pass**
- Existing GUI tests: **all pass** (callback rename fixed)
- Full suite regression: **all pass**

---

## 2026-02-01 (Session 3) — Mocap Master Launcher & Colcon Debug Fix

### Summary

Created a dedicated master system launcher that uses BVH motion capture replay data instead of dummy sinusoidal inputs. This enables cross-machine testing with real human motion: run the mocap master on one PC and the MuJoCo slave on another.

Also fixed a stale colcon build issue causing `ImportError: bvhio` when running `run_mocap_replay.py --view`.

### Bug Fix

| Issue | Root Cause | Fix |
|-------|-----------|-----|
| `run_mocap_replay.py --view` fails with `ImportError: bvhio` | Stale colcon build at `ros2_ws/build/teleop_system/` on PYTHONPATH shadows source tree | Rebuilt with `colcon build --packages-select teleop_system --symlink-install` |

### New Files

| File | Description |
|------|-------------|
| `launch/master_mocap.launch.py` | ROS2 launch: BVH replay publisher + all teleop nodes + optional viewer |
| `scripts/launch_master_mocap.py` | Python script launcher for mocap master (no `ros2 launch` needed) |

### Architecture

```
master_mocap.launch.py / launch_master_mocap.py
    ├── BVH Replay Publisher (replaces dummy_tracker_pub + dummy_glove_pub + dummy_hmd_pub)
    │     publishes: /master/tracker/*, /master/hand/*, /master/hmd/orientation
    ├── Arm Teleop Node
    ├── Locomotion Node
    ├── Hand Teleop Node
    ├── Camera Teleop Node
    └── (Optional) RGB-D Viewer
```

### Usage

```bash
# ROS2 launch
ros2 launch teleop_system master_mocap.launch.py bvh_file:=/path/to/file.bvh
ros2 launch teleop_system master_mocap.launch.py bvh_file:=/path/to/file.bvh playback_speed:=0.5 launch_viewer:=true

# Python script
python3 scripts/launch_master_mocap.py --bvh data/bvh/cmu/002/02_01.bvh
python3 scripts/launch_master_mocap.py --bvh data/bvh/cmu/002/02_01.bvh --playback-speed 0.5 --launch-viewer

# Cross-machine: slave on Machine A, mocap master on Machine B
# Machine A:
ros2 launch teleop_system slave_mujoco.launch.py launch_viewer:=true
# Machine B:
ros2 launch teleop_system master_mocap.launch.py bvh_file:=/path/to/walking.bvh
```

### Test Results

- **249/249 tests pass** (no regressions)

---

## 2025-02-01 (Session 2) — Motion Capture Dataset Replay Infrastructure

### Summary

Built a complete BVH motion capture replay and analysis pipeline to replace dummy sinusoidal inputs with real human motion data. This enables testing the teleop pipeline (arm IK, locomotion, hand retargeting) against recorded human motion.

### New Package: `teleop_system/mocap/`

| File | Description |
|------|-------------|
| `bvh_loader.py` | BVH file parser using `bvhio`, coordinate conversion (BVH Y-up → ROS2 Z-up) |
| `skeleton_mapper.py` | Maps BVH skeleton joints to TrackerRole Pose6D (6 roles: hands, waist, feet, head) |
| `bvh_tracker_adapter.py` | `BVHTrackerAdapter(IMasterTracker)` — time-indexed BVH playback, loop/pause/speed |
| `bvh_hand_adapter.py` | `BVHHandAdapter(IHandInput)` — derives 20-joint finger motion from wrist angular velocity |
| `bvh_replay_publisher.py` | ROS2 node publishing BVH data on tracker/hand/HMD topics (replaces dummy publishers) |
| `data_recorder.py` | Records input/output channels to `.npz` for offline analysis |
| `metrics.py` | Tracking error, velocity saturation, workspace utilization, smoothness (jerk) |
| `skeleton_viewer.py` | Matplotlib 3D skeleton viewer with frame slider |
| `dual_viewer.py` | Side-by-side human skeleton + tracker position viewer |

### New Scripts

| Script | Description |
|--------|-------------|
| `scripts/download_cmu_bvh.py` | Downloads CMU BVH files from GitHub (walking, arm reaching, full body) |
| `scripts/run_mocap_replay.py` | Standalone BVH replay with metrics generation (no ROS2 needed) |
| `scripts/run_parameter_sweep.py` | Iterates scale/tuning parameters, outputs comparison table |

### Data Pipeline Architecture

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

### Configuration

- `config/mocap/default.yaml` — playback settings (scale, speed, loop)
- `config/mocap/cmu_joint_mapping.yaml` — CMU BVH joint names → TrackerRole

### Coordinate Transform

BVH (X=lateral, Y=up, Z=forward) → ROS2 (X=forward, Y=left, Z=up):
- `ros2_pos = (bvh_z, bvh_x, bvh_y) * scale`
- `ros2_quat_xyzw = (bvh_z, bvh_x, bvh_y, bvh_w)`
- Default scale: 0.056 (CMU BVH → meters, hips at ~0.95m)

### Test Results

- **249/249 tests pass** (192 existing + 57 new mocap tests, no regressions)

### Usage

```bash
# Download CMU BVH data
python3 scripts/download_cmu_bvh.py

# Standalone replay with metrics
python3 scripts/run_mocap_replay.py --bvh data/bvh/cmu/002/02_01.bvh

# Parameter sweep
python3 scripts/run_parameter_sweep.py --bvh data/bvh/cmu/002/02_01.bvh

# ROS2 replay (replaces dummy publishers with BVH data)
ros2 launch teleop_system mocap_replay.launch.py bvh_file:=/path/to/file.bvh

# With MuJoCo visualization
ros2 launch teleop_system mocap_replay.launch.py bvh_file:=/path/to/file.bvh launch_bridge:=true
```

---

## 2025-02-01 (Session 1) — Bug Fixes, ros2-server Mode, Auto-Launch Features, Documentation

### Issues Addressed

| # | Issue | Root Cause | Fix |
|---|-------|-----------|-----|
| 1 | `camera_fps:=30` crashes bridge on `ros2 launch` | ROS2 `LaunchConfiguration` passes all values as strings; node tried `.double_value` on an integer-typed parameter | Added `_get_float_param()`, `_get_bool_param()`, `_get_int_param()` type-coercion helpers |
| 2 | TCP client/server mode camera doesn't sync with robot motion | `--mode server` creates its own MuJoCo instance that doesn't receive teleop commands | Added `--mode ros2-server` that bridges ROS2 camera topics to TCP (single MuJoCo instance) |
| 3 | Master system should auto-launch viewer; slave should auto-launch streaming | No auto-launch support in launch files or scripts | Added `launch_viewer`, `launch_streaming`, `launch_camera_viewer` args to launch files and scripts |
| 4 | PRD.md, TRD.md, Tasks.md should be in docs/ | Files were in project root | Moved to `docs/` |

### Files Modified

| File | Change |
|------|--------|
| `teleop_system/simulators/mujoco_ros2_bridge.py` | Added `_get_float_param()`, `_get_bool_param()`, `_get_int_param()` helpers; updated `__init__` to use them |
| `scripts/demo_rgbd_streaming.py` | Added `ros2-server` mode and `run_ros2_server()` function |
| `launch/master_sim.launch.py` | Added `launch_viewer` arg + `ExecuteProcess` for `ros2-viewer` |
| `launch/slave_mujoco.launch.py` | Added `launch_streaming` + `streaming_port` args + `ExecuteProcess` for `ros2-server` |
| `launch/teleop_sim_full.launch.py` | Added `launch_camera_viewer` arg, passes to master_sim |
| `scripts/launch_master.py` | Added `--launch-viewer` flag with script-based process support |
| `scripts/launch_slave.py` | Added `--launch-streaming` + `--streaming-port` flags, multi-process management |
| `scripts/launch_all.py` | Added `--launch-camera-viewer` flag with auto-enable of `--publish-camera` |
| `docs/user_guide.md` | Added ros2-server docs, cross-machine workflow diagram, camera mode selection guide, updated launch options |
| `docs/developer_guide.md` | Added ros2-server architecture section, updated directory structure with moved docs |

### Architecture: ros2-server Mode

```
MuJoCo Bridge (ROS2 node)
    | publishes /slave/camera/* topics
    v
ros2-server (ROS2RGBDSubscriber -> RGBDStreamServer)
    | TCP streaming (lz4 compressed)
    v
TCP client (point cloud viewer)
    | sends head orientation via reverse channel
    v
ros2-server publishes /slave/camera/pan_tilt_cmd
    |
    v
MuJoCo Bridge moves head actuators (ctrl[22:23])
```

### New Launch Options

```bash
# Slave with auto-streaming
ros2 launch teleop_system slave_mujoco.launch.py publish_camera:=true launch_streaming:=true

# Master with auto-viewer
ros2 launch teleop_system master_sim.launch.py launch_viewer:=true

# Full system with camera viewer
ros2 launch teleop_system teleop_sim_full.launch.py publish_camera:=true launch_camera_viewer:=true

# Python script equivalents
python3 scripts/launch_slave.py --publish-camera --launch-streaming
python3 scripts/launch_master.py --launch-viewer
python3 scripts/launch_all.py --publish-camera --launch-camera-viewer
```

### Test Results

- **192/192 tests pass** (no regressions)

---

## Previous Development (see CHANGE_LOG.md for details)

### Phase 6 — Colcon Build, Launchers, Documentation
- Fixed colcon build infrastructure (resource/, setup.py data_files)
- Created slave/master/full ROS2 launch files
- Created Python script launchers (launch_slave.py, launch_master.py, launch_all.py)
- Created user_guide.md and developer_guide.md

### Phase 5b — First-Person View, Streaming Fixes
- Added first-person mode to PointCloudViewer (GLFW+OpenGL)
- Fixed EGL threading bug in RGBDStreamServer (main-thread capture pattern)
- Created verify_first_person_view.py headless verification script

### Phase 5a — Camera/RGB-D & Head Teleoperation
- Created SimCameraStream (ICameraStream wrapping MuJoCo head camera)
- Created CameraController (HMD quaternion -> pan/tilt with smoothing)
- Created CameraTeleopNode (ROS2 lifecycle node)
- Created DummyHMDPub (simulated HMD orientation publisher)
- Created PointCloudViewer (interactive GLFW+OpenGL viewer)
- Created demo_rgbd_streaming.py (TCP streaming: server/client/local modes)
- Fixed MuJoCo intrinsics computation from camera fovy

### Phase 5 — Point Cloud Pipeline
- Created PointCloudGenerator (depth -> 3D points conversion)
- Created pointcloud_viewer.py (Open3D/GLFW visualization)
- Verified pipeline: 130k-140k points/frame at 37 Hz

### Phase 4 — Locomotion & Hand Teleoperation
- Implemented LocomotionController + GaitDetector (foot trackers -> wheel velocity)
- Implemented HandController + HandRetargeting (glove joints -> gripper commands)
- Created DummyGlovePub (simulated 20-DOF hand data)
- Added MuJoCoHand adapter (20-DOF -> single gripper with 20x scaling)

### Phase 3 — Multi-Chain IK (Arm Teleoperation)
- Implemented PinkIKSolver with 3-chain simultaneous IK
- Created ArmController with ROS2 adapters
- Created MuJoCoROS2Bridge (physics + joint command/state bridge)
- Created DummyTrackerPub (sinusoidal test tracker)

### Phases 1-2 — Foundation
- Interface-driven architecture (ABCs for all hardware components)
- Coordinate transforms (pure numpy, SteamVR <-> ROS2 conversion)
- Configuration system (Hydra/OmegaConf YAML loading)
- Simulated devices (SimulatedTracker, SimulatedHand)
