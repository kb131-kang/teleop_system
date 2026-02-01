# Development Summary

This document tracks development progress for the RB-Y1 teleoperation system. Each session's changes are recorded in reverse chronological order (newest first).

---

## 2025-02-01 — Bug Fixes, ros2-server Mode, Auto-Launch Features, Documentation

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
