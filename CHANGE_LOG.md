Implementation Summary
New Files Created
File	Description
proportional_mapper.py	SimpleProportionalMapper fallback IK + create_ik_solver() factory
ros2_adapters.py	ROS2TrackerAdapter + ROS2ArmCommandPublisher (bridges ROS2 ↔ interfaces)
mujoco_ros2_bridge.py	MuJoCo physics node: subscribes to joint cmds, publishes joint states
dummy_tracker_pub.py	Sinusoidal test tracker publisher (replaces real Vive for testing)
vive_tracker_pub.py	Real Vive Tracker → ROS2 PoseStamped publisher (requires SteamVR)
test_arm_teleop_standalone.py	Standalone test (no ROS2) with MuJoCo viewer
run_mujoco_bridge.py	Entry point for the bridge node
teleop_mujoco_bridge.launch.py	ROS2 launch: all 3 nodes together
model_pinocchio.urdf	Pinocchio-compatible URDF (patched geometry + limits)
Modified Files
arm_teleop_node.py — Now integrates ArmController with ROS2 adapters
ros2_helpers.py — Added MUJOCO_JOINT_STATES topic
solvers/__init__.py — Exports new classes
setup.py — Entry points for all new ROS2 nodes
Data Flow (verified working)

DummyTrackerPub → PoseStamped → ArmTeleopNode → JointState → MuJoCoROS2Bridge → physics + viewer
How to Run
Standalone (no ROS2):


MUJOCO_GL=glfw python3 scripts/test_arm_teleop_standalone.py
ROS2 pipeline (3 terminals):


source /opt/ros/jazzy/setup.bash
# T1: MUJOCO_GL=glfw python3 scripts/run_mujoco_bridge.py --launch-viewer
# T2: python3 -c "from teleop_system.simulators.dummy_tracker_pub import main; main()"
# T3: python3 -c "from teleop_system.modules.arm_teleop.arm_teleop_node import main; main()"
Real Vive Trackers (requires SteamVR):


python3 -c "from teleop_system.devices.vive_tracker_pub import main; main()" \
    --ros-args -p tracker_mapping.right_hand:=LHR-XXXXXXXX
Test Results
139/139 pytest tests pass (no regressions)
Standalone PinkIKSolver: 300/300 IK success
ROS2 integration: 94 tracker poses → 58 arm commands → 42 joint states in 5s, arms confirmed moving in MuJoCo

------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------

All tasks complete. Here's the summary:

Locomotion Module Integration - Complete
Files Created
File	Description
ros2_adapters.py	ROS2BaseCommandPublisher(IMobileBase) - publishes Twist when send_velocity() called
test_locomotion_standalone.py	Standalone test (no ROS2): foot trackers → gait detection → MuJoCo base movement
Files Modified
File	Change
locomotion_node.py	Integrated LocomotionController with ROS2 adapters, auto-calibration on first tracker data, 50Hz control loop
dummy_tracker_pub.py	Added RIGHT_FOOT + LEFT_FOOT trackers (anti-phase walking pattern), publish_feet parameter
setup.py	Added locomotion_node entry point
teleop_mujoco_bridge.launch.py	Added locomotion node to launch
Data Flow (verified)

DummyTrackerPub → PoseStamped on /master/tracker/{right_foot,left_foot}
    → LocomotionNode (GaitDetector: foot displacement → velocity)
        → Twist on /slave/base/cmd_vel
            → MuJoCoROS2Bridge (differential drive → wheel actuators)
Key Fix
LocomotionNode defers controller.enable() until the control loop detects both foot trackers have data. This handles the ROS2 startup race condition where the node initializes before any tracker messages arrive.

Test Results
139/139 pytest tests pass (no regressions)
Standalone test: 87.6% non-zero velocity commands, robot moves
ROS2 pipeline: 296 foot poses → 112 cmd_vel commands → 232 joint states in 6s, auto-calibration succeeded
How to Run

# Standalone (no ROS2)
MUJOCO_GL=glfw python3 scripts/test_locomotion_standalone.py

# ROS2 pipeline (3 terminals)
source /opt/ros/jazzy/setup.bash
# T1: MUJOCO_GL=glfw /usr/bin/python3.12 scripts/run_mujoco_bridge.py --launch-viewer
# T2: /usr/bin/python3.12 -c "from teleop_system.simulators.dummy_tracker_pub import main; main()"
# T3: /usr/bin/python3.12 -c "from teleop_system.modules.locomotion.locomotion_node import main; main()"


------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------

Gripper/Hand Teleoperation - Implementation Summary
Files Created
ros2_adapters.py - ROS2GloveAdapter(IHandInput) + ROS2HandCommandPublisher(ISlaveHand) bridging ROS2 messages to HandController interfaces
dummy_glove_pub.py - ROS2 node publishing simulated 20-DOF hand joint data on /master/hand/{left,right}/joints
test_hand_teleop_standalone.py - Standalone test (no ROS2): SimulatedHand → HandController → MuJoCoHand → MuJoCo physics
Files Modified
mujoco_sim.py - Added MuJoCoHand(ISlaveHand) adapter mapping 20-DOF hand commands → single gripper ctrl value via mean + 20x scaling
hand_teleop_node.py - Full rewrite from stub: dual HandController (left+right) with ROS2 adapters, 100Hz control loop, auto-connect on glove data. Fixed callback_group → callback_groups bug.
mujoco_ros2_bridge.py - Added 20x scaling to _gripper_cmd_callback for visible gripper motion
setup.py - Added hand_teleop_node and dummy_glove_pub entry points
teleop_mujoco_bridge.launch.py - Added DummyGlovePub and HandTeleopNode nodes
Verification Results
139/139 tests pass (no regressions)
Standalone test: 500/500 commands (100%), gripper ctrl oscillating 0-20, gripper qpos reaching full range
ROS2 pipeline: 143 hand commands in 6s, gripper ctrl=14.8 (properly scaled)
Works with both dummy (SimulatedHand) and real hardware (ManusGlove) — same topic interface

------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------

Camera/RGB-D & Head Teleoperation Module -- Implementation Summary
Files Created (7)
File	Description
sim_camera_stream.py	SimCameraStream(ICameraStream) -- wraps MuJoCo head camera with pan/tilt control, double buffering, intrinsics from fovy
camera_controller.py	CameraController -- pure logic: HMD quaternion -> euler -> pan/tilt with EMA smoothing + velocity limiting
ros2_adapters.py	ROS2HMDAdapter(IMasterTracker) + ROS2CameraAdapter(ICameraStream) for ROS2 bridging
camera_node.py	CameraTeleopNode(LifecycleNode) -- subscribes HMD orientation, publishes pan/tilt commands at 30Hz
dummy_hmd_pub.py	DummyHMDPub(Node) -- publishes simulated HMD orientation at 90Hz for testing
test_camera_teleop_standalone.py	Standalone test script (no ROS2) with viewer support and point cloud generation
test_phase5_camera_head.py	21 pytest tests covering SimCameraStream, CameraController, pipeline integration
Files Modified (4)
File	Change
mujoco_sim.py	Fixed get_camera_rgbd() to compute proper intrinsics from camera fovy (was returning np.eye(3))
mujoco_ros2_bridge.py	Added head pan/tilt subscription (CTRL_HEAD was defined but never subscribed)
setup.py	Added camera_teleop_node and dummy_hmd_pub entry points
teleop_mujoco_bridge.launch.py	Added DummyHMDPub and CameraTeleopNode nodes
Verification Results
160 tests pass (139 existing + 21 new, zero regressions)
Standalone test: 150/150 commands, pan/tilt vary, RGB-D frames 640x480, ~101K points per frame, intrinsics fx=415.7
ROS2 pipeline: DummyHMDPub(90Hz) -> CameraTeleopNode(30Hz) -> Bridge -> ctrl[22]=0.13, ctrl[23]=0.08


------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------


테스트 방법 정리
1. 자동 테스트 (pytest)

MUJOCO_GL=egl python3 -m pytest tests/test_phase5_camera_head.py -v
21개 테스트: SimCameraStream, CameraController, 파이프라인 통합, point cloud 생성

2. 스트리밍 품질 검증 (이미지 저장)

MUJOCO_GL=egl python3 scripts/test_camera_streaming.py
5개 방향 RGB/depth 이미지 + PLY point cloud 저장, FPS/throughput 측정

3. 파이프라인 동작 검증 (standalone)

MUJOCO_GL=egl python3 scripts/test_camera_teleop_standalone.py --no-viewer --duration 5
HMD → CameraController → SimCameraStream → MuJoCo → RGB-D → point cloud 전체 루프

4. ROS2 파이프라인 검증

source /opt/ros/jazzy/setup.bash
# 터미널 1: 브릿지
/usr/bin/python3.12 scripts/run_mujoco_bridge.py --launch-viewer
# 터미널 2: 더미 HMD
/usr/bin/python3.12 -c "from teleop_system.simulators.dummy_hmd_pub import main; main()"
# 터미널 3: 카메라 노드
/usr/bin/python3.12 -c "from teleop_system.modules.camera.camera_node import main; main()"
# 확인:
ros2 topic hz /slave/camera/pan_tilt_cmd    # ~30Hz
ros2 topic echo /master/hmd/orientation --once
2. 실제 HW에서 달라지는 것
구분	Simulation (현재)	Real Hardware
HMD 입력	DummyHMDPub (SimulatedTracker)	Vive HMD 또는 Quest → /master/hmd/orientation 퍼블리시
카메라	SimCameraStream (MuJoCo 렌더링)	RealSenseCamera (realsense_camera.py) — ROS2 토픽으로 RGB-D 수신
로봇 제어	MuJoCo Bridge ctrl[22:24]	RBY1 실제 로봇 head joint 드라이버
Point Cloud	동일 (PointCloudGenerator)	동일 — 인터페이스 동일하므로 코드 변경 없음
변경이 필요 없는 것:

CameraController — 순수 제어 로직, 인터페이스만 사용
CameraTeleopNode — /master/hmd/orientation 토픽만 구독하므로, 퍼블리셔가 뭐든 동작
PointCloudGenerator — RGBDFrame만 받으면 됨
변경이 필요한 것:

HMD 퍼블리셔: DummyHMDPub 대신 실제 VR SDK 노드 실행

SteamVR: vive_tracker_pub에 HEAD role 추가하거나 별도 HMD publisher
Quest: WebXR/OpenXR → ROS2 bridge
카메라 소스: launch에서 SimCameraStream → RealSenseCamera 전환

RealSenseCamera는 이미 구현됨 (realsense_camera.py)
ROS2 토픽 /camera/color/image_raw, /camera/aligned_depth_to_color/image_raw 구독
set_orientation()이 /slave/camera/pan_tilt_cmd로 퍼블리시 (이미 구현)
로봇 드라이버: MuJoCo bridge 대신 RBY1 하드웨어 드라이버가 /slave/camera/pan_tilt_cmd 구독

즉, launch 파일에서 노드만 교체하면 되고 CameraTeleopNode 코드는 변경 없이 동작합니다. 이것이 인터페이스 기반 설계의 장점입니다.


------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------



Results
The full camera/point cloud streaming pipeline is verified and working:

Metric	Result
Point cloud points per frame	130k-140k
Pipeline throughput	37.1 Hz (target: 15 Hz)
Point cloud generation	23-49 ms per frame
Head tracking	pan: 0.127→0.147 rad, tilt: 0.066→0.096 rad
All tests	160/160 pass
Files created
File	Purpose
pointcloud_viewer.py	Interactive GLFW+OpenGL viewer (for workstations with display)
demo_pointcloud_viewer.py	Real-time streaming demo (GLFW window, mouse orbit, live stats)
verify_pointcloud_pipeline.py	Headless verification (saves PNG images + PLY files)
Environment note
This environment is headless (no X11/Wayland display server), so the interactive GLFW viewer cannot open a window here. The demo_pointcloud_viewer.py script will work on any workstation with a display. For verification in this headless environment, verify_pointcloud_pipeline.py renders point cloud snapshots as 2D projections (top-down + side views) using Pillow and saves them as PNG files.

Output images are in output/pc_verify/ -- the composite image shows RGB, depth, top-down point cloud, and side-view point cloud for each frame captured during the streaming pipeline test.


------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------


demo_pointcloud_viewer.py는 마스터 시스템이 아닙니다. 이 스크립트는 슬레이브+마스터가 하나의 프로세스에 합쳐진 올인원 데모입니다:


demo_pointcloud_viewer.py (단일 프로세스)
┌─────────────────────────────────────────────────┐
│  MuJoCo 시뮬레이션 (슬레이브 역할)               │
│    └→ SimCameraStream → RGB-D 생성               │
│                                                   │
│  SimulatedTracker + CameraController              │
│    └→ HMD 시뮬레이션 → pan/tilt 제어              │
│                                                   │
│  PointCloudGenerator → PointCloudViewer           │
│    └→ RGB-D를 직접 받아서 포인트클라우드 표시 (마스터 역할)  │
│                                                   │
│  ※ 네트워크 통신 없음 — 모두 같은 프로세스 내 메모리 전달   │
└─────────────────────────────────────────────────┘
그래서 demo_rgbd_streaming.py 없이도 화면을 볼 수 있습니다. RGB-D 데이터가 네트워크를 거치지 않고 메모리에서 직접 전달되기 때문입니다.

demo_rgbd_streaming.py는 실제 무선통신 시나리오를 위한 것입니다:


--mode server (슬레이브/로봇 측)        --mode client (마스터/오퍼레이터 측)
┌──────────────────────┐              ┌──────────────────────┐
│ MuJoCo 시뮬레이션     │              │ RGBDStreamClient     │
│   └→ SimCameraStream │              │   (ICameraStream)    │
│        │             │              │        │             │
│ RGBDStreamServer     │── TCP/무선 ──→│ PointCloudGenerator  │
│  (JPEG+lz4 압축)     │              │   └→ PointCloudViewer│
│  (pan/tilt 역방향 수신)│←─ TCP/무선 ──│  (마우스→pan/tilt전송)│
└──────────────────────┘              └──────────────────────┘
--mode local은 두 개를 같은 프로세스에서 localhost TCP로 연결해 테스트하는 모드입니다.

정리하면:

스크립트	역할	네트워크 사용
demo_pointcloud_viewer.py	올인원 데모 (시뮬+뷰어 한 프로세스)	없음
demo_rgbd_streaming.py --mode server	슬레이브 (RGB-D 송신)	TCP 서버
demo_rgbd_streaming.py --mode client	마스터 (RGB-D 수신 + 뷰어)	TCP 클라이언트
demo_rgbd_streaming.py --mode local	서버+클라이언트 한 프로세스 (테스트용)	localhost TCP
실제 HW 배포 시에는 로봇에서 --mode server, 오퍼레이터 PC에서 --mode client --host <로봇IP> 를 실행하게 됩니다.


------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------


The EGL threading bug is fixed. Here's a summary of the changes:

Root cause: RGBDStreamServer._server_loop() ran in a background thread and called self._camera.get_rgbd(), which invokes MuJoCo's EGL renderer. EGL contexts are thread-local, so rendering from a different thread than the one that created the context causes EGL_BAD_ACCESS.

Fix — 3 files changed:

rgbd_streaming.py — Restructured RGBDStreamServer:

Added capture() method that must be called from the main thread. It captures a frame from the camera, encodes it, and stores the bytes in a thread-safe buffer (_encoded_frame + _frame_event).
Changed _server_loop() background thread to only handle TCP accept/send. It waits on _frame_event for new encoded data instead of calling camera.get_rgbd() directly.
demo_rgbd_streaming.py — Updated both run_server() and run_local():

run_server(): Calls server.capture() at stream FPS from the main physics loop.
run_local(): Calls server.capture() from the main loop alongside viewer updates.
test_rgbd_streaming.py — Updated integration tests:

Added _run_capture_loop() helper and _start_capture() method to pump server.capture() from a test thread (safe since MockCameraStream has no EGL dependency).
All 179 tests pass.