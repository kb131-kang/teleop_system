# ⚙ RB-Y1 텔레오퍼레이션 시스템 — 기술 사양서 (TRD)

## 1. 권장 스택

| 영역 | 기술 | 버전 | 비고 |
|------|------|------|------|
| **언어 (주)** | Python | 3.12 | ROS2 Jazzy 기본 Python |
| **언어 (보조)** | C++ | 17 | 실시간 제어 루프, 성능 크리티컬 모듈 |
| **미들웨어** | ROS2 Jazzy Jalisco | Patch 7+ | LTS (2024-2029), Ubuntu 24.04 |
| **시뮬레이터 (1차)** | Isaac Lab | 2.3.0 (Isaac Sim 5.1) | Manus+Vive 빌트인 지원, GPU 가속 |
| **시뮬레이터 (2차)** | MuJoCo | 3.4.0 | 경량, 설치 간편, 빠른 프로토타이핑 |
| **IK 솔버** | Pink (+ Pinocchio) | Pink 3.4.0 / Pinocchio 2.7.0 | Differential IK, 가중 태스크 기반 |
| **VR/Tracker** | PyOpenVR (OpenVR SDK) | 2.12.1401 (SDK 2.12.14) | Vive Tracker 6DoF 포즈 |
| **글러브** | MANUS SDK + ROS2 패키지 | MANUS Core 3.0.1+ | 공식 ROS2 토픽 퍼블리셔 |
| **로봇 SDK** | rby1-sdk | latest | Python/C++ API, URDF 포함 |
| **핸드 SDK** | DELTO_M_ROS2 | latest | DG-5F ros2_control 드라이버, Isaac Sim/Gazebo 연동 |
| **포인트 클라우드** | Open3D | 0.19.0 | 포인트 클라우드 생성/처리/렌더링 |
| **비디오 스트리밍** | GStreamer + WebRTC | GStreamer 1.24+ | 저지연 RGB-D 전송 |
| **GUI** | Dear PyGui | 2.1.1 | 즉시 모드 GUI, 빠른 프로토타이핑 |
| **설정 관리** | Hydra (OmegaConf) | 1.3+ | YAML 기반 계층적 설정 |
| **로깅** | Python logging + ROS2 logging | 표준 | 모듈별 로거 |

## 2. 선정 이유

### ROS2 Jazzy Jalisco
- **LTS 릴리즈** (2029년까지 지원)로 장기 유지보수 안정성 확보
- Ubuntu 24.04 + Python 3.12 기본 지원
- RB-Y1 SDK(`rbpodo_ros2`), MANUS SDK, TESOLLO SDK 모두 ROS2 패키지 제공
- ros2_control 프레임워크로 하드웨어 추상화 용이

### Isaac Lab 2.3 (Primary Simulator)
- **Manus + Vive Tracker 텔레오퍼레이션이 빌트인** (`ManusVive` 디바이스 클래스)
- CloudXR 기반 VR 스트리밍 지원
- Pinocchio IK 통합 (`--enable_pinocchio`)
- GPU 가속 물리 시뮬레이션 (PhysX)
- 이미테이션 러닝 데이터 수집 파이프라인 포함

### MuJoCo 3.4.0 (Secondary Simulator)
- `pip install mujoco`만으로 즉시 사용 가능
- GPU 불필요, CPU만으로 빠른 시뮬레이션
- URDF 로드 가능, ROS2 연동 용이
- 가벼운 단위 테스트 및 빠른 프로토타이핑에 적합

### Pink + Pinocchio (IK)
- Pinocchio: C++ 기반 고성능 강체 역학 라이브러리, Python 바인딩 제공
- Pink: 가중 태스크 기반 differential IK — 휴머노이드처럼 **다중 킨메틱 체인**에 적합
- 널스페이스 포스처 태스크로 자연스러운 자세 유지 가능
- ROS2 패키지로도 제공 (`pinocchio` ROS 인덱스)

### VR 포인트 클라우드 방식 (멀미 방지)
- **핵심 원리**: HMD 회전은 즉각 로컬 렌더링 (60fps+), RGB-D 데이터는 비동기 갱신 (15fps+)
- Open3D로 포인트 클라우드 생성 후 OpenGL/Vulkan 렌더링
- 최신 연구(Reality Fusion, EuroXR 2025)에서 검증된 방식
- 5G 환경에서 100ms 이하 end-to-end 지연 달성 사례 존재

---

## 3. 핵심 아키텍처

### 3.1 전체 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────────┐
│                     GUI Control Panel (ROS2 Node)                    │
│  Dear PyGui: Status │ Tracker 3D │ Joint Plots │ Parameters         │
│  [Calibrate] [RGB-D Viewer] [E-Stop] [Record]                      │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ ROS2 Topics/Services
┌──────────────────────────────┴──────────────────────────────────────┐
│                        ROS2 Middleware                               │
│                    (Jazzy Jalisco, DDS)                              │
├────────┬──────────┬──────────┬──────────┬───────────┬───────────────┤
│        │          │          │          │           │               │
│ ┌──────┴──┐ ┌────┴────┐ ┌──┴───┐ ┌───┴────┐ ┌────┴────┐ ┌───────┴───┐
│ │  Arm    │ │Locomotion│ │ Hand │ │ Camera │ │Calibra- │ │   Mocap   │
│ │ Module  │ │ Module   │ │Module│ │ Module │ │  tion   │ │  Replay   │
│ │ (Pink   │ │ (Gait   │ │(Retar│ │(P.Cloud│ │ (A-Pose │ │  (BVH     │
│ │  IK)    │ │Detector)│ │geting│ │Stream) │ │ Offset) │ │ Publisher)│
│ └────┬────┘ └────┬────┘ └──┬───┘ └───┬────┘ └────┬────┘ └─────┬─────┘
│      │           │         │         │            │            │     │
├──────┴───────────┴─────────┴─────────┴────────────┴────────────┴────┤
│                     Interface Layer (ABC)                            │
│  IMasterTracker │ ISlaveArm │ ISlaveHand │ IMobileBase               │
│  IHandInput │ IIKSolver │ ICameraStream │ ISimulator                 │
├─────────────────────────────────────────────────────────────────────┤
│                     Hardware / Simulator                             │
│  Vive Tracker │ Manus Glove │ RB-Y1 │ DG-5F │ RGB-D Camera         │
│  MuJoCo │ Isaac Lab │ Simulated/Dummy (구현체)                       │
└─────────────────────────────────────────────────────────────────────┘
```

> **[2026-02-01 Updated]** GUI를 ROS2 Node로 전환 (4탭: Status/Tracker 3D/Joint Plots/Parameters), Calibration(A-Pose 오프셋) 모듈 추가, Mocap Replay(BVH Publisher) 모듈 추가. Interface Layer 이름을 실제 구현(`IMasterTracker`, `ISlaveArm` 등)과 일치하도록 수정.

### 3.2 인터페이스 설계 (SOLID 원칙)

모든 HW 의존 컴포넌트는 **추상 인터페이스(ABC)**로 정의하여, 구현체 교체만으로 다른 장비에 대응합니다.

```python
# 예시: 마스터 디바이스 인터페이스
from abc import ABC, abstractmethod
from dataclasses import dataclass
import numpy as np

@dataclass
class Pose6D:
    position: np.ndarray    # (3,) xyz
    orientation: np.ndarray  # (4,) quaternion xyzw

class IMasterTracker(ABC):
    """마스터 트래커 인터페이스 (Vive Tracker, 시뮬레이터 등)"""

    @abstractmethod
    def get_pose(self) -> Pose6D:
        """현재 6DoF 포즈 반환"""
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        ...

class IHandInput(ABC):
    """핸드 입력 인터페이스 (Manus Glove, 시뮬레이터 등)"""

    @abstractmethod
    def get_joint_angles(self) -> np.ndarray:
        """손가락 관절 각도 배열 반환"""
        ...

class ISlaveArm(ABC):
    """슬레이브 로봇 팔 인터페이스"""

    @abstractmethod
    def send_joint_command(self, joint_positions: np.ndarray) -> None:
        ...

    @abstractmethod
    def get_joint_state(self) -> np.ndarray:
        ...

class ISlaveHand(ABC):
    """슬레이브 핸드 인터페이스 (DG-5F, 일반 그리퍼 등)"""

    @abstractmethod
    def send_joint_command(self, joint_positions: np.ndarray) -> None:
        ...

class IMobileBase(ABC):
    """이동부 인터페이스"""

    @abstractmethod
    def send_velocity(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        ...

class IIKSolver(ABC):
    """IK 솔버 인터페이스"""

    @abstractmethod
    def solve(self, target_pose: Pose6D, current_joints: np.ndarray) -> np.ndarray:
        ...

class ICameraStream(ABC):
    """카메라 스트림 인터페이스"""

    @abstractmethod
    def get_rgbd(self) -> tuple:  # (rgb, depth)
        ...

    @abstractmethod
    def set_orientation(self, pan: float, tilt: float) -> None:
        ...
```

### 3.3 구현체 구조

> **[2025-01-31 Updated]** 실제 구현 완료 상태 반영.

| 인터페이스 | 실제 HW 구현체 | 시뮬레이터 구현체 | 상태 |
|-----------|---------------|-----------------|------|
| `IMasterTracker` | `ViveTracker` (PyOpenVR) | `SimulatedTracker` (합성 포즈) | HW: SDK 없이 graceful fallback, Sim: 완료 |
| `IHandInput` | `ManusGlove` (MANUS SDK ROS2) | `SimulatedHand` (합성 관절) | HW: SDK 없이 graceful fallback, Sim: 완료 |
| `ISlaveArm` | `RBY1Arm` (rby1-sdk) | `MuJoCoArm` (mujoco_sim.py 내부 클래스) | HW: SDK 없이 graceful fallback, Sim: 완료 |
| `ISlaveHand` | `DG5FHand` (ros2_control) | `MuJoCoHand` (mujoco_sim.py 내부 클래스) | HW: SDK 없이 graceful fallback, Sim: 완료 |
| `IMobileBase` | `RBY1Base` (rby1_arm.py 내 통합) | `MuJoCoBase` (mujoco_sim.py 내부 클래스) | HW: SDK 없이 graceful fallback, Sim: 완료 |
| `IIKSolver` | `PinkIKSolver` | `PinkIKSolver` (동일) + `SimpleProportionalMapper` (폴백) | 완료 |
| `ICameraStream` | `RealSenseCamera` (ROS2 토픽) | `SimCameraStream` (MuJoCo 렌더링) | HW: 구현됨, Sim: 완료 |
| `ISimulator` | — | `MuJoCoSimulator` (mujoco_sim.py) | 완료 |

**참고**: `MuJoCoArm`, `MuJoCoBase`, `MuJoCoHand`는 별도 파일이 아닌 `mujoco_sim.py` 내 nested class로 구현됨.
`RBY1Base`는 `rby1_base.py` 대신 `rby1_arm.py`에 통합되어 있음.

---

## 4. 디렉토리 구조

> **[2025-01-31 Updated]** 실제 구현 결과를 반영하여 디렉토리 구조를 업데이트함.
> 각 모듈에 `ros2_adapters.py`가 추가되었고, 시뮬레이터에 더미 퍼블리셔 및 MuJoCo-ROS2 브릿지가 추가됨.

```
teleop_system/
├── config/                          # Hydra/YAML 설정 파일
│   ├── default.yaml                 # 기본 설정
│   ├── simulation/
│   │   ├── isaac_lab.yaml
│   │   └── mujoco.yaml
│   ├── hardware/
│   │   ├── rby1.yaml
│   │   ├── vive_tracker.yaml
│   │   ├── manus_glove.yaml
│   │   └── dg5f.yaml
│   ├── teleop/
│   │   ├── arm.yaml                 # IK 게인, 속도 제한 등
│   │   ├── locomotion.yaml          # 걸음 감지 파라미터
│   │   └── hand.yaml               # 리타겟팅 파라미터
│   ├── mocap/                       # 모캡 재생 설정
│   │   ├── default.yaml
│   │   └── cmu_joint_mapping.yaml
│   └── calibration/                 # 캘리브레이션 설정
│       └── a_pose_reference.yaml    # A-Pose 레퍼런스 위치
│
├── teleop_system/                   # 메인 Python 패키지
│   ├── __init__.py
│   │
│   ├── interfaces/                  # 추상 인터페이스 (ABC)
│   │   ├── __init__.py
│   │   ├── master_device.py         # IMasterTracker, IHandInput, Pose6D, HandJointState, TrackerRole
│   │   ├── slave_robot.py           # ISlaveArm, ISlaveHand, IMobileBase, JointState, VelocityCommand, ArmSide
│   │   ├── ik_solver.py             # IIKSolver, IKResult
│   │   ├── camera_stream.py         # ICameraStream, RGBDFrame
│   │   └── simulator.py             # ISimulator, SimState
│   │
│   ├── modules/                     # 기능 모듈 (독립 실행 가능)
│   │   ├── __init__.py
│   │   ├── arm_teleop/              # 양팔 + 토르소 텔레오퍼레이션
│   │   │   ├── __init__.py
│   │   │   ├── arm_teleop_node.py   # ROS2 Lifecycle 노드
│   │   │   ├── arm_controller.py    # 제어 로직 (순수 Python, ROS2 무관)
│   │   │   └── ros2_adapters.py     # ROS2TrackerAdapter, ROS2ArmCommandPublisher
│   │   ├── locomotion/              # AMR 이동부 텔레오퍼레이션
│   │   │   ├── __init__.py
│   │   │   ├── locomotion_node.py   # ROS2 Lifecycle 노드
│   │   │   ├── locomotion_controller.py  # 제어 로직 (순수 Python)
│   │   │   ├── gait_detector.py     # 보행 패턴 감지 알고리즘
│   │   │   └── ros2_adapters.py     # ROS2BaseCommandPublisher
│   │   ├── hand_teleop/             # 핸드 텔레오퍼레이션
│   │   │   ├── __init__.py
│   │   │   ├── hand_teleop_node.py  # ROS2 Lifecycle 노드
│   │   │   ├── hand_controller.py   # 제어 로직 (순수 Python)
│   │   │   ├── retargeting.py       # Manus → DG-5F 매핑 (HandRetargeting)
│   │   │   └── ros2_adapters.py     # ROS2GloveAdapter, ROS2HandCommandPublisher
│   │   └── camera/                  # VR 카메라 스트리밍 + 헤드 텔레오퍼레이션
│   │       ├── __init__.py
│   │       ├── camera_node.py       # ROS2 Lifecycle 노드 (HMD→Pan/Tilt)
│   │       ├── camera_controller.py # 제어 로직 (HMD→euler→pan/tilt, EMA 스무딩)
│   │       ├── pointcloud_generator.py  # RGB-D → Open3D PointCloud 변환
│   │       ├── pointcloud_viewer.py # GLFW+OpenGL 포인트 클라우드 뷰어
│   │       └── ros2_adapters.py     # ROS2HMDAdapter, ROS2CameraAdapter
│   │
│   ├── devices/                     # HW 디바이스 구현체
│   │   ├── __init__.py
│   │   ├── vive_tracker.py          # ViveTracker, ViveTrackerManager (IMasterTracker)
│   │   ├── vive_tracker_pub.py      # Vive Tracker → ROS2 PoseStamped 퍼블리셔
│   │   ├── manus_glove.py           # ManusGlove (IHandInput)
│   │   ├── rby1_arm.py              # RBY1Arm (ISlaveArm) + RBY1Base (IMobileBase)
│   │   ├── dg5f_hand.py             # DG5FHand (ISlaveHand)
│   │   └── realsense_camera.py      # RealSenseCamera (ICameraStream)
│   │
│   ├── simulators/                  # 시뮬레이터 구현체
│   │   ├── __init__.py
│   │   ├── mujoco_sim.py            # MuJoCoSimulator + MuJoCoArm + MuJoCoBase + MuJoCoHand
│   │   ├── sim_camera_stream.py     # SimCameraStream (ICameraStream, MuJoCo 렌더링)
│   │   ├── simulated_tracker.py     # SimulatedTracker (IMasterTracker, 합성 포즈)
│   │   ├── simulated_hand.py        # SimulatedHand (IHandInput, 합성 관절)
│   │   ├── dummy_tracker_pub.py     # 더미 ROS2 트래커 퍼블리셔 (양손+허리+양발)
│   │   ├── dummy_hmd_pub.py         # 더미 HMD 오리엔테이션 퍼블리셔
│   │   ├── dummy_glove_pub.py       # 더미 글러브 관절 퍼블리셔
│   │   └── mujoco_ros2_bridge.py    # MuJoCo ↔ ROS2 물리 상태 브릿지
│   │
│   ├── solvers/                     # IK 솔버 등 알고리즘
│   │   ├── __init__.py
│   │   ├── pink_ik_solver.py        # PinkIKSolver (프로덕션, differential IK)
│   │   └── proportional_mapper.py   # SimpleProportionalMapper (폴백 IK, Pink 불필요)
│   │
│   ├── gui/                         # GUI 제어 패널
│   │   ├── __init__.py
│   │   ├── control_panel.py         # Dear PyGui 기반 (4탭: Status/Tracker/Joints/Params)
│   │   └── gui_node.py              # ROS2 Node 래퍼 (토픽 구독 + DearPyGui 메인루프)
│   │
│   ├── calibration/                 # 트래커 캘리브레이션
│   │   ├── __init__.py
│   │   ├── pose_calibrator.py       # A-Pose 캘리브레이션 상태 머신 + 오프셋 계산
│   │   └── calibration_node.py      # ROS2 서비스/퍼블리셔 노드
│   │
│   ├── mocap/                       # 모션 캡처 재생 인프라
│   │   ├── __init__.py
│   │   ├── bvh_loader.py            # BVH 파일 파서 + 좌표 변환
│   │   ├── skeleton_mapper.py       # BVH 조인트 → TrackerRole 매핑
│   │   ├── bvh_tracker_adapter.py   # IMasterTracker 구현 (BVH 재생)
│   │   ├── bvh_hand_adapter.py      # IHandInput 구현 (BVH 손 데이터)
│   │   ├── bvh_replay_publisher.py  # ROS2 BVH 데이터 퍼블리셔
│   │   ├── metrics.py               # 트래킹 에러, 스무스니스 등 메트릭
│   │   ├── skeleton_viewer.py       # Matplotlib 3D 스켈레톤 시각화
│   │   └── dual_viewer.py           # 병렬 비교 뷰어
│   │
│   └── utils/                       # 유틸리티
│       ├── __init__.py
│       ├── transforms.py            # 좌표 변환, 쿼터니언 연산 (순수 numpy)
│       ├── ros2_helpers.py          # QoS 프로파일, 토픽/서비스 이름 상수
│       ├── config_loader.py         # Hydra/OmegaConf 설정 로더
│       └── logger.py                # 모듈별 로거 팩토리
│
├── launch/                          # ROS2 launch 파일
│   ├── teleop_full.launch.py        # 전체 시스템 실행 (HW 모드)
│   ├── teleop_sim.launch.py         # 시뮬레이션 모드 실행
│   ├── teleop_sim_full.launch.py    # 시뮬레이션 전체 (Master+Slave)
│   ├── teleop_mujoco_bridge.launch.py  # MuJoCo 브릿지 + 전체 더미 노드
│   ├── master_sim.launch.py         # 마스터 시스템 (시뮬레이션 입력)
│   ├── master_mocap.launch.py       # 마스터 시스템 (BVH 모캡 입력)
│   ├── slave_mujoco.launch.py       # 슬레이브 시스템 (MuJoCo)
│   ├── arm_only.launch.py           # 팔 모듈만 실행
│   └── hand_only.launch.py          # 핸드 모듈만 실행
│
├── models/                          # URDF, MJCF 모델 파일
│   ├── rby1/                        # RB-Y1 MJCF/URDF + 60개 이상 메시
│   │   ├── model_teleop.xml         # MuJoCo 텔레옵 전용 모델 (추천)
│   │   ├── rby1.xml / rby1.urdf     # 기본 MJCF/URDF
│   │   └── assets/                  # OBJ/충돌 메시 (LINK_1..20, NECK, PAN_TILT 등)
│   └── dg5f/                        # DG-5F URDF + 좌/우 메시
│       ├── dg5f.urdf
│       └── meshes/                  # visual/ + collision/ STL
│
├── scripts/                         # 실행 스크립트 및 예제
│   ├── run_teleop.py                # 메인 진입점 (--mode, --sim-backend, --modules)
│   ├── run_mujoco_bridge.py         # MuJoCo↔ROS2 브릿지 실행
│   ├── demo_teleop_sim.py           # 프로포셔널 IK 데모 (Pink 불필요)
│   ├── demo_mujoco_viewer.py        # MuJoCo 뷰어
│   ├── demo_pointcloud_viewer.py    # 포인트 클라우드 시각화 데모
│   ├── test_arm_teleop_standalone.py       # 팔 독립 테스트 (ROS2 불필요)
│   ├── test_hand_teleop_standalone.py      # 핸드 독립 테스트
│   ├── test_locomotion_standalone.py       # 이동부 독립 테스트
│   ├── test_camera_teleop_standalone.py    # 카메라 독립 테스트
│   ├── test_camera_streaming.py            # RGB-D 스트리밍 품질 검증
│   ├── verify_pointcloud_pipeline.py       # 포인트 클라우드 파이프라인 검증 (headless)
│   └── examples/
│       ├── test_ik_solver.py        # IK 솔버 API 데모
│       └── test_multichain_ik.py    # 3체인 IK 수렴 데모
│
├── tests/                           # Phase별 단위/통합 테스트 (160개)
│   ├── test_transforms.py           # Phase 1: 쿼터니언/오일러/프레임 변환
│   ├── test_interfaces.py           # Phase 1: 인터페이스 데이터클래스
│   ├── test_config.py               # Phase 1: 설정 로딩
│   ├── test_arm_teleop.py           # Phase 2: ArmController 단위 테스트
│   ├── test_simulation.py           # Phase 2: MuJoCo 시뮬레이터
│   ├── test_phase3_multichain.py    # Phase 3: 3체인 IK 통합
│   ├── test_phase4_locomotion_hand.py  # Phase 4: 보행 감지 + 핸드 리타겟팅
│   ├── test_phase5_camera_gui.py    # Phase 5: 카메라 + GUI
│   ├── test_phase5_camera_head.py   # Phase 5: 헤드 트래킹 + 포인트 클라우드
│   └── test_phase6_devices.py       # Phase 6: 하드웨어 드라이버 (SDK 없이 동작)
│
├── setup.py                         # Python 패키지 설정 + ROS2 entry points
├── setup.cfg
├── package.xml                      # ROS2 패키지 메타데이터
├── requirements.txt
├── CLAUDE.md                        # AI 코딩 가이드
├── PRD.md                           # 요구사항 정의서
├── TRD.md                           # 기술 사양서
├── Tasks.md                         # 개발 태스크 목록
└── CHANGE_LOG.md                    # 변경 이력
```

> **참고**: 초기 계획에 있던 `docker/`, `CMakeLists.txt`, `README.md`는 아직 미생성.
> `rby1_base.py`는 별도 파일 대신 `rby1_arm.py`에 `RBY1Base` 클래스로 통합됨.
> `isaac_lab_sim.py`는 아직 stub 상태 (Isaac Lab 연동은 Phase 6 진행 중).

---

## 5. ROS2 토픽/서비스 설계

### 5.1 주요 토픽

| 토픽명 | 메시지 타입 | 발행자 | 구독자 | 주기 |
|--------|-----------|--------|--------|------|
| `/master/tracker/{left,right,waist}` | `geometry_msgs/PoseStamped` | Vive Tracker Node | Arm Teleop | 100Hz |
| `/master/tracker/{left,right}_foot` | `geometry_msgs/PoseStamped` | Vive Tracker Node | Locomotion | 50Hz |
| `/master/hmd/orientation` | `geometry_msgs/QuaternionStamped` | VR HMD Node | Camera Module | 90Hz |
| `/master/hand/{left,right}/joints` | `sensor_msgs/JointState` | Manus Node | Hand Teleop | 100Hz |
| `/slave/arm/{left,right}/joint_cmd` | `sensor_msgs/JointState` | Arm Teleop | Robot Driver | 100Hz |
| `/slave/torso/joint_cmd` | `sensor_msgs/JointState` | Arm Teleop | Robot Driver | 100Hz |
| `/slave/hand/{left,right}/joint_cmd` | `sensor_msgs/JointState` | Hand Teleop | Hand Driver | 100Hz |
| `/slave/base/cmd_vel` | `geometry_msgs/Twist` | Locomotion | Base Driver | 50Hz |
| `/slave/camera/rgbd` | `sensor_msgs/PointCloud2` | Camera Node | VR Renderer | 15Hz |
| `/slave/camera/pan_tilt_cmd` | `sensor_msgs/JointState` | Camera Module | Camera Driver | 30Hz |
| `/system/status` | Custom `SystemStatus` | All Modules | GUI | 10Hz |

### 5.2 서비스

| 서비스명 | 용도 |
|---------|------|
| `/teleop/set_mode` | 시뮬레이션/실로봇 모드 전환 |
| `/teleop/enable_module` | 개별 모듈 활성화/비활성화 |
| `/teleop/calibrate` | Vive Tracker 캘리브레이션 |

---

## 6. AI 코딩 주의사항

### 반드시 준수할 사항

1. **인터페이스 우선**: 모든 HW 의존 코드는 반드시 `interfaces/` 디렉토리의 ABC를 상속받아 구현할 것. 직접 HW 호출 금지.

2. **모듈 독립성**: 각 모듈(`arm_teleop`, `locomotion`, `hand_teleop`, `camera`)은 독립적으로 실행 가능해야 함. 모듈 간 직접 임포트 금지, 반드시 ROS2 토픽/서비스를 통해 통신.

3. **설정 외부화**: 하드코딩된 파라미터 금지. 모든 설정값은 `config/` 디렉토리의 YAML 파일에서 로드.

4. **ROS2 표준 준수**:
   - 노드 생명주기 관리(Lifecycle Node) 사용 권장
   - QoS 설정 명시적으로 지정 (센서 데이터: `BEST_EFFORT`, 명령: `RELIABLE`)
   - `rclpy.spin()` 대신 `MultiThreadedExecutor` 사용

5. **IK 설정**:
   - Pink의 `Configuration` 객체로 각 매니퓰레이터(좌팔, 우팔, 토르소) 개별 태스크 정의
   - 널스페이스 포스처 태스크를 반드시 추가하여 자연스러운 자세 유지
   - 조인트 리밋은 URDF에서 자동 로드

6. **VR 렌더링**:
   - HMD 회전은 **로컬에서 즉각 처리** (포인트 클라우드 뷰포인트 변경만)
   - RGB-D → 포인트 클라우드 변환은 **별도 스레드**에서 비동기 수행
   - 두 프로세스가 서로 블로킹하지 않도록 double buffering 사용

7. **시뮬레이터 어댑터**:
   - Isaac Lab과 MuJoCo는 동일한 인터페이스(`ISimulator`)를 구현
   - URDF 로드 경로, 물리 파라미터 등은 시뮬레이터별 YAML로 분리
   - `config/simulation/` 하위 파일 선택으로 백엔드 전환

8. **더미 입력 생성기**:
   - `SimulatedTracker`는 사전 녹화된 모션 데이터(예: CMU MoCap) 재생 또는 sin/cos 기반 주기적 모션 생성
   - 설정 파일에서 `data_source: "mocap_file"` 또는 `data_source: "synthetic"` 선택

### 실수 방지 가이드

- **좌표계 주의**: Vive Tracker(SteamVR 좌표계, Y-up) → ROS2(Z-up) → 로봇(URDF 기준) 간 변환을 `utils/transforms.py`에 집중. 각 모듈에서 좌표 변환 직접 하지 말 것.
- **쿼터니언 컨벤션**: ROS2는 `xyzw`, PyOpenVR는 `wxyz`. 반드시 변환 유틸 사용.
- **단위 통일**: 거리 = meters, 각도 = radians, 시간 = seconds
- **스레드 안전**: ROS2 콜백과 제어 루프 간 공유 데이터는 `threading.Lock` 또는 ROS2 `MutuallyExclusiveCallbackGroup` 사용
- **Isaac Lab 특이사항**: `ManusVive` 클래스는 Vive Tracker 2개까지 자동 매핑. 5개 사용 시 커스텀 매핑 로직 필요.

---

## 7. 필수 라이브러리 목록

### Python 패키지 (requirements.txt)

```
# ROS2 (apt로 설치, pip 호환용 참조)
# ros-jazzy-desktop (apt)
# ros-jazzy-ros2-control (apt)
# ros-jazzy-ros2-controllers (apt)

# IK / Robotics
pin>=2.7.0                    # Pinocchio
pin-pink>=3.4.0               # Pink IK solver
numpy>=1.26.0
scipy>=1.12.0

# VR / Tracking
openvr>=2.12.1401             # PyOpenVR (Vive Tracker)

# Simulation
mujoco>=3.4.0                 # MuJoCo (secondary sim)

# Visualization & Point Cloud
open3d>=0.19.0                # 포인트 클라우드 처리/렌더링

# GUI
dearpygui>=2.1.1              # Dear PyGui

# Configuration
hydra-core>=1.3.0
omegaconf>=2.3.0

# Utilities
transforms3d>=0.4.1           # 좌표 변환 — 주의: setup.py에 선언되어 있으나 런타임에 미사용. utils/transforms.py가 순수 numpy로 구현.
pyyaml>=6.0
```

### 시스템 패키지 (apt)

```
# ROS2 Jazzy
ros-jazzy-desktop
ros-jazzy-ros2-control
ros-jazzy-ros2-controllers
ros-jazzy-joint-state-publisher
ros-jazzy-robot-state-publisher
ros-jazzy-rviz2

# 비디오 스트리밍
gstreamer1.0-tools
gstreamer1.0-plugins-good
gstreamer1.0-plugins-bad
libgstreamer1.0-dev

# SteamVR (별도 설치)
steam
```

### Isaac Lab (별도 설치)

Isaac Lab 2.3.0은 Isaac Sim 5.1 기반으로, [공식 설치 가이드](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html)를 따를 것. Docker 사용 권장.

---

## 8. 개발 순서 (점진적 통합)

SOLID 원칙에 따라 작은 단위부터 동작을 확인하고 기능을 추가합니다.

> **[2025-01-31 Updated]** 실제 진행 상태 반영

```
Phase 1: 기반 구축  ✅ 완료
  ├─ 인터페이스 정의 (interfaces/) ✅
  ├─ 설정 시스템 (config/ + utils/config_loader.py) ✅
  ├─ 좌표 변환 유틸 (utils/transforms.py) ✅ (순수 numpy, transforms3d 미사용)
  └─ MuJoCo에서 RB-Y1 URDF 로드 및 시각화 확인 ✅

Phase 2: IK + 단일 팔 텔레오퍼레이션  ✅ 완료
  ├─ PinkIKSolver 구현 및 단위 테스트 ✅ (300/300 IK 성공)
  ├─ SimpleProportionalMapper 폴백 IK 추가 ✅ (원래 계획에 없던 추가 구현)
  ├─ SimulatedTracker로 더미 입력 → IK → MuJoCo 시뮬레이션 ✅
  ├─ ROS2 어댑터 패턴 확립 (ros2_adapters.py) ✅
  └─ 단일 팔 동작 확인 ✅ (Standalone + ROS2 파이프라인)

Phase 3: 양팔 + 토르소 통합  ✅ 완료
  ├─ 3개 매니퓰레이터 동시 IK (ChainConfig 기반 가중치) ✅
  ├─ 널스페이스 포스처 태스크 추가 ✅
  └─ 시뮬레이터에서 전체 상체 동작 확인 ✅

Phase 4: 이동부 + 핸드 추가  ✅ 완료
  ├─ 보행 감지 알고리즘 (GaitDetector) ✅ (87.6% 비제로 속도)
  ├─ 핸드 리타겟팅 (HandRetargeting 20DoF→1 그리퍼) ✅
  ├─ MuJoCoHand 어댑터 (20DoF→단일 ctrl, 20x 스케일링) ✅
  └─ 시뮬레이터에서 전신 + 핸드 동작 확인 ✅

Phase 5: VR 스트리밍 + GUI  ✅ 완료
  ├─ SimCameraStream (MuJoCo 렌더링, double buffering) ✅
  ├─ CameraController (HMD→Pan/Tilt, EMA 스무딩) ✅
  ├─ 포인트 클라우드 생성 파이프라인 (37.1Hz, 130K+ 포인트/프레임) ✅
  ├─ PointCloudViewer (GLFW+OpenGL) ✅
  ├─ GUI 제어 패널 (Dear PyGui) ✅
  └─ Launch 파일 작성 ✅

Phase 6: 실제 HW 연동  🔧 부분 완료
  ├─ ViveTracker + ViveTrackerPub 구현 ✅ (SDK 없이 graceful fallback)
  ├─ ManusGlove 구현 ✅ (SDK 없이 graceful fallback)
  ├─ RBY1Arm + RBY1Base 구현 ✅ (SDK 없이 graceful fallback)
  ├─ DG5FHand 구현 ✅ (SDK 없이 graceful fallback)
  ├─ RealSenseCamera 구현 ✅ (ROS2 토픽 기반)
  └─ Isaac Lab 시뮬레이터 연동 ⏳ (미착수)
```

**테스트 현황**: 160개 pytest 테스트 전체 통과 (Phase 1~6 커버)
