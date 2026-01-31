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
┌─────────────────────────────────────────────────────────────────┐
│                        GUI Control Panel                         │
│                    (Dear PyGui, 상태 모니터링)                    │
└──────────────────────────────┬──────────────────────────────────┘
                               │ ROS2 Topics/Services
┌──────────────────────────────┴──────────────────────────────────┐
│                        ROS2 Middleware                            │
│                    (Jazzy Jalisco, DDS)                           │
├─────────┬──────────┬──────────┬──────────┬──────────────────────┤
│         │          │          │          │                       │
│  ┌──────┴───┐ ┌────┴────┐ ┌──┴───┐ ┌───┴────┐ ┌─────────────┐ │
│  │ Arm      │ │Locomotion│ │ Hand │ │ Camera │ │ Simulation  │ │
│  │ Module   │ │ Module   │ │Module│ │ Module │ │ Module      │ │
│  │          │ │          │ │      │ │        │ │             │ │
│  │ IK Solver│ │ Gait     │ │Retar-│ │ P.Cloud│ │ Isaac Lab / │ │
│  │ (Pink)   │ │ Detector │ │geting│ │ Stream │ │ MuJoCo      │ │
│  └──────┬───┘ └────┬────┘ └──┬───┘ └───┬────┘ └─────────────┘ │
│         │          │          │          │                       │
├─────────┴──────────┴──────────┴──────────┴──────────────────────┤
│                     Interface Layer (ABC)                         │
│  IMasterDevice │ ISlaveRobot │ IHandDevice │ ICameraDevice       │
│  IIKSolver     │ ILocomotionController │ ISimulator              │
├─────────────────────────────────────────────────────────────────┤
│                     Hardware / Simulator                          │
│  Vive Tracker │ Manus Glove │ RB-Y1 │ DG-5F │ RGB-D Camera     │
│  (또는 Dummy/Simulator 구현체)                                    │
└─────────────────────────────────────────────────────────────────┘
```

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

| 인터페이스 | 실제 HW 구현체 | 시뮬레이터 구현체 |
|-----------|---------------|-----------------|
| `IMasterTracker` | `ViveTracker` (PyOpenVR) | `SimulatedTracker` (모션 데이터 재생) |
| `IHandInput` | `ManusGlove` (MANUS SDK) | `SimulatedHand` (더미/모션 데이터) |
| `ISlaveArm` | `RBY1Arm` (rby1-sdk) | `IsaacLabArm` / `MuJoCoArm` |
| `ISlaveHand` | `DG5FHand` (DELTO_M_ROS2) | `IsaacLabHand` / `MuJoCoHand` |
| `IMobileBase` | `RBY1Base` (rby1-sdk) | `IsaacLabBase` / `MuJoCoBase` |
| `IIKSolver` | `PinkIKSolver` | `PinkIKSolver` (동일) |
| `ICameraStream` | `RealSenseCamera` (ROS2) | `SimCameraStream` (시뮬레이터 렌더) |

---

## 4. 디렉토리 구조

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
│   └── teleop/
│       ├── arm.yaml                 # IK 게인, 속도 제한 등
│       ├── locomotion.yaml          # 걸음 감지 파라미터
│       └── hand.yaml               # 리타겟팅 파라미터
│
├── teleop_system/                   # 메인 Python 패키지
│   ├── __init__.py
│   │
│   ├── interfaces/                  # 추상 인터페이스 (ABC)
│   │   ├── __init__.py
│   │   ├── master_device.py         # IMasterTracker, IHandInput
│   │   ├── slave_robot.py           # ISlaveArm, ISlaveHand, IMobileBase
│   │   ├── ik_solver.py             # IIKSolver
│   │   ├── camera_stream.py         # ICameraStream
│   │   └── simulator.py             # ISimulator
│   │
│   ├── modules/                     # 기능 모듈 (독립 실행 가능)
│   │   ├── __init__.py
│   │   ├── arm_teleop/              # 양팔 + 토르소 텔레오퍼레이션
│   │   │   ├── __init__.py
│   │   │   ├── arm_teleop_node.py   # ROS2 노드
│   │   │   └── arm_controller.py    # 제어 로직
│   │   ├── locomotion/              # AMR 이동부 텔레오퍼레이션
│   │   │   ├── __init__.py
│   │   │   ├── locomotion_node.py
│   │   │   └── gait_detector.py     # 보행 패턴 감지
│   │   ├── hand_teleop/             # 핸드 텔레오퍼레이션
│   │   │   ├── __init__.py
│   │   │   ├── hand_teleop_node.py
│   │   │   └── retargeting.py       # Manus → DG-5F 매핑
│   │   └── camera/                  # VR 카메라 스트리밍
│   │       ├── __init__.py
│   │       ├── camera_node.py
│   │       ├── pointcloud_generator.py
│   │       └── vr_renderer.py       # 포인트 클라우드 VR 렌더링
│   │
│   ├── devices/                     # HW 디바이스 구현체
│   │   ├── __init__.py
│   │   ├── vive_tracker.py          # IMasterTracker 구현
│   │   ├── manus_glove.py           # IHandInput 구현
│   │   ├── rby1_arm.py              # ISlaveArm 구현
│   │   ├── rby1_base.py             # IMobileBase 구현
│   │   ├── dg5f_hand.py             # ISlaveHand 구현
│   │   └── realsense_camera.py      # ICameraStream 구현
│   │
│   ├── simulators/                  # 시뮬레이터 구현체
│   │   ├── __init__.py
│   │   ├── isaac_lab_sim.py         # Isaac Lab 백엔드
│   │   ├── mujoco_sim.py            # MuJoCo 백엔드
│   │   ├── simulated_tracker.py     # IMasterTracker 시뮬레이터
│   │   └── simulated_hand.py        # IHandInput 시뮬레이터
│   │
│   ├── solvers/                     # IK 솔버 등 알고리즘
│   │   ├── __init__.py
│   │   └── pink_ik_solver.py        # Pink 기반 IK 구현
│   │
│   ├── gui/                         # GUI 제어 패널
│   │   ├── __init__.py
│   │   └── control_panel.py         # Dear PyGui 기반
│   │
│   └── utils/                       # 유틸리티
│       ├── __init__.py
│       ├── transforms.py            # 좌표 변환, 쿼터니언 연산
│       ├── ros2_helpers.py          # ROS2 토픽/서비스 유틸
│       ├── config_loader.py         # Hydra 설정 로더
│       └── logger.py                # 모듈별 로거 설정
│
├── launch/                          # ROS2 launch 파일
│   ├── teleop_full.launch.py        # 전체 시스템 실행
│   ├── teleop_sim.launch.py         # 시뮬레이션 모드 실행
│   ├── arm_only.launch.py           # 팔 모듈만 실행
│   └── hand_only.launch.py          # 핸드 모듈만 실행
│
├── models/                          # URDF, MJCF 모델 파일
│   ├── rby1/                        # RB-Y1 URDF
│   └── dg5f/                        # DG-5F URDF/메시
│
├── scripts/                         # 실행 스크립트 및 예제
│   ├── run_teleop.py                # 메인 진입점
│   └── examples/
│       ├── test_ik_solver.py        # IK 단위 테스트
│       ├── test_vive_tracker.py     # Vive 연결 테스트
│       └── test_manus_glove.py      # Manus 연결 테스트
│
├── tests/                           # 단위/통합 테스트
│   ├── test_interfaces.py
│   ├── test_arm_teleop.py
│   ├── test_locomotion.py
│   ├── test_hand_teleop.py
│   └── test_simulation.py
│
├── docker/                          # Docker 설정
│   ├── Dockerfile
│   └── docker-compose.yaml
│
├── setup.py                         # Python 패키지 설정
├── setup.cfg
├── package.xml                      # ROS2 패키지 메타데이터
├── CMakeLists.txt                   # C++ 빌드 (필요 시)
├── requirements.txt
├── PRD.md
├── TRD.md
├── Tasks.md
└── README.md
```

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
transforms3d>=0.4.1           # 좌표 변환
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

```
Phase 1: 기반 구축
  ├─ 인터페이스 정의 (interfaces/)
  ├─ 설정 시스템 (config/ + utils/config_loader.py)
  ├─ 좌표 변환 유틸 (utils/transforms.py)
  └─ MuJoCo에서 RB-Y1 URDF 로드 및 시각화 확인

Phase 2: IK + 단일 팔 텔레오퍼레이션
  ├─ PinkIKSolver 구현 및 단위 테스트
  ├─ SimulatedTracker로 더미 입력 → IK → MuJoCo 시뮬레이션
  └─ 단일 팔 동작 확인

Phase 3: 양팔 + 토르소 통합
  ├─ 3개 매니퓰레이터 동시 IK
  ├─ 널스페이스 포스처 태스크 추가
  └─ 시뮬레이터에서 전체 상체 동작 확인

Phase 4: 이동부 + 핸드 추가
  ├─ 보행 감지 알고리즘 구현 및 테스트
  ├─ 핸드 리타겟팅 구현
  └─ 시뮬레이터에서 전신 + 핸드 동작 확인

Phase 5: VR 스트리밍 + GUI
  ├─ 포인트 클라우드 생성 파이프라인
  ├─ VR 렌더링 (로컬 회전 + 비동기 갱신)
  └─ GUI 제어 패널

Phase 6: 실제 HW 연동
  ├─ Vive Tracker / Manus Glove 디바이스 드라이버
  ├─ RB-Y1 / DG-5F 실 로봇 연동
  └─ Isaac Lab 시뮬레이터 연동
```
