# RB-Y1 텔레오퍼레이션 시스템 — 개발 태스크 목록

> **[2025-01-31 Updated]** 실제 구현 결과를 반영하여 태스크 상태, 추가 구현 사항, 변경점을 업데이트함.

## 시스템 프롬프트

너는 **Python 3.12 + ROS2 Jazzy + Pink(Pinocchio) IK + MuJoCo/Isaac Lab** 기반 로보틱스 시스템의 **수석 개발자**야.

첨부한 **[요구사항 정의서(PRD.md)]**의 기능을 구현하되, **[기술 사양서(TRD.md)]**의 스택과 아키텍처를 **엄격하게** 준수해.

절대 생략하거나 너의 임의대로 판단하지 말고, 사양서에 명시된 라이브러리만 사용해.

---

## 핵심 규칙

1. **인터페이스 먼저**: 모든 HW 의존 컴포넌트는 `interfaces/` 디렉토리의 ABC를 먼저 정의하고, 구현체를 별도로 작성한다.
2. **모듈 독립성**: `modules/` 하위의 각 모듈(arm_teleop, locomotion, hand_teleop, camera)은 독립적으로 실행 가능해야 한다. 모듈 간 직접 import 금지, ROS2 토픽/서비스로만 통신한다.
3. **설정 외부화**: 모든 파라미터는 `config/` 디렉토리의 YAML 파일에서 Hydra로 로드한다. 코드 내 하드코딩 금지.
4. **좌표 변환 집중화**: 모든 좌표계 변환은 `utils/transforms.py`에서만 수행한다. 쿼터니언 컨벤션(ROS2: xyzw, OpenVR: wxyz)에 주의.
5. **점진적 개발**: 각 Phase를 순서대로 완료하고, 각 단계마다 동작 확인 가능한 테스트 코드를 작성한다.
6. **ROS2 Best Practice**: Lifecycle Node, MultiThreadedExecutor, 명시적 QoS 설정을 사용한다.
7. **SOLID 원칙**: 단일 책임, 개방-폐쇄, 리스코프 치환, 인터페이스 분리, 의존성 역전을 준수한다.

---

## 개발 태스크 (Phase별)

### Phase 1: 기반 구축 (Foundation) — ✅ 완료

**목표**: 프로젝트 골격, 인터페이스, 설정 시스템, 유틸리티 완성

```
Task 1.1: 프로젝트 폴더 구조 생성 ✅
  - TRD.md의 "디렉토리 구조" 섹션을 따라 폴더/파일 생성
  - setup.py, package.xml 기본 설정
  - requirements.txt 작성
  ※ 변경사항: CMakeLists.txt, docker/, README.md는 미생성 (필요 시 추후 추가)

Task 1.2: 인터페이스 정의 (interfaces/) ✅
  - IMasterTracker, IHandInput (master_device.py) + TrackerRole(Enum), Pose6D, HandJointState
  - ISlaveArm, ISlaveHand, IMobileBase (slave_robot.py) + ArmSide(Enum), JointState, VelocityCommand
  - IIKSolver (ik_solver.py) + IKResult
  - ICameraStream (camera_stream.py) + RGBDFrame
  - ISimulator (simulator.py) + SimState
  - 각 인터페이스에 타입 힌트와 docstring 포함

Task 1.3: 유틸리티 구현 (utils/) ✅
  - transforms.py: SteamVR↔ROS2↔URDF 좌표 변환, 쿼터니언 xyzw↔wxyz 변환
    ※ 순수 numpy 구현, transforms3d 런타임 미사용
    ※ "sxyz" Euler convention만 지원
  - config_loader.py: Hydra/OmegaConf 기반 YAML 설정 로더
  - ros2_helpers.py: QoS 프로파일 정의(SENSOR_DATA, COMMAND, STATUS, PARAMETER), TopicNames/ServiceNames 상수
  - logger.py: 모듈별 로거 팩토리 (teleop.{name} 접두사)

Task 1.4: 기본 설정 파일 생성 (config/) ✅
  - default.yaml: system.mode, robot 설정, modules 활성화, simulation.backend
  - simulation/mujoco.yaml, simulation/isaac_lab.yaml
  - teleop/arm.yaml, teleop/hand.yaml, teleop/locomotion.yaml
  - hardware/vive_tracker.yaml, hardware/manus_glove.yaml, hardware/rby1.yaml, hardware/dg5f.yaml

Task 1.5: MuJoCo에서 RB-Y1 로드 확인 ✅
  - model_teleop.xml (MuJoCo teleop 전용 모델) 로드 성공
  - demo_mujoco_viewer.py 스크립트로 시각화 확인
  - dg5f.urdf 핸드 모델도 포함

검증: ✅ config 로드, URDF 시각화, 좌표 변환 단위 테스트 통과
```

### Phase 2: IK + 단일 팔 텔레오퍼레이션 (Single Arm) — ✅ 완료

**목표**: IK 솔버 동작 확인 → 더미 입력으로 단일 팔 제어

```
Task 2.1: PinkIKSolver 구현 (solvers/pink_ik_solver.py) ✅
  - IIKSolver 인터페이스 구현
  - Pinocchio로 RB-Y1 URDF 로드 (model_pinocchio.urdf 패치 버전 생성)
  - Pink Configuration으로 end-effector 태스크 정의 (ChainConfig)
  - 널스페이스 포스처 태스크 추가
  - 조인트 리밋 자동 적용
  - QP 솔버 자동 감지 (proxqp > osqp > daqp)

Task 2.1b: SimpleProportionalMapper 구현 (solvers/proportional_mapper.py) ✅ [추가]
  - IIKSolver 인터페이스 구현 (Pink 불필요 폴백)
  - 비례 제어 기반 position→joint 매핑
  - create_ik_solver() 팩토리 함수

Task 2.2: SimulatedTracker 구현 (simulators/simulated_tracker.py) ✅
  - IMasterTracker 인터페이스 구현
  - sin/cos 기반 주기적 모션 생성
  - TrackerRole별 기본 위치 (hand, waist, foot, head)
  - 설정 가능한 amplitude, frequency, phase offset
  ※ 변경사항: CMU MoCap 재생 모드는 미구현 (합성 모션으로 충분)

Task 2.3: MuJoCo 시뮬레이터 어댑터 (simulators/mujoco_sim.py) ✅
  - MuJoCoSimulator(ISimulator) 메인 클래스
  - MuJoCoArm(ISlaveArm) — mujoco_sim.py 내부 nested class
  - MuJoCoBase(IMobileBase) — mujoco_sim.py 내부 nested class
  - MuJoCo 물리 시뮬레이션 + 렌더링
  - 조인트 명령 수신 → 시뮬레이션 반영

Task 2.4: ArmTeleop 모듈 (modules/arm_teleop/) ✅
  - arm_controller.py: 순수 제어 로직 (ROS2 의존 없음)
  - arm_teleop_node.py: ROS2 Lifecycle Node
  - ros2_adapters.py: ROS2TrackerAdapter(IMasterTracker), ROS2ArmCommandPublisher(ISlaveArm) [추가]
  ※ 설계 패턴: 순수 로직(Controller) + ROS2 래퍼(Node) + 어댑터(ros2_adapters) 3-layer 구조 확립

Task 2.5: 통합 테스트 ✅
  - test_arm_teleop_standalone.py: Standalone (ROS2 없이) MuJoCo 뷰어 테스트
  - dummy_tracker_pub.py: 더미 ROS2 트래커 퍼블리셔 [추가]
  - mujoco_ros2_bridge.py: MuJoCo↔ROS2 물리 상태 브릿지 [추가]
  - teleop_mujoco_bridge.launch.py: 전체 ROS2 파이프라인 런치 [추가]

검증: ✅ 300/300 IK 성공, ROS2 파이프라인 94 poses → 58 arm cmds → 42 joint states/5s
```

### Phase 3: 양팔 + 토르소 통합 (Full Upper Body) — ✅ 완료

**목표**: 3개 매니퓰레이터(좌팔, 우팔, 토르소) 동시 IK 제어

```
Task 3.1: 3-매니퓰레이터 IK 확장 ✅
  - PinkIKSolver에 좌팔, 우팔, 토르소 각각의 FrameTask 정의
  - ChainConfig 기반 per-chain 가중치 (position_cost, orientation_cost, lm_damping)
  - 3개 태스크 동시 QP 솔빙
  - 토르소 조인트 널스페이스 포스처 태스크 (직립 자세 유지)
  - per-chain ChainError 에러 추적

Task 3.2: ArmTeleop 확장 ✅
  - 좌팔, 우팔, 토르소 3채널 트래커 입력 동시 처리
  - per-chain joint index 매핑으로 올바른 조인트 디스패치
  - Workspace offset 캘리브레이션 (enable 시)
  - 각 채널 독립적 IK → 통합 조인트 명령 발행

Task 3.3: 통합 테스트 ✅
  - test_phase3_multichain.py: 3체인 IK 초기화, per-chain 가중치, 수렴, joint slicing, 트래커 연결 해제 핸들링, 전체 파이프라인
  - SimulatedTracker 3개 → ArmTeleop → MuJoCo 전체 상체 동작 확인

검증: ✅ 3개 트래커의 독립적 움직임이 상체 전체에 올바르게 반영됨
```

### Phase 4: 이동부 + 핸드 추가 (Full Body + Hands) — ✅ 완료

**목표**: AMR 이동 + DG-5F 핸드 텔레오퍼레이션

```
Task 4.1: GaitDetector 구현 (modules/locomotion/gait_detector.py) ✅
  - 양발 Vive Tracker 포즈에서 보행 패턴 감지
  - 발 위치 변화량 → 선속도(linear_x, linear_y) 매핑
  - 발 방향 변화 → 각속도(angular_z) 매핑
  - 데드존, 스케일 팩터, 스무딩 윈도우 파라미터화

Task 4.1b: LocomotionController 구현 (modules/locomotion/locomotion_controller.py) ✅ [추가]
  - 순수 제어 로직 (GaitDetector 래핑)
  - 캘리브레이션, enable/disable, recalibration 관리

Task 4.2: Locomotion 모듈 (modules/locomotion/) ✅
  - locomotion_node.py: ROS2 Lifecycle Node (50Hz 제어 루프)
  - ros2_adapters.py: ROS2BaseCommandPublisher(IMobileBase) [추가]
  - MuJoCo AMR differential drive 시뮬레이션 연동
  ※ 핵심 수정: 첫 트래커 데이터 수신 시까지 enable 지연 (ROS2 스타트업 레이스 컨디션 해결)

Task 4.3: SimulatedHand 구현 (simulators/simulated_hand.py) ✅
  - IHandInput 인터페이스 구현
  - open/close 사이클 + 손가락별 phase delay
  - 원위부 관절은 근위부보다 적게 움직임 (생체역학 반영)

Task 4.3b: DummyGlovePub 구현 (simulators/dummy_glove_pub.py) ✅ [추가]
  - ROS2 노드: 합성 20DoF 핸드 관절 데이터 퍼블리시

Task 4.4: HandRetargeting 구현 (modules/hand_teleop/retargeting.py) ✅
  - Manus Glove 관절 데이터 → DG-5F 20DoF 매핑
  - linear_scale() 직접 매핑 방식
  - 매핑 테이블을 config 파일(hand.yaml)로 외부화

Task 4.4b: HandController 구현 (modules/hand_teleop/hand_controller.py) ✅ [추가]
  - 순수 제어 로직 (HandRetargeting 래핑)
  - 속도 제한, 지수 스무딩

Task 4.5: HandTeleop 모듈 (modules/hand_teleop/) ✅
  - hand_teleop_node.py: ROS2 Lifecycle Node (100Hz, dual left+right)
  - ros2_adapters.py: ROS2GloveAdapter(IHandInput), ROS2HandCommandPublisher(ISlaveHand) [추가]

Task 4.5b: MuJoCoHand 어댑터 (simulators/mujoco_sim.py 내부) ✅ [추가]
  - MuJoCoHand(ISlaveHand) nested class
  - 20DoF → 단일 그리퍼 ctrl 값 (mean + 20x 스케일링)

Task 4.6: 전체 통합 테스트 ✅
  - test_phase4_locomotion_hand.py: GaitDetector, LocomotionController, HandRetargeting, HandController 테스트
  - test_hand_teleop_standalone.py, test_locomotion_standalone.py: Standalone 통합 테스트
  - dummy_tracker_pub.py에 양발 트래커 추가 (anti-phase 보행 패턴)
  - teleop_mujoco_bridge.launch.py에 locomotion + hand 노드 추가

검증: ✅ Standalone: 500/500 핸드 명령, 87.6% 비제로 속도. ROS2: 296 foot poses → 112 cmd_vel, 143 hand cmds/6s
```

### Phase 5: VR 스트리밍 + GUI (Visualization) — ✅ 완료

**목표**: 포인트 클라우드 VR 스트리밍, GUI 제어 패널

```
Task 5.1: PointCloudGenerator 구현 (modules/camera/pointcloud_generator.py) ✅
  - RGB-D 이미지 → Open3D PointCloud 변환
  - 카메라 내부 파라미터(intrinsics) 기반 역투영
  - MuJoCo 시뮬레이터 RGB-D 렌더러에서 테스트
  - 130K+ 포인트/프레임

Task 5.1b: SimCameraStream 구현 (simulators/sim_camera_stream.py) ✅ [추가]
  - ICameraStream 인터페이스 구현 (MuJoCo 렌더링)
  - Pan-Tilt 제어 via ctrl[22:24]
  - intrinsics를 fovy + render dimensions에서 계산
  - Double buffering

Task 5.2: PointCloudViewer 구현 (modules/camera/pointcloud_viewer.py) ✅
  ※ 변경사항: 원래 계획의 vr_renderer.py 대신 pointcloud_viewer.py로 구현
  - GLFW + OpenGL 기반 포인트 클라우드 뷰어
  - 마우스 오빗, 실시간 스탯 표시
  ※ headless 환경에서는 verify_pointcloud_pipeline.py로 2D 프로젝션 검증 가능

Task 5.2b: CameraController 구현 (modules/camera/camera_controller.py) ✅ [추가]
  - 순수 제어 로직 (HMD quaternion → euler → pan/tilt)
  - EMA 스무딩 + velocity limiting

Task 5.3: Camera 모듈 (modules/camera/) ✅
  - camera_node.py: ROS2 Lifecycle Node (30Hz, HMD → Pan-Tilt 명령)
  - ros2_adapters.py: ROS2HMDAdapter(IMasterTracker), ROS2CameraAdapter(ICameraStream) [추가]
  - dummy_hmd_pub.py: 더미 HMD 오리엔테이션 퍼블리셔 (90Hz) [추가]

Task 5.4: GUI 제어 패널 (gui/control_panel.py) ✅
  - Dear PyGui 기반 (dearpygui import, SDK 없을 시 graceful fallback)
  - ModuleStatus 데이터클래스
  - 모듈별 연결 상태 표시, 활성화/비활성화 토글

Task 5.5: Launch 파일 작성 ✅
  - teleop_full.launch.py: 전체 시스템 (HW 모드)
  - teleop_sim.launch.py: 시뮬레이션 전용
  - teleop_mujoco_bridge.launch.py: MuJoCo 브릿지 + 전체 더미 노드 [추가]
  - arm_only.launch.py, hand_only.launch.py: 모듈별

검증: ✅ 160/160 테스트 통과. 37.1Hz 파이프라인 throughput. 포인트 클라우드 130K+/프레임. Pan/Tilt 정상 추종.
```

### Phase 6: 실제 HW 연동 (Hardware Integration) — 🔧 부분 완료

**목표**: 실 장비 구현체 추가 및 검증

```
Task 6.1: ViveTracker 구현 (devices/vive_tracker.py) ✅
  - ViveTracker(IMasterTracker) + ViveTrackerManager 구현
  - PyOpenVR로 SteamVR에서 트래커 데이터 수신
  - 트래커 시리얼 → TrackerRole 매핑 (config)
  - try/except ImportError로 SDK 없이 graceful fallback
  ※ 추가: vive_tracker_pub.py — Vive Tracker → ROS2 PoseStamped 퍼블리셔

Task 6.2: ManusGlove 구현 (devices/manus_glove.py) ✅
  - IHandInput 인터페이스 구현
  - MANUS SDK ROS2 토픽 구독
  - Ergonomics 데이터 파싱
  - SDK 없이 graceful fallback

Task 6.3: RBY1Arm / RBY1Base 구현 ✅
  ※ 변경사항: rby1_base.py 별도 파일 대신 rby1_arm.py에 RBY1Base 통합
  - rby1_arm.py: RBY1Arm(ISlaveArm) + RBY1Base(IMobileBase)
  - rby1-sdk Python API 활용
  - SDK 없이 graceful fallback

Task 6.4: DG5FHand 구현 (devices/dg5f_hand.py) ✅
  - DELTO_M_ROS2 ros2_control 드라이버 연동
  - ISlaveHand 구현
  - SDK 없이 graceful fallback

Task 6.5: RealSenseCamera 구현 (devices/realsense_camera.py) ✅
  - ROS2 표준 카메라 토픽 구독 (/camera/color/image_raw, /camera/aligned_depth_to_color/image_raw)
  - ICameraStream 구현
  - set_orientation()이 /slave/camera/pan_tilt_cmd로 퍼블리시

Task 6.6: Isaac Lab 시뮬레이터 연동 (simulators/isaac_lab_sim.py) ⏳ 미착수
  - Isaac Lab 2.3의 ManusVive 디바이스 클래스 활용
  - CloudXR VR 스트리밍 연동
  - Pinocchio IK 통합 (--enable_pinocchio)

검증: 테스트 160/160 통과 (SDK 없이 graceful fallback 정상 동작)
실 HW 검증은 실제 장비 환경에서 수행 필요.
```

---

## 추가 구현 사항 (원래 태스크에 없던 것들)

개발 과정에서 아키텍처 개선을 위해 추가된 구현 사항:

### 아키텍처 패턴
1. **ros2_adapters.py 패턴**: 각 모듈에 ROS2 어댑터 레이어 추가 — 인터페이스 ABC를 ROS2 토픽으로 래핑하여 순수 로직(Controller)과 ROS2(Node)를 완전 분리
2. **Controller 레이어**: 각 모듈에 순수 Python 컨트롤러 클래스 추가 (arm_controller, hand_controller, locomotion_controller, camera_controller) — ROS2 없이 독립 테스트 가능
3. **Standalone 테스트 스크립트**: ROS2 없이 전체 파이프라인을 검증하는 standalone 스크립트 6종

### 시뮬레이션/브릿지
4. **SimpleProportionalMapper**: Pink/Pinocchio 없이 동작하는 폴백 IK 솔버
5. **mujoco_ros2_bridge.py**: MuJoCo 물리 상태를 ROS2 토픽으로 동기화하는 브릿지 노드
6. **더미 퍼블리셔 3종**: dummy_tracker_pub, dummy_hmd_pub, dummy_glove_pub — ROS2 파이프라인 테스트용 합성 입력
7. **MuJoCoHand**: 20DoF 핸드 명령을 단일 그리퍼 ctrl로 변환하는 어댑터

### 검증/테스트
8. **verify_pointcloud_pipeline.py**: headless 환경에서 포인트 클라우드 파이프라인을 2D 프로젝션으로 검증
9. **test_camera_streaming.py**: RGB-D 스트리밍 품질 + FPS 벤치마크

---

## 기술 스택 요약 (빠른 참조)

| 항목 | 값 |
|------|-----|
| Python | 3.12 |
| ROS2 | Jazzy Jalisco (LTS, ~2029) |
| IK | Pink 3.4.0 + Pinocchio 2.7.0 (+ SimpleProportionalMapper 폴백) |
| 시뮬레이터 | MuJoCo 3.4.0 (주력) / Isaac Lab 2.3.0 (미착수) |
| VR Tracking | PyOpenVR 2.12.1401 |
| 글러브 | MANUS SDK 3.0.1+ ROS2 |
| 로봇 | rby1-sdk |
| 핸드 | DELTO_M_ROS2 |
| 포인트 클라우드 | Open3D 0.19.0 |
| GUI | Dear PyGui 2.1.1 |
| 설정 | Hydra 1.3+ (OmegaConf) |

---

## 남은 태스크 (Next Steps)

### 미완료 항목
- [ ] Isaac Lab 시뮬레이터 연동 (Task 6.6)
- [ ] Docker 설정 (Dockerfile, docker-compose.yaml)
- [ ] README.md 작성
- [ ] CMakeLists.txt (C++ 모듈 필요 시)

### Post-MVP 고도화 (PRD 참조)
- [ ] 햅틱 피드백 (Manus Glove 진동 피드백)
- [ ] 3D Gaussian Splatting 기반 고품질 VR 렌더링
- [ ] 충돌 감지 및 안전 제한 (Force Feedback)
- [ ] 다중 오퍼레이터 지원
- [ ] 데이터 로깅 및 이미테이션 러닝 데이터 수집
- [ ] 네트워크 지연 보상 (예측 기반)
- [ ] transforms3d를 setup.py에서 제거 (런타임 미사용)

### 테스트 현황
- **총 테스트**: 160개 pytest
- **커버리지**: Phase 1~6 전체
- **실행**: `MUJOCO_GL=egl python3 -m pytest tests/ -v`
