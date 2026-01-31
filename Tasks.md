# 🤖 RB-Y1 텔레오퍼레이션 시스템 — AI 코딩 착수용 프롬프트

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

### Phase 1: 기반 구축 (Foundation)

**목표**: 프로젝트 골격, 인터페이스, 설정 시스템, 유틸리티 완성

```
Task 1.1: 프로젝트 폴더 구조 생성
  - TRD.md의 "디렉토리 구조" 섹션을 정확히 따라서 폴더/파일 생성
  - setup.py, package.xml, CMakeLists.txt 기본 설정
  - requirements.txt 작성

Task 1.2: 인터페이스 정의 (interfaces/)
  - IMasterTracker, IHandInput (master_device.py)
  - ISlaveArm, ISlaveHand, IMobileBase (slave_robot.py)
  - IIKSolver (ik_solver.py)
  - ICameraStream (camera_stream.py)
  - ISimulator (simulator.py)
  - 각 인터페이스에 타입 힌트와 docstring 포함

Task 1.3: 유틸리티 구현 (utils/)
  - transforms.py: SteamVR↔ROS2↔URDF 좌표 변환, 쿼터니언 xyzw↔wxyz 변환
  - config_loader.py: Hydra 기반 YAML 설정 로더
  - ros2_helpers.py: QoS 프로파일 정의, 토픽 유틸리티
  - logger.py: 모듈별 로거 팩토리

Task 1.4: 기본 설정 파일 생성 (config/)
  - default.yaml, simulation/mujoco.yaml, teleop/arm.yaml 등
  - 로봇 URDF 경로, IK 파라미터, 제어 주기 등

Task 1.5: MuJoCo에서 RB-Y1 로드 확인
  - rby1-sdk에서 URDF 가져오기
  - MuJoCo에 로드 및 시각화 확인 스크립트 (scripts/examples/)

검증: config 로드, URDF 시각화, 좌표 변환 단위 테스트 통과
```

### Phase 2: IK + 단일 팔 텔레오퍼레이션 (Single Arm)

**목표**: IK 솔버 동작 확인 → 더미 입력으로 단일 팔 제어

```
Task 2.1: PinkIKSolver 구현 (solvers/pink_ik_solver.py)
  - IIKSolver 인터페이스 구현
  - Pinocchio로 RB-Y1 URDF 로드
  - Pink Configuration으로 end-effector 태스크 정의
  - 널스페이스 포스처 태스크 추가
  - 조인트 리밋 자동 적용

Task 2.2: SimulatedTracker 구현 (simulators/simulated_tracker.py)
  - IMasterTracker 인터페이스 구현
  - 모드 1: sin/cos 기반 주기적 모션 생성
  - 모드 2: 사전 녹화 모션 데이터 재생 (CMU MoCap 등)
  - 설정 파일에서 모드 및 파라미터 로드

Task 2.3: MuJoCo 시뮬레이터 어댑터 (simulators/mujoco_sim.py)
  - ISlaveArm 인터페이스 구현 (MuJoCoArm)
  - MuJoCo 물리 시뮬레이션 + 렌더링
  - 조인트 명령 수신 → 시뮬레이션 반영

Task 2.4: ArmTeleop 모듈 (modules/arm_teleop/)
  - ROS2 Lifecycle Node로 구현
  - 마스터 트래커 토픽 구독 → IK 풀기 → 조인트 명령 발행
  - 단일 팔(우측)만 먼저 구현

Task 2.5: 통합 테스트
  - SimulatedTracker → ArmTeleop → MuJoCoArm 파이프라인
  - MuJoCo 시각화에서 팔 움직임 확인

검증: 더미 입력으로 MuJoCo 내 RB-Y1 우측 팔이 올바르게 IK 추종하는지 시각적 확인
```

### Phase 3: 양팔 + 토르소 통합 (Full Upper Body)

**목표**: 3개 매니퓰레이터(좌팔, 우팔, 토르소) 동시 IK 제어

```
Task 3.1: 3-매니퓰레이터 IK 확장
  - PinkIKSolver에 좌팔, 우팔, 토르소 각각의 태스크 정의
  - 3개 태스크를 동시에 풀되, 충돌하는 조인트가 없도록 태스크 우선순위 설정
  - 토르소 조인트의 널스페이스 포스처 태스크 (직립 자세 유지)

Task 3.2: ArmTeleop 확장
  - 좌팔, 우팔, 토르소 3채널 트래커 입력 동시 처리
  - 각 채널 독립적 IK → 통합 조인트 명령 발행

Task 3.3: 통합 테스트
  - SimulatedTracker 3개 → ArmTeleop → MuJoCo 전체 상체 동작 확인

검증: 3개 트래커의 독립적 움직임이 상체 전체에 올바르게 반영되는지 확인
```

### Phase 4: 이동부 + 핸드 추가 (Full Body + Hands)

**목표**: AMR 이동 + DG-5F 핸드 텔레오퍼레이션

```
Task 4.1: GaitDetector 구현 (modules/locomotion/gait_detector.py)
  - 양발 Vive Tracker 포즈에서 보행 패턴 감지
  - 발 위치 변화량 → 선속도(linear_x, linear_y) 매핑
  - 발 방향 변화 → 각속도(angular_z) 매핑
  - 데드존, 스케일 팩터 등 파라미터화

Task 4.2: Locomotion 모듈 (modules/locomotion/)
  - ROS2 노드: 양발 트래커 토픽 → cmd_vel 발행
  - MuJoCo AMR 시뮬레이션 연동

Task 4.3: SimulatedHand 구현 (simulators/simulated_hand.py)
  - IHandInput 인터페이스 구현
  - 더미 손가락 관절 데이터 생성

Task 4.4: HandRetargeting 구현 (modules/hand_teleop/retargeting.py)
  - Manus Glove 관절 데이터 → DG-5F 20DoF 매핑
  - MANUS SDK의 Ergonomics 데이터 활용
  - 매핑 테이블을 config 파일로 외부화

Task 4.5: HandTeleop 모듈 (modules/hand_teleop/)
  - ROS2 노드: 글러브 토픽 → 리타겟팅 → 핸드 조인트 명령

Task 4.6: 전체 통합 테스트
  - 상체(팔+토르소) + 이동부 + 핸드 동시 동작

검증: MuJoCo에서 RB-Y1 전신 + DG-5F 핸드가 더미 입력에 따라 움직이는지 확인
```

### Phase 5: VR 스트리밍 + GUI (Visualization)

**목표**: 포인트 클라우드 VR 스트리밍, GUI 제어 패널

```
Task 5.1: PointCloudGenerator 구현 (modules/camera/pointcloud_generator.py)
  - RGB-D 이미지 → Open3D PointCloud 변환
  - 카메라 내부 파라미터(intrinsics) 기반 역투영
  - 시뮬레이터 RGB-D 렌더러에서 테스트

Task 5.2: VR 렌더러 구현 (modules/camera/vr_renderer.py)
  - 포인트 클라우드를 OpenGL/Vulkan으로 렌더링
  - HMD 오리엔테이션에 따른 뷰포인트 실시간 변경 (로컬 처리, 60fps+)
  - RGB-D 데이터 도착 시 포인트 클라우드 비동기 갱신 (별도 스레드)
  - Double buffering으로 렌더 스레드/갱신 스레드 분리

Task 5.3: Camera 모듈 (modules/camera/)
  - ROS2 노드: HMD 오리엔테이션 → 카메라 Pan-Tilt 명령
  - RGB-D 토픽 구독 → 포인트 클라우드 생성 → VR 렌더러 전달

Task 5.4: GUI 제어 패널 (gui/control_panel.py)
  - Dear PyGui 기반 단일 윈도우
  - 모듈별 연결 상태 표시 (Green/Red)
  - 모듈별 활성화/비활성화 토글 버튼
  - 시뮬레이션/실로봇 모드 전환 버튼
  - IK 게인, 속도 스케일 등 실시간 슬라이더

Task 5.5: Launch 파일 작성
  - teleop_full.launch.py: 전체 시스템
  - teleop_sim.launch.py: 시뮬레이션 전용
  - arm_only.launch.py, hand_only.launch.py: 모듈별

검증: 시뮬레이터에서 전체 파이프라인(더미 입력 → IK → 시뮬레이션 → 포인트 클라우드 → 렌더링) 동작, GUI에서 모듈 토글 가능
```

### Phase 6: 실제 HW 연동 (Hardware Integration)

**목표**: 실 장비 구현체 추가 및 검증

```
Task 6.1: ViveTracker 구현 (devices/vive_tracker.py)
  - IMasterTracker 인터페이스 구현
  - PyOpenVR로 SteamVR에서 5개 트래커 데이터 수신
  - 트래커 ID → 신체 부위 매핑 (config로 설정)

Task 6.2: ManusGlove 구현 (devices/manus_glove.py)
  - IHandInput 인터페이스 구현
  - MANUS SDK ROS2 토픽 구독
  - Ergonomics 데이터 파싱

Task 6.3: RBY1Arm / RBY1Base 구현 (devices/rby1_arm.py, rby1_base.py)
  - rby1-sdk Python API 활용
  - ISlaveArm, IMobileBase 구현

Task 6.4: DG5FHand 구현 (devices/dg5f_hand.py)
  - DELTO_M_ROS2 ros2_control 드라이버 연동
  - ISlaveHand 구현

Task 6.5: RealSenseCamera 구현 (devices/realsense_camera.py)
  - ROS2 표준 카메라 드라이버 연동
  - ICameraStream 구현
  - Pan-Tilt 조인트 제어

Task 6.6: Isaac Lab 시뮬레이터 연동 (simulators/isaac_lab_sim.py)
  - Isaac Lab 2.3의 ManusVive 디바이스 클래스 활용
  - CloudXR VR 스트리밍 연동
  - Pinocchio IK 통합 (--enable_pinocchio)

검증: 실 HW에서 전체 텔레오퍼레이션 동작 확인
```

---

## 기술 스택 요약 (빠른 참조)

| 항목 | 값 |
|------|-----|
| Python | 3.12 |
| ROS2 | Jazzy Jalisco (LTS, ~2029) |
| IK | Pink 3.4.0 + Pinocchio 2.7.0 |
| 시뮬레이터 | MuJoCo 3.4.0 (개발용) / Isaac Lab 2.3.0 (통합) |
| VR Tracking | PyOpenVR 2.12.1401 |
| 글러브 | MANUS SDK 3.0.1+ ROS2 |
| 로봇 | rby1-sdk |
| 핸드 | DELTO_M_ROS2 |
| 포인트 클라우드 | Open3D 0.19.0 |
| GUI | Dear PyGui 2.1.1 |
| 설정 | Hydra 1.3+ |

---

## 시작 명령

```
먼저 프로젝트 폴더 구조부터 잡아줘.
TRD.md의 "디렉토리 구조"를 정확히 따라서 모든 폴더와 __init__.py 파일을 생성하고,
Task 1.1부터 순서대로 진행해줘.
각 Task 완료 시마다 동작 확인 가능한 테스트를 작성해줘.
```
