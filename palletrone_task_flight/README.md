# Palletrone task flight

독립 신규 패키지이다. 기존 px4_ros_com, quadrotor_controller, px4_msgs, OptiTrack,
allocation, DOB, payload, ServoAngle 코드를 수정하지 않는다. 소스와 설치 메시지를
검사하고 단위 테스트/별도 경로 빌드를 수행해도 실제 비행 검증을 대신하지 않는다.
기체에 탑재한 PX4의 최근 position-only 구현과 DDS 매핑은 별도 확인 대상이다.

## 기본 동작

`task_flight.launch.py`는 비행 노드만 실행한다. Agent, 모캡, allocation, 드라이버를
자동 실행하지 않는다. 두 YAML 모두 `dry_run: true`이고 J1 보정은 미확인 상태이다.

DISARMED 관측 → 유효한 local position/heading/status/J1 → 준비 자세 확인 →
현재 위치 목표와 heartbeat 1.5초 사전 송신 → READY → **사용자가 QGC에서 Offboard
선택 및 Arm** → 실제 ARMED+OFFBOARD 확인 → 시작 P0/yaw0 저장 및 계획 재검증 →
상승 → 초기 hover → 자세1의 0/30/60도 접근·목표 유지·후퇴 → 60도에서 자세2 →
60/30/0도 접근·목표 유지·후퇴 → 수평 봉 복귀 → 초기 hover 안정화 → NAV_LAND 요청 →
실제 AUTO_LAND 인계. AUTO_LAND는 착륙 완료가 아니다. 자동 Disarm/Kill/재Arm은 없다.
QGC **Takeoff는 AUTO_TAKEOFF 명령이므로 이 임무의 시작 방법이 아니다.**

## 기하학과 초기 배치

local NED와 body FRD를 사용한다. **작업 중 roll/pitch가 수평인 fully actuated
운용을 전제**한다. 이 노드는 roll/pitch 명령을 생성하거나 실제 수평 여부를 감시하지
않는다. 실제 기체 자세 편차, 봉 휨, 위치 오차가 있으면 실제 M/E/C 정렬은 달라진다.

P는 PX4 기준점, B는 기체 하단 중앙, M은 J1 회전축, E는 봉 끝, C는 고정 목표점이다.
`m = px4_reference_to_bottom_frd_m + j1_mount_from_bottom_frd_m`.
`u(theta)=[cos(theta),0,sin(theta)]`, `Rz(yaw)`는 FRD→NED yaw 회전이다.

```
M = P + Rz*m
E = P + Rz*(m + L*u)
P_sp = C - Rz*(m + (L+d)*u)
E_sp = C - d*Rz*u
P_hover = P0 + [0,0,-hover_height_m]
C = P_hover + Rz(yaw0)*(m+(L+d_standoff)*u(pose1))
```

C는 실제 시작을 확인한 시점에 한 번 계산한다. **실제 물체를 인식한 접촉점이 아니다.**
사용자가 초기 위치/heading/높이를 목표물에 맞춰 배치해야 한다. 초기 수직 상승은 작업
자세에 도달하기 위한 별도 단계이다. M–E–C 목표 정렬 보장은 초기 hover부터 작업 경로에
적용된다. 초기 상승 중까지 고정 C와의 정렬을 유지한다는 뜻은 아니다.

작업 이동에서는 d, yaw, theta 중 해당 변수만 quintic smoothstep으로 보간한다.
독립적인 XYZ clipping은 하지 않는다. C는 고정되고 자세 전환 중 E도 움직인다.
접근은 d를 0까지만 줄이며 C 너머로 밀지 않는다. `AT_CONTACT_TARGET`는 목표 도달이고
힘 센서에 의한 접촉 검출이 아니다. 물체와의 힘/충돌 제어는 구현하지 않았다.

기본값: 30도 접근은 수평 +0.259808m, NED z +0.15m(하강)이다.
0→30도 대기 자세 전환은 전방 +0.120577m, NED z -0.45m(상승)이다.
초기 상승 0.5m일 때 **계획상 최대 상승량은 0.95m**이다. P와 B가 다르면 두 오프셋을
실측해서 입력한다. 초기 yaw=0 기준 기본 계획의 최대 수평 이동 반경은 약 0.79m이다.
실제 비행의 overshoot/안전 여유를 이 수치에 포함하지 않았다.

## 파라미터

전체 기본값은 `config/task_flight.yaml`에 있다. 시작 시 읽기 전용 파라미터로 선언하므로
변경은 다음 프로세스 실행부터 반영한다. 설정값은 실비행 검증값이 아니다.

| 항목 | 기본값 / 의미 |
|---|---|
| dry_run | true: 실제 PX4/J1 명령 publisher를 만들지 않음 |
| hover_height_m / climb_rate_mps | 0.5m / 0.2m/s. 초기 상대 상승과 목표 최대 상승 속도 |
| initial_hover_duration_s / final_hover_duration_s | 10s / 3s, 실제 오차 조건을 유지하는 연속 시간 |
| rod_length_m / standoff_distance_m | 0.60m / 0.30m. d는 봉 방향의 실제 3D 거리 |
| px4_reference_to_bottom_frd_m | [0,0,0]은 초기 제안이며 실측 필요 |
| j1_mount_from_bottom_frd_m | [-0.11,0,0]m |
| pose1_angle_deg / pose2_angle_deg | 0 / 30. pose1=0은 수평 착륙 자세로 고정 검증 |
| azimuth_offsets_deg | 시작 yaw 기준 [0,30,60], 서로 다른 세 값, 첫 값 0 |
| reverse_second_pass | true: 두 번째 순서는 60/30/0. false: 0/30/60 후 초기 방위각 복귀 |
| approach/retract/orbit/posture_change_duration_s | 각 10s. 전체 구간 속도 제한을 넘으면 자동 연장 |
| contact_hold_duration_s / segment_settle_time_s | 2s / 1s. 실제 위치/yaw/J1 모두 허용 오차 필요 |
| position/yaw/joint tolerance | 0.05m / 3도 / 2도 |
| publish_rate_hz / max_step_dt_s | 20Hz / 0.1s. 콜백 지연으로 경로를 건너뛰지 않음 |
| position/status/joint_feedback_timeout_s | 0.5 / 2.0 / 0.5s. 수신 시각은 monotonic |
| control_timeout_s / joint_tracking_timeout_s | 0.3 / 2s |
| climb_timeout_s | 상승 전체 허용 wall time 30s |
| segment_arrival_timeout_s | 각 단계의 조정된 reference duration 외 추가 허용 wall time 30s |
| mission_timeout_s | 실제 시작 이후 전체 작업 600s; 상승 timeout과 별개 |
| max_reference_speed_mps | 0.2m/s, 모든 이동의 분석적 속도 상한 |
| max_reference_yaw_rate_deg_s / max_joint_rate_deg_s | 10 / 15도/s |
| require_heading_good_for_control | false여도 NaN/Inf heading 거부. PX4 preflight check 해제 아님 |
| prestream_duration_s / discovery_wait_s | 1.5 / 2s |
| land_retry_interval_s / land_max_attempts | 2s / 3회, 실제 nav_state 확인 |
| target_system/component / source_system/component ID | 1/1, 245/191 |

smoothstep의 최대 미분 1.875와 봉/장착 길이를 이용해 전 구간 속도·각속도를 제한한다.
시작/끝 목표 속도는 0이다. 시간 연장 후 전체 nominal duration이 mission timeout을 넘으면
설정 오류로 거부한다. reference duration은 실제 추종, 정착 및 중간 hold 때문에 더 길어질 수 있다.

## 피드백과 중단 정책

| 상황 | 동작 |
|---|---|
| 준비 입력/보정/수평 J1 미확인 | 임무와 명령 송신 시작 금지 |
| 노드 시작 당시 Armed | 먼저 Disarmed 관측 필요. READY 이전 Arm은 중단 |
| 위치/yaw/J1이 직전 목표를 추종하지 못함 | 진행률 정지. 목표를 계속 멀리 보내지 않음 |
| 단계 끝 | 실제 XYZ/yaw/J1 오차가 모두 만족된 상태로 1s 정착 필요 |
| J1 지연 2s, J1 피드백/ready 손실 | FAULT_HOLD 고정. PX4 위치/상태 정상 시 마지막 유효 위치·yaw와 heartbeat 유지. J1 명령·Land 금지 |
| 단계/전체 작업 timeout | FAULT_HOLD 고정, 정상 임무 완료처럼 Land하지 않음 |
| 위치/상태 timeout·무효, EKF 위치/heading/reference reset | ABORTED, 목표/heartbeat/J1/Land 송신 중단. 이전 좌표 재사용·reset 보정 없음 |
| RC/QGC 모드 이탈·Disarm·failsafe·제어 루프 지연 | ABORTED, 자동 재시작·재진입 없음 |
| 정상 작업 종료 | 봉 수평과 최종 hover 도달 확인 후 Land 요청 |
| Land ACK accepted | 아직 인계 아님. AUTO_LAND 관측 전 hold/heartbeat 유지 |
| Land 재시도 소진 | LAND_FAILED_HOLD. 건강한 Offboard hold 유지, 사용자 개입 필요 |
| 실제 AUTO_LAND | 모든 임무 명령 중단. 착륙 완료/접지/자동 Disarm은 관측하지 않음 |

FAULT_HOLD에서 J1이 회복되어도 임무를 재개하지 않는다. 사용자 RC/QGC 모드 인계가 필요하다.
PX4 입력마저 상실하면 FAULT_HOLD도 ABORTED로 전환한다. heartbeat 중단 후 실제 동작은
기체의 Offboard-loss failsafe 설정에 의존하며 이 패키지가 파라미터를 변경하지 않는다.
스틱 움직임만으로 override된다고 가정하지 말고 실제 RC 모드 스위치/QGC 모드 전환을 확인한다.

PX4 timestamp/sample timestamp, J1 JointState timestamp가 같으면 신선도를 갱신하지 않는다.
각각 증가한 실제 수신만 인정하며 역행은 중단한다. 원시 mocap/GPS 단절 중에도 EKF가
유효한 local position을 계속 발행하면 이 노드는 EKF 유효성 판단을 따른다.
GPS/global/Home/RTK fixed를 노드 준비 조건으로 요구하지 않는다. 실내 yaw 관측원, EV frame
정렬, 실외 RTK 전환/EKF reset 및 실제 estimator 품질은 별도 검증 대상이다.

## J1 및 기존 ID1..4 계약

**기존 `quadrotor_controller/dynamixel_fix`와 새 드라이버를 함께 실행하지 않는다.**
새 드라이버가 같은 버스의 유일한 소유자이다. 프로세스 fd 검사, 장치별 flock,
Linux TIOCEXCL, ROS publisher 중복 검사를 사용한다. 권한으로 보이지 않는 다른 프로세스나
CAP_SYS_ADMIN 소유자의 우회까지 보장하는 보안 장치는 아니므로 사용자가 실제 포트 소유자를 확인한다.
실행 중인 기존 드라이버를 이 패키지가 종료하지 않는다. 교체는 지상에서 사용자가 수행한다.

ID1..4: `servo_angle_cmd` 길이 5의 앞 네 값(rad) → 기존 3.14/4096 변환 → ID1..4.
`servo_angle` 길이 5 앞 네 값은 실제 성공한 읽기의 기존 역변환이다.
다섯 번째 입력은 기존 payload 슬롯으로 유지하고 **J1에 연결하지 않는다**.
기준 코드에서 ID5 read/write가 비활성화되어 다섯 번째 feedback은 기존처럼 raw=0의
변환값(-3.14)으로 남는다. 이는 J1 측정값이 아니다. PX4 ServoAngle에 J1을 추가하지 않는다.

| J1 토픽 | 의미 |
|---|---|
| /j1_angle_cmd Float32 | 물리 theta 목표 rad, 실제 구동에 사용 |
| /j1_angle Float32 | 성공한 실제 읽기의 물리 theta rad |
| /j1_state JointState | 동일 실제 측정 + ROS timestamp, name=[j1], position=[theta]. 임무가 사용하는 피드백 |
| /j1_ready Bool | 보정/초기화/읽기 성공 상태; 오류 시 false로 고정 |
| /j1_limits Float64MultiArray | [물리 최소 rad, 최대 rad, 최대 각속도 rad/s]. 계획과 보정 범위가 맞는지 사전 검사 |
| /j1_angle_target Float32 | rate-limit 후 보낸 목표 로그. 실제 측정이 아님 |

Float32만으로는 동일값 재전송과 새로운 측정을 구분할 수 없어, 임무 판단에는 stamped
JointState를 사용한다. 읽기 실패 시 이전 측정을 새 timestamp로 발행하지 않는다.
J1 limits도 성공한 읽기 때만 발행한다. 계획 자세 또는 task 최대 J1 각속도가 드라이버
범위를 벗어나면 준비를 차단한다. 실행 중 limits가 사라지거나 맞지 않으면 FAULT_HOLD이다.

라이브 시작 조건: dry_run=false, calibration_confirmed=true, control_table_confirmed=true,
expected_j1_model_number가 실제 모델과 일치, J1/ID1..4 모두 Position mode(3), J1 torque OFF.
초기 실제 raw tick 읽기 → 범위 검사 → 동일 raw 목표 기록 → J1 torque enable 확인 순서이다.
임의 0도/0tick 명령으로 초기화하지 않는다. ID1..4도 현재 위치로 목표를 초기화한다.
기존 PID, allocation, DOB 파라미터를 변경하지 않으며 운전 모드도 자동 변경하지 않는다.

ID1..4 I/O는 100Hz 타이머의 sync write/read, J1은 기본 20Hz로 한 포트에서 직렬 처리한다.
ID1..4 read timeout 기본 4ms, J1 read 두 건 각각 2ms로 제한한다. 실제 USB latency가
이 값에 맞는지는 실측 필요하며 hard real-time 보장이 아니다. J1 오류 후 추가 J1 통신을
중단하고 ready=false만 발행하며 ID1..4 처리를 계속한다. TxOnly 성공은 모터가 명령을
수락/추종했다는 증거가 아니므로 실제 읽기와 임무 추종 조건으로 확인한다.
명령 timeout 시 새 목표 전송을 멈추며 기존 위치 목표를 유지한다. J1 오류나 프로세스 종료만으로
ID1..4 torque를 끄지 않는다. 종료 후 남은 토크 상태는 사용자가 관리해야 한다.

### 실제 보정 전에 확인할 항목

이번 구현/검증에서는 시리얼 포트를 열거나 모터를 움직이지 않는다. 향후 별도 벤치 작업에서:

1. 실제 모터 모델, Protocol 2.0, control table 주소/단위, baudrate, ID 충돌을 확인한다.
   기본 주소 mode=11, torque=64, goal=116, present=132는 기존 드라이버 계약에서 가져온 것이며
   새로운 J1 모델에 맞다는 사실은 아직 확인되지 않았다.
2. 봉이 물리적으로 수평(+X)일 때의 raw를 `zero_tick`에 기록한다. 2048은 미보정 초기값이다.
3. 아래쪽 theta 증가가 raw 증가인지 감소인지 확인하여 direction=+1/-1을 정한다.
4. 실제 ticks_per_rev, 각도·raw 제한, 케이블/기체 간섭, 지면 여유, 하중 상태를 확인한다.
5. 물리 최소/최대 각도를 raw로 변환해 제한 안에 들어가는지 검증한다.
   Status Return Level, Return Delay Time, USB latency와 sync read 지원도 확인한다.
   송신 성공과 실제 모터 수락은 다르며, 기본 수신 timeout은 실측으로 검증해야 한다.
6. 별도 장비에서 보정/준비 자세를 만든 후 확인한 값과 플래그를 설정한다.
   이 패키지는 보정되지 않은 상태에서 모터를 움직여 보정하는 기능을 제공하지 않는다.

## 별도 경로 빌드와 테스트

아래 명령은 사용자가 나중에 실행할 안내이다. 기존 install을 덮어쓰지 않는다.

```bash
cd /home/palletrone/ros2_ws
source /opt/ros/humble/setup.bash
source /home/palletrone/ros2_ws/install/setup.bash
TASK_BUILD=$(mktemp -d /home/palletrone/palletrone_task_build.XXXXXX)
PYTHONDONTWRITEBYTECODE=1 colcon --log-base "$TASK_BUILD/log" build \
  --base-paths src/PX4-Modifying-Lattepanda/palletrone_task_flight \
  --build-base "$TASK_BUILD/build" --install-base "$TASK_BUILD/install" \
  --packages-select palletrone_task_flight --symlink-install
PYTHONDONTWRITEBYTECODE=1 colcon --log-base "$TASK_BUILD/log" test \
  --base-paths src/PX4-Modifying-Lattepanda/palletrone_task_flight \
  --build-base "$TASK_BUILD/build" --install-base "$TASK_BUILD/install" \
  --packages-select palletrone_task_flight --ctest-args --output-on-failure
colcon --log-base "$TASK_BUILD/log" test-result --test-result-base "$TASK_BUILD/build/palletrone_task_flight" --verbose
source "$TASK_BUILD/install/local_setup.bash"
ros2 pkg prefix palletrone_task_flight
```

source만으로 기존 프로세스의 메시지나 환경이 갱신되지 않는다. 필요한 재시작은 사용자가
지상에서 수행한다. 메시지 정의는 변경하지 않으며 의존성이 없을 때 가짜 px4_msgs를 만들지 않는다.

### ROS graph 없는 오프라인 미리보기

```bash
ros2 run palletrone_task_flight preview_task_flight.py \
  --config /home/palletrone/ros2_ws/src/PX4-Modifying-Lattepanda/palletrone_task_flight/config/task_flight.yaml \
  --start 0 0 0 --yaw-deg 0 --output "$TASK_BUILD/preview"
```

이 실행 파일은 rclpy를 import하지 않고 ROS 노드나 시리얼 연결을 만들지 않는다.
`preview.html`과 `path_preview.png`를 같은 폴더에 두고 연다. CSV/JSON도 출력한다.
슬라이더로 P/M/E/C와 봉 방향을 확인한다. 정적 그림은 상면·측면·XYZ·yaw/J1 시계열과
여섯 접근 구간을 표시한다.

## 향후 실행 순서 — 이번 작업에서 실행하지 않음

1. 실제 탑재 펌웨어/메시지/DDS 매핑, 기체 failsafe, 실내/실외 EKF를 확인한다.
2. 기존 Agent와 센서 수신 환경은 사용자가 기존 검증된 방법으로 준비한다.
3. 기존 `listen_and_speak_ros` allocation은 유지한다. 기존 목표 생성기/keyboard는 함께 쓰지 않는다.
4. 기존 dynamixel_fix가 같은 포트를 사용하지 않는 지상 구성에서 보정 확인된 새 드라이버를 준비한다.
5. task YAML의 dry_run=true로 계획을 먼저 확인한다. 실제 운용 전 별도 검토를 거쳐 false로 설정한다.
6. READY와 계획 범위를 확인한 뒤 **QGC Offboard 선택 → Arm**. Takeoff를 누르지 않는다.
7. 수동 인계/중단 시 임무가 자동 재시작하지 않는다. 새 프로세스로 비행 중 재시도하지 않는다.
8. AUTO_LAND 이후에도 allocation/서보 드라이버는 유지하며 착륙 확인은 사용자/PX4가 담당한다.

```bash
# 각각 별도 터미널. 기본 YAML은 dry_run=true이므로 실제 명령/포트 접근 없음.
ros2 launch palletrone_task_flight j1_driver.launch.py
ros2 launch palletrone_task_flight task_flight.launch.py
```

함께 실행하지 않는 항목: 기존 palletrone_hover_land, px4_ros_com/px4_offboard의 Offboard 예제,
custom_trajectory_generation, control_mode_flags를 발행하는 keyboard, 이를 포함한 v3 launch,
같은 포트의 기존 dynamixel_fix. 기존 v3 전체를 비행 중 종료하면 필수 allocation까지 꺼질 수 있다.
custom_control_mode는 새 비행 노드가 덮어쓰지 않는다. 기존 allocator를 재사용할 때 캐시된
trajectory/DOB/CoM/L1 플래그를 확인한다. 필요한 support publisher 하나씩만 허용한다.
publisher 존재는 실제 액추에이터 건강 상태를 증명하지 않는다.
`custom_control_mode.trajectory_flag=0`의 신선한 수신도 준비 조건이다. 기존 키보드의 `p`는
flags[1](주석상 CoM)을 켜고 **실제 타이머 15초 뒤 flags[2] trajectory를 켠다**.
키보드 로그/주석의 10/40/45초 표기와 실제 타이머 값이 다르므로 키보드를 함께 실행하지 않는다.
DOB/CoM/L1은 기존 정책 그대로이며 이 노드가 설정하지 않는다. `custom_mode_flag`가
탑재 펌웨어에서 fully actuated 진입 자체를 결정하는지까지는 현재 ROS 소스로 확정할 수 없다.
실제 펌웨어에서 그 의미와 수평 유지/allocation 경로를 확인해야 한다.

기존 v3 launch 대신 지상에서 필요한 allocator만 준비할 때의 명령은 다음과 같다.
실행 즉시 기존 servo/payload 명령 생성이 시작되므로 이 작업에서는 실행하지 않았다.

```bash
ros2 run px4_ros_com listen_and_speak_ros
```

## 사용자가 확인할 조회 명령

```bash
ros2 topic info -v /fmu/in/offboard_control_mode
ros2 topic info -v /fmu/in/trajectory_setpoint
ros2 topic info -v /fmu/in/vehicle_command
ros2 topic info -v /j1_angle_cmd
ros2 topic info -v /servo_angle
ros2 topic info -v /servo_angle_cmd
ros2 topic info -v /control_mode_flags
ros2 topic echo /task_flight/preview --qos-reliability best_effort --once
ros2 topic echo /j1_state --qos-reliability best_effort --once
ros2 topic echo /j1_ready --qos-reliability best_effort --once
ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort --once
ros2 topic echo /fmu/out/vehicle_status --qos-reliability best_effort --once
ros2 topic echo /fmu/out/vehicle_command_ack --qos-reliability best_effort
ros2 topic hz /j1_state
ros2 topic hz /fmu/in/trajectory_setpoint
```

PX4 NSH에서 사용자가 확인: `ver all`, `listener offboard_control_mode`,
`listener offboard_trajectory_setpoint`, `listener vehicle_control_mode`,
`listener vehicle_local_position_setpoint`, `listener vehicle_status`, `mc_pos_control status`.
ROS 발행 토픽은 `/fmu/in/trajectory_setpoint`이고 전용 uORB 이름으로 바꾸지 않는다.

프로펠러 제거 시험에서는 실제 상승/접근이 없으므로 실제 도달 조건이 충족되지 않고
정상 전체 작업을 완료할 수 없다. 단위 테스트는 가상 피드백으로 상태 로직을 검증하지만
실기에서 위치/상태를 위조하거나 preflight check를 해제하는 절차를 제공하지 않는다.
실제 추종, J1 부하/열/기계 한계, 실제 접촉 안전성, 수평 유지, 통신 지연, 착륙 성능은 미검증이다.
