# 구현 및 검증 기록 — 2026-11-07

## 확인 대상과 보존 결과

- SSH: `palletrone@192.168.10.152`, 실제 hostname `palletrone`, 최초 경로 `/home/palletrone`.
- 저장소: `/home/palletrone/ros2_ws/src/PX4-Modifying-Lattepanda`.
- 브랜치: `Offboard_control_autonomousFLight` — 요청 기준과 일치, 전환하지 않음.
- HEAD: `651beba113b6eee3307b6457d9d8cefaaf7a092b` — 작업 중 변경 없음.
- origin: `https://github.com/kamgaa/PX4-Modifying-Lattepanda` — 변경 없음.
- 상위 디렉터리 및 생성 디렉터리를 제외한 저장소에서 AGENTS.md를 찾지 못함.
- 처음 Git 상태: 아래 브랜치 줄만 표시됐으며 변경/미추적 표시 없음.
- 마지막 Git 상태: 아래와 같음. 기존 파일 수정이 필요한 미적용 diff는 없음.

```text
## Offboard_control_autonomousFLight...origin/Offboard_control_autonomousFLight
?? palletrone_task_flight/
```

신규 패키지는 시작 시 존재하지 않았다. 기존 일반 파일 **571개의 SHA-256이 모두 일치**하고,
기존 추적 항목 576개에 대한 마지막 Git diff와 staged diff는 비어 있다.
기존 무시된 캐시/rosbag/백업/빌드 디렉터리 전체를 해시 검사한 것은 아니다.
해당 파일을 삭제하거나 덮어쓰는 작업은 하지 않았다.

**로그 부작용 및 복구:** 결과 집계 명령 `colcon test-result`가 기존 추적 symlink
`log/latest`를 `latest_build`에서 `latest_test-result`로 바꾼 것을 발견했다.
최초 clean 상태, HEAD의 원래 대상, 현재 대상을 확인한 뒤 이 링크만 원상 복구했다.
로그 삭제/정리는 하지 않았다. 집계 명령이 추가한 무시된
`log/test-result_2026-11-07_19-38-59/`, `log/test-result_2026-11-07_19-42-27/`,
`log/latest_test-result`는 남아 있다. 향후 명령에는 결과 집계에도 별도 `--log-base`를 지정했다.

## 읽은 기존 구현과 메시지

기존 palletrone_hover_land.py, hover_land_logic.py, hover_land_messages.py, YAML/launch/테스트,
listen_and_speak_ros.cpp, custom_trajectory_generation.cpp, v3 launch, CMake 설치 규칙,
quadrotor_controller/src/dynamixel_fix.cpp, keyboard_teleop.cpp, 관련 메시지와 SDK 계약을 읽었다.
현재 hover의 position/status timeout 10초는 복사하지 않았다. 새 기본값은 0.5초/2초이다.

실제 `px4_msgs` prefix는 `/home/palletrone/ros2_ws/install/px4_msgs`이다.
생성 Python도 그 아래 `local/lib/python3.10/dist-packages/px4_msgs/msg/`에서 로딩된다.
주석/상수/TOPICS를 직렬화 필드로 취급하지 않고 이름·타입·배열 길이·순서를 비교했다.
정수 상수도 별도로 비교했고 대상에서 불일치를 발견하지 못했다.

| 메시지 | 직렬화 필드 수 | 소스 ↔ 실제 설치본 | 실제 탑재 펌웨어 |
|---|---:|---|---|
| OffboardControlMode | 8 | 일치 | 미확인 |
| TrajectorySetpoint | 7 | 일치 | 미확인 |
| VehicleCommand | 15 | 일치 | 미확인 |
| VehicleCommandAck | 8 | 일치 | 미확인 |
| VehicleLocalPosition | 49 | 일치 | 미확인 |
| VehicleStatus | 41 | 일치 | 미확인 |
| CustomControlMode | 6 | 일치 | 미확인 |
| ServoAngle | 2 | 일치, 기존 배열 관련 코드 보존 | 미확인 |
| VehicleOdometry | 13 | 일치 | 미확인 |

메시지 정의/펌웨어는 변경하지 않았다. 실제 설치된 TrajectorySetpoint로 native serialization
roundtrip을 시험했다. 이는 실제 DDS 수신이나 탑재 펌웨어 ABI 일치의 증거는 아니다.
사용자가 보고한 position-only 계약으로 구현했으며 ROS 토픽은 `/fmu/in/trajectory_setpoint`,
PX4 확인 대상은 `offboard_trajectory_setpoint`이다.

## 신규 파일 및 업로드 대상 — 27개

아래 경로는 신규 패키지 `palletrone_task_flight/` 기준이다.

```text
CMakeLists.txt
package.xml
LICENSE
NOTICE.md
README.md
VALIDATION.md
scripts/task_flight.py
scripts/preview_task_flight.py
palletrone_task_flight/__init__.py
palletrone_task_flight/configuration.py
palletrone_task_flight/geometry.py
palletrone_task_flight/trajectory.py
palletrone_task_flight/state_machine.py
palletrone_task_flight/messages.py
palletrone_task_flight/output.py
palletrone_task_flight/preview.py
include/palletrone_task_flight/joint_math.hpp
src/dynamixel_fix_with_j1.cpp
config/task_flight.yaml
config/j1_driver.yaml
launch/task_flight.launch.py
launch/j1_driver.launch.py
test/conftest.py
test/test_geometry.py
test/test_state_machine.py
test/test_messages.py
test/test_joint_math.cpp
```

GitHub 업로드 대상도 위 소스/설정/문서/테스트이다. build/install/log, 캐시, rosbag,
백업 및 임시 파일은 제외한다. 이번 작업에서는 commit/push하지 않았다.

## 실행한 검증

검증 루트: `/home/palletrone/palletrone_task_audit_ig0jz522`.
build/install/log 경로를 모두 이 별도 디렉터리에 지정하고 신규 패키지만 빌드했다.
기존 운용 install은 px4_msgs/SDK 등의 underlay로 사용했다.

- C++17 드라이버/순수 C++ 시험 컴파일·링크 및 신규 패키지 설치 성공. 드라이버는 실행하지 않음.
- colcon test 성공. 결과 집계: **62 tests, 0 errors, 0 failures, 0 skipped**.
- 실제 pytest case: geometry 17, state 34, messages/adapter 7, 총 **58개**.
- 순수 C++ 시험 executable 1개 통과. 62라는 집계에는 세 Python suite를 감싼 CTest 항목도 포함된다.
- 봉 길이/정렬/d=0/offset/yaw/연속성/속도 상한, 지정한 30도 변위 수치를 검증했다.
- 정상 여섯 접근, 실제 feedback 도달 gating, J1 loss/lag, 입력 timeout, reset, RC/QGC 이탈,
  Disarm, 제어 지연, 무재시작, Land retry/ACK/mode 구분, dry-run 출력 차단을 검증했다.
- 실제 설치된 ROS 메시지를 사용했다. 가짜 ROS 메시지 패키지는 만들지 않았다.
- legacy ID1..4 변환: raw 0..4095 전 범위 및 -0.7..0.7rad의 1,401개 목표에서 기존 식과 비교했다.
- J1 raw 변환/방향/보정/제한, payload 슬롯과 J1의 독립성, 계획과 driver limits 일치를 검증했다.
- dry-run/충돌 검사는 순수 출력 경계와 실제 adapter 메서드로 수행했다.
  ROS 노드 생성/DDS graph에서의 발행 시험은 아니다.

첫 시험의 부동소수점 보간 초과와 최초 package.xml 이메일 형식 오류를 수정한 뒤 최종 검증했다.
수정은 이번에 만든 신규 파일에 적용했다. 시험 로그는 검증 루트에 보관한다.

## 경로 미리보기

검증 루트 `preview/`에 HTML, PNG, plan.json, trajectory.csv를 생성했다.
실제 기하학/경로 함수를 공유하며 ROS graph/serial을 만들지 않는다.
PNG 상면·측면·시계열을 열어 확인했고 HTML JavaScript 구문 검사도 통과했다.

기본 P0=[0,0,0], yaw0=0일 때:

- 고정 C=[0.79,0,-0.5]m.
- 초기 상승 0.5m, **최대 상승 0.95m**, 최대 수평 이동 반경 약 **0.79m**.
- 자세2 접근: 수평 0.259808m, 하강 0.15m.
- 자세1→2 대기 전환: 전방 0.120577m, 상승 0.45m.
- 목표 시간 + 단계별 정착 시간의 nominal 합계 **236.6875초**.
  실제 feedback 지연/정착/hold 때문에 실행 시간이 증가할 수 있다.
- 그림/CSV 시간축은 reference 시간이며 추가 정착과 추종 지연은 포함하지 않는다.
- 실제 overshoot, 물체 간섭 및 안전 여유는 이 계획 범위에 포함하지 않는다.

원격 Matplotlib의 Axes3D import 경고가 있었지만 사용하는 2D 네 패널과 HTML은 생성됐다.
Windows에는 matplotlib가 없어 로컬 재렌더링은 수행하지 않았다. 원격 결과를 전송하고 ZIP
CRC 검사 후 확인했다. 전역 패키지 설치나 기존 Matplotlib 변경은 하지 않았다.

## 이후 실행·보정·조회 절차

전체 YAML 설명, J1 보정, 새 빌드/실행/조회 명령은 [README.md](README.md)에 있다.
이미 만든 격리 설치본을 선택하고 오프라인 계획을 다시 만드는 명령은 다음과 같다.

```bash
source /opt/ros/humble/setup.bash
source /home/palletrone/ros2_ws/install/setup.bash
source /home/palletrone/palletrone_task_audit_ig0jz522/install/local_setup.bash
ros2 pkg prefix palletrone_task_flight
ros2 run palletrone_task_flight preview_task_flight.py \
  --config /home/palletrone/ros2_ws/src/PX4-Modifying-Lattepanda/palletrone_task_flight/config/task_flight.yaml \
  --start 0 0 0 --yaw-deg 0 --output /home/palletrone/task_preview
```

J1은 실제 모델/주소/baudrate/ID5, 수평 zero_tick, 방향, ticks/rev, 물리·raw 범위,
Position mode, 토크 상태, 오프셋/봉 간섭 및 통신 지연 확인이 필요하다.
`calibration_confirmed=false`, `control_table_confirmed=false`, 모델번호 0은 미확인 기본값이다.
검증 없이 플래그와 모델번호만 채워 동작을 강제하지 않는다. 두 YAML 모두 dry_run=true이다.

사용자가 지상에서 구성할 노드는 Agent/EKF 경로 + 기존 listen_and_speak_ros + 신규 combined
driver 하나 + task node 하나이다. **기존 dynamixel_fix와 신규 driver는 같은 포트에서 동시에
실행하면 안 된다.** 기존 hover/Offboard 예제/custom trajectory/keyboard/v3 전체 launch도
작업 노드와 함께 실행하지 않는다. 실내/실외 모두 task는 PX4 local position만 사용한다.
OptiTrack hostname/rigid body는 변경하지 않는다.

보정과 실제 운용 검토 후 사용자가 dry_run=false를 선택했을 때의 시작 순서는:
DISARMED 관측 → 입력/보정범위 확인 → 사전 송신 → READY/계획 확인 →
**QGC Offboard 선택 및 Arm**이다. QGC Takeoff를 누르지 않는다.
J1 오류 시 PX4가 건강하면 hold하며 수동 인계가 필요하다. 좌표 reset/입력 손실이면
송신 중단 후 PX4 failsafe에 의존한다. Land ACK만으로 종료하지 않으며 실제 AUTO_LAND
확인 후 임무 송신을 멈춘다. 서보/allocation/Agent는 종료하지 않는다.

## 실제 장비에서 남은 확인

- 탑재 펌웨어 소스/바이너리 동일성, 실제 DDS 수신, mode 명령 수락: 미확인.
- estimator/yaw/mocap 정렬/RTK 전환, Home/RTL/failsafe: 미확인.
- fully actuated 수평 유지와 custom_control_mode의 firmware 의미: 미확인.
- 실제 serial 독점, motor 보정·추종·부하·열·간섭, ROS 주기/중복 검출: 미시험.
- 실제 접촉 인식/힘 제한 기능은 없음. 접촉 목표 도달만 판정한다.
- 실제 비행/접근/후퇴/hover/착륙 성능과 접지 확인: 미시험.

사용자가 프로펠러 제거 상태에서 먼저 README의 topic info/echo/hz 및 PX4 `ver all`,
`listener offboard_control_mode`, `listener offboard_trajectory_setpoint`,
`listener vehicle_control_mode`, `listener vehicle_local_position_setpoint`,
`listener vehicle_status`, `mc_pos_control status`를 확인한다.
이 상태에서는 실제 상승이 없어 도달 조건을 만족하지 않으므로 정상 임무 완료를 기대하지 않는다.
이번 작업에서는 비행/서보 노드 실행, actuator 조작, 파라미터 변경, 펌웨어 업로드,
기존 프로세스 종료/재시작, GitHub push를 수행하지 않았다.
