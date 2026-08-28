# PX4 Mocap Bridge

`px4_mocap_bridge`는 OptiTrack/Motive에서 추적한 특정 rigid body의 위치를
PX4 EKF2의 외부 위치 입력으로 전달하는 ROS 2 Humble 패키지입니다. 1차
구현은 **위치만** 융합하며, mocap 자세와 외부 선속도/각속도는 전달하지
않습니다. 위치를 미분해 속도를 만들지도 않습니다.

```text
OptiTrack/Motive
  -> IMRCLab motion_capture_tracking
  -> /poses (motion_capture_tracking_interfaces/msg/NamedPoseArrayV2)
  -> px4_mocap_bridge
  -> /fmu/in/vehicle_visual_odometry (px4_msgs/msg/VehicleOdometry)
  -> uXRCE-DDS
  -> PX4 EKF2
```

## 요구 환경과 설치

- Ubuntu 22.04
- ROS 2 Humble
- PX4와 LattePanda 사이의 정상적인 uXRCE-DDS 연결
- 이 저장소의 `px4_msgs`
- IMRCLab `motion_capture_tracking`의 ROS 2 브랜치

`motion_capture_tracking`은 내부 라이브러리를 Git submodule로 사용하므로
반드시 `--recurse-submodules`로 복제해야 합니다.

```bash
cd ~/ros2_ws
git clone --recurse-submodules --branch ros2 \
  https://github.com/IMRCLab/motion_capture_tracking.git
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --packages-select \
  px4_msgs motion_capture_tracking_interfaces \
  motion_capture_tracking px4_mocap_bridge
source install/setup.bash
```

이미 submodule 없이 복제했다면 해당 저장소에서 다음을 먼저 실행합니다.

```bash
git submodule update --init --recursive
```

Motive의 Streaming 설정에서 NatNet, Rigid Bodies, multicast, Z-up을 활성화하고
기본 NatNet 포트(1510/1511)를 사용합니다. 이 패키지의 예제는 Motive가 고유한
rigid body를 이미 추적한다고 가정하므로 `config/optitrack.yaml`에 custom marker
tracker용 `rigid_bodies` 항목을 두지 않습니다.

## 실행

브리지만 실행하려면 `/poses` publisher와 uXRCE-DDS 연결을 별도로 시작한 뒤:

```bash
ros2 launch px4_mocap_bridge bridge.launch.py \
  rigid_body_name:=drone \
  input_topic:=/poses \
  output_topic:=/fmu/in/vehicle_visual_odometry
```

다른 설정 파일은 `config_file:=/absolute/path/to/bridge.yaml`로 지정할 수
있습니다. OptiTrack 수신 노드와 브리지를 함께 실행하려면:

```bash
ros2 launch px4_mocap_bridge optitrack_px4.launch.py \
  hostname:=192.168.0.100 \
  rigid_body_name:=drone
```

결합 launch는 `motion_capture_tracking_node`를 직접 실행하며 RViz를 자동으로
시작하지 않습니다. 외부 패키지가 설치되어 있지 않거나 현재 shell에서 source
되지 않았다면 설치 방법을 포함한 오류를 냅니다.

기존 `px4_ros_com`의 v3 비행 launch와 OptiTrack 수신/브리지를 한 명령으로
실행하려면 다음 새 wrapper launch를 사용합니다.

```bash
ros2 launch px4_mocap_bridge v3_optitrack_flight_launcher.launch.py \
  hostname:=192.168.0.100 \
  rigid_body_name:=drone
```

이 wrapper는 기존 `px4_ros_com/launch/v3_flight_launcher.launch.py`를 수정하거나
복제하지 않고 그대로 포함합니다. 따라서 기존 v3 노드와 rosbag 기록에 더해
`motion_capture_tracking_node`와 `mocap_bridge_node`가 함께 실행됩니다. Micro XRCE-DDS
Agent와 Dynamixel 구동 프로그램은 기존 실행 방법을 그대로 사용하며 이 wrapper가
자동으로 시작하지 않습니다.

## 브리지 파라미터

기본값은 `config/bridge.yaml`에 있습니다.

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `rigid_body_name` | `drone` | `NamedPoseArrayV2.poses`에서 선택할 이름 |
| `input_topic` | `/poses` | `NamedPoseArrayV2` 입력 토픽 |
| `output_topic` | `/fmu/in/vehicle_visual_odometry` | PX4 `VehicleOdometry` 입력 토픽 |
| `max_publish_rate_hz` | `50.0` | callback 기반 최대 발행 빈도 |
| `max_input_age_ms` | `100.0` | 허용할 입력 `header.stamp`의 최대 나이 |
| `position_stddev_xy` | `0.02` | x/y 위치 표준편차(m) |
| `position_stddev_z` | `0.02` | z 위치 표준편차(m) |
| `quality` | `100` | PX4 `VehicleOdometry.quality` 값 |
| `mocap_to_px4_matrix` | `diag(1,-1,-1)` | mocap 위치에 곱할 행 우선 3x3 회전행렬 |
| `position_offset_px4` | `[0,0,0]` | 변환 뒤 더할 PX4 FRD 오프셋(m) |

표준편차는 출력 메시지에서 제곱되어 위치 분산이 됩니다. 변환행렬은 원소 9개,
유한값, 근사 직교성, determinant 약 `+1` 조건을 만족해야 합니다. 입력과 출력
QoS는 best-effort, volatile, keep-last, depth 1을 사용해 센서 표본이 쌓이지 않게
합니다.

지정한 rigid body가 없거나 입력이 오래되었거나 위치에 NaN/Inf가 있으면 브리지는
마지막 값이나 `[0, 0, 0]`을 대신 발행하지 않습니다. 경고를 rate-limit해 출력하고,
정상 입력이 다시 들어오면 자동으로 발행을 재개합니다. 입력 watchdog timer는
dropout 경고만 만들며 저장된 위치를 발행하지 않습니다.

## 좌표계 변환과 확인

Motive world frame이 Z-up이고 `/poses`가 ROS FLU 규약과 맞는다는 기본 가정에서
PX4 FRD 위치 변환은 다음과 같습니다.

```text
p_px4 = R_mocap_to_px4 * p_mocap + position_offset_px4
R_mocap_to_px4 = diag(1, -1, -1)
[x, y, z] -> [x, -y, -z]
```

실험실 Motive world frame은 축 설정에 따라 다를 수 있습니다. 프로펠러를 제거한
기체를 직접 움직이면서 출력이 다음과 같은지 확인하고, 다르면
`mocap_to_px4_matrix`와 `position_offset_px4`를 수정하십시오.

```text
기체를 앞으로 이동   -> PX4 x 증가
기체를 오른쪽으로 이동 -> PX4 y 증가
기체를 위로 이동     -> PX4 z 감소
```

## PX4 설정

이 브리지는 PX4가 외부 비전 위치 입력에 사용하는
`/fmu/in/vehicle_visual_odometry`에 `px4_msgs/msg/VehicleOdometry`를 발행합니다.
이 경로를 사용해야 EKF2가 uXRCE-DDS를 통해 수신한 외부 위치를 visual odometry로
처리할 수 있습니다. `VehicleMocapOdometry` 메시지나 PX4/`px4_msgs`의 메시지
정의는 변경하지 않습니다.

초기 PX4 파라미터 예시는 다음과 같습니다. 기체 및 PX4 버전에 맞게 실제
파라미터 의미와 지원 여부를 다시 확인하십시오.

```text
EKF2_EV_CTRL = 3
EKF2_HGT_REF = 3
UXRCE_DDS_SYNCT = 1
```

`EKF2_EV_CTRL=3`은 외부 비전의 수평 위치와 수직 위치만 융합하고, 외부 속도나
yaw는 융합하지 않는 설정입니다. `EKF2_HGT_REF=3`은 외부 비전을 고도 기준으로
사용하며, `UXRCE_DDS_SYNCT=1`은 PX4와 DDS 측 시간 동기화를 활성화합니다.

이 구현은 외부 yaw를 제공하지 않습니다. 자력계까지 비활성화하면 EKF의 yaw
관측원이 사라질 수 있으므로 비행 전에 반드시 별도의 yaw 관측 가능성을
확인하십시오. 외부 yaw 융합은 후속 기능 범위입니다.

## 실기 점검

ROS 2 측 입력 빈도와 실제 값을 확인합니다.

```bash
ros2 topic hz /poses
ros2 topic echo /poses
ros2 topic echo /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/out/vehicle_local_position
```

PX4 NSH에서는 다음을 확인합니다.

```text
listener vehicle_visual_odometry
listener estimator_status_flags
ekf2 status
```

첫 비행 전에는 **프로펠러를 제거한 상태**에서 좌표축 방향을 먼저 확인하고,
rigid body 가림이나 네트워크 중단 시 오래된 위치/영점이 발행되지 않는지 검증해야
합니다. 그 다음 EKF innovation과 estimator 상태를 확인한 후 제한된 안전 구역에서
단계적으로 시험하십시오.

## 테스트

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_mocap_bridge
colcon test --packages-select px4_mocap_bridge
colcon test-result --verbose
```

`motion_capture_tracking_interfaces`가 없다면 가짜 메시지 패키지를 만들거나 시스템
전역에 임의 설치하지 말고, 위 절차대로 원본 저장소와 submodule을 준비하십시오.
