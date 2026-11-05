# Palletrone: relative climb, hover, Land handover

This node is a one-shot scenario for the reported custom position-only Offboard
firmware contract. It does not establish that this firmware is installed.
Audit the deployed firmware message definitions and DDS mapping before operation.

## Ownership and coordinates

QGC selects Offboard and Arms. ROS never sends Arm, Disarm, Kill or Offboard mode
commands. It only requests `VEHICLE_CMD_NAV_LAND` after the measured hover.
The node first needs a fresh DISARMED observation after process startup. Starting
the process while already Armed cannot trigger a climb. Arm before READY aborts.

Both OptiTrack and RTK use `/fmu/out/vehicle_local_position`. No `/poses`, GPS fix,
global position or Home dependency is introduced. The estimator must provide
valid finite local XYZ and `heading_good_for_control`.

After ARMED+OFFBOARD, one snapshot defines `x_start,y_start,z_start,yaw_start`.
The final target is `[x_start,y_start,z_start-hover_height_m]`, absolute PX4 local
NED. Height means displacement from that snapshot, not AMSL, AGL, Home altitude
or RTK base altitude. XY/yaw stay fixed. The z setpoint changes gradually.
No body-heading-relative horizontal route is generated.

The ROS input topic remains `/fmu/in/trajectory_setpoint`; the reported firmware
receives it on internal uORB `offboard_trajectory_setpoint`. Do not rename the
ROS topic. Position-only mode flags are explicit. Unused velocity, acceleration,
jerk and yaw speed are NaN. Land's optional yaw/global coordinates are also NaN.

## Timing and parameters

| Parameter | Default | Meaning |
|---|---:|---|
| hover_height_m | 1.0 | Relative ascent, metres |
| hover_duration_s | 10.0 | Continuous measured hover inside tolerance |
| climb_rate_mps | 0.2 | Maximum normal z-setpoint progression rate |
| publish_rate_hz | 20.0 | Goal and heartbeat rate |
| position_tolerance_m | 0.1 | Euclidean XYZ distance to final goal |
| arrival_settle_time_s | 1.0 | Continuous in-tolerance time before hover timing |
| prestream_duration_s | 1.5 | Current-pose streaming before READY; must exceed 1 s |
| position_timeout_s | 0.5 | Maximum age/gap of advancing local-position samples |
| status_timeout_s | 1.0 | Maximum age/gap of advancing status messages |
| climb_timeout_s | 30.0 | Arrival/reacquisition deadline from initial climb start |
| land_retry_interval_s | 2.0 | Per-attempt wait for actual AUTO_LAND |
| land_max_attempts | 3 | Total requests, including the first |
| max_step_dt_s | 0.1 | Cap on elapsed time used for each z increment |
| control_timeout_s | 0.3 | Maximum timer gap while streaming |
| target_system_id / target_component_id | 1 / 1 | Must match VehicleStatus IDs |
| source_system_id / source_component_id | 245 / 191 | Distinguish this command sender from QGC |

Parameters are read-only after startup. Hardware use requires `use_sim_time=false`.
The first two seconds are discovery warm-up without control publication. After
valid inputs and a Disarmed observation, prestreaming starts. Minimum READY time
is therefore about 3.5 s from startup, possibly longer while waiting for inputs.

ROS clock microseconds populate outgoing timestamps. A steady timer and Python
`monotonic()` drive elapsed time and timeouts. ROS clock regression/stall stops
the stream. uXRCE-DDS/PX4 time synchronization still requires independent checking.
Repeated position sample timestamps do not refresh freshness. Regressing PX4
timestamps abort. Reception-age checks do not prove end-to-end clock alignment.

The normal ramp takes about five seconds. There is no velocity/acceleration
feedforward, acceleration bound or guarantee that the vehicle follows at exactly
0.2 m/s. Actual tracking depends on the existing firmware controller and tuning.

## State and failure policy

| Condition/state | Position goal | Heartbeat | Land requests |
|---|---|---|---|
| No valid input / no post-start Disarmed observation | none | none | none |
| PRESTREAM / READY | latest valid current pose | 20 Hz | none |
| CLIMB | anchored XY/yaw, bounded z ramp | 20 Hz | none |
| HOLD | fixed final goal | 20 Hz | none |
| LAND_REQUESTED, still Armed+Offboard, inputs healthy | final goal | 20 Hz | at most 3, at least 2 s apart |
| LAND_FAILED_HOLD, still Armed+Offboard, inputs healthy | final goal | 20 Hz | no more requests |
| Actual AUTO_LAND observed after Land request | none | none | none |
| RC/QGC Offboard exit, Disarm, invalid/stale input, reset, clock fault, conflict | none | none | none |

The path must finish its z ramp and measured XYZ must settle for 1 s before the
10 s hover timer begins. Leaving tolerance resets settling and hover time; the
original arrival/reacquisition deadline is not extended. Arrival timeout aborts
without a Land request. A long timer delay cannot produce a large z step: up to
0.1 s is integrated; a gap greater than 0.3 s aborts.

Land ACKs are filtered by command, recipient IDs, origin and increasing timestamp.
They are diagnostic; ACCEPTED and IN_PROGRESS do not establish a mode transition.
All ACK results, missing ACKs, and absent AUTO_LAND share the bounded retry window.
Three default requests occur approximately at 0, 2 and 4 s; at approximately 6 s
without AUTO_LAND the node logs LAND_FAILED_HOLD and holds while inputs remain valid.
Late AUTO_LAND still stops all Offboard publication. There is no landing-complete
detector and no exit-on-Land-request. AUTO_LAND_HANDOVER is not "landed".

Aborts latch for the process lifetime. Recovery of telemetry or Offboard mode does
not restart the scenario. XY/Z/heading reset counters and local `ref_timestamp`
changes abort once streaming begins. No reset delta is applied, avoiding an
unverified second correction on top of firmware correction. If the firmware
changes frames before ROS receives the notification, zero-latency reaction is not
possible; this is not a substitute for a verified firmware reset policy.

On invalid/stale input or abort, no zero pose or old pose is republished with a new
timestamp. If PX4 is still in Offboard, subsequent action depends on its configured
Offboard-loss failsafe. Check `COM_OF_LOSS_T` and `COM_OBL_RC_ACT` on the deployed
firmware; this code never changes them. Complete DDS loss prevents a reliable Land
command. Do not interpret stopping the stream as an immediate confirmed Land.

## Required support nodes and conflicts

Keep the existing uXRCE-DDS Agent, `px4_ros_com/listen_and_speak_ros`, and
`quadrotor_controller/dynamixel_fix` running separately from this mission launch.
The allocator computes servo angles from PX4 thrust/torque and per-motor thrust,
publishes `servo_angle_cmd`, receives `servo_angle` from the driver, and returns
`/fmu/in/servo_angle` to PX4. The firmware motor allocation remains in place.
Driver startup can energize servos. Its existing implementation is unchanged.

Indoor operation additionally needs motion_capture_tracking and px4_mocap_bridge.
Use the existing OptiTrack launch with the explicit current hostname and actual
rigid-body name. Outdoor operation uses PX4 RTK/EKF; do not accidentally continue
fusing an unwanted indoor external-vision source. This node does not change EKF
parameters or require RTK fixed. Those are separate operational decisions.

Do not run old Offboard examples, `custom_trajectory_generation`, or the unmodified
keyboard publisher with this scenario. The v3 launch starts the allocator AND a
custom trajectory generator, so use the allocator's standalone executable instead.
Stopping v3 in flight also stops the allocator. No existing v3 file is changed.

The existing keyboard's p action sets custom_mode_flag and schedules trajectory
activation after 15 s. In public old firmware 2c151963, custom_mode_flag enables
the CoM-estimation branch; it is not the Offboard/fully-actuated selector. That
meaning must be checked against the actually deployed modified firmware. A fresh
allocator starts its existing flags at zero. If reusing an allocator process,
check cached `/fmu/in/custom_control_mode` flags before arming; this node does not
overwrite them. The existing payload and ServoAngle code remain untouched.

A same-host advisory lock prevents a second instance of this new executable.
Every 0.5 s, ROS graph checks reject competing Offboard/VehicleCommand publishers,
legacy custom-goal/keyboard publishers, and duplicate servo/support publishers.
Discovery can be delayed or partitioned; publisher count is neither an exclusive
distributed lock nor proof of data flow, estimator validity or actuator health.
Check required nodes and actual data before QGC Arm. The new launch starts no
support nodes and never respawns the mission automatically.

## Bench and flight separation

First verify communication, coordinate signs, targets and mode handover with
propellers removed. Without real ascent, the measured arrival condition is not
satisfied: the normal scenario will not start its hover timer or automatic Land
sequence, and will abort at its arrival timeout. Do not fake the estimate or
disable arming checks to make this pass. QGC can independently select Land to
check authority handover; bounded automatic Land logic is covered by unit tests.

Real tracking, hover precision and landing performance require a separately
authorized controlled flight after firmware/message/estimator checks. Keep QGC/RC
takeover available. After abort or completed handover, a new mission needs a new
process and a new post-start Disarmed observation; never restart it in flight.
