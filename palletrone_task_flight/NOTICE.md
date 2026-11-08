# Sources and compatibility

Inspected repository: https://github.com/kamgaa/PX4-Modifying-Lattepanda
Branch: Offboard_control_autonomousFLight
Revision: 651beba113b6eee3307b6457d9d8cefaaf7a092b

`messages.py` is adapted from `px4_ros_com/px4_ros_com/hover_land_messages.py`.
The existing hover mission's separation of pure logic, ROS-clock message timestamps,
monotonic timeouts and ACK-vs-mode confirmation informed this package. The upstream
px4_ros_com BSD-3-Clause license and PX4 Development Team notice are retained in LICENSE.
No dependency on the existing hover Python modules is introduced.

`dynamixel_fix_with_j1.cpp` is a new implementation of the observed bus/topic contract,
not a copy or modification of `quadrotor_controller/src/dynamixel_fix.cpp`.
That package's manifest declares `TODO: License declaration`; no license is inferred
for the old driver. Protocol compatibility facts are retained: IDs 1..4, protocol 2,
4096 ticks/revolution, the original 3.14-based legacy conversion, Position mode,
five-slot Float32MultiArray input/output, and inactive legacy payload feedback slot.
Do not attribute a new license to the original driver on the basis of this package.

Existing `listen_and_speak_ros.cpp`, allocation formulas, DOB, payload, ServoAngle,
OptiTrack configuration, firmware and px4_msgs definitions are unchanged.
