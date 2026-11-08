#!/usr/bin/env python3
"""ROS adapter. This executable never opens a serial port or starts another node."""
from dataclasses import fields
from time import monotonic
import json
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray, Bool, String
from px4_msgs.msg import (VehicleLocalPosition, VehicleStatus, VehicleCommandAck,
                         OffboardControlMode, TrajectorySetpoint, VehicleCommand, CustomControlMode)
from palletrone_task_flight.configuration import Settings, check_driver_limits
from palletrone_task_flight.state_machine import TaskMachine, Position, Status, Joint
from palletrone_task_flight.trajectory import Plan
from palletrone_task_flight.messages import check_publish_schemas
from palletrone_task_flight.output import command_topics, emit


class TaskFlight(Node):
    def __init__(self):
        super().__init__('task_flight')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('hardware node requires use_sim_time=false')
        defaults = Settings()
        ro = ParameterDescriptor(read_only=True)
        values = {}
        for f in fields(Settings):
            value = getattr(defaults, f.name)
            if isinstance(value, tuple):
                value = list(value)
            values[f.name] = self.declare_parameter(f.name, value, ro).value
        self.p = Settings(**values)
        self.core = TaskMachine(self.p)
        check_publish_schemas()
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self._command_publishers = {}
        if command_topics(self.p.dry_run):
            for key, cls, topic in [('mode', OffboardControlMode, '/fmu/in/offboard_control_mode'),
                                    ('goal', TrajectorySetpoint, '/fmu/in/trajectory_setpoint'),
                                    ('command', VehicleCommand, '/fmu/in/vehicle_command'),
                                    ('joint', Float32, '/j1_angle_cmd')]:
                self._command_publishers[key] = self.create_publisher(cls, topic, qos)
        self.preview_pub = self.create_publisher(String, '/task_flight/preview', qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_position, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.on_status, qos)
        self.create_subscription(JointState, '/j1_state', self.on_joint, qos)
        self.create_subscription(Bool, '/j1_ready', self.on_ready, qos)
        self.create_subscription(Float64MultiArray, '/j1_limits', self.on_limits, qos)
        self.create_subscription(CustomControlMode, '/fmu/in/custom_control_mode', self.on_control_mode, qos)
        ack_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                             reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.on_ack, ack_qos)
        self.driver_ready = False
        self.driver_ready_at = None
        self.driver_limits_at = self.custom_mode_at = None
        self.custom_mode_stamp = 0
        self.legacy_trajectory_off = False
        self.support_reason = 'Waiting for J1 limits and legacy trajectory_flag=0'
        self.last_ros_stamp = self.last_ack_stamp = 0
        self.started = monotonic()
        self.last_graph = self.last_preview = float('-inf')
        self.last_notice = None
        self.support_ready = False
        self.ids = (self.p.target_system_id, self.p.target_component_id,
                    self.p.source_system_id, self.p.source_component_id)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1/self.p.publish_rate_hz, self.on_timer, clock=self.steady)
        self.get_logger().info(f'dry_run={self.p.dry_run}; no automatic Arm/Offboard/Disarm. QGC Takeoff is not the start trigger.')

    def on_position(self, m):
        self.core.receive_position(monotonic(), Position(
            m.timestamp, m.timestamp_sample, (m.x,m.y,m.z), m.heading,
            m.xy_valid,m.z_valid,m.heading_good_for_control,
            (m.xy_reset_counter,m.z_reset_counter,m.heading_reset_counter,m.ref_timestamp)))

    def on_status(self, m):
        self.core.receive_status(monotonic(), Status(
            m.timestamp,m.arming_state==VehicleStatus.ARMING_STATE_ARMED,
            m.arming_state==VehicleStatus.ARMING_STATE_DISARMED,
            m.nav_state==VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            m.nav_state==VehicleStatus.NAVIGATION_STATE_AUTO_LAND,
            not m.failsafe and m.system_id==self.p.target_system_id and m.component_id==self.p.target_component_id))

    def on_ready(self, m):
        self.driver_ready, self.driver_ready_at = bool(m.data), monotonic()
        if not m.data and self.core.streaming:
            self.core.fault_hold('Driver reports J1 not ready')

    def on_limits(self, m):
        try:
            check_driver_limits(self.p, m.data)
            self.driver_limits_at = monotonic()
        except (ValueError, TypeError) as e:
            self.driver_limits_at = None
            self.support_reason = str(e)
            self.core.fault_hold(str(e))

    def on_control_mode(self, m):
        if m.timestamp <= 0 or m.timestamp < self.custom_mode_stamp:
            if self.core.streaming:
                self.core.abort('Allocator custom mode timestamp invalid/regressed')
            return
        if m.timestamp == self.custom_mode_stamp:
            return
        self.custom_mode_stamp, self.custom_mode_at = m.timestamp, monotonic()
        self.legacy_trajectory_off = m.trajectory_flag == 0
        if not self.legacy_trajectory_off:
            self.support_reason = 'Allocator retains trajectory_flag != 0; prepare a clean ground configuration'
            if self.core.streaming:
                self.core.abort(self.support_reason)

    def on_joint(self, m):
        now = monotonic()
        ready = (self.driver_ready and self.driver_ready_at is not None
                 and now-self.driver_ready_at <= self.p.joint_feedback_timeout_s
                 and self.driver_limits_at is not None
                 and now-self.driver_limits_at <= self.p.joint_feedback_timeout_s)
        if m.name != ['j1'] or len(m.position) != 1:
            self.core.fault_hold('Invalid J1 JointState shape')
            return
        stamp = m.header.stamp.sec*1000000000+m.header.stamp.nanosec
        self.core.receive_joint(now, Joint(stamp,float(m.position[0]),ready))

    def on_ack(self, m):
        if (m.command != VehicleCommand.VEHICLE_CMD_NAV_LAND
                or m.target_system != self.p.source_system_id or m.target_component != self.p.source_component_id
                or m.timestamp <= self.last_ack_stamp):
            return
        self.last_ack_stamp = m.timestamp
        if self.core.state == 'LAND_REQUESTED':
            self.core.receive_ack(m.result)
            self.get_logger().info(f'Land ACK result={m.result}, from_external={m.from_external}; actual AUTO_LAND still required')

    def guard_graph(self):
        own = (self.get_name(), self.get_namespace())
        for topic in command_topics(False):
            infos = self.get_publishers_info_by_topic(topic)
            if len(infos)>1 or any((i.node_name,i.node_namespace)!=own for i in infos):
                self.core.abort('Competing command publisher: '+topic)
        for topic in ('/fmu/in/custom_command_position_mode','/fmu/in/custom_command_velocity_mode','/control_mode_flags'):
            if self.get_publishers_info_by_topic(topic):
                self.core.abort('Legacy goal/keyboard publisher: '+topic)
        for topic in ('/servo_angle','/servo_angle_cmd','/fmu/in/custom_control_mode','/j1_angle','/j1_state','/j1_ready','/j1_limits'):
            if len(self.get_publishers_info_by_topic(topic))>1:
                self.core.abort('Duplicate support/feedback publisher: '+topic)
        required=('/servo_angle','/servo_angle_cmd','/fmu/in/custom_control_mode','/j1_state','/j1_ready','/j1_limits')
        self.support_ready=all(len(self.get_publishers_info_by_topic(topic))==1 for topic in required)
        if not self.support_ready and self.core.streaming:
            self.core.abort('Required allocation/driver endpoint disappeared')

    def preview(self, plan, label):
        summary = plan.summary()
        summary['label'] = label
        summary['assumption'] = 'Level roll/pitch; C is calculated from initial placement, not detected'
        message = String(); message.data = json.dumps(summary)
        self.preview_pub.publish(message)
        self.get_logger().info(f'{label}: C={plan.contact}; max rise={summary["max_rise_from_start_m"]:.3f}m; '
                               f'max horizontal distance={summary["max_horizontal_distance_from_start_m"]:.3f}m; '
                               f'nominal with settling={plan.nominal_duration:.1f}s')

    def on_timer(self):
        now = monotonic()
        try:
            if self.p.dry_run:
                if self.core.px4_fresh(now) and now-self.last_preview >= 2.0:
                    self.preview(Plan(self.core.position.xyz,self.core.position.yaw,self.p),'DRY_RUN_PREVIEW')
                    self.last_preview = now
                return
            if now-self.started < self.p.discovery_wait_s:
                return
            if now-self.last_graph >= 0.5:
                self.guard_graph(); self.last_graph=now
            if not self.support_ready and not self.core.streaming:
                self.core.reason='Waiting for one allocator and one independent J1 driver; graph presence is not hardware health proof'
                self.announce()
                return
            custom_ok=(self.legacy_trajectory_off and self.custom_mode_at is not None
                       and now-self.custom_mode_at <= self.p.status_timeout_s)
            limits_ok=(self.driver_limits_at is not None
                       and now-self.driver_limits_at <= self.p.joint_feedback_timeout_s)
            if not custom_ok:
                if self.core.streaming:
                    self.core.abort('Allocator custom mode stale or legacy trajectory enabled')
                else:
                    self.core.reason=self.support_reason
                    self.announce()
                    return
            if not limits_ok:
                if self.core.streaming:
                    self.core.fault_hold('J1 calibrated limits stale or incompatible')
                else:
                    self.core.reason=self.support_reason
                    self.announce()
                    return
            if self.core.streaming and (self.driver_ready_at is None or now-self.driver_ready_at > self.p.joint_feedback_timeout_s):
                self.core.fault_hold('J1 readiness timeout')
            before = self.core.state
            output = self.core.tick(now)
            if self.core.state in {'PRESTREAM','RUNNING'} and self.core.state != before:
                self.preview(self.core.plan, 'FINAL_CAPTURED_PLAN' if self.core.state=='RUNNING' else 'PREARM_PLAN')
                self.last_preview=now
            elif self.core.state=='READY' and not self.core.status.armed and now-self.last_preview>=2.0:
                self.preview(Plan(self.core.position.xyz,self.core.position.yaw,self.p),'PREARM_PLAN')
                self.last_preview=now
            if output.target is not None:
                stamp = self.get_clock().now().nanoseconds//1000
                if stamp <= self.last_ros_stamp or stamp <= 0:
                    self.core.abort('ROS clock stopped/regressed')
                else:
                    self.last_ros_stamp = stamp
                    emit(self.p.dry_run,output,stamp,self._command_publishers,self.ids,self.core.land_attempts)
                    if output.request_land:
                        self.get_logger().warning(f'Land request {self.core.land_attempts}/{self.p.land_max_attempts}')
        except Exception as e:
            self.core.abort('Callback failure: '+str(e))
        self.announce()

    def announce(self):
        notice=(self.core.state,self.core.phase,self.core.reason)
        if notice != self.last_notice:
            self.last_notice=notice
            self.get_logger().info(' | '.join(notice))


def main(args=None):
    import fcntl
    import os
    lock=open(f'/tmp/palletrone_task_flight_{os.getuid()}.lock','a',encoding='utf-8')
    try:
        fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise SystemExit('Another task_flight process holds the mission lock')
    node=None
    try:
        rclpy.init(args=args)
        node=TaskFlight()
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.close()


if __name__=='__main__':
    main()
