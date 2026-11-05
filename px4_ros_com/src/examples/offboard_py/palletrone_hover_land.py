#!/usr/bin/env python3
"""QGC arms/selects Offboard; this one-shot node climbs, holds and requests Land."""

from dataclasses import fields
from time import monotonic

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck,
    VehicleLocalPosition, VehicleStatus,
)
from px4_ros_com.hover_land_logic import HoverLand, Position, Settings, Status
from px4_ros_com.hover_land_messages import check_publish_schemas, land_command, position_messages


class PalletroneHoverLand(Node):
    def __init__(self):
        super().__init__('palletrone_hover_land')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('This hardware node requires use_sim_time=false')
        descriptor = ParameterDescriptor(read_only=True)
        
        self.require_heading_good_for_control = self.declare_parameter(
            'require_heading_good_for_control', True, descriptor
        ).value

        if not self.require_heading_good_for_control:
            self.get_logger().info(
                'Using existing PX4 heading; heading_good_for_control gate disabled'
            )

        defaults = Settings()
        values = {f.name: self.declare_parameter(f.name, getattr(defaults, f.name), descriptor).value
                  for f in fields(Settings)}
        self.core = HoverLand(Settings(**values))
        self.ids = [self.declare_parameter(name, default, descriptor).value for name, default in [
            ('target_system_id', 1), ('target_component_id', 1),
            ('source_system_id', 245), ('source_component_id', 191)]]
        for value, maximum in zip(self.ids, [255, 255, 255, 65535]):
            if type(value) is not int or not 1 <= value <= maximum:
                raise ValueError('Invalid command system/component ID')
        if self.ids[:2] == self.ids[2:]:
            raise ValueError('Source and target IDs must differ')
        check_publish_schemas()
        self.qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', self.qos)
        self.goal_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.qos)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', self.qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_position, self.qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.on_status, self.qos)
        ack_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.on_ack, ack_qos)
        self.started_at = monotonic()
        self.last_graph_check = float('-inf')
        self.last_ros_stamp = 0
        self.last_ack_stamp = 0
        self.last_notice = None
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0 / self.core.p.publish_rate_hz, self.on_timer, clock=self.steady_clock)
        self.get_logger().info('No automatic Arm/Offboard/Disarm commands. Waiting for DISARMED and valid inputs.')

    def on_position(self, msg):
        self.core.receive_position(monotonic(), Position(
            stamp=msg.timestamp, sample_stamp=msg.timestamp_sample,
            xyz=(msg.x, msg.y, msg.z), yaw=msg.heading,
            xy_valid=msg.xy_valid,
            z_valid=msg.z_valid,

            heading_valid=(
                msg.heading_good_for_control
                or not self.require_heading_good_for_control
            ),           
            #heading_valid=msg.heading_good_for_control,
            resets=(msg.xy_reset_counter, msg.z_reset_counter,
                    msg.heading_reset_counter, msg.ref_timestamp)))

    def on_status(self, msg):
        self.core.receive_status(monotonic(), Status(
            stamp=msg.timestamp,
            armed=msg.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            disarmed=msg.arming_state == VehicleStatus.ARMING_STATE_DISARMED,
            offboard=msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            auto_land=msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND,
            healthy=(not msg.failsafe and msg.system_id == self.ids[0]
                     and msg.component_id == self.ids[1])))

    def on_ack(self, msg):
        if (msg.command != VehicleCommand.VEHICLE_CMD_NAV_LAND
                or msg.target_system != self.ids[2] or msg.target_component != self.ids[3]
                or msg.from_external or msg.timestamp <= self.last_ack_stamp):
            return
        self.last_ack_stamp = msg.timestamp
        if self.core.state == 'LAND_REQUESTED':
            self.core.receive_ack(monotonic(), int(msg.result))
            self.get_logger().info(
                f'Land ACK result={msg.result}; this does NOT confirm AUTO_LAND or landing completion')

    def check_publishers(self):
        # Discovery is a conflict guard, not proof of data flow or actuator health.
        owned = ['/fmu/in/offboard_control_mode', '/fmu/in/trajectory_setpoint', '/fmu/in/vehicle_command']
        own_name = (self.get_name(), self.get_namespace())
        for topic in owned:
            infos = self.get_publishers_info_by_topic(topic)
            if len(infos) > 1 or any((i.node_name, i.node_namespace) != own_name for i in infos):
                self.core.abort(f'Competing command publisher: {topic}')
        forbidden = ['/fmu/in/custom_command_position_mode', '/fmu/in/custom_command_velocity_mode',
                     '/control_mode_flags']
        for topic in forbidden:
            if self.get_publishers_info_by_topic(topic):
                self.core.abort(f'Legacy trajectory/keyboard publisher present: {topic}')
        for topic in ['/servo_angle_cmd', '/servo_angle', '/fmu/in/custom_control_mode']:
            if len(self.get_publishers_info_by_topic(topic)) > 1:
                self.core.abort(f'Duplicate actuator/support publisher: {topic}')

    def announce(self):
        notice = (self.core.state, self.core.reason)
        if notice != self.last_notice:
            self.last_notice = notice
            message = ': '.join(notice)
            if self.core.state in {'ABORTED', 'LAND_FAILED_HOLD'}:
                self.get_logger().error(message)
            else:
                self.get_logger().info(message)

    def on_timer(self):
        now = monotonic()
        try:
            if now - self.started_at < 2.0:
                return  # Discovery warm-up, no control publication.
            if now - self.last_graph_check >= 0.5:
                self.check_publishers()
                self.last_graph_check = now
            output = self.core.tick(now)
            if output.target is not None:
                stamp = self.get_clock().now().nanoseconds // 1000
                if stamp <= 0 or stamp <= self.last_ros_stamp:
                    self.core.abort('ROS clock stopped/regressed; streams stopped')
                else:
                    self.last_ros_stamp = stamp
                    mode, goal = position_messages(stamp, output.target, output.yaw)
                    self.goal_pub.publish(goal)
                    self.mode_pub.publish(mode)
                    if output.request_land:
                        self.command_pub.publish(land_command(stamp, *self.ids, self.core.attempts))
                        self.get_logger().warning(
                            f'Land request {self.core.attempts}/{self.core.p.land_max_attempts}; awaiting AUTO_LAND')
        except Exception as exc:
            self.core.abort(f'Control callback failed: {exc}')
        self.announce()


def main(args=None):
    # Same-host guard, independent of ROS node name/namespace. Keep the file:
    # unlinking a live flock file would permit a second process to bypass the lock.
    import fcntl
    import os
    lock = open(f'/tmp/palletrone_hover_land_{os.getuid()}.lock', 'a', encoding='utf-8')
    try:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise SystemExit('Another palletrone_hover_land process is already running')
    node = None
    try:
        rclpy.init(args=args)
        node = PalletroneHoverLand()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.close()


if __name__ == '__main__':
    main()
