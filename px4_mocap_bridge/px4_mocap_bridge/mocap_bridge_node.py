"""Bridge NamedPoseArrayV2 motion-capture positions to PX4 EKF2."""

import math
import time
from typing import Dict, Optional

import rclpy
from motion_capture_tracking_interfaces.msg import NamedPoseArrayV2
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from .conversion import (
    extract_position,
    make_vehicle_odometry,
    select_named_pose,
    stamp_to_nanoseconds,
)
from .transforms import (
    DEFAULT_MOCAP_TO_PX4_MATRIX,
    DEFAULT_POSITION_OFFSET_PX4,
    is_finite_position,
    validate_position_offset,
    validate_rotation_matrix,
)


class MocapBridgeNode(Node):
    """Publish fresh, position-only external vision samples for PX4 EKF2."""

    _WARNING_THROTTLE_NS = 5_000_000_000

    def __init__(self) -> None:
        super().__init__('px4_mocap_bridge')

        self.declare_parameter('rigid_body_name', 'drone')
        self.declare_parameter('input_topic', '/poses')
        self.declare_parameter(
            'output_topic',
            '/fmu/in/vehicle_visual_odometry',
        )
        self.declare_parameter('max_publish_rate_hz', 50.0)
        self.declare_parameter('max_input_age_ms', 100.0)
        self.declare_parameter('position_stddev_xy', 0.02)
        self.declare_parameter('position_stddev_z', 0.02)
        self.declare_parameter('quality', 100)
        self.declare_parameter(
            'mocap_to_px4_matrix',
            list(DEFAULT_MOCAP_TO_PX4_MATRIX),
        )
        self.declare_parameter(
            'position_offset_px4',
            list(DEFAULT_POSITION_OFFSET_PX4),
        )

        self._rigid_body_name = str(
            self.get_parameter('rigid_body_name').value
        )
        self._input_topic = str(self.get_parameter('input_topic').value)
        self._output_topic = str(self.get_parameter('output_topic').value)
        self._max_publish_rate_hz = float(
            self.get_parameter('max_publish_rate_hz').value
        )
        self._max_input_age_ms = float(
            self.get_parameter('max_input_age_ms').value
        )
        self._position_stddev_xy = float(
            self.get_parameter('position_stddev_xy').value
        )
        self._position_stddev_z = float(
            self.get_parameter('position_stddev_z').value
        )
        self._quality = int(self.get_parameter('quality').value)
        self._mocap_to_px4_matrix = validate_rotation_matrix(
            self.get_parameter('mocap_to_px4_matrix').value
        )
        self._position_offset_px4 = validate_position_offset(
            self.get_parameter('position_offset_px4').value
        )
        self._validate_parameters()

        self._minimum_publish_interval_ns = int(
            1_000_000_000 / self._max_publish_rate_hz
        )
        self._max_input_age_ns = int(self._max_input_age_ms * 1_000_000)
        self._last_publish_monotonic_ns: Optional[int] = None
        self._started_monotonic_ns = time.monotonic_ns()
        self._last_input_monotonic_ns: Optional[int] = None
        self._warning_times_ns: Dict[str, int] = {}
        self._input_degraded = False

        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        px4_input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._publisher = self.create_publisher(
            VehicleOdometry,
            self._output_topic,
            px4_input_qos,
        )
        self._subscription = self.create_subscription(
            NamedPoseArrayV2,
            self._input_topic,
            self._poses_callback,
            input_qos,
        )
        watchdog_period_sec = min(
            1.0,
            max(0.05, self._max_input_age_ms / 1_000.0),
        )
        self._watchdog_timer = self.create_timer(
            watchdog_period_sec,
            self._watchdog_callback,
        )

        self.get_logger().info(
            f'Bridging rigid body "{self._rigid_body_name}" from '
            f'{self._input_topic} to {self._output_topic} '
            f'(maximum {self._max_publish_rate_hz:g} Hz)'
        )

    def _validate_parameters(self) -> None:
        if not self._rigid_body_name:
            raise ValueError('rigid_body_name cannot be empty')
        if not self._input_topic:
            raise ValueError('input_topic cannot be empty')
        if not self._output_topic:
            raise ValueError('output_topic cannot be empty')
        if (
            not math.isfinite(self._max_publish_rate_hz)
            or self._max_publish_rate_hz <= 0.0
        ):
            raise ValueError('max_publish_rate_hz must be finite and positive')
        if (
            not math.isfinite(self._max_input_age_ms)
            or self._max_input_age_ms <= 0.0
        ):
            raise ValueError('max_input_age_ms must be finite and positive')
        if (
            not math.isfinite(self._position_stddev_xy)
            or self._position_stddev_xy < 0.0
        ):
            raise ValueError(
                'position_stddev_xy must be finite and non-negative'
            )
        if (
            not math.isfinite(self._position_stddev_z)
            or self._position_stddev_z < 0.0
        ):
            raise ValueError(
                'position_stddev_z must be finite and non-negative'
            )
        if self._quality < -1 or self._quality > 100:
            raise ValueError('quality must be in the PX4 range [-1, 100]')

    def _warn_throttled(self, key: str, message: str) -> None:
        now_ns = time.monotonic_ns()
        previous_ns = self._warning_times_ns.get(key)
        if (
            previous_ns is None
            or now_ns - previous_ns >= self._WARNING_THROTTLE_NS
        ):
            self.get_logger().warning(message)
            self._warning_times_ns[key] = now_ns
        self._input_degraded = True

    def _watchdog_callback(self) -> None:
        """Warn about input dropout without ever republishing a saved pose."""
        now_ns = time.monotonic_ns()
        reference_ns = (
            self._last_input_monotonic_ns
            if self._last_input_monotonic_ns is not None
            else self._started_monotonic_ns
        )
        if now_ns - reference_ns > self._max_input_age_ns:
            age_ms = (now_ns - reference_ns) / 1_000_000.0
            self._warn_throttled(
                'input_timeout',
                f'No mocap input received for {age_ms:.1f} ms; not publishing',
            )

    def _poses_callback(self, message: NamedPoseArrayV2) -> None:
        self._last_input_monotonic_ns = time.monotonic_ns()
        now = self.get_clock().now()
        timestamp_ns = max(0, int(now.nanoseconds))

        try:
            sample_timestamp_ns = stamp_to_nanoseconds(message.header.stamp)
        except ValueError as exc:
            self._warn_throttled('invalid_stamp', f'Ignoring input: {exc}')
            return

        if (
            sample_timestamp_ns != 0
            and timestamp_ns - sample_timestamp_ns > self._max_input_age_ns
        ):
            age_ms = (timestamp_ns - sample_timestamp_ns) / 1_000_000.0
            self._warn_throttled(
                'stale',
                f'Ignoring stale mocap input ({age_ms:.1f} ms old; '
                f'limit {self._max_input_age_ms:g} ms)',
            )
            return

        named_pose = select_named_pose(
            message.poses,
            self._rigid_body_name,
        )
        if named_pose is None:
            self._warn_throttled(
                'missing_body',
                f'Rigid body "{self._rigid_body_name}" is absent from '
                f'{self._input_topic}; not publishing',
            )
            return

        try:
            position = extract_position(named_pose)
        except ValueError as exc:
            self._warn_throttled('invalid_position', f'Ignoring input: {exc}')
            return
        if not is_finite_position(position):
            self._warn_throttled(
                'nonfinite_position',
                f'Rigid body "{self._rigid_body_name}" contains NaN or Inf; '
                'not publishing',
            )
            return

        monotonic_ns = time.monotonic_ns()
        if (
            self._last_publish_monotonic_ns is not None
            and monotonic_ns - self._last_publish_monotonic_ns
            < self._minimum_publish_interval_ns
        ):
            return

        odometry = make_vehicle_odometry(
            message,
            rigid_body_name=self._rigid_body_name,
            mocap_to_px4_matrix=self._mocap_to_px4_matrix,
            position_offset_px4=self._position_offset_px4,
            timestamp_us=timestamp_ns // 1_000,
            position_stddev_xy=self._position_stddev_xy,
            position_stddev_z=self._position_stddev_z,
            quality=self._quality,
            vehicle_odometry_type=VehicleOdometry,
        )
        if odometry is None:
            self._warn_throttled(
                'conversion_failed',
                'Mocap sample could not be converted; not publishing',
            )
            return

        self._publisher.publish(odometry)
        self._last_publish_monotonic_ns = monotonic_ns
        if self._input_degraded:
            self.get_logger().info('Valid mocap input restored; publishing resumed')
            self._input_degraded = False


def main(args=None) -> None:
    """Run the ROS 2 bridge node."""
    rclpy.init(args=args)
    node: Optional[MocapBridgeNode] = None
    try:
        node = MocapBridgeNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
