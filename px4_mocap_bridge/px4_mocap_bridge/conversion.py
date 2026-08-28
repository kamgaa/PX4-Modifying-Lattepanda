"""ROS-independent conversion helpers for the mocap bridge."""

from math import isfinite
from typing import Any, Iterable, Optional, Type, TypeVar

from .transforms import is_finite_position, transform_position


VehicleOdometryT = TypeVar('VehicleOdometryT')


def stamp_to_nanoseconds(stamp: Any) -> int:
    """Convert a builtin_interfaces/Time-like object to nanoseconds."""
    try:
        seconds = int(stamp.sec)
        nanoseconds = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError('header stamp must provide integer sec and nanosec') from exc

    if seconds < 0:
        raise ValueError('header stamp cannot be negative')
    if nanoseconds < 0 or nanoseconds >= 1_000_000_000:
        raise ValueError('header stamp nanosec must be in [0, 1000000000)')
    return seconds * 1_000_000_000 + nanoseconds


def stamp_to_microseconds(stamp: Any) -> int:
    """Convert a builtin_interfaces/Time-like object to whole microseconds."""
    return stamp_to_nanoseconds(stamp) // 1_000


def select_named_pose(
    poses: Iterable[Any],
    rigid_body_name: str,
) -> Optional[Any]:
    """Return the first pose whose name exactly matches rigid_body_name."""
    for named_pose in poses:
        if getattr(named_pose, 'name', None) == rigid_body_name:
            return named_pose
    return None


def extract_position(named_pose: Any) -> tuple[float, float, float]:
    """Extract xyz coordinates from a NamedPose-like object."""
    try:
        position = named_pose.pose.position
        return (float(position.x), float(position.y), float(position.z))
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError('named pose does not contain a valid xyz position') from exc


def make_vehicle_odometry(
    named_pose_array: Any,
    *,
    rigid_body_name: str,
    mocap_to_px4_matrix: Iterable[float],
    position_offset_px4: Iterable[float],
    timestamp_us: int,
    position_stddev_xy: float,
    position_stddev_z: float,
    quality: int,
    vehicle_odometry_type: Type[VehicleOdometryT],
) -> Optional[VehicleOdometryT]:
    """Convert one named pose to a position-only VehicleOdometry-like object.

    No ROS modules are imported here. Passing the generated VehicleOdometry
    class creates a real PX4 message; tests can pass a lightweight stand-in.
    ``None`` means that the requested rigid body was absent or its position was
    not finite.
    """
    if timestamp_us < 0:
        raise ValueError('timestamp_us cannot be negative')
    if not isfinite(position_stddev_xy) or position_stddev_xy < 0.0:
        raise ValueError('position_stddev_xy must be finite and non-negative')
    if not isfinite(position_stddev_z) or position_stddev_z < 0.0:
        raise ValueError('position_stddev_z must be finite and non-negative')
    if quality < -128 or quality > 127:
        raise ValueError('quality must fit in the VehicleOdometry int8 field')

    named_pose = select_named_pose(named_pose_array.poses, rigid_body_name)
    if named_pose is None:
        return None

    position_mocap = extract_position(named_pose)
    if not is_finite_position(position_mocap):
        return None
    position_px4 = transform_position(
        position_mocap,
        mocap_to_px4_matrix,
        position_offset_px4,
    )

    timestamp_sample_us = stamp_to_microseconds(named_pose_array.header.stamp)
    if timestamp_sample_us == 0:
        timestamp_sample_us = timestamp_us

    nan = float('nan')
    message = vehicle_odometry_type()
    message.timestamp = int(timestamp_us)
    message.timestamp_sample = int(timestamp_sample_us)
    message.pose_frame = vehicle_odometry_type.POSE_FRAME_FRD
    message.position = [float(value) for value in position_px4]
    message.q = [nan, nan, nan, nan]
    message.velocity_frame = vehicle_odometry_type.VELOCITY_FRAME_UNKNOWN
    message.velocity = [nan, nan, nan]
    message.angular_velocity = [nan, nan, nan]
    message.position_variance = [
        float(position_stddev_xy ** 2),
        float(position_stddev_xy ** 2),
        float(position_stddev_z ** 2),
    ]
    message.orientation_variance = [nan, nan, nan]
    message.velocity_variance = [nan, nan, nan]
    message.reset_counter = 0
    message.quality = int(quality)
    return message
