"""Unit tests for message conversion without a ROS installation."""

from math import isclose
from math import isnan
from types import SimpleNamespace

import pytest

from px4_mocap_bridge.conversion import make_vehicle_odometry
from px4_mocap_bridge.conversion import select_named_pose
from px4_mocap_bridge.transforms import DEFAULT_MOCAP_TO_PX4_MATRIX
from px4_mocap_bridge.transforms import DEFAULT_POSITION_OFFSET_PX4


class FakeVehicleOdometry:
    """Small stand-in exposing the constants used by the pure converter."""

    POSE_FRAME_FRD = 2
    VELOCITY_FRAME_UNKNOWN = 0


def make_named_pose(name, x, y, z):
    """Create a NamedPose-shaped object from standard-library types."""
    position = SimpleNamespace(x=x, y=y, z=z)
    pose = SimpleNamespace(position=position)
    return SimpleNamespace(name=name, pose=pose)


def make_pose_array(poses, sec=0, nanosec=0):
    """Create a NamedPoseArrayV2-shaped object for a conversion test."""
    stamp = SimpleNamespace(sec=sec, nanosec=nanosec)
    header = SimpleNamespace(stamp=stamp)
    return SimpleNamespace(header=header, poses=poses)


def convert(
    pose_array,
    rigid_body_name='drone',
    timestamp_us=9_000_000,
    position_stddev_xy=0.02,
    position_stddev_z=0.02,
):
    """Convert with the bridge's documented default parameters."""
    return make_vehicle_odometry(
        pose_array,
        rigid_body_name=rigid_body_name,
        mocap_to_px4_matrix=DEFAULT_MOCAP_TO_PX4_MATRIX,
        position_offset_px4=DEFAULT_POSITION_OFFSET_PX4,
        timestamp_us=timestamp_us,
        position_stddev_xy=position_stddev_xy,
        position_stddev_z=position_stddev_z,
        quality=100,
        vehicle_odometry_type=FakeVehicleOdometry,
    )


@pytest.mark.parametrize('invalid_value', [float('nan'), float('inf')])
def test_nan_and_inf_positions_are_rejected(invalid_value):
    """Non-finite rigid-body positions never create output messages."""
    pose_array = make_pose_array([
        make_named_pose('drone', 1.0, invalid_value, 3.0),
    ])

    assert convert(pose_array) is None


def test_rigid_body_is_selected_by_exact_name():
    """Selection returns the matching named pose, not another body."""
    other = make_named_pose('drone_2', 10.0, 20.0, 30.0)
    target = make_named_pose('drone', 1.0, 2.0, 3.0)

    selected = select_named_pose([other, target], 'drone')

    assert selected is target


def test_missing_rigid_body_does_not_create_a_message():
    """An array without the configured name produces no odometry."""
    pose_array = make_pose_array([
        make_named_pose('another_drone', 1.0, 2.0, 3.0),
    ])

    assert convert(pose_array) is None


def test_position_only_message_marks_pose_and_velocity_unknown():
    """Orientation and all velocity-related values are explicitly NaN."""
    pose_array = make_pose_array([
        make_named_pose('drone', 1.0, 2.0, 3.0),
    ])

    message = convert(pose_array)

    assert message is not None
    assert message.pose_frame == FakeVehicleOdometry.POSE_FRAME_FRD
    assert message.position == pytest.approx([1.0, -2.0, -3.0])
    assert message.velocity_frame == (
        FakeVehicleOdometry.VELOCITY_FRAME_UNKNOWN
    )
    assert all(isnan(value) for value in message.q)
    assert all(isnan(value) for value in message.velocity)
    assert all(isnan(value) for value in message.angular_velocity)
    assert all(isnan(value) for value in message.orientation_variance)
    assert all(isnan(value) for value in message.velocity_variance)
    assert message.reset_counter == 0
    assert message.quality == 100


def test_position_standard_deviations_are_squared_into_variances():
    """The PX4 field receives variances rather than standard deviations."""
    pose_array = make_pose_array([
        make_named_pose('drone', 1.0, 2.0, 3.0),
    ])

    message = convert(pose_array, position_stddev_z=0.03)

    assert message is not None
    assert all(
        isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-12)
        for actual, expected in zip(
            message.position_variance,
            (0.0004, 0.0004, 0.0009),
        )
    )


def test_nonzero_header_stamp_becomes_timestamp_sample_microseconds():
    """The ROS header stamp is truncated from nanoseconds to microseconds."""
    pose_array = make_pose_array(
        [make_named_pose('drone', 1.0, 2.0, 3.0)],
        sec=3,
        nanosec=456_789_999,
    )

    message = convert(pose_array, timestamp_us=9_876_543)

    assert message is not None
    assert message.timestamp == 9_876_543
    assert message.timestamp_sample == 3_456_789


def test_zero_header_stamp_falls_back_to_current_timestamp():
    """A zero ROS header stamp uses the message creation timestamp."""
    pose_array = make_pose_array([
        make_named_pose('drone', 1.0, 2.0, 3.0),
    ])

    message = convert(pose_array, timestamp_us=1_234_567)

    assert message is not None
    assert message.timestamp == 1_234_567
    assert message.timestamp_sample == 1_234_567
