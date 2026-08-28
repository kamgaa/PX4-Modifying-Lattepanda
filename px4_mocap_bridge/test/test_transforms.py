"""Unit tests for the ROS-independent coordinate transforms."""

import pytest

from px4_mocap_bridge.transforms import DEFAULT_MOCAP_TO_PX4_MATRIX
from px4_mocap_bridge.transforms import transform_position
from px4_mocap_bridge.transforms import validate_rotation_matrix


def test_default_flu_to_frd_position_transform():
    """The default matrix maps ROS FLU position into PX4 FRD."""
    result = transform_position((1.0, 2.0, 3.0))

    assert result == pytest.approx((1.0, -2.0, -3.0))


def test_custom_rotation_matrix_is_applied():
    """A caller-provided proper rotation replaces the default mapping."""
    rotate_90_degrees_about_z = (
        0.0, -1.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 0.0, 1.0,
    )

    result = transform_position(
        (1.0, 2.0, 3.0),
        matrix=rotate_90_degrees_about_z,
    )

    assert result == pytest.approx((-2.0, 1.0, 3.0))


def test_position_offset_is_applied_after_rotation():
    """The PX4-frame offset is added after the matrix multiplication."""
    result = transform_position(
        (1.0, 2.0, 3.0),
        matrix=DEFAULT_MOCAP_TO_PX4_MATRIX,
        offset=(0.5, 1.0, -2.0),
    )

    assert result == pytest.approx((1.5, -1.0, -5.0))


def test_invalid_matrix_size_is_rejected():
    """A rotation matrix must contain exactly nine row-major values."""
    with pytest.raises(ValueError, match='exactly 9 elements'):
        validate_rotation_matrix((1.0,) * 8)


def test_non_orthogonal_matrix_is_rejected():
    """A finite 3x3 matrix is not sufficient unless it is orthogonal."""
    matrix = (
        1.0, 0.0, 0.0,
        0.0, 2.0, 0.0,
        0.0, 0.0, 1.0,
    )

    with pytest.raises(ValueError, match='orthogonal'):
        validate_rotation_matrix(matrix)


def test_reflection_matrix_is_rejected():
    """An orthogonal matrix with determinant -1 is not a rotation."""
    reflection = (
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, -1.0,
    )

    with pytest.raises(ValueError, match=r'determinant.*\+1'):
        validate_rotation_matrix(reflection)
