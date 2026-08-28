"""Pure position-frame transformations used by the mocap bridge."""

from math import isfinite
from typing import Iterable, Tuple


DEFAULT_MOCAP_TO_PX4_MATRIX = (
    1.0, 0.0, 0.0,
    0.0, -1.0, 0.0,
    0.0, 0.0, -1.0,
)
DEFAULT_POSITION_OFFSET_PX4 = (0.0, 0.0, 0.0)


def _finite_tuple(
    values: Iterable[float],
    expected_size: int,
    name: str,
) -> Tuple[float, ...]:
    """Convert an iterable to a finite float tuple of the requested size."""
    try:
        converted = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must contain numeric values') from exc

    if len(converted) != expected_size:
        raise ValueError(
            f'{name} must contain exactly {expected_size} elements; '
            f'got {len(converted)}'
        )
    if not all(isfinite(value) for value in converted):
        raise ValueError(f'{name} must contain only finite values')
    return converted


def determinant_3x3(matrix: Iterable[float]) -> float:
    """Return the determinant of a row-major 3x3 matrix."""
    values = _finite_tuple(matrix, 9, 'matrix')
    return (
        values[0] * (values[4] * values[8] - values[5] * values[7])
        - values[1] * (values[3] * values[8] - values[5] * values[6])
        + values[2] * (values[3] * values[7] - values[4] * values[6])
    )


def validate_rotation_matrix(
    matrix: Iterable[float],
    tolerance: float = 1.0e-3,
) -> Tuple[float, ...]:
    """Validate and return a row-major proper 3x3 rotation matrix.

    The matrix must be finite, approximately orthogonal, and have a
    determinant close to +1. Reflections are intentionally rejected.
    """
    values = _finite_tuple(matrix, 9, 'mocap_to_px4_matrix')
    if not isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError('tolerance must be finite and greater than zero')

    for row in range(3):
        for column in range(3):
            dot_product = sum(
                values[row * 3 + index] * values[column * 3 + index]
                for index in range(3)
            )
            expected = 1.0 if row == column else 0.0
            if abs(dot_product - expected) > tolerance:
                raise ValueError(
                    'mocap_to_px4_matrix must be approximately orthogonal'
                )

    determinant = determinant_3x3(values)
    if abs(determinant - 1.0) > tolerance:
        raise ValueError(
            'mocap_to_px4_matrix determinant must be approximately +1; '
            f'got {determinant:.6g}'
        )
    return values


def validate_position_offset(offset: Iterable[float]) -> Tuple[float, ...]:
    """Validate and return a three-element position offset."""
    return _finite_tuple(offset, 3, 'position_offset_px4')


def is_finite_position(position: Iterable[float]) -> bool:
    """Return whether position contains exactly three finite coordinates."""
    try:
        _finite_tuple(position, 3, 'position')
    except ValueError:
        return False
    return True


def transform_position(
    position: Iterable[float],
    matrix: Iterable[float] = DEFAULT_MOCAP_TO_PX4_MATRIX,
    offset: Iterable[float] = DEFAULT_POSITION_OFFSET_PX4,
) -> Tuple[float, float, float]:
    """Apply ``p_px4 = R_mocap_to_px4 * p_mocap + offset``."""
    position_values = _finite_tuple(position, 3, 'position')
    matrix_values = validate_rotation_matrix(matrix)
    offset_values = validate_position_offset(offset)

    transformed = tuple(
        sum(
            matrix_values[row * 3 + column] * position_values[column]
            for column in range(3)
        ) + offset_values[row]
        for row in range(3)
    )
    return transformed  # type: ignore[return-value]
