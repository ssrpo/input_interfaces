from __future__ import annotations

from typing import Sequence, Tuple

Vector3 = Tuple[float, float, float]


def normalize_mapping(
    *,
    axes: Sequence[int],
    signs: Sequence[float],
) -> Tuple[Tuple[int, int, int], Tuple[float, float, float]]:
    if len(axes) != 3:
        raise ValueError("axes must have exactly 3 items")
    if len(signs) != 3:
        raise ValueError("signs must have exactly 3 items")

    mapped_axes = tuple(int(v) for v in axes)
    mapped_signs = tuple(float(v) for v in signs)

    if any(v < 0 or v > 2 for v in mapped_axes):
        raise ValueError("axes values must be in range [0, 2]")

    return mapped_axes, mapped_signs


def remap(values: Sequence[float], axes: Sequence[int], signs: Sequence[float]) -> Vector3:
    if len(values) != 3:
        raise ValueError("values must have exactly 3 items")
    if len(axes) != 3 or len(signs) != 3:
        raise ValueError("axes and signs must have exactly 3 items")

    v = (float(values[0]), float(values[1]), float(values[2]))
    return (
        v[int(axes[0])] * float(signs[0]),
        v[int(axes[1])] * float(signs[1]),
        v[int(axes[2])] * float(signs[2]),
    )


def map_and_scale(
    *,
    linear_values: Sequence[float],
    angular_values: Sequence[float],
    linear_axes: Sequence[int],
    linear_signs: Sequence[float],
    angular_axes: Sequence[int],
    angular_signs: Sequence[float],
    linear_scale: float,
    angular_scale: float,
    swap_xy: bool = False,
) -> Tuple[Vector3, Vector3]:
    linear = remap(linear_values, linear_axes, linear_signs)
    angular = remap(angular_values, angular_axes, angular_signs)

    if swap_xy:
        linear = (linear[1], linear[0], linear[2])
        angular = (angular[1], angular[0], angular[2])

    linear_scaled: Vector3 = (
        linear[0] * float(linear_scale),
        linear[1] * float(linear_scale),
        linear[2] * float(linear_scale),
    )
    angular_scaled: Vector3 = (
        angular[0] * float(angular_scale),
        angular[1] * float(angular_scale),
        angular[2] * float(angular_scale),
    )
    return linear_scaled, angular_scaled
