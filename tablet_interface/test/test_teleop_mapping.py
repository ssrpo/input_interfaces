import pytest

from tablet_interface.teleop_mapping import map_and_scale, normalize_mapping, remap


def test_normalize_mapping_valid() -> None:
    axes, signs = normalize_mapping(
        axes=[1, 0, 2],
        signs=[-1.0, -1.0, 1.0],
    )
    assert axes == (1, 0, 2)
    assert signs == (-1.0, -1.0, 1.0)


@pytest.mark.parametrize(
    "axes,signs",
    [
        ([0, 1], [1.0, 1.0, 1.0]),
        ([0, 1, 2], [1.0, 1.0]),
        ([0, 1, 3], [1.0, 1.0, 1.0]),
    ],
)
def test_normalize_mapping_invalid(axes, signs) -> None:
    with pytest.raises(ValueError):
        normalize_mapping(axes=axes, signs=signs)


def test_remap_explorer_linear_signs() -> None:
    result = remap(
        values=(0.3, -0.4, 0.5),
        axes=(1, 0, 2),
        signs=(-1.0, -1.0, 1.0),
    )
    assert result == (0.4, -0.3, 0.5)


def test_map_and_scale_profile() -> None:
    linear, angular = map_and_scale(
        linear_values=(0.3, -0.4, 0.5),
        angular_values=(-0.2, 0.1, 0.6),
        linear_axes=(1, 0, 2),
        linear_signs=(-1.0, -1.0, 1.0),
        angular_axes=(0, 1, 2),
        angular_signs=(1.0, -1.0, -1.0),
        linear_scale=0.035,
        angular_scale=0.2,
    )
    assert linear == pytest.approx((0.014, -0.0105, 0.0175))
    assert angular == pytest.approx((-0.04, -0.02, -0.12))


def test_map_and_scale_swap_xy() -> None:
    linear, angular = map_and_scale(
        linear_values=(0.1, 0.4, -0.3),
        angular_values=(0.7, -0.2, 0.5),
        linear_axes=(0, 1, 2),
        linear_signs=(1.0, 1.0, 1.0),
        angular_axes=(0, 1, 2),
        angular_signs=(1.0, 1.0, 1.0),
        linear_scale=1.0,
        angular_scale=1.0,
        swap_xy=True,
    )
    assert linear == pytest.approx((0.4, 0.1, -0.3))
    assert angular == pytest.approx((-0.2, 0.7, 0.5))
