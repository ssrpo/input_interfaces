from __future__ import annotations

from geometry_msgs.msg import Twist

from tablet_interface.teleop_command_service import (
    TeleopCommandService,
    TeleopCommandSettings,
)


def _create_service(*, accept_mode_from_client: bool = True) -> TeleopCommandService:
    return TeleopCommandService(
        TeleopCommandSettings(
            linear_axes=(0, 1, 2),
            linear_signs=(1.0, -1.0, 1.0),
            angular_axes=(2, 1, 0),
            angular_signs=(1.0, 1.0, -1.0),
            linear_scale=0.5,
            angular_scale=0.25,
            swap_xy=False,
            default_mode=2,
            accept_mode_from_client=accept_mode_from_client,
        )
    )


def test_map_and_scale_cmd_applies_mapping_and_scaling() -> None:
    service = _create_service()

    twist = service.map_and_scale_cmd(
        linear_values=(1.0, 0.5, -0.5),
        angular_values=(0.2, -0.4, 0.8),
    )

    assert twist.linear.x == 0.5
    assert twist.linear.y == -0.25
    assert twist.linear.z == -0.25
    assert twist.angular.x == 0.2
    assert twist.angular.y == -0.1
    assert twist.angular.z == -0.05


def test_resolve_mode_honors_accept_mode_from_client_flag() -> None:
    assert _create_service(accept_mode_from_client=True).resolve_mode(1) == 1
    assert _create_service(accept_mode_from_client=False).resolve_mode(1) == 2


def test_copy_twist_returns_independent_copy() -> None:
    twist = Twist()
    twist.linear.x = 1.0
    twist.linear.y = 2.0
    twist.angular.z = 3.0

    copied = TeleopCommandService.copy_twist(twist)
    twist.linear.x = 9.0

    assert copied.linear.x == 1.0
    assert copied.linear.y == 2.0
    assert copied.angular.z == 3.0
