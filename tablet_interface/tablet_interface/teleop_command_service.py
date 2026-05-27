from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

from geometry_msgs.msg import Twist

from tablet_interface.teleop_mapping import map_and_scale


@dataclass(frozen=True)
class TeleopCommandSettings:
    linear_axes: tuple[int, int, int]
    linear_signs: tuple[float, float, float]
    angular_axes: tuple[int, int, int]
    angular_signs: tuple[float, float, float]
    linear_scale: float
    angular_scale: float
    swap_xy: bool
    default_mode: int
    accept_mode_from_client: bool


class TeleopCommandService:
    def __init__(self, settings: TeleopCommandSettings) -> None:
        self._settings = settings

    def map_and_scale_cmd(
        self,
        *,
        linear_values: Tuple[float, float, float],
        angular_values: Tuple[float, float, float],
    ) -> Twist:
        linear, angular = map_and_scale(
            linear_values=linear_values,
            angular_values=angular_values,
            linear_axes=self._settings.linear_axes,
            linear_signs=self._settings.linear_signs,
            angular_axes=self._settings.angular_axes,
            angular_signs=self._settings.angular_signs,
            linear_scale=self._settings.linear_scale,
            angular_scale=self._settings.angular_scale,
            swap_xy=self._settings.swap_xy,
        )
        twist = Twist()
        twist.linear.x = linear[0]
        twist.linear.y = linear[1]
        twist.linear.z = linear[2]
        twist.angular.x = angular[0]
        twist.angular.y = angular[1]
        twist.angular.z = angular[2]
        return twist

    def resolve_mode(self, requested_mode: int) -> int:
        if self._settings.accept_mode_from_client:
            return requested_mode
        return self._settings.default_mode

    @staticmethod
    def copy_twist(twist: Twist) -> Twist:
        out = Twist()
        out.linear.x = float(twist.linear.x)
        out.linear.y = float(twist.linear.y)
        out.linear.z = float(twist.linear.z)
        out.angular.x = float(twist.angular.x)
        out.angular.y = float(twist.angular.y)
        out.angular.z = float(twist.angular.z)
        return out
