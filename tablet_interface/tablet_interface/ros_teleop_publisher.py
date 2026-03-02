from __future__ import annotations

import threading
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Float32MultiArray, String

from extender_msgs.msg import TeleopCommand

from tablet_interface.safety_gate import SafetyGate, TeleopCmd, Twist
from tablet_interface.teleop_mapping import map_and_scale, normalize_mapping


class TabletInterfaceNode(Node):
    def __init__(self) -> None:
        super().__init__("tablet_interface_node")

        self.declare_parameter("teleop_cmd_topic", "/teleop_cmd")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("watchdog_timeout_ms", 200)
        self.declare_parameter("max_linear_mps", 0.2)
        self.declare_parameter("max_linear_z_mps", 0.2)
        self.declare_parameter("max_angular_rps", 0.5)
        self.declare_parameter("linear_scale", 0.2)
        self.declare_parameter("angular_scale", 0.5)
        self.declare_parameter("z_scale", 0.2)
        self.declare_parameter("linear_axes", [0, 1, 2])
        self.declare_parameter("linear_signs", [1.0, 1.0, 1.0])
        self.declare_parameter("angular_axes", [0, 1, 2])
        self.declare_parameter("angular_signs", [1.0, 1.0, 1.0])
        self.declare_parameter("default_mode", 0)
        self.declare_parameter("accept_mode_from_client", True)
        self.declare_parameter("state_publish_hz", 5.0)
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("bind_port", 8765)
        self.declare_parameter("ws_path", "/ws/control")
        self.declare_parameter(
            "state_machine_topic", "/petanque_state_machine/change_state"
        )
        self.declare_parameter("hub_digital_output_topic", "/hub/digital_output")
        self.declare_parameter("hub_electromagnet_channel", 2.0)
        self.declare_parameter("petanque_param_service", "/petanque_throw/set_parameters")
        self.declare_parameter("petanque_total_duration_param", "total_duration")
        self.declare_parameter(
            "petanque_angle_between_start_and_finish_param",
            "angle_between_start_and_finish",
        )
        self.declare_parameter("param_call_timeout_sec", 1.5)

        self.teleop_cmd_topic = self.get_parameter("teleop_cmd_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.watchdog_timeout_ms = int(self.get_parameter("watchdog_timeout_ms").value)
        self.max_linear_mps = float(self.get_parameter("max_linear_mps").value)
        self.max_linear_z_mps = float(self.get_parameter("max_linear_z_mps").value)
        self.max_angular_rps = float(self.get_parameter("max_angular_rps").value)
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.z_scale = float(self.get_parameter("z_scale").value)
        linear_axes_param = list(self.get_parameter("linear_axes").value)
        linear_signs_param = list(self.get_parameter("linear_signs").value)
        angular_axes_param = list(self.get_parameter("angular_axes").value)
        angular_signs_param = list(self.get_parameter("angular_signs").value)
        self.default_mode = int(self.get_parameter("default_mode").value)
        self.accept_mode_from_client = bool(self.get_parameter("accept_mode_from_client").value)
        self.state_publish_hz = float(self.get_parameter("state_publish_hz").value)
        self.bind_host = str(self.get_parameter("bind_host").value)
        self.bind_port = int(self.get_parameter("bind_port").value)
        self.ws_path = str(self.get_parameter("ws_path").value)
        self.state_machine_topic = str(self.get_parameter("state_machine_topic").value)
        self.hub_digital_output_topic = str(
            self.get_parameter("hub_digital_output_topic").value
        )
        self.hub_electromagnet_channel = float(
            self.get_parameter("hub_electromagnet_channel").value
        )
        self.petanque_param_service = str(self.get_parameter("petanque_param_service").value)
        self.petanque_total_duration_param = str(
            self.get_parameter("petanque_total_duration_param").value
        )
        self.petanque_angle_between_start_and_finish_param = str(
            self.get_parameter("petanque_angle_between_start_and_finish_param").value
        )
        self.param_call_timeout_sec = float(self.get_parameter("param_call_timeout_sec").value)
        try:
            self.linear_axes, self.linear_signs = normalize_mapping(
                axes=linear_axes_param,
                signs=linear_signs_param,
            )
            self.angular_axes, self.angular_signs = normalize_mapping(
                axes=angular_axes_param,
                signs=angular_signs_param,
            )
        except ValueError as exc:
            self.get_logger().warning(
                f"Invalid axis mapping parameters, using identity mapping: {exc}"
            )
            self.linear_axes = (0, 1, 2)
            self.linear_signs = (1.0, 1.0, 1.0)
            self.angular_axes = (0, 1, 2)
            self.angular_signs = (1.0, 1.0, 1.0)

        self._gate = SafetyGate(
            watchdog_timeout_ms=self.watchdog_timeout_ms,
            max_linear_mps=self.max_linear_mps,
            max_linear_z_mps=self.max_linear_z_mps,
            max_angular_rps=self.max_angular_rps,
            default_mode=self.default_mode,
        )

        self._lock = threading.Lock()
        self._last_cmd_received_ms: Optional[int] = None
        self._last_seq: int = 0
        self._connected: bool = False
        self._last_events: List[str] = []

        self._publisher = self.create_publisher(TeleopCommand, self.teleop_cmd_topic, 10)
        self._state_cmd_publisher = self.create_publisher(
            String, self.state_machine_topic, 10
        )
        self._hub_digital_output_publisher = self.create_publisher(
            Float32MultiArray, self.hub_digital_output_topic, 10
        )
        self._petanque_param_client = self.create_client(
            SetParameters, self.petanque_param_service
        )
        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info("Tablet interface node initialized")
        self.get_logger().info(
            "Safety params: watchdog_timeout_ms={0} max_linear_mps={1:.3f} "
            "max_linear_z_mps={2:.3f} max_angular_rps={3:.3f} default_mode={4}".format(
                self.watchdog_timeout_ms,
                self.max_linear_mps,
                self.max_linear_z_mps,
                self.max_angular_rps,
                self.default_mode,
            )
        )
        self.get_logger().info(
            "WS params: bind_host={0} bind_port={1} ws_path={2} state_publish_hz={3:.1f}".format(
                self.bind_host,
                self.bind_port,
                self.ws_path,
                self.state_publish_hz,
            )
        )
        self.get_logger().info(
            "Scale params: linear_scale={0:.3f} z_scale={1:.3f} angular_scale={2:.3f}".format(
                self.linear_scale,
                self.z_scale,
                self.angular_scale,
            )
        )
        self.get_logger().info(
            "Mapping params: linear_axes={0} linear_signs={1} "
            "angular_axes={2} angular_signs={3}".format(
                self.linear_axes,
                self.linear_signs,
                self.angular_axes,
                self.angular_signs,
            )
        )
        self.get_logger().info(
            "Teleop params: topic={0} publish_rate_hz={1:.1f} accept_mode_from_client={2}".format(
                self.teleop_cmd_topic,
                self.publish_rate_hz,
                str(self.accept_mode_from_client).lower(),
            )
        )
        self.get_logger().info(
            "Petanque bridge: state_machine_topic={0} param_service={1} "
            "duration_param={2} angle_param={3}".format(
                self.state_machine_topic,
                self.petanque_param_service,
                self.petanque_total_duration_param,
                self.petanque_angle_between_start_and_finish_param,
            )
        )
        self.get_logger().info(
            "Hub bridge: digital_output_topic={0} electromagnet_channel={1:.1f}".format(
                self.hub_digital_output_topic,
                self.hub_electromagnet_channel,
            )
        )

    def map_and_scale_cmd(
        self,
        *,
        linear_values: Tuple[float, float, float],
        angular_values: Tuple[float, float, float],
    ) -> Twist:
        linear, angular = map_and_scale(
            linear_values=linear_values,
            angular_values=angular_values,
            linear_axes=self.linear_axes,
            linear_signs=self.linear_signs,
            angular_axes=self.angular_axes,
            angular_signs=self.angular_signs,
            linear_scale=self.linear_scale,
            z_scale=self.z_scale,
            angular_scale=self.angular_scale,
        )
        twist = Twist()
        twist.linear.x = linear[0]
        twist.linear.y = linear[1]
        twist.linear.z = linear[2]
        twist.angular.x = angular[0]
        twist.angular.y = angular[1]
        twist.angular.z = angular[2]
        return twist

    def update_latest_cmd(
        self,
        *,
        twist: Twist,
        mode: int,
        seq: int,
        received_ms: Optional[int] = None,
    ) -> None:
        now_ms = self._now_ms()
        if received_ms is None:
            received_ms = now_ms

        if not self.accept_mode_from_client:
            mode = self.default_mode

        cmd = TeleopCmd(
            twist=twist,
            mode=int(mode),
            seq=int(seq),
            received_ms=int(received_ms),
        )

        with self._lock:
            self._gate.update_cmd(cmd)
            self._last_cmd_received_ms = int(received_ms)
            self._last_seq = int(seq)

    def send_state_command(self, command: str) -> bool:
        normalized = command.strip().lower()
        if normalized not in {
            "teleop",
            "activate_throw",
            "go_to_start",
            "throw",
            "pick_up",
            "stop",
        }:
            self.get_logger().warning(f"Invalid state machine command: {command}")
            return False

        msg = String()
        msg.data = normalized
        self._state_cmd_publisher.publish(msg)
        self.get_logger().info(f"Published state machine command: {normalized}")
        return True

    def set_electromagnet(self, enabled: bool) -> bool:
        msg = Float32MultiArray()
        msg.data = [
            float(self.hub_electromagnet_channel),
            1.0 if enabled else 0.0,
        ]
        self._hub_digital_output_publisher.publish(msg)
        self.get_logger().info(
            "Published hub digital output: channel={0:.1f} value={1:.1f}".format(
                self.hub_electromagnet_channel,
                msg.data[1],
            )
        )
        return True

    def set_petanque_total_duration(self, total_duration: float) -> bool:
        if total_duration <= 0.0:
            self.get_logger().warning(
                f"Invalid total_duration={total_duration:.3f}; expected > 0"
            )
            return False

        return self._set_petanque_double_parameter(
            parameter_name=self.petanque_total_duration_param,
            value=float(total_duration),
        )

    def set_petanque_angle_between_start_and_finish(self, angle: float) -> bool:
        return self._set_petanque_double_parameter(
            parameter_name=self.petanque_angle_between_start_and_finish_param,
            value=float(angle),
        )

    def _set_petanque_double_parameter(self, *, parameter_name: str, value: float) -> bool:
        if not parameter_name:
            self.get_logger().warning("Petanque parameter name is empty")
            return False

        if not self._petanque_param_client.wait_for_service(
            timeout_sec=self.param_call_timeout_sec
        ):
            self.get_logger().warning(
                f"Service unavailable: {self.petanque_param_service}"
            )
            return False

        param = Parameter(
            name=parameter_name,
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value),
            ),
        )
        req = SetParameters.Request(parameters=[param])
        future = self._petanque_param_client.call_async(req)

        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=self.param_call_timeout_sec):
            self.get_logger().warning(
                f"Timeout while setting parameter {parameter_name}"
            )
            return False

        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover
            self.get_logger().warning(f"SetParameters call failed: {exc}")
            return False

        if not result or not result.results:
            self.get_logger().warning("SetParameters returned empty result")
            return False

        if not result.results[0].successful:
            reason = result.results[0].reason or "unknown error"
            self.get_logger().warning(
                f"Failed to set {parameter_name}: {reason}"
            )
            return False

        self.get_logger().info(
            f"Updated {parameter_name}={value:.3f}"
        )
        return True

    def set_connected(self, connected: bool) -> None:
        with self._lock:
            self._connected = bool(connected)

    def get_state(self) -> Dict[str, object]:
        with self._lock:
            now_ms = self._now_ms()
            cmd_age_ms = None
            if self._last_cmd_received_ms is not None:
                cmd_age_ms = int(now_ms) - int(self._last_cmd_received_ms)

            return {
                "connected": self._connected,
                "cmd_age_ms": cmd_age_ms,
                "watchdog_timeout_ms": self.watchdog_timeout_ms,
                "last_seq": self._last_seq,
                "publishing_rate_hz": float(self.publish_rate_hz),
                "current_mode": int(self._gate._last_mode),
                "events": list(self._last_events),
            }

    def _on_timer(self) -> None:
        now_ms = self._now_ms()
        with self._lock:
            twist, mode, events = self._gate.process(now_ms=now_ms)
            self._last_events = list(events)

        msg = TeleopCommand()
        msg.twist = twist
        msg.mode = int(mode)
        self._publisher.publish(msg)

    def _now_ms(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1_000_000)


__all__ = ["TabletInterfaceNode"]
