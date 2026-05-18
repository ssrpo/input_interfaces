from __future__ import annotations

import json
import threading
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String

from extender_msgs.msg import TeleopCommand

from tablet_interface.config import (
    declare_tablet_interface_parameters,
    load_tablet_interface_config,
)
from tablet_interface.actuator_bridge import ActuatorBridge
from tablet_interface.camera_bridge import CameraBridge
from tablet_interface.generic_publishers import GenericPublisherCache
from tablet_interface.measure_bridge import MeasureBridge
from tablet_interface.measure_codec import load_demo_measure_image_data_url
from tablet_interface.petanque_bridge import PetanqueBridge
from tablet_interface.runtime_state import TabletRuntimeState
from tablet_interface.sandbox_bridge import SandboxBridge
from tablet_interface.teleop_mapping import map_and_scale, normalize_mapping
from tablet_interface.typed_message_publishers import TypedMessagePublisherCache

MEASURE_DEMO_VECTORS_JSON = json.dumps(
    {
        "source": "image_measures_demo",
        "distances_cm": [27.9],
    },
    separators=(",", ":"),
)


class TabletInterfaceNode(Node):
    def __init__(self) -> None:
        super().__init__("tablet_interface_node")

        declare_tablet_interface_parameters(self)
        config = load_tablet_interface_config(self)

        self.teleop_cmd_topic = config.teleop_cmd_topic
        self.publish_rate_hz = config.publish_rate_hz
        self.linear_scale = config.linear_scale
        self.angular_scale = config.angular_scale
        self.swap_xy = config.swap_xy
        linear_axes_param = config.linear_axes
        linear_signs_param = config.linear_signs
        angular_axes_param = config.angular_axes
        angular_signs_param = config.angular_signs
        self.default_mode = config.default_mode
        self.accept_mode_from_client = config.accept_mode_from_client
        self.state_publish_hz = config.state_publish_hz
        self.bind_host = config.bind_host
        self.bind_port = config.bind_port
        self.ws_path = config.ws_path
        self.state_machine_topic = config.state_machine_topic
        self.gripper_topic = config.gripper_topic
        self.gripper_open_position = config.gripper_open_position
        self.gripper_close_position = config.gripper_close_position
        self.hub_digital_output_topic = config.hub_digital_output_topic
        self.hub_electromagnet_channel = config.hub_electromagnet_channel
        self.petanque_param_service = config.petanque_param_service
        self.petanque_total_duration_param = config.petanque_total_duration_param
        self.petanque_angle_between_start_and_finish_param = (
            config.petanque_angle_between_start_and_finish_param
        )
        self.petanque_alpha_param = config.petanque_alpha_param
        self.measure_request_image_topic = config.measure_request_image_topic
        self.measure_result_image_topic = config.measure_result_image_topic
        self.measure_result_vectors_topic = config.measure_result_vectors_topic
        self.sandbox_ee_pose_topic = config.sandbox_ee_pose_topic
        self.sandbox_velocity_command_topic = config.sandbox_velocity_command_topic
        self.sandbox_joint_pose_topic = config.sandbox_joint_pose_topic
        self.sandbox_toggle_output_topic = config.sandbox_toggle_output_topic
        self.sandbox_toggle_output_message_type = (
            config.sandbox_toggle_output_message_type
        )
        self.sandbox_toggle_output_mode = config.sandbox_toggle_output_mode
        self.config_storage_db_path = config.config_storage_db_path
        self.param_call_timeout_sec = config.param_call_timeout_sec
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

        self._cmd_lock = threading.Lock()
        self._latest_twist = Twist()
        self._measure_demo_vectors_json: str = MEASURE_DEMO_VECTORS_JSON
        self._measure_demo_image_data_url: str | None = (
            load_demo_measure_image_data_url(__file__)
        )
        self._runtime_state = TabletRuntimeState(
            default_mode=self.default_mode,
            publish_rate_hz=self.publish_rate_hz,
            gripper_open_position=self.gripper_open_position,
            gripper_close_position=self.gripper_close_position,
            measure_demo_vectors_json=self._measure_demo_vectors_json,
            measure_demo_image_data_url=self._measure_demo_image_data_url,
        )
        self._generic_publishers = GenericPublisherCache(self)
        self._typed_publishers = TypedMessagePublisherCache(self)
        self._ensure_sandbox_toggle_output_publisher()

        self._publisher = self.create_publisher(TeleopCommand, self.teleop_cmd_topic, 10)
        self._state_cmd_publisher = self.create_publisher(
            String, self.state_machine_topic, 10
        )
        self._gripper_publisher = self.create_publisher(
            Float64MultiArray, self.gripper_topic, 10
        )
        self._hub_digital_output_publisher = self.create_publisher(
            Float32MultiArray, self.hub_digital_output_topic, 10
        )
        self._measure_request_image_publisher = self.create_publisher(
            CompressedImage, self.measure_request_image_topic, 10
        )
        self._gripper_subscription = self.create_subscription(
            Float64MultiArray, self.gripper_topic, self._on_gripper_command, 10
        )
        self._measure_result_image_subscription = self.create_subscription(
            CompressedImage,
            self.measure_result_image_topic,
            self._on_measure_result_image,
            10,
        )
        self._measure_result_vectors_subscription = self.create_subscription(
            String,
            self.measure_result_vectors_topic,
            self._on_measure_result_vectors,
            10,
        )
        self._sandbox_ee_pose_subscription = (
            self.create_subscription(
                PoseStamped,
                self.sandbox_ee_pose_topic,
                self._on_sandbox_ee_pose,
                10,
            )
            if self.sandbox_ee_pose_topic
            else None
        )
        self._sandbox_velocity_subscription = (
            self.create_subscription(
                TwistStamped,
                self.sandbox_velocity_command_topic,
                self._on_sandbox_velocity_command,
                10,
            )
            if self.sandbox_velocity_command_topic
            else None
        )
        self._sandbox_joint_pose_subscription = (
            self.create_subscription(
                Float64MultiArray,
                self.sandbox_joint_pose_topic,
                self._on_sandbox_joint_pose,
                10,
            )
            if self.sandbox_joint_pose_topic
            else None
        )
        self._petanque_param_client = self.create_client(
            SetParameters, self.petanque_param_service
        )
        self._petanque_bridge = PetanqueBridge(
            logger=self.get_logger(),
            state_cmd_publisher=self._state_cmd_publisher,
            param_client=self._petanque_param_client,
            param_service=self.petanque_param_service,
            total_duration_param=self.petanque_total_duration_param,
            angle_between_start_and_finish_param=(
                self.petanque_angle_between_start_and_finish_param
            ),
            alpha_param=self.petanque_alpha_param,
            param_call_timeout_sec=self.param_call_timeout_sec,
        )
        self._measure_bridge = MeasureBridge(
            logger=self.get_logger(),
            request_image_publisher=self._measure_request_image_publisher,
            runtime_state=self._runtime_state,
            request_image_topic=self.measure_request_image_topic,
            result_image_topic=self.measure_result_image_topic,
            result_vectors_topic=self.measure_result_vectors_topic,
        )
        self._sandbox_bridge = SandboxBridge(runtime_state=self._runtime_state)
        self._actuator_bridge = ActuatorBridge(
            logger=self.get_logger(),
            gripper_publisher=self._gripper_publisher,
            hub_digital_output_publisher=self._hub_digital_output_publisher,
            runtime_state=self._runtime_state,
            gripper_topic=self.gripper_topic,
            gripper_open_position=self.gripper_open_position,
            gripper_close_position=self.gripper_close_position,
            hub_digital_output_topic=self.hub_digital_output_topic,
            hub_electromagnet_channel=self.hub_electromagnet_channel,
        )
        self._camera_bridge = CameraBridge(
            logger=self.get_logger(),
            publishers=self._generic_publishers,
        )
        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info("Tablet interface node initialized")
        self.get_logger().info("SafetyGate disabled for debug: raw mapped command forwarding")
        self.get_logger().info(
            "WS params: bind_host={0} bind_port={1} ws_path={2} state_publish_hz={3:.1f}".format(
                self.bind_host,
                self.bind_port,
                self.ws_path,
                self.state_publish_hz,
            )
        )
        self.get_logger().info(
            "Scale params: linear_scale={0:.3f} angular_scale={1:.3f}".format(
                self.linear_scale,
                self.angular_scale,
            )
        )
        self.get_logger().info(
            "Mapping params: linear_axes={0} linear_signs={1} "
            "angular_axes={2} angular_signs={3} swap_xy={4}".format(
                self.linear_axes,
                self.linear_signs,
                self.angular_axes,
                self.angular_signs,
                str(self.swap_xy).lower(),
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
            "duration_param={2} angle_param={3} alpha_param={4}".format(
                self.state_machine_topic,
                self.petanque_param_service,
                self.petanque_total_duration_param,
                self.petanque_angle_between_start_and_finish_param,
                self.petanque_alpha_param,
            )
        )
        self.get_logger().info(
            "Gripper bridge: topic={0} open={1:.3f} close={2:.3f}".format(
                self.gripper_topic,
                self.gripper_open_position,
                self.gripper_close_position,
            )
        )
        self.get_logger().info(
            "Hub bridge: digital_output_topic={0} electromagnet_channel={1:.1f}".format(
                self.hub_digital_output_topic,
                self.hub_electromagnet_channel,
            )
        )
        self.get_logger().info(
            "Measure bridge: request_image_topic={0} result_image_topic={1} vectors_topic={2}".format(
                self.measure_request_image_topic,
                self.measure_result_image_topic,
                self.measure_result_vectors_topic,
            )
        )
        self.get_logger().info(
            "Sandbox feedback: ee_pose_topic={0} velocity_topic={1} joint_pose_topic={2} toggle_output_topic={3} toggle_output_message_type={4}".format(
                self.sandbox_ee_pose_topic or "disabled",
                self.sandbox_velocity_command_topic or "disabled",
                self.sandbox_joint_pose_topic or "disabled",
                self.sandbox_toggle_output_topic or "disabled",
                self._resolve_sandbox_toggle_output_message_type() or "disabled",
            )
        )
        self.get_logger().info(
            "Config storage: db_path={0}".format(
                self.config_storage_db_path or "disabled",
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
            angular_scale=self.angular_scale,
            swap_xy=self.swap_xy,
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

        with self._cmd_lock:
            self._latest_twist = self._copy_twist(twist)
        self._runtime_state.update_command_meta(
            mode=mode,
            seq=seq,
            received_ms=received_ms,
        )

    def send_state_command(self, command: str) -> bool:
        return self._petanque_bridge.send_state_command(command)

    def set_gripper(self, action: str) -> bool:
        return self._actuator_bridge.set_gripper(action)

    def set_electromagnet(self, enabled: bool) -> bool:
        return self._actuator_bridge.set_electromagnet(enabled)

    def publish_ui_button(self, topic: str, payload: str) -> bool:
        ok = self._generic_publishers.publish_string(topic, payload)
        if not ok:
            return False
        self.get_logger().info(
            "Published generic UI button: topic={0} payload={1}".format(
                topic.strip(),
                payload,
            )
        )
        return True

    def publish_ui_bool(self, topic: str, value: bool) -> bool:
        ok = self._generic_publishers.publish_bool(topic, value)
        if not ok:
            return False
        self.get_logger().info(
            "Published generic UI bool: topic={0} value={1}".format(
                topic.strip(),
                str(bool(value)).lower(),
            )
        )
        return True

    def publish_ui_typed(
        self,
        topic: str,
        message_type: str,
        payload_text: str,
    ) -> bool:
        try:
            ok = self._typed_publishers.publish(topic, message_type, payload_text)
        except ValueError as exc:
            self.get_logger().warning(f"Failed to publish typed UI message: {exc}")
            return False

        if not ok:
            return False

        self.get_logger().info(
            "Published typed UI message: topic={0} message_type={1}".format(
                topic.strip(),
                message_type.strip(),
            )
        )
        return True

    def publish_ui_scalar(self, topic: str, value: float) -> bool:
        ok = self._generic_publishers.publish_float(topic, value)
        if not ok:
            return False
        self.get_logger().info(
            "Published generic UI scalar: topic={0} value={1:.3f}".format(
                topic.strip(),
                float(value),
            )
        )
        return True

    def _resolve_sandbox_toggle_output_message_type(self) -> str:
        normalized_message_type = self.sandbox_toggle_output_message_type.strip()
        if normalized_message_type:
            return normalized_message_type

        legacy_mode = self.sandbox_toggle_output_mode.strip().lower()
        if legacy_mode == "boolean":
            return "std_msgs/msg/Bool"
        if legacy_mode in {"", "numeric"}:
            return "std_msgs/msg/Float64"
        self.get_logger().warning(
            "Unsupported sandbox_toggle_output_mode={0}, falling back to Float64".format(
                self.sandbox_toggle_output_mode
            )
        )
        return "std_msgs/msg/Float64"

    def _ensure_sandbox_toggle_output_publisher(self) -> None:
        normalized_topic = self.sandbox_toggle_output_topic.strip()
        if not normalized_topic:
            return

        resolved_message_type = self._resolve_sandbox_toggle_output_message_type()
        try:
            publisher = self._typed_publishers.ensure_publisher(
                normalized_topic,
                resolved_message_type,
            )
        except ValueError as exc:
            self.get_logger().warning(
                f"Failed to prepare eager sandbox toggle publisher: {exc}"
            )
            return

        if publisher is not None:
            self.get_logger().info(
                "Eager sandbox toggle publisher ready: topic={0} message_type={1}".format(
                    normalized_topic,
                    resolved_message_type,
                )
            )

    def _on_gripper_command(self, msg: Float64MultiArray) -> None:
        self._actuator_bridge.on_gripper_command(msg)

    def _set_gripper_state_from_position(self, position: float) -> None:
        self._runtime_state.update_gripper_position(position)

    def set_petanque_total_duration(self, total_duration: float) -> bool:
        return self._petanque_bridge.set_total_duration(total_duration)

    def set_petanque_angle_between_start_and_finish(self, angle: float) -> bool:
        return self._petanque_bridge.set_angle_between_start_and_finish(angle)

    def set_petanque_alpha(self, alpha: float) -> bool:
        return self._petanque_bridge.set_alpha(alpha)

    def publish_measure_request_image(self, image_data_url: str) -> bool:
        return self._measure_bridge.publish_request_image(image_data_url)

    def get_measure_result_snapshot(self) -> Dict[str, object]:
        return self._measure_bridge.get_result_snapshot()

    def _on_measure_result_image(self, msg: CompressedImage) -> None:
        self._measure_bridge.on_result_image(msg, now_ms=self._now_ms())

    def _on_measure_result_vectors(self, msg: String) -> None:
        self._measure_bridge.on_result_vectors(msg, now_ms=self._now_ms())

    def _on_sandbox_ee_pose(self, msg: PoseStamped) -> None:
        self._sandbox_bridge.on_ee_pose(msg)

    def _on_sandbox_velocity_command(self, msg: TwistStamped) -> None:
        self._sandbox_bridge.on_velocity_command(msg)

    def _on_sandbox_joint_pose(self, msg: Float64MultiArray) -> None:
        self._sandbox_bridge.on_joint_pose(msg)

    def publish_camera_frame(self, *, topic: str, image_data_url: str) -> bool:
        return self._camera_bridge.publish_frame(
            topic=topic,
            image_data_url=image_data_url,
        )

    def set_connected(self, connected: bool) -> None:
        self._runtime_state.set_connected(connected)

    def get_state(self) -> Dict[str, object]:
        return self._runtime_state.get_state(now_ms=self._now_ms())

    def _on_timer(self) -> None:
        with self._cmd_lock:
            twist = self._copy_twist(self._latest_twist)
        mode = self._runtime_state.get_current_mode()
        self._runtime_state.clear_events()

        msg = TeleopCommand()
        msg.twist = twist
        msg.mode = int(mode)
        self._publisher.publish(msg)

    @staticmethod
    def _copy_twist(twist: Twist) -> Twist:
        out = Twist()
        out.linear.x = float(twist.linear.x)
        out.linear.y = float(twist.linear.y)
        out.linear.z = float(twist.linear.z)
        out.angular.x = float(twist.angular.x)
        out.angular.y = float(twist.angular.y)
        out.angular.z = float(twist.angular.z)
        return out

    def _now_ms(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1_000_000)


__all__ = ["TabletInterfaceNode"]
