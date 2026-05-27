from __future__ import annotations

from typing import Protocol


class LoggerLike(Protocol):
    def info(self, message: str) -> None:
        ...

    def warning(self, message: str) -> None:
        ...


class GenericPublishersLike(Protocol):
    def publish_string(self, topic: str, payload: str) -> bool:
        ...

    def publish_bool(self, topic: str, value: bool) -> bool:
        ...

    def publish_float(self, topic: str, value: float) -> bool:
        ...


class TypedPublishersLike(Protocol):
    def publish(self, topic: str, message_type: str, payload_text: str) -> bool:
        ...

    def ensure_publisher(self, topic: str, message_type: str):
        ...


class UiTopicPublisherService:
    def __init__(
        self,
        *,
        logger: LoggerLike,
        generic_publishers: GenericPublishersLike,
        typed_publishers: TypedPublishersLike,
        sandbox_toggle_output_topic: str,
        sandbox_toggle_output_message_type: str,
        sandbox_toggle_output_mode: str,
    ) -> None:
        self._logger = logger
        self._generic_publishers = generic_publishers
        self._typed_publishers = typed_publishers
        self._sandbox_toggle_output_topic = sandbox_toggle_output_topic
        self._sandbox_toggle_output_message_type = sandbox_toggle_output_message_type
        self._sandbox_toggle_output_mode = sandbox_toggle_output_mode

    def publish_button(self, topic: str, payload: str) -> bool:
        ok = self._generic_publishers.publish_string(topic, payload)
        if not ok:
            return False
        self._logger.info(
            "Published generic UI button: topic={0} payload={1}".format(
                topic.strip(),
                payload,
            )
        )
        return True

    def publish_bool(self, topic: str, value: bool) -> bool:
        ok = self._generic_publishers.publish_bool(topic, value)
        if not ok:
            return False
        self._logger.info(
            "Published generic UI bool: topic={0} value={1}".format(
                topic.strip(),
                str(bool(value)).lower(),
            )
        )
        return True

    def publish_scalar(self, topic: str, value: float) -> bool:
        ok = self._generic_publishers.publish_float(topic, value)
        if not ok:
            return False
        self._logger.info(
            "Published generic UI scalar: topic={0} value={1:.3f}".format(
                topic.strip(),
                float(value),
            )
        )
        return True

    def publish_typed(
        self,
        topic: str,
        message_type: str,
        payload_text: str,
    ) -> bool:
        try:
            ok = self._typed_publishers.publish(topic, message_type, payload_text)
        except ValueError as exc:
            self._logger.warning(f"Failed to publish typed UI message: {exc}")
            return False

        if not ok:
            return False

        self._logger.info(
            "Published typed UI message: topic={0} message_type={1}".format(
                topic.strip(),
                message_type.strip(),
            )
        )
        return True

    def resolve_sandbox_toggle_output_message_type(self) -> str:
        normalized_message_type = self._sandbox_toggle_output_message_type.strip()
        if normalized_message_type:
            return normalized_message_type

        legacy_mode = self._sandbox_toggle_output_mode.strip().lower()
        if legacy_mode == "boolean":
            return "std_msgs/msg/Bool"
        if legacy_mode in {"", "numeric"}:
            return "std_msgs/msg/Float64"
        self._logger.warning(
            "Unsupported sandbox_toggle_output_mode={0}, falling back to Float64".format(
                self._sandbox_toggle_output_mode
            )
        )
        return "std_msgs/msg/Float64"

    def ensure_sandbox_toggle_output_publisher(self) -> None:
        normalized_topic = self._sandbox_toggle_output_topic.strip()
        if not normalized_topic:
            return

        resolved_message_type = self.resolve_sandbox_toggle_output_message_type()
        try:
            publisher = self._typed_publishers.ensure_publisher(
                normalized_topic,
                resolved_message_type,
            )
        except ValueError as exc:
            self._logger.warning(
                f"Failed to prepare eager sandbox toggle publisher: {exc}"
            )
            return

        if publisher is not None:
            self._logger.info(
                "Eager sandbox toggle publisher ready: topic={0} message_type={1}".format(
                    normalized_topic,
                    resolved_message_type,
                )
            )
