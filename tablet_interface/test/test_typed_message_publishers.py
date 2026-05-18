from __future__ import annotations

import pytest

from tablet_interface.typed_message_publishers import (
    TypedMessagePublisherCache,
    build_typed_message,
    parse_message_payload_text,
)


class FakePublisher:
    def __init__(self, message_cls: type, topic: str) -> None:
        self.message_cls = message_cls
        self.topic = topic
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class FakeLogger:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def warning(self, message: str) -> None:
        self.warnings.append(message)


class FakeNode:
    def __init__(self) -> None:
        self.logger = FakeLogger()
        self.created_publishers: list[tuple[type, str, int]] = []

    def get_logger(self) -> FakeLogger:
        return self.logger

    def create_publisher(self, message_cls: type, topic: str, qos: int) -> FakePublisher:
        self.created_publishers.append((message_cls, topic, qos))
        return FakePublisher(message_cls, topic)


def test_build_typed_message_from_mapping_payload() -> None:
    message = build_typed_message(
        "std_msgs/msg/String",
        "{data: 'activate_throw'}",
    )

    assert message.data == "activate_throw"


def test_build_typed_message_wraps_scalar_payload_for_single_field_messages() -> None:
    message = build_typed_message(
        "std_msgs/msg/Bool",
        "true",
    )

    assert message.data is True


def test_build_typed_message_supports_array_payloads() -> None:
    message = build_typed_message(
        "std_msgs/msg/Int32MultiArray",
        "{data: [4, 1]}",
    )

    assert list(message.data) == [4, 1]


def test_build_typed_message_supports_custom_field_mappings() -> None:
    message = build_typed_message(
        "geometry_msgs/msg/Vector3",
        "{x: 0.1, y: 0.0, z: -0.2}",
    )

    assert message.x == pytest.approx(0.1)
    assert message.y == pytest.approx(0.0)
    assert message.z == pytest.approx(-0.2)


def test_parse_message_payload_text_rejects_empty_payloads() -> None:
    with pytest.raises(ValueError, match="Typed message payload is empty"):
        parse_message_payload_text("   ")


def test_build_typed_message_rejects_scalar_payload_for_multi_field_messages() -> None:
    with pytest.raises(
        ValueError,
        match="Typed message payload must be a mapping for multi-field ROS messages",
    ):
        build_typed_message(
            "geometry_msgs/msg/Vector3",
            "0.5",
        )


def test_typed_message_publisher_cache_reuses_publishers_per_topic_and_type() -> None:
    node = FakeNode()
    cache = TypedMessagePublisherCache(node)

    cache.publish(
        "/arduino/command",
        "std_msgs/msg/Int32MultiArray",
        "{data: [4, 1]}",
    )
    cache.publish(
        "/arduino/command",
        "std_msgs/msg/Int32MultiArray",
        "{data: [4, 0]}",
    )

    assert len(node.created_publishers) == 1


def test_typed_message_publisher_cache_rejects_invalid_payload() -> None:
    node = FakeNode()
    cache = TypedMessagePublisherCache(node)

    with pytest.raises(ValueError, match="Invalid typed message payload"):
        cache.publish(
            "/arduino/command",
            "std_msgs/msg/Int32MultiArray",
            "{data: [4, }",
        )


def test_typed_message_publisher_cache_rejects_empty_topic() -> None:
    node = FakeNode()
    cache = TypedMessagePublisherCache(node)

    with pytest.raises(ValueError, match="Typed message publisher topic is empty"):
        cache.ensure_publisher("   ", "std_msgs/msg/String")
