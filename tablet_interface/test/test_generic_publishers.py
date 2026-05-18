from __future__ import annotations

from tablet_interface.generic_publishers import GenericPublisherCache


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


def test_publishers_cache_and_publish_supported_message_types() -> None:
    node = FakeNode()
    cache = GenericPublisherCache(node)

    assert cache.publish_bool("/sandbox/enabled", True) is True
    assert cache.publish_string("/sandbox/state", "teleop") is True
    assert cache.publish_float("/sandbox/max_velocity", 0.75) is True
    assert cache.publish_compressed_image(
        "/sandbox/camera",
        image_format="jpeg",
        image_bytes=b"\x00\x01",
    ) is True
    assert cache.publish_bool("/sandbox/enabled", False) is True

    assert len(node.created_publishers) == 4
    bool_publisher = cache.ensure_bool_publisher("/sandbox/enabled")
    assert bool_publisher is not None
    assert [message.data for message in bool_publisher.messages] == [True, False]
    string_publisher = cache.ensure_string_publisher("/sandbox/state")
    assert string_publisher is not None
    assert string_publisher.messages[0].data == "teleop"
    float_publisher = cache.ensure_float_publisher("/sandbox/max_velocity")
    assert float_publisher is not None
    assert float_publisher.messages[0].data == 0.75


def test_empty_topics_are_rejected_with_clear_warnings() -> None:
    node = FakeNode()
    cache = GenericPublisherCache(node)

    assert cache.ensure_bool_publisher("   ") is None
    assert cache.ensure_string_publisher("   ") is None
    assert cache.ensure_float_publisher("   ") is None
    assert (
        cache.publish_compressed_image(
            "   ",
            image_format="png",
            image_bytes=b"\x00",
        )
        is False
    )

    assert node.logger.warnings == [
        "Bool publisher topic is empty",
        "String publisher topic is empty",
        "Float publisher topic is empty",
        "CompressedImage publisher topic is empty",
    ]
