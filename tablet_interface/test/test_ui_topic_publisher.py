from __future__ import annotations

from tablet_interface.ui_topic_publisher import UiTopicPublisherService


class FakeLogger:
    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)

    def warning(self, message: str) -> None:
        self.warnings.append(message)


class FakeGenericPublishers:
    def __init__(self) -> None:
        self.string_calls: list[tuple[str, str]] = []
        self.bool_calls: list[tuple[str, bool]] = []
        self.float_calls: list[tuple[str, float]] = []
        self.string_ok = True
        self.bool_ok = True
        self.float_ok = True

    def publish_string(self, topic: str, payload: str) -> bool:
        self.string_calls.append((topic, payload))
        return self.string_ok

    def publish_bool(self, topic: str, value: bool) -> bool:
        self.bool_calls.append((topic, value))
        return self.bool_ok

    def publish_float(self, topic: str, value: float) -> bool:
        self.float_calls.append((topic, value))
        return self.float_ok


class FakeTypedPublishers:
    def __init__(self) -> None:
        self.publish_calls: list[tuple[str, str, str]] = []
        self.ensure_calls: list[tuple[str, str]] = []
        self.publish_ok = True
        self.ensure_return = object()
        self.raise_publish_error = False
        self.raise_ensure_error = False

    def publish(self, topic: str, message_type: str, payload_text: str) -> bool:
        self.publish_calls.append((topic, message_type, payload_text))
        if self.raise_publish_error:
            raise ValueError("bad payload")
        return self.publish_ok

    def ensure_publisher(self, topic: str, message_type: str):
        self.ensure_calls.append((topic, message_type))
        if self.raise_ensure_error:
            raise ValueError("bad publisher")
        return self.ensure_return


def _create_service(
    *,
    sandbox_toggle_output_message_type: str = "",
    sandbox_toggle_output_mode: str = "numeric",
):
    logger = FakeLogger()
    generic_publishers = FakeGenericPublishers()
    typed_publishers = FakeTypedPublishers()
    service = UiTopicPublisherService(
        logger=logger,
        generic_publishers=generic_publishers,
        typed_publishers=typed_publishers,
        sandbox_toggle_output_topic="/sandbox/digital_output",
        sandbox_toggle_output_message_type=sandbox_toggle_output_message_type,
        sandbox_toggle_output_mode=sandbox_toggle_output_mode,
    )
    return service, logger, generic_publishers, typed_publishers


def test_publish_methods_delegate_and_log_on_success() -> None:
    service, logger, generic_publishers, typed_publishers = _create_service()

    assert service.publish_button("/topic/button", "throw") is True
    assert service.publish_bool("/topic/bool", True) is True
    assert service.publish_scalar("/topic/scalar", 0.75) is True
    assert service.publish_typed("/topic/typed", "std_msgs/msg/String", "{data: 'go'}") is True

    assert generic_publishers.string_calls == [("/topic/button", "throw")]
    assert generic_publishers.bool_calls == [("/topic/bool", True)]
    assert generic_publishers.float_calls == [("/topic/scalar", 0.75)]
    assert typed_publishers.publish_calls == [
        ("/topic/typed", "std_msgs/msg/String", "{data: 'go'}")
    ]
    assert logger.infos == [
        "Published generic UI button: topic=/topic/button payload=throw",
        "Published generic UI bool: topic=/topic/bool value=true",
        "Published generic UI scalar: topic=/topic/scalar value=0.750",
        "Published typed UI message: topic=/topic/typed message_type=std_msgs/msg/String",
    ]


def test_publish_typed_returns_false_and_logs_warning_on_value_error() -> None:
    service, logger, _, typed_publishers = _create_service()
    typed_publishers.raise_publish_error = True

    assert service.publish_typed("/topic/typed", "std_msgs/msg/String", "{") is False
    assert logger.warnings == ["Failed to publish typed UI message: bad payload"]


def test_resolve_sandbox_toggle_output_message_type_supports_legacy_modes() -> None:
    numeric_service, _, _, _ = _create_service(
        sandbox_toggle_output_message_type="",
        sandbox_toggle_output_mode="numeric",
    )
    boolean_service, _, _, _ = _create_service(
        sandbox_toggle_output_message_type="",
        sandbox_toggle_output_mode="boolean",
    )
    explicit_service, _, _, _ = _create_service(
        sandbox_toggle_output_message_type="std_msgs/msg/Int32",
        sandbox_toggle_output_mode="numeric",
    )

    assert numeric_service.resolve_sandbox_toggle_output_message_type() == "std_msgs/msg/Float64"
    assert boolean_service.resolve_sandbox_toggle_output_message_type() == "std_msgs/msg/Bool"
    assert explicit_service.resolve_sandbox_toggle_output_message_type() == "std_msgs/msg/Int32"


def test_resolve_sandbox_toggle_output_message_type_warns_on_unknown_legacy_mode() -> None:
    service, logger, _, _ = _create_service(
        sandbox_toggle_output_message_type="",
        sandbox_toggle_output_mode="legacy_weird_mode",
    )

    assert service.resolve_sandbox_toggle_output_message_type() == "std_msgs/msg/Float64"
    assert logger.warnings == [
        "Unsupported sandbox_toggle_output_mode=legacy_weird_mode, falling back to Float64"
    ]


def test_ensure_sandbox_toggle_output_publisher_handles_success_and_errors() -> None:
    service, logger, _, typed_publishers = _create_service(
        sandbox_toggle_output_message_type="std_msgs/msg/Bool",
        sandbox_toggle_output_mode="boolean",
    )

    service.ensure_sandbox_toggle_output_publisher()

    assert typed_publishers.ensure_calls == [
        ("/sandbox/digital_output", "std_msgs/msg/Bool")
    ]
    assert logger.infos == [
        "Eager sandbox toggle publisher ready: topic=/sandbox/digital_output message_type=std_msgs/msg/Bool"
    ]

    service, logger, _, typed_publishers = _create_service()
    typed_publishers.raise_ensure_error = True

    service.ensure_sandbox_toggle_output_publisher()

    assert logger.warnings == [
        "Failed to prepare eager sandbox toggle publisher: bad publisher"
    ]
