from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace


NODE_MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "tablet_interface" / "tablet_interface_node.py"
)


class FakeTeleopCommand:
    TRANSLATION_ROTATION = 7

    def __init__(self) -> None:
        self.mode = None


class FakePublisher:
    def __init__(self, message_cls: type, topic: str, qos: int) -> None:
        self.message_cls = message_cls
        self.topic = topic
        self.qos = qos
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class FakeLogger:
    def __init__(self) -> None:
        self.infos: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)


class FakeBaseNode:
    created_instances: list["FakeBaseNode"] = []

    def __init__(self, node_name: str) -> None:
        self.node_name = node_name
        self.logger = FakeLogger()
        self.parameters: dict[str, object] = {}
        self.publishers: list[FakePublisher] = []
        self.timers: list[tuple[float, object]] = []
        self.destroyed = False
        FakeBaseNode.created_instances.append(self)

    def declare_parameter(self, name: str, default: object) -> None:
        self.parameters[name] = default

    def create_publisher(self, message_cls: type, topic: str, qos: int) -> FakePublisher:
        publisher = FakePublisher(message_cls, topic, qos)
        self.publishers.append(publisher)
        return publisher

    def get_parameter(self, name: str) -> SimpleNamespace:
        return SimpleNamespace(value=self.parameters[name])

    def create_timer(self, period: float, callback):
        self.timers.append((period, callback))
        return {"period": period, "callback": callback}

    def get_logger(self) -> FakeLogger:
        return self.logger

    def destroy_node(self) -> None:
        self.destroyed = True


def _load_tablet_interface_node_module(monkeypatch, *, spin_side_effect: Exception | None = None):
    FakeBaseNode.created_instances.clear()
    calls = {
        "init": 0,
        "spin_nodes": [],
        "shutdown": 0,
    }

    def init() -> None:
        calls["init"] += 1

    def spin(node: object) -> None:
        calls["spin_nodes"].append(node)
        if spin_side_effect is not None:
            raise spin_side_effect

    def shutdown() -> None:
        calls["shutdown"] += 1

    fake_rclpy = ModuleType("rclpy")
    fake_rclpy.init = init
    fake_rclpy.spin = spin
    fake_rclpy.shutdown = shutdown
    fake_rclpy_node = ModuleType("rclpy.node")
    fake_rclpy_node.Node = FakeBaseNode
    fake_extender_msgs = ModuleType("extender_msgs.msg")
    fake_extender_msgs.TeleopCommand = FakeTeleopCommand

    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", fake_rclpy_node)
    monkeypatch.setitem(sys.modules, "extender_msgs.msg", fake_extender_msgs)

    spec = importlib.util.spec_from_file_location(
        "tablet_interface_node_under_test",
        NODE_MODULE_PATH,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, calls


def test_tablet_interface_node_initializes_publisher_and_timer(monkeypatch) -> None:
    module, _ = _load_tablet_interface_node_module(monkeypatch)

    node = module.TabletInterfaceNode()

    assert node.node_name == "tablet_interface_node"
    assert node.parameters == {
        "publish_rate_hz": 30.0,
        "start_mode": "TRANSLATION_ROTATION",
    }
    assert len(node.publishers) == 1
    assert node.publishers[0].message_cls is FakeTeleopCommand
    assert node.publishers[0].topic == "/teleop_cmd"
    assert node.timers == [(1.0 / 30.0, node._on_timer)]
    assert node.logger.infos == ["Tablet interface node initialized"]


def test_tablet_interface_node_timer_publishes_default_mode(monkeypatch) -> None:
    module, _ = _load_tablet_interface_node_module(monkeypatch)
    node = module.TabletInterfaceNode()

    node._on_timer()

    assert len(node.publishers[0].messages) == 1
    assert node.publishers[0].messages[0].mode == FakeTeleopCommand.TRANSLATION_ROTATION


def test_tablet_interface_node_main_shuts_down_even_on_keyboard_interrupt(
    monkeypatch,
) -> None:
    module, calls = _load_tablet_interface_node_module(
        monkeypatch,
        spin_side_effect=KeyboardInterrupt(),
    )

    module.main()

    assert calls["init"] == 1
    assert len(calls["spin_nodes"]) == 1
    assert calls["shutdown"] == 1
    assert FakeBaseNode.created_instances[0].destroyed is True
