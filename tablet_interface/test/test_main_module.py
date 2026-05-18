from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace


MAIN_PATH = Path(__file__).resolve().parents[1] / "tablet_interface" / "main.py"


class FakeLogger:
    def __init__(self) -> None:
        self.infos: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)


class FakeTabletInterfaceNode:
    created_instances: list["FakeTabletInterfaceNode"] = []

    def __init__(self) -> None:
        self.logger = FakeLogger()
        self.destroyed = False
        FakeTabletInterfaceNode.created_instances.append(self)

    def get_logger(self) -> FakeLogger:
        return self.logger

    def destroy_node(self) -> None:
        self.destroyed = True


class FakeThread:
    created_instances: list["FakeThread"] = []

    def __init__(self, *, target, args, daemon: bool) -> None:
        self.target = target
        self.args = args
        self.daemon = daemon
        self.started = False
        FakeThread.created_instances.append(self)

    def start(self) -> None:
        self.started = True


def _load_main_module(monkeypatch, *, spin_side_effect: Exception | None = None):
    FakeTabletInterfaceNode.created_instances.clear()
    FakeThread.created_instances.clear()
    calls = {
        "init": 0,
        "spin_nodes": [],
        "shutdown": 0,
        "ws_nodes": [],
    }

    def init() -> None:
        calls["init"] += 1

    def spin(node: object) -> None:
        calls["spin_nodes"].append(node)
        if spin_side_effect is not None:
            raise spin_side_effect

    def shutdown() -> None:
        calls["shutdown"] += 1

    def run_uvicorn_server(node: object) -> None:
        calls["ws_nodes"].append(node)

    fake_rclpy = ModuleType("rclpy")
    fake_rclpy.init = init
    fake_rclpy.spin = spin
    fake_rclpy.shutdown = shutdown
    fake_threading = ModuleType("threading")
    fake_threading.Thread = FakeThread
    fake_ros_module = ModuleType("tablet_interface.ros_teleop_publisher")
    fake_ros_module.TabletInterfaceNode = FakeTabletInterfaceNode
    fake_ws_module = ModuleType("tablet_interface.ws_server")
    fake_ws_module.run_uvicorn_server = run_uvicorn_server

    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "threading", fake_threading)
    monkeypatch.setitem(
        sys.modules, "tablet_interface.ros_teleop_publisher", fake_ros_module
    )
    monkeypatch.setitem(sys.modules, "tablet_interface.ws_server", fake_ws_module)

    spec = importlib.util.spec_from_file_location(
        "tablet_interface_main_under_test",
        MAIN_PATH,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, calls


def test_main_starts_ws_thread_and_shuts_down_cleanly(monkeypatch) -> None:
    module, calls = _load_main_module(monkeypatch)

    module.main()

    assert calls["init"] == 1
    assert len(FakeTabletInterfaceNode.created_instances) == 1
    node = FakeTabletInterfaceNode.created_instances[0]
    assert calls["spin_nodes"] == [node]
    assert calls["shutdown"] == 1
    assert node.destroyed is True
    assert node.logger.infos == [
        "tablet_interface_node started (ROS spinning + WS server thread)."
    ]
    assert len(FakeThread.created_instances) == 1
    thread = FakeThread.created_instances[0]
    assert thread.target is not None
    assert thread.args == (node,)
    assert thread.daemon is True
    assert thread.started is True


def test_main_handles_keyboard_interrupt_and_still_shuts_down(monkeypatch) -> None:
    module, calls = _load_main_module(monkeypatch, spin_side_effect=KeyboardInterrupt())

    module.main()

    assert calls["init"] == 1
    assert calls["shutdown"] == 1
    assert FakeTabletInterfaceNode.created_instances[0].destroyed is True
