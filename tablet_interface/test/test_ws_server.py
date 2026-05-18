from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from tablet_interface import ws_server


class FakeLogger:
    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []
        self.errors: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)

    def warning(self, message: str) -> None:
        self.warnings.append(message)

    def error(self, message: str) -> None:
        self.errors.append(message)


class FakeNode:
    def __init__(self, *, db_path: str = "") -> None:
        self.config_storage_db_path = db_path
        self.state_publish_hz = 5.0
        self.logger = FakeLogger()
        self.params = {
            "bind_host": "127.0.0.1",
            "bind_port": 8765,
            "ws_path": "/ws/control",
            "config_storage_db_path": db_path,
        }

    def get_parameter(self, name: str) -> SimpleNamespace:
        return SimpleNamespace(value=self.params[name])

    def get_logger(self) -> FakeLogger:
        return self.logger

    def get_state(self) -> dict[str, object]:
        return {
            "connected": False,
            "cmd_age_ms": None,
            "watchdog_timeout_ms": 0,
            "last_seq": 0,
            "publishing_rate_hz": 5.0,
            "current_mode": 0,
            "gripper_state": None,
            "ee_pose": None,
            "tcp_speed_mps": None,
            "joint_positions": None,
        }

    def get_measure_result_snapshot(self) -> dict[str, object]:
        return {"revision": 0, "image_data_url": None, "vectors_json": None}

    def set_connected(self, connected: bool) -> None:
        self.connected = connected


def test_create_config_storage_returns_none_when_db_path_is_blank() -> None:
    node = FakeNode(db_path="   ")

    assert ws_server._create_config_storage(node) is None


def test_create_config_storage_initializes_database_and_logs(tmp_path: Path) -> None:
    db_path = tmp_path / "nested" / "ui_storage.sqlite3"
    node = FakeNode(db_path=str(db_path))

    storage = ws_server._create_config_storage(node)

    assert storage is not None
    assert db_path.exists()
    assert node.logger.infos == [f"Config storage API enabled at {db_path}"]


def test_create_app_registers_storage_routes_only_when_storage_is_enabled(
    tmp_path: Path,
) -> None:
    enabled_app = ws_server.create_app(FakeNode(db_path=str(tmp_path / "ui.sqlite3")))
    disabled_app = ws_server.create_app(FakeNode(db_path=""))

    enabled_paths = {route.path for route in enabled_app.routes}
    disabled_paths = {route.path for route in disabled_app.routes}
    assert "/api/storage/snapshot" in enabled_paths
    assert "/api/storage/snapshot" not in disabled_paths


def test_run_uvicorn_server_logs_and_returns_when_fastapi_stack_is_missing(
    monkeypatch,
) -> None:
    node = FakeNode()
    monkeypatch.setattr(ws_server, "uvicorn", None)

    ws_server.run_uvicorn_server(node)

    assert node.logger.errors == [
        "fastapi/uvicorn not available. WebSocket server cannot start."
    ]


def test_run_uvicorn_server_invokes_uvicorn_run(monkeypatch) -> None:
    node = FakeNode()
    called: dict[str, object] = {}

    def fake_run(app, *, host: str, port: int, log_level: str) -> None:
        called["app"] = app
        called["host"] = host
        called["port"] = port
        called["log_level"] = log_level

    monkeypatch.setattr(
        ws_server,
        "uvicorn",
        SimpleNamespace(run=fake_run),
    )

    ws_server.run_uvicorn_server(node)

    assert called["host"] == "127.0.0.1"
    assert called["port"] == 8765
    assert called["log_level"] == "info"
    assert "/ws/control" in {route.path for route in called["app"].routes}
