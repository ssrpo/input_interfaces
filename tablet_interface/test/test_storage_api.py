from __future__ import annotations

import contextlib
import json
import socket
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import uvicorn

from tablet_interface.config_storage import SQLiteConfigStorage
from tablet_interface.ws_server import create_app


class _SilentLogger:
    def info(self, _: str) -> None:
        pass

    def warning(self, _: str) -> None:
        pass

    def error(self, _: str) -> None:
        pass


class _FakeNode:
    def __init__(self, *, config_storage_db_path: str) -> None:
        self.config_storage_db_path = config_storage_db_path
        self.state_publish_hz = 5.0
        self._logger = _SilentLogger()
        self._params = {
            "bind_host": "127.0.0.1",
            "bind_port": 8765,
            "ws_path": "/ws/control",
            "config_storage_db_path": config_storage_db_path,
        }

    def get_parameter(self, name: str) -> SimpleNamespace:
        return SimpleNamespace(value=self._params[name])

    def get_logger(self) -> _SilentLogger:
        return self._logger


def _find_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


@contextlib.contextmanager
def _run_http_app(node: _FakeNode):
    app = create_app(node)
    port = _find_free_port()
    server = uvicorn.Server(
        uvicorn.Config(
            app,
            host="127.0.0.1",
            port=port,
            log_level="error",
            access_log=False,
            lifespan="off",
        )
    )
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{port}"
    _wait_for_http_server(base_url)
    try:
        yield base_url
    finally:
        server.should_exit = True
        thread.join(timeout=5)
        if thread.is_alive():
            raise RuntimeError("Timed out while stopping uvicorn test server")


def _wait_for_http_server(base_url: str) -> None:
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            with urlopen(f"{base_url}/openapi.json", timeout=0.5) as response:
                if response.status == 200:
                    return
        except URLError:
            time.sleep(0.05)
    raise RuntimeError("Timed out while waiting for uvicorn test server")


def _request_json(
    base_url: str,
    path: str,
    *,
    method: str = "GET",
    payload: dict | None = None,
) -> tuple[int, dict]:
    body = None if payload is None else json.dumps(payload).encode("utf-8")
    headers = {"Accept": "application/json"}
    if body is not None:
        headers["Content-Type"] = "application/json"
    request = Request(f"{base_url}{path}", data=body, method=method, headers=headers)

    try:
        with urlopen(request, timeout=2.0) as response:
            return int(response.status), json.loads(response.read().decode("utf-8"))
    except HTTPError as exc:
        payload_text = exc.read().decode("utf-8")
        return int(exc.code), json.loads(payload_text)


def test_storage_snapshot_is_exposed_over_http(tmp_path: Path) -> None:
    db_path = tmp_path / "ui_storage.sqlite3"
    storage = SQLiteConfigStorage(db_path)
    storage.initialize()
    storage.upsert_configuration(
        name="sandbox_control",
        payload={
            "name": "sandbox_control",
            "widgets": [
                {
                    "id": "sandbox-toggle-output",
                    "kind": "ros-message-toggle",
                    "messageType": "std_msgs/msg/Int32MultiArray",
                    "onPayload": "{data: [13, 1]}",
                    "offPayload": "{data: [13, 0]}",
                }
            ],
            "poses": [],
            "canvas": {"presetId": "hd", "runtimeMode": "fit"},
            "updatedAt": "2026-05-18T10:00:00Z",
        },
        updated_at="2026-05-18T10:00:00Z",
    )

    with _run_http_app(_FakeNode(config_storage_db_path=str(db_path))) as base_url:
        status_code, response = _request_json(base_url, "/api/storage/snapshot")

    assert status_code == 200
    assert response == {
        "applications": [],
        "configurations": [
            {
                "name": "sandbox_control",
                "widgets": [
                    {
                        "id": "sandbox-toggle-output",
                        "kind": "ros-message-toggle",
                        "messageType": "std_msgs/msg/Int32MultiArray",
                        "onPayload": "{data: [13, 1]}",
                        "offPayload": "{data: [13, 0]}",
                    }
                ],
                "poses": [],
                "canvas": {"presetId": "hd", "runtimeMode": "fit"},
                "updatedAt": "2026-05-18T10:00:00Z",
            }
        ],
    }


def test_storage_snapshot_import_is_additive_over_http(tmp_path: Path) -> None:
    db_path = tmp_path / "ui_storage.sqlite3"
    storage = SQLiteConfigStorage(db_path)
    storage.initialize()
    storage.upsert_application(
        application_id="application-sandbox",
        name="Sandbox",
        payload={
            "id": "application-sandbox",
            "name": "Sandbox",
            "screenIds": ["sandbox_control"],
            "homeScreenId": "sandbox_control",
            "updatedAt": "2026-05-18T10:00:00Z",
        },
        updated_at="2026-05-18T10:00:00Z",
    )

    with _run_http_app(_FakeNode(config_storage_db_path=str(db_path))) as base_url:
        status_code, response = _request_json(
            base_url,
            "/api/storage/snapshot/import",
            method="POST",
            payload={
                "applications": [
                    {
                        "id": "application-sandbox-copy",
                        "name": "Sandbox Copy",
                        "screenIds": ["sandbox_control_copy"],
                        "homeScreenId": "sandbox_control_copy",
                        "updatedAt": "2026-05-18T10:05:00Z",
                    }
                ],
                "configurations": [
                    {
                        "name": "sandbox_control_copy",
                        "widgets": [],
                        "poses": [],
                        "canvas": {"presetId": "hd", "runtimeMode": "fit"},
                        "updatedAt": "2026-05-18T10:05:00Z",
                    }
                ],
            },
        )
        snapshot_status, snapshot_response = _request_json(
            base_url,
            "/api/storage/snapshot",
        )

    assert status_code == 200
    assert response == {
        "applicationsImported": 1,
        "configurationsImported": 1,
    }
    assert snapshot_status == 200
    assert [application["id"] for application in snapshot_response["applications"]] == [
        "application-sandbox",
        "application-sandbox-copy",
    ]
    assert [configuration["name"] for configuration in snapshot_response["configurations"]] == [
        "sandbox_control_copy"
    ]


def test_storage_snapshot_import_returns_422_over_http(tmp_path: Path) -> None:
    db_path = tmp_path / "ui_storage.sqlite3"
    storage = SQLiteConfigStorage(db_path)
    storage.initialize()

    with _run_http_app(_FakeNode(config_storage_db_path=str(db_path))) as base_url:
        status_code, response = _request_json(
            base_url,
            "/api/storage/snapshot/import",
            method="POST",
            payload={
                "applications": [{"name": "Missing id"}],
                "configurations": [],
            },
        )

    assert status_code == 422
    assert response == {
        "detail": "application payload must include a non-empty 'id'"
    }


def test_storage_routes_are_not_registered_without_db_path(tmp_path: Path) -> None:
    with _run_http_app(_FakeNode(config_storage_db_path="")) as base_url:
        status_code, response = _request_json(base_url, "/api/storage/snapshot")

    assert status_code == 404
    assert response == {"detail": "Not Found"}
