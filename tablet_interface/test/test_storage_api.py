from __future__ import annotations

from pathlib import Path

from fastapi import HTTPException

from tablet_interface.config_storage import SQLiteConfigStorage
from tablet_interface.storage_api import StorageSnapshotBody, create_storage_router


def _resolve_route_endpoint(storage: SQLiteConfigStorage, path: str, method: str):
    router = create_storage_router(storage)
    for route in router.routes:
        if getattr(route, "path", None) != path:
            continue
        if method.upper() not in getattr(route, "methods", set()):
            continue
        return route.endpoint
    raise AssertionError(f"Route {method} {path} was not registered")


def _create_storage(tmp_path: Path) -> SQLiteConfigStorage:
    storage = SQLiteConfigStorage(tmp_path / "ui_storage.sqlite3")
    storage.initialize()
    return storage


def test_get_snapshot_returns_structured_payloads(tmp_path: Path) -> None:
    storage = _create_storage(tmp_path)
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

    endpoint = _resolve_route_endpoint(storage, "/api/storage/snapshot", "GET")
    response = endpoint()

    assert response.model_dump() == {
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


def test_import_snapshot_is_additive(tmp_path: Path) -> None:
    storage = _create_storage(tmp_path)
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

    endpoint = _resolve_route_endpoint(storage, "/api/storage/snapshot/import", "POST")
    response = endpoint(
        StorageSnapshotBody(
            applications=[
                {
                    "id": "application-sandbox-copy",
                    "name": "Sandbox Copy",
                    "screenIds": ["sandbox_control_copy"],
                    "homeScreenId": "sandbox_control_copy",
                    "updatedAt": "2026-05-18T10:05:00Z",
                }
            ],
            configurations=[
                {
                    "name": "sandbox_control_copy",
                    "widgets": [],
                    "poses": [],
                    "canvas": {"presetId": "hd", "runtimeMode": "fit"},
                    "updatedAt": "2026-05-18T10:05:00Z",
                }
            ],
        )
    )

    assert response == {
        "applicationsImported": 1,
        "configurationsImported": 1,
    }
    exported = storage.export_payload_snapshot()
    assert [application["id"] for application in exported.applications] == [
        "application-sandbox",
        "application-sandbox-copy",
    ]
    assert [configuration["name"] for configuration in exported.configurations] == [
        "sandbox_control_copy"
    ]


def test_import_snapshot_rejects_missing_identifiers(tmp_path: Path) -> None:
    storage = _create_storage(tmp_path)
    endpoint = _resolve_route_endpoint(storage, "/api/storage/snapshot/import", "POST")

    try:
        endpoint(
            StorageSnapshotBody(
                applications=[{"name": "Missing id"}],
                configurations=[],
            )
        )
    except HTTPException as exc:
        assert exc.status_code == 422
        assert str(exc.detail) == "application payload must include a non-empty 'id'"
    else:
        raise AssertionError("Expected HTTPException for missing application id")
