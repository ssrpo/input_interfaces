from __future__ import annotations

from pathlib import Path

from tablet_interface.config_storage import (
    SQLiteConfigStorage,
    StoragePayloadSnapshot,
    StorageSnapshot,
    StoredApplication,
    StoredConfiguration,
)


def test_initialize_creates_schema(tmp_path: Path) -> None:
    storage = SQLiteConfigStorage(tmp_path / "ui_storage.sqlite3")

    storage.initialize()

    assert storage.schema_version() == "1"


def test_upsert_and_list_applications_round_trip_payloads(tmp_path: Path) -> None:
    storage = SQLiteConfigStorage(tmp_path / "ui_storage.sqlite3")
    storage.initialize()

    storage.upsert_application(
        application_id="application-sandbox-copy",
        name="Sandbox Copy",
        payload={
            "id": "application-sandbox-copy",
            "name": "Sandbox Copy",
            "screenIds": ["sandbox_control"],
            "homeScreenId": "sandbox_control",
            "updatedAt": "2026-05-18T10:00:00Z",
        },
        updated_at="2026-05-18T10:00:00Z",
    )

    applications = storage.list_applications()

    assert len(applications) == 1
    assert applications[0].application_id == "application-sandbox-copy"
    assert applications[0].payload["screenIds"] == ["sandbox_control"]


def test_upsert_and_list_configurations_round_trip_payloads(tmp_path: Path) -> None:
    storage = SQLiteConfigStorage(tmp_path / "ui_storage.sqlite3")
    storage.initialize()

    storage.upsert_configuration(
        name="sandbox_control_copy",
        payload={
            "name": "sandbox_control_copy",
            "widgets": [{"id": "widget-1", "kind": "text", "label": "Sandbox Copy"}],
            "poses": [],
            "canvas": {"presetId": "hd", "runtimeMode": "fit"},
            "updatedAt": "2026-05-18T10:00:00Z",
        },
        updated_at="2026-05-18T10:00:00Z",
    )

    configurations = storage.list_configurations()

    assert len(configurations) == 1
    assert configurations[0].name == "sandbox_control_copy"
    assert configurations[0].payload["widgets"][0]["label"] == "Sandbox Copy"


def test_import_snapshot_is_additive_and_does_not_drop_existing_records(tmp_path: Path) -> None:
    storage = SQLiteConfigStorage(tmp_path / "ui_storage.sqlite3")
    storage.initialize()

    storage.upsert_application(
        application_id="application-sandbox",
        name="Sandbox",
        payload={"id": "application-sandbox", "name": "Sandbox"},
        updated_at="2026-05-18T10:00:00Z",
    )

    snapshot = StorageSnapshot(
        applications=[
            StoredApplication(
                application_id="application-sandbox-copy",
                name="Sandbox Copy",
                payload={"id": "application-sandbox-copy", "name": "Sandbox Copy"},
                created_at="2026-05-18T10:01:00Z",
                updated_at="2026-05-18T10:01:00Z",
            )
        ],
        configurations=[
            StoredConfiguration(
                name="sandbox_control_copy",
                payload={"name": "sandbox_control_copy", "widgets": []},
                created_at="2026-05-18T10:01:00Z",
                updated_at="2026-05-18T10:01:00Z",
            )
        ],
    )

    storage.import_snapshot(snapshot)
    exported = storage.export_snapshot()

    assert {application.application_id for application in exported.applications} == {
        "application-sandbox-copy",
        "application-sandbox",
    }
    assert [configuration.name for configuration in exported.configurations] == [
        "sandbox_control_copy"
    ]


def test_export_snapshot_preserves_structured_payloads_without_loss(tmp_path: Path) -> None:
    storage = SQLiteConfigStorage(tmp_path / "ui_storage.sqlite3")
    storage.initialize()

    original_payload = {
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

    storage.upsert_configuration(
        name="sandbox_control",
        payload=original_payload,
        updated_at="2026-05-18T10:00:00Z",
    )

    exported = storage.export_snapshot()

    assert exported.configurations[0].payload == original_payload


def test_import_and_export_payload_snapshot_round_trip_without_mutation(
    tmp_path: Path,
) -> None:
    storage = SQLiteConfigStorage(tmp_path / "ui_storage.sqlite3")
    storage.initialize()

    snapshot = StoragePayloadSnapshot(
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
                "updatedAt": "2026-05-18T10:05:00Z",
            }
        ],
    )

    storage.import_payload_snapshot(snapshot)
    exported = storage.export_payload_snapshot()
    exported.configurations[0]["widgets"][0]["onPayload"] = "mutated"

    assert storage.export_payload_snapshot() == snapshot
