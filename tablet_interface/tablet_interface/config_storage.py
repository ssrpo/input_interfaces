from __future__ import annotations

import json
import sqlite3
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _encode_payload(payload: dict[str, Any]) -> str:
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def _decode_payload(payload_json: str) -> dict[str, Any]:
    decoded = json.loads(payload_json)
    if not isinstance(decoded, dict):
        raise ValueError("Stored payload must decode to a JSON object")
    return decoded


@dataclass(frozen=True)
class StoredApplication:
    application_id: str
    name: str
    payload: dict[str, Any]
    created_at: str
    updated_at: str


@dataclass(frozen=True)
class StoredConfiguration:
    name: str
    payload: dict[str, Any]
    created_at: str
    updated_at: str


@dataclass(frozen=True)
class StorageSnapshot:
    applications: list[StoredApplication]
    configurations: list[StoredConfiguration]


@dataclass(frozen=True)
class StoragePayloadSnapshot:
    applications: list[dict[str, Any]]
    configurations: list[dict[str, Any]]


def _clone_payload(payload: dict[str, Any]) -> dict[str, Any]:
    return _decode_payload(_encode_payload(payload))


def _payload_updated_at(payload: dict[str, Any]) -> str | None:
    value = payload.get("updatedAt")
    return value if isinstance(value, str) and value.strip() else None


def _require_payload_string_field(
    payload: dict[str, Any], field_name: str, *, entity_name: str
) -> str:
    value = payload.get(field_name)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{entity_name} payload must include a non-empty '{field_name}'")
    return value.strip()


class SQLiteConfigStorage:
    def __init__(self, db_path: str | Path) -> None:
        self._db_path = Path(db_path)

    def initialize(self) -> None:
        with self._connect() as connection:
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS metadata (
                    key TEXT PRIMARY KEY,
                    value TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS applications (
                    application_id TEXT PRIMARY KEY,
                    name TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS configurations (
                    name TEXT PRIMARY KEY,
                    payload_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );

                INSERT OR IGNORE INTO metadata(key, value)
                VALUES ('schema_version', '1');
                """
            )

    def schema_version(self) -> str:
        with self._connect() as connection:
            row = connection.execute(
                "SELECT value FROM metadata WHERE key = 'schema_version'"
            ).fetchone()
        if row is None:
            raise ValueError("Schema has not been initialized")
        return str(row["value"])

    def upsert_application(
        self,
        *,
        application_id: str,
        name: str,
        payload: dict[str, Any],
        updated_at: str | None = None,
    ) -> StoredApplication:
        timestamp = updated_at or _utc_now_iso()
        with self._connect() as connection:
            connection.execute(
                """
                INSERT INTO applications (
                    application_id,
                    name,
                    payload_json,
                    created_at,
                    updated_at
                ) VALUES (
                    :application_id,
                    :name,
                    :payload_json,
                    :created_at,
                    :updated_at
                )
                ON CONFLICT(application_id) DO UPDATE SET
                    name = excluded.name,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at
                """,
                {
                    "application_id": application_id,
                    "name": name,
                    "payload_json": _encode_payload(payload),
                    "created_at": timestamp,
                    "updated_at": timestamp,
                },
            )
            row = connection.execute(
                """
                SELECT application_id, name, payload_json, created_at, updated_at
                FROM applications
                WHERE application_id = ?
                """,
                (application_id,),
            ).fetchone()
        if row is None:
            raise ValueError("Application upsert failed")
        return self._row_to_application(row)

    def upsert_configuration(
        self,
        *,
        name: str,
        payload: dict[str, Any],
        updated_at: str | None = None,
    ) -> StoredConfiguration:
        timestamp = updated_at or _utc_now_iso()
        with self._connect() as connection:
            connection.execute(
                """
                INSERT INTO configurations (
                    name,
                    payload_json,
                    created_at,
                    updated_at
                ) VALUES (
                    :name,
                    :payload_json,
                    :created_at,
                    :updated_at
                )
                ON CONFLICT(name) DO UPDATE SET
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at
                """,
                {
                    "name": name,
                    "payload_json": _encode_payload(payload),
                    "created_at": timestamp,
                    "updated_at": timestamp,
                },
            )
            row = connection.execute(
                """
                SELECT name, payload_json, created_at, updated_at
                FROM configurations
                WHERE name = ?
                """,
                (name,),
            ).fetchone()
        if row is None:
            raise ValueError("Configuration upsert failed")
        return self._row_to_configuration(row)

    def list_applications(self) -> list[StoredApplication]:
        with self._connect() as connection:
            rows = connection.execute(
                """
                SELECT application_id, name, payload_json, created_at, updated_at
                FROM applications
                ORDER BY name ASC, application_id ASC
                """
            ).fetchall()
        return [self._row_to_application(row) for row in rows]

    def list_configurations(self) -> list[StoredConfiguration]:
        with self._connect() as connection:
            rows = connection.execute(
                """
                SELECT name, payload_json, created_at, updated_at
                FROM configurations
                ORDER BY name ASC
                """
            ).fetchall()
        return [self._row_to_configuration(row) for row in rows]

    def import_snapshot(self, snapshot: StorageSnapshot) -> None:
        for application in snapshot.applications:
            self.upsert_application(
                application_id=application.application_id,
                name=application.name,
                payload=application.payload,
                updated_at=application.updated_at,
            )
        for configuration in snapshot.configurations:
            self.upsert_configuration(
                name=configuration.name,
                payload=configuration.payload,
                updated_at=configuration.updated_at,
            )

    def export_snapshot(self) -> StorageSnapshot:
        return StorageSnapshot(
            applications=self.list_applications(),
            configurations=self.list_configurations(),
        )

    def import_payload_snapshot(self, snapshot: StoragePayloadSnapshot) -> None:
        for payload in snapshot.applications:
            application_id = _require_payload_string_field(
                payload,
                "id",
                entity_name="application",
            )
            self.upsert_application(
                application_id=application_id,
                name=_require_payload_string_field(
                    payload,
                    "name",
                    entity_name="application",
                ),
                payload=_clone_payload(payload),
                updated_at=_payload_updated_at(payload),
            )
        for payload in snapshot.configurations:
            configuration_name = _require_payload_string_field(
                payload,
                "name",
                entity_name="configuration",
            )
            self.upsert_configuration(
                name=configuration_name,
                payload=_clone_payload(payload),
                updated_at=_payload_updated_at(payload),
            )

    def export_payload_snapshot(self) -> StoragePayloadSnapshot:
        snapshot = self.export_snapshot()
        return StoragePayloadSnapshot(
            applications=[
                _clone_payload(application.payload)
                for application in snapshot.applications
            ],
            configurations=[
                _clone_payload(configuration.payload)
                for configuration in snapshot.configurations
            ],
        )

    def _connect(self) -> sqlite3.Connection:
        self._db_path.parent.mkdir(parents=True, exist_ok=True)
        connection = sqlite3.connect(self._db_path)
        connection.row_factory = sqlite3.Row
        return connection

    @staticmethod
    def _row_to_application(row: sqlite3.Row) -> StoredApplication:
        return StoredApplication(
            application_id=str(row["application_id"]),
            name=str(row["name"]),
            payload=_decode_payload(str(row["payload_json"])),
            created_at=str(row["created_at"]),
            updated_at=str(row["updated_at"]),
        )

    @staticmethod
    def _row_to_configuration(row: sqlite3.Row) -> StoredConfiguration:
        return StoredConfiguration(
            name=str(row["name"]),
            payload=_decode_payload(str(row["payload_json"])),
            created_at=str(row["created_at"]),
            updated_at=str(row["updated_at"]),
        )
