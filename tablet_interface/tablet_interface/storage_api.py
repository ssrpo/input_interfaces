from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from tablet_interface.config_storage import SQLiteConfigStorage, StoragePayloadSnapshot


class StorageSnapshotBody(BaseModel):
    applications: list[dict[str, Any]] = Field(default_factory=list)
    configurations: list[dict[str, Any]] = Field(default_factory=list)


def create_storage_router(storage: SQLiteConfigStorage) -> APIRouter:
    router = APIRouter(prefix="/api/storage", tags=["storage"])

    @router.get("/snapshot", response_model=StorageSnapshotBody)
    def get_storage_snapshot() -> StorageSnapshotBody:
        snapshot = storage.export_payload_snapshot()
        return StorageSnapshotBody(
            applications=snapshot.applications,
            configurations=snapshot.configurations,
        )

    @router.post("/snapshot/import")
    def import_storage_snapshot(body: StorageSnapshotBody) -> dict[str, int]:
        try:
            storage.import_payload_snapshot(
                StoragePayloadSnapshot(
                    applications=body.applications,
                    configurations=body.configurations,
                )
            )
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc

        return {
            "applicationsImported": len(body.applications),
            "configurationsImported": len(body.configurations),
        }

    return router
