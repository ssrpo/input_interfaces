from __future__ import annotations

from typing import Annotated, Any

from fastapi import APIRouter, Depends, FastAPI, HTTPException, Request
from pydantic import BaseModel, Field

from tablet_interface.config_storage import SQLiteConfigStorage, StoragePayloadSnapshot


class StorageSnapshotBody(BaseModel):
    applications: list[dict[str, Any]] = Field(default_factory=list)
    configurations: list[dict[str, Any]] = Field(default_factory=list)


storage_router = APIRouter(prefix="/api/storage", tags=["storage"])


def install_storage_api(app: FastAPI, storage: SQLiteConfigStorage) -> None:
    app.state.config_storage = storage
    app.include_router(storage_router)


def resolve_storage(request: Request) -> SQLiteConfigStorage:
    storage = getattr(request.app.state, "config_storage", None)
    if not isinstance(storage, SQLiteConfigStorage):
        raise HTTPException(status_code=503, detail="storage API is not configured")
    return storage


StorageDependency = Annotated[SQLiteConfigStorage, Depends(resolve_storage)]


@storage_router.get("/snapshot", response_model=StorageSnapshotBody)
def get_storage_snapshot(storage: StorageDependency) -> StorageSnapshotBody:
    snapshot = storage.export_payload_snapshot()
    return StorageSnapshotBody(
        applications=snapshot.applications,
        configurations=snapshot.configurations,
    )


@storage_router.post("/snapshot/import")
def import_storage_snapshot(
    body: StorageSnapshotBody,
    storage: StorageDependency,
) -> dict[str, int]:
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
