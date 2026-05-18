from __future__ import annotations

import asyncio
from pathlib import Path

try:
    import uvicorn
    from fastapi import FastAPI, WebSocket
    from fastapi.websockets import WebSocketState
    from pydantic import ValidationError
except Exception:  # pragma: no cover
    uvicorn = None  # type: ignore
    FastAPI = None  # type: ignore
    WebSocket = None  # type: ignore
    WebSocketState = None  # type: ignore
    ValidationError = Exception  # type: ignore

from tablet_interface.ros_teleop_publisher import TabletInterfaceNode
from tablet_interface.config_storage import SQLiteConfigStorage
from tablet_interface.storage_api import create_storage_router
from tablet_interface.ws_handlers import (
    build_state_message,
    handle_ws_payload,
    send_event,
    send_measure_result,
)


def create_app(node: TabletInterfaceNode) -> FastAPI:
    if FastAPI is None or WebSocket is None:
        raise RuntimeError("fastapi is not available")

    app = FastAPI()
    storage = _create_config_storage(node)
    if storage is not None:
        app.include_router(create_storage_router(storage))

    host = node.get_parameter("bind_host").value
    port = int(node.get_parameter("bind_port").value)
    ws_path = node.get_parameter("ws_path").value

    async def _state_sender(websocket: WebSocket) -> None:
        interval = 1.0 / max(node.state_publish_hz, 1e-3)
        while True:
            await asyncio.sleep(interval)
            state = node.get_state()
            await websocket.send_json(build_state_message(state))

    async def _measure_sender(websocket: WebSocket) -> None:
        last_revision = -1
        while True:
            await asyncio.sleep(0.2)
            snapshot = node.get_measure_result_snapshot()
            revision = int(snapshot.get("revision") or 0)
            if revision <= last_revision:
                continue
            last_revision = revision

            image_data_url = snapshot.get("image_data_url")
            vectors_json = snapshot.get("vectors_json")
            if image_data_url is None and vectors_json is None:
                continue

            await send_measure_result(
                websocket,
                image_data_url=image_data_url if isinstance(image_data_url, str) else None,
                vectors_json=vectors_json if isinstance(vectors_json, str) else None,
                updated_at_ms=(
                    int(snapshot["updated_at_ms"])
                    if isinstance(snapshot.get("updated_at_ms"), int)
                    else None
                ),
            )

    @app.websocket(ws_path)
    async def ws_endpoint(websocket: WebSocket) -> None:
        await websocket.accept()
        node.set_connected(True)
        node.get_logger().info("WS client connected")
        await send_event(websocket, "WS_CONNECTED", "info", "WS connected")

        state_task = asyncio.create_task(_state_sender(websocket))
        measure_task = asyncio.create_task(_measure_sender(websocket))
        try:
            while True:
                payload = await websocket.receive_json()
                try:
                    await handle_ws_payload(node, websocket, payload)
                except ValidationError as exc:
                    node.get_logger().warning(f"WS cmd invalid: {exc}")
                    await send_event(
                        websocket,
                        code="CMD_INVALID",
                        severity="warning",
                        message=str(exc),
                    )
        except Exception:
            pass
        finally:
            state_task.cancel()
            measure_task.cancel()
            node.set_connected(False)
            node.get_logger().info("WS client disconnected")
            if WebSocketState is not None and websocket.client_state == WebSocketState.CONNECTED:
                try:
                    await send_event(
                        websocket,
                        "WS_DISCONNECTED",
                        "info",
                        "WS disconnected",
                    )
                except Exception:
                    pass

    node.get_logger().info(f"WebSocket listening on ws://{host}:{port}{ws_path}")
    return app


def _create_config_storage(node: TabletInterfaceNode) -> SQLiteConfigStorage | None:
    raw_db_path = getattr(node, "config_storage_db_path", "") or str(
        node.get_parameter("config_storage_db_path").value
    )
    db_path = raw_db_path.strip()
    if not db_path:
        return None

    resolved_db_path = Path(db_path).expanduser()
    storage = SQLiteConfigStorage(resolved_db_path)
    storage.initialize()
    node.get_logger().info(f"Config storage API enabled at {resolved_db_path}")
    return storage


def run_uvicorn_server(node: TabletInterfaceNode) -> None:
    if uvicorn is None or FastAPI is None:
        node.get_logger().error(
            "fastapi/uvicorn not available. WebSocket server cannot start."
        )
        return

    try:
        from tablet_interface import ws_models as _ws_models  # noqa: F401
    except Exception as exc:  # pragma: no cover
        node.get_logger().error(
            f"pydantic not available. WebSocket server cannot start: {exc}"
        )
        return

    host = node.get_parameter("bind_host").value
    port = int(node.get_parameter("bind_port").value)
    app = create_app(node)
    uvicorn.run(app, host=host, port=port, log_level="info")
