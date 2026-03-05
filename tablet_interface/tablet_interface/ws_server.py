from __future__ import annotations

import asyncio

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


def run_uvicorn_server(node: TabletInterfaceNode) -> None:
    if uvicorn is None or FastAPI is None:
        node.get_logger().error(
            "fastapi/uvicorn not available. WebSocket server cannot start."
        )
        return

    try:
        from tablet_interface.ws_models import (
            CmdMessage,
            EventMessage,
            GripperCmdMessage,
            PetanqueConfigMessage,
            StateCmdMessage,
            UiButtonMessage,
        )
    except Exception as exc:  # pragma: no cover
        node.get_logger().error(
            f"pydantic not available. WebSocket server cannot start: {exc}"
        )
        return

    host = node.get_parameter("bind_host").value
    port = int(node.get_parameter("bind_port").value)
    ws_path = node.get_parameter("ws_path").value

    app = FastAPI()

    async def _send_event(
        websocket: WebSocket,
        code: str,
        severity: str,
        message: str,
    ) -> None:
        event = EventMessage(type="event", severity=severity, code=code, message=message)
        await websocket.send_json(event.model_dump())

    async def _state_sender(websocket: WebSocket) -> None:
        interval = 1.0 / max(node.state_publish_hz, 1e-3)
        while True:
            await asyncio.sleep(interval)
            state = node.get_state()
            message = {
                "type": "state",
                "connected": state["connected"],
                "cmd_age_ms": state["cmd_age_ms"],
                "watchdog_timeout_ms": state["watchdog_timeout_ms"],
                "last_seq": state["last_seq"],
                "publishing_rate_hz": state["publishing_rate_hz"],
                "current_mode": state["current_mode"],
            }
            await websocket.send_json(message)

    @app.websocket(ws_path)
    async def ws_endpoint(websocket: WebSocket) -> None:
        await websocket.accept()
        node.set_connected(True)
        node.get_logger().info("WS client connected")
        await _send_event(websocket, "WS_CONNECTED", "info", "WS connected")

        state_task = asyncio.create_task(_state_sender(websocket))
        try:
            while True:
                payload = await websocket.receive_json()
                msg_type = payload.get("type") if isinstance(payload, dict) else None
                try:
                    if msg_type == "teleop_cmd":
                        cmd = CmdMessage.model_validate(payload)
                        twist = node.map_and_scale_cmd(
                            linear_values=(cmd.linear.x, cmd.linear.y, cmd.linear.z),
                            angular_values=(cmd.angular.x, cmd.angular.y, cmd.angular.z),
                        )
                        node.update_latest_cmd(
                            twist=twist,
                            mode=int(cmd.mode),
                            seq=int(cmd.seq),
                            received_ms=node._now_ms(),
                        )
                        node.get_logger().debug(
                            f"WS teleop_cmd accepted seq={cmd.seq} mode={cmd.mode}"
                        )
                        continue

                    if msg_type == "state_cmd":
                        state_cmd = StateCmdMessage.model_validate(payload)
                        ok = node.send_state_command(state_cmd.command)
                        await _send_event(
                            websocket,
                            code="STATE_CMD_OK" if ok else "STATE_CMD_FAILED",
                            severity="info" if ok else "warning",
                            message=f"state_cmd={state_cmd.command}",
                        )
                        continue

                    if msg_type == "gripper_cmd":
                        gripper_cmd = GripperCmdMessage.model_validate(payload)
                        ok = node.set_gripper(gripper_cmd.action)
                        await _send_event(
                            websocket,
                            code="GRIPPER_CMD_OK" if ok else "GRIPPER_CMD_FAILED",
                            severity="info" if ok else "warning",
                            message=f"gripper_cmd={gripper_cmd.action}",
                        )
                        continue

                    if msg_type == "petanque_cfg":
                        cfg = PetanqueConfigMessage.model_validate(payload)
                        ok = True
                        updated_fields: list[str] = []
                        if cfg.total_duration is not None:
                            ok = node.set_petanque_total_duration(cfg.total_duration) and ok
                            updated_fields.append(f"total_duration={cfg.total_duration:.3f}")
                        if cfg.angle_between_start_and_finish is not None:
                            ok = (
                                node.set_petanque_angle_between_start_and_finish(
                                    cfg.angle_between_start_and_finish
                                )
                                and ok
                            )
                            updated_fields.append(
                                "angle_between_start_and_finish="
                                f"{cfg.angle_between_start_and_finish:.3f}"
                            )
                        if updated_fields:
                            node.get_logger().info(
                                "Applied petanque_cfg: " + ", ".join(updated_fields)
                            )
                        await _send_event(
                            websocket,
                            code="PETANQUE_CFG_OK" if ok else "PETANQUE_CFG_FAILED",
                            severity="info" if ok else "warning",
                            message=", ".join(updated_fields),
                        )
                        continue

                    if msg_type == "ui_button":
                        button = UiButtonMessage.model_validate(payload)
                        handled = False
                        if button.topic == node.state_machine_topic:
                            handled = True
                            ok = node.send_state_command(button.payload)
                            await _send_event(
                                websocket,
                                code="STATE_CMD_OK" if ok else "STATE_CMD_FAILED",
                                severity="info" if ok else "warning",
                                message=f"ui_button payload={button.payload}",
                            )
                        if button.topic == node.hub_digital_output_topic:
                            normalized = button.payload.strip().lower()
                            enable_values = {
                                "electromagnet_on",
                                "on",
                                "1",
                                "true",
                                "activate",
                            }
                            disable_values = {
                                "electromagnet_off",
                                "off",
                                "0",
                                "false",
                                "deactivate",
                            }
                            if normalized in enable_values | disable_values:
                                handled = True
                                enabled = normalized in enable_values
                                ok = node.set_electromagnet(enabled)
                                await _send_event(
                                    websocket,
                                    code="HUB_DIGITAL_OUTPUT_OK"
                                    if ok
                                    else "HUB_DIGITAL_OUTPUT_FAILED",
                                    severity="info" if ok else "warning",
                                    message=(
                                        f"electromagnet={'on' if enabled else 'off'}"
                                    ),
                                )
                        if not handled:
                            await _send_event(
                                websocket,
                                code="UI_BUTTON_IGNORED",
                                severity="warning",
                                message=f"unsupported ui_button topic={button.topic}",
                            )
                        continue

                    await _send_event(
                        websocket,
                        code="CMD_INVALID_TYPE",
                        severity="warning",
                        message=f"unsupported type={msg_type}",
                    )
                except ValidationError as exc:
                    node.get_logger().warning(f"WS cmd invalid: {exc}")
                    await _send_event(
                        websocket,
                        code="CMD_INVALID",
                        severity="warning",
                        message=str(exc),
                    )
        except Exception:
            pass
        finally:
            state_task.cancel()
            node.set_connected(False)
            node.get_logger().info("WS client disconnected")
            if WebSocketState is not None and websocket.client_state == WebSocketState.CONNECTED:
                try:
                    await _send_event(
                        websocket,
                        "WS_DISCONNECTED",
                        "info",
                        "WS disconnected",
                    )
                except Exception:
                    pass

    node.get_logger().info(f"WebSocket listening on ws://{host}:{port}{ws_path}")
    uvicorn.run(app, host=host, port=port, log_level="info")
