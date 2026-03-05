import pytest
from pydantic import ValidationError

from tablet_interface.ws_models import (
    CmdMessage,
    EventMessage,
    GripperCmdMessage,
    PetanqueConfigMessage,
    StateCmdMessage,
    StateMessage,
    UiButtonMessage,
)


def test_cmd_message_valid() -> None:
    payload = {
        "type": "teleop_cmd",
        "seq": 12,
        "mode": 2,
        "linear": {"x": 0.1, "y": 0.2, "z": 0.0},
        "angular": {"x": -0.1, "y": 0.0, "z": 0.3},
    }
    msg = CmdMessage.model_validate(payload)
    assert msg.mode == 2
    assert msg.linear.x == 0.1


@pytest.mark.parametrize(
    "payload",
    [
        {  # invalid mode
            "type": "teleop_cmd",
            "seq": 1,
            "mode": 4,
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        },
        {  # missing linear field
            "type": "teleop_cmd",
            "seq": 1,
            "mode": 0,
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        },
        {  # wrong types
            "type": "teleop_cmd",
            "seq": "1",
            "mode": 0,
            "linear": {"x": {}, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        },
        {  # out-of-range vector component
            "type": "teleop_cmd",
            "seq": 1,
            "mode": 0,
            "linear": {"x": 21.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        },
    ],
)
def test_cmd_message_invalid(payload: dict) -> None:
    with pytest.raises(ValidationError):
        CmdMessage.model_validate(payload)


def test_state_cmd_valid() -> None:
    payload = {
        "type": "state_cmd",
        "command": "go_to_start",
    }
    msg = StateCmdMessage.model_validate(payload)
    assert msg.command == "go_to_start"


def test_state_cmd_pick_up_valid() -> None:
    payload = {
        "type": "state_cmd",
        "command": "pick_up",
    }
    msg = StateCmdMessage.model_validate(payload)
    assert msg.command == "pick_up"


def test_state_cmd_invalid() -> None:
    with pytest.raises(ValidationError):
        StateCmdMessage.model_validate(
            {
                "type": "state_cmd",
                "command": "go_to_end",
            }
        )


def test_gripper_cmd_valid() -> None:
    payload = {
        "type": "gripper_cmd",
        "action": "open",
        "speed": 0.8,
        "force": 0.4,
    }
    msg = GripperCmdMessage.model_validate(payload)
    assert msg.action == "open"
    assert msg.speed == pytest.approx(0.8)
    assert msg.force == pytest.approx(0.4)


@pytest.mark.parametrize(
    "payload",
    [
        {
            "type": "gripper_cmd",
            "action": "toggle",
        },
        {
            "type": "gripper_cmd",
            "action": "close",
            "speed": 1.2,
        },
        {
            "type": "gripper_cmd",
            "action": "open",
            "force": -0.1,
        },
    ],
)
def test_gripper_cmd_invalid(payload: dict) -> None:
    with pytest.raises(ValidationError):
        GripperCmdMessage.model_validate(payload)


def test_petanque_cfg_valid() -> None:
    payload = {
        "type": "petanque_cfg",
        "total_duration": 1.25,
    }
    msg = PetanqueConfigMessage.model_validate(payload)
    assert msg.total_duration == pytest.approx(1.25)
    assert msg.angle_between_start_and_finish is None


def test_petanque_cfg_angle_valid() -> None:
    payload = {
        "type": "petanque_cfg",
        "angle_between_start_and_finish": 0.7,
    }
    msg = PetanqueConfigMessage.model_validate(payload)
    assert msg.total_duration is None
    assert msg.angle_between_start_and_finish == pytest.approx(0.7)


def test_petanque_cfg_both_fields_valid() -> None:
    payload = {
        "type": "petanque_cfg",
        "total_duration": 1.2,
        "angle_between_start_and_finish": 0.4,
    }
    msg = PetanqueConfigMessage.model_validate(payload)
    assert msg.total_duration == pytest.approx(1.2)
    assert msg.angle_between_start_and_finish == pytest.approx(0.4)


def test_petanque_cfg_invalid() -> None:
    with pytest.raises(ValidationError):
        PetanqueConfigMessage.model_validate(
            {
                "type": "petanque_cfg",
                "total_duration": 0,
            }
        )

    with pytest.raises(ValidationError):
        PetanqueConfigMessage.model_validate(
            {
                "type": "petanque_cfg",
            }
        )


def test_ui_button_valid() -> None:
    payload = {
        "type": "ui_button",
        "topic": "/petanque_state_machine/change_state",
        "payload": "throw",
        "widget_id": "pet-throw",
    }
    msg = UiButtonMessage.model_validate(payload)
    assert msg.payload == "throw"


def test_ui_button_invalid() -> None:
    with pytest.raises(ValidationError):
        UiButtonMessage.model_validate(
            {
                "type": "ui_button",
                "topic": "",
                "payload": "throw",
            }
        )


def test_state_message_valid() -> None:
    payload = {
        "type": "state",
        "connected": True,
        "cmd_age_ms": 20,
        "watchdog_timeout_ms": 200,
        "last_seq": 10,
        "publishing_rate_hz": 30.0,
        "current_mode": 1,
        "gripper_state": "open",
    }
    msg = StateMessage.model_validate(payload)
    assert msg.current_mode == 1
    assert msg.gripper_state == "open"


@pytest.mark.parametrize(
    "payload",
    [
        {  # invalid mode
            "type": "state",
            "connected": True,
            "cmd_age_ms": 20,
            "watchdog_timeout_ms": 200,
            "last_seq": 10,
            "publishing_rate_hz": 30.0,
            "current_mode": 9,
        },
        {  # invalid publishing_rate_hz type
            "type": "state",
            "connected": True,
            "cmd_age_ms": 20,
            "watchdog_timeout_ms": 200,
            "last_seq": 10,
            "publishing_rate_hz": "30.0",
            "current_mode": 1,
        },
    ],
)
def test_state_message_invalid(payload: dict) -> None:
    with pytest.raises(ValidationError):
        StateMessage.model_validate(payload)


def test_event_message_valid() -> None:
    payload = {
        "type": "event",
        "severity": "warning",
        "code": "W001",
        "message": "Low battery",
    }
    msg = EventMessage.model_validate(payload)
    assert msg.severity == "warning"


@pytest.mark.parametrize(
    "payload",
    [
        {"type": "event", "severity": "debug", "code": "W001", "message": "x"},
        {"type": "event", "severity": "info", "code": "", "message": "x"},
        {"type": "event", "severity": "info", "code": "W001", "message": ""},
    ],
)
def test_event_message_invalid(payload: dict) -> None:
    with pytest.raises(ValidationError):
        EventMessage.model_validate(payload)
