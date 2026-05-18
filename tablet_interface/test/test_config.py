from __future__ import annotations

from types import SimpleNamespace

from tablet_interface.config import (
    PARAMETER_DEFAULTS,
    declare_tablet_interface_parameters,
    load_tablet_interface_config,
)


class FakeNode:
    def __init__(self) -> None:
        self.declared_parameters: dict[str, object] = {}

    def declare_parameter(self, name: str, default: object) -> None:
        self.declared_parameters[name] = default

    def get_parameter(self, name: str) -> SimpleNamespace:
        return SimpleNamespace(value=self.declared_parameters[name])


def test_declare_tablet_interface_parameters_uses_default_values() -> None:
    node = FakeNode()

    declare_tablet_interface_parameters(node)

    assert node.declared_parameters == PARAMETER_DEFAULTS


def test_load_tablet_interface_config_reads_all_declared_parameters() -> None:
    node = FakeNode()
    declare_tablet_interface_parameters(node)
    node.declared_parameters.update(
        {
            "teleop_cmd_topic": "/custom/teleop_cmd",
            "publish_rate_hz": 42.0,
            "swap_xy": True,
            "bind_port": 9000,
            "sandbox_toggle_output_message_type": "std_msgs/msg/Bool",
            "config_storage_db_path": "/tmp/extender/storage.sqlite3",
        }
    )

    config = load_tablet_interface_config(node)

    assert config.teleop_cmd_topic == "/custom/teleop_cmd"
    assert config.publish_rate_hz == 42.0
    assert config.swap_xy is True
    assert config.bind_port == 9000
    assert config.sandbox_toggle_output_message_type == "std_msgs/msg/Bool"
    assert config.config_storage_db_path == "/tmp/extender/storage.sqlite3"
