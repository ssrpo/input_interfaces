# Tablet Interface

ROS 2 input interface that publishes `/teleop_cmd` from a WebSocket client.

## Run the node

```bash
ros2 run tablet_interface tablet_interface_node --ros-args --params-file config/tablet_interface_parameters_explorer.yaml
```

## Makefile (uv run)

Use the Makefile to run the node and tools with uv:

```bash
make -C . run-node
make -C . run-ws-client
make -C . test
```

## Developer WS client test

The test client streams `cmd` messages at 50 Hz and prints `state` messages from the server.

```bash
python3 scripts/ws_client_test.py --host 127.0.0.1 --port 8765 --path /ws/control
```

### WebSocket command format

UI sends normalized values in [-1, 1]. Backend applies axis mapping and scaling.

```json
{
	"type": "teleop_cmd",
	"seq": 42,
	"mode": 0,
	"linear": { "x": 0.2, "y": -0.1, "z": 0.0 },
	"angular": { "x": 0.0, "y": 0.0, "z": 0.0 }
}
```

For pétanque integration, backend also supports:

```json
{ "type": "state_cmd", "command": "teleop" }
```

Allowed `command` values: `teleop`, `activate_throw`, `go_to_start`, `throw`, `pick_up`, `stop`.
These are published as `std_msgs/String` on `/petanque_state_machine/change_state`.

```json
{ "type": "petanque_cfg", "total_duration": 1.0 }
```

This updates `/petanque_throw` parameter `total_duration` through `/petanque_throw/set_parameters`.

`ui_button` messages are also accepted for compatibility. If `topic` matches
`/petanque_state_machine/change_state`, backend forwards `payload` to the same bridge.

### Mapping and scaling

The backend can remap and invert tablet axes before publishing `/teleop_cmd`.
This is configured through the ROS params file:

- `linear_axes` / `linear_signs`
- `angular_axes` / `angular_signs`
- `linear_scale`, `angular_scale`
- `swap_xy`

By default, tablet mapping is identity (`x->x`, `y->y`, `z->z`) and can be tuned per robot.
Robot profiles are provided in:
- `config/tablet_interface_parameters_explorer.yaml`
- `config/tablet_interface_parameters_kinova.yaml`

**Expected behavior**
- While the script runs, `/teleop_cmd` should be non-zero.
- `/teleop_cmd` follows the latest UI command and mode.
