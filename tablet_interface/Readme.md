# Tablet Interface

`tablet_interface` is the ROS 2 websocket backend used by
[`extender_ui`](https://github.com/ISIR-EXTENDER/extender_ui). It receives
generic UI messages from the tablet, validates them, republishes them to ROS 2,
and streams robot state, topic snapshots, and backend events back to the UI.

The current production-style frontend app for new work is **Sandbox V0.0**.
It now drives the robot through
[`cartesian_manager`](https://github.com/ISIR-EXTENDER/cartesian_manager), which
routes Cartesian commands to `qontrol_controller`. Petanque support and the
legacy `sandbox_controller` path are preserved for compatibility, but new
controller/UI development should target the manager stack.

<p align="center">
  <img alt="ROS 2" src="https://img.shields.io/badge/ROS%202-Humble-22314e?style=for-the-badge" />
  <img alt="Python" src="https://img.shields.io/badge/Python-3.10-3776ab?style=for-the-badge&logo=python&logoColor=white" />
  <img alt="WebSocket" src="https://img.shields.io/badge/WebSocket-backend-4b5563?style=for-the-badge" />
</p>

<p align="center">
  <a href="#current-state">Current State</a> ·
  <a href="#sandbox-v00">Sandbox V0.0</a> ·
  <a href="#architecture">Architecture</a> ·
  <a href="#websocket-contract">WebSocket Contract</a> ·
  <a href="#ros-contract">ROS Contract</a> ·
  <a href="#configuration">Configuration</a> ·
  <a href="#development">Development</a> ·
  <a href="#bloom">Bloom</a>
</p>

## Current State

- WebSocket server for `extender_ui` at `/ws/control`.
- Teleoperation bridge from frontend joystick/slider/mode state to
  `geometry_msgs/msg/TwistStamped` Cartesian commands for `cartesian_manager`.
- Structured `std_msgs/msg/String` mode requests on `/mode_request`, validated
  against the manager grammar before publication.
- Legacy `extender_msgs/msg/TeleopCommand` output kept behind the
  `command_backend` parameter as a rollback path.
- Generic typed ROS publishers for frontend widgets.
- Topic monitor bridge for small diagnostic ROS messages.
- Browser-captured camera frame bridge to ROS compressed image topics.
- Robot feedback bridge for end-effector pose, end-effector velocity, and joint
  state, reading either `sensor_msgs/msg/JointState` or the legacy
  `std_msgs/msg/Float64MultiArray`.
- Legacy Petanque, measure, gripper, and actuator bridges kept for
  compatibility.
- Runtime state and backend events streamed back to the UI.

## Sandbox V0.0

Sandbox V0.0 is the recommended app/backend path for new Extender development.
The intended flow is:

```text
extender_ui Sandbox V0.0 screen
  -> websocket message
  -> tablet_interface
  -> ROS topic
  -> cartesian_manager -> qontrol_controller, or a perception node
  -> ROS feedback
  -> tablet_interface
  -> UI state, topic monitor, or event
```

Current Sandbox V0.0 screens on the frontend:

| Frontend screen | Backend support |
| --- | --- |
| `sandbox_control` | `teleop_cmd`, sandbox feedback state, typed widget publishing. |
| `sandbox_teleop_config` | Teleop mapping and reusable widget configuration. |
| `control_panel` | Teleop, webcam/camera frame flow, gripper, visual-servoing controls, and compact telemetry. |
| `snake_control` | Teleop plus a `geometric/snake` mode request held on `/mode_request`. |
| `visual_servoing` | Typed ON/OFF and save-tag commands. |
| `visual_servoing_monitor` | Topic snapshots for AprilTag and servo telemetry. |

Use [`cartesian_manager`](https://github.com/ISIR-EXTENDER/cartesian_manager)
plus `qontrol_controller` for new robot control experiments.
`sandbox_controller` and the Petanque bridges remain available as rollback
paths, but they should not be the default for new features.

## Architecture

`tablet_interface` is intentionally split into a generic websocket/ROS core and
domain bridges.

```text
browser / tablet UI
  -> ws_server.py
  -> ws_handlers.py
  -> bridge or publisher module
  -> ROS 2 publisher/subscriber/service
  -> runtime_state.py / websocket response
  -> browser / tablet UI
```

Main modules:

| Module | Responsibility |
| --- | --- |
| `main.py` | Package entry point. |
| `ros_teleop_publisher.py` | ROS node orchestration, timers, and bridge wiring. |
| `ws_server.py` | WebSocket server lifecycle. |
| `ws_handlers.py` | Incoming message dispatch and outgoing UI responses. |
| `ws_models.py` | Pydantic websocket message validation models. |
| `runtime_state.py` | Shared backend state streamed to the UI. |
| `safety_gate.py` | Teleop watchdog and safety filtering. |
| `teleop_mapping.py` | UI vector mapping to ROS `TeleopCommand`. |
| `generic_publishers.py` | Generic string/scalar/bool ROS publishers. |
| `typed_message_publishers.py` | Typed ROS message publishing for configurable UI widgets. |
| `topic_monitor_bridge.py` | Generic ROS topic subscriptions and `topic_snapshot` output. |
| `sandbox_bridge.py` | Sandbox controller feedback subscriptions. |
| `camera_bridge.py` | Browser `camera_frame` to ROS compressed image bridge. |
| `actuator_bridge.py` | Gripper and actuator-related output bridges. |
| `petanque_bridge.py` | Legacy Petanque state-machine compatibility. |
| `measure_bridge.py` | Legacy Petanque measure compatibility. |

## WebSocket Contract

Default endpoint:

```text
ws://localhost:8765/ws/control
```

Incoming messages from the UI:

| Message type | Purpose |
| --- | --- |
| `teleop_cmd` | Main normalized teleoperation command. |
| `mode_request` | Structured `cartesian_manager` mode request. |
| `ui_button` | Generic string command from button widgets. |
| `ui_scalar` | Generic numeric command from slider/gain widgets. |
| `ui_bool` | Generic boolean command. |
| `ui_typed` | Typed ROS message publication from configurable widgets. |
| `camera_frame` | Browser-captured image frame as a data URL. |
| `topic_subscribe` | Runtime request for ROS topic snapshots. |
| `gripper_cmd` | Gripper open/close command. |
| `state_cmd` | Legacy Petanque state-machine command. |
| `petanque_cfg` | Legacy Petanque parameter update command. |
| `measure_request` | Legacy measure request with captured image. |
| `measure_refresh` | Legacy measure result refresh request. |

Outgoing messages to the UI:

| Message type | Purpose |
| --- | --- |
| `state` | Backend connectivity, watchdog, mode, gripper, and sandbox feedback. |
| `event` | Backend info/warning/error events, including subscription failures. |
| `topic_snapshot` | Latest compact ROS message snapshot for monitored topics. |
| `measure_result` | Legacy measure result payload. |

### Teleoperation Message

```json
{
  "type": "teleop_cmd",
  "seq": 42,
  "mode": 0,
  "linear": { "x": 0.2, "y": -0.1, "z": 0.0 },
  "angular": { "x": 0.0, "y": 0.0, "z": 0.0 }
}
```

The backend validates finite values, applies mapping/scaling, applies safety
rules, and republishes the command as ROS.

### Mode Request Message

```json
{
  "type": "mode_request",
  "mode": "geometric/snake",
  "widget_id": "snake-hold"
}
```

The backend validates the mode against the `cartesian_manager` grammar and
answers with a `MODE_REQUEST_OK` or `MODE_REQUEST_REJECTED` event.

### Typed UI Message

```json
{
  "type": "ui_typed",
  "topic": "/mode_request",
  "message_type": "std_msgs/msg/String",
  "payload_text": "{data: geometric/snake}",
  "widget_id": "snake-hold"
}
```

This powers widgets such as `ROS Message Toggle` and `Momentary ROS Message`.
Use it when a frontend control should publish a specific ROS message type and
payload.

A `ui_typed` message addressed to `mode_request_topic` is routed through the same
validation as a native `mode_request`, so a widget configured with a bad mode
gets operator feedback instead of failing silently at the manager.

### Topic Subscribe Message

```json
{
  "type": "topic_subscribe",
  "topics": [
    {
      "topic": "/tag_detections",
      "message_type": "extender_msgs/msg/SharedControlGoalArray"
    }
  ]
}
```

When a monitored ROS message arrives, the backend emits:

```json
{
  "type": "topic_snapshot",
  "topic": "/tag_detections",
  "message_type": "extender_msgs/msg/SharedControlGoalArray",
  "updated_at_ms": 123456,
  "revision": 1,
  "data": {},
  "error": null
}
```

Topic monitors are for small diagnostic messages. The backend rejects image and
video topic types such as `sensor_msgs/msg/Image` and
`sensor_msgs/msg/CompressedImage` so large video streams do not pass through the
topic snapshot path.

## ROS Contract

### Teleoperation

The output depends on the `command_backend` parameter.

`command_backend: cartesian_manager` (default):

| ROS topic | Message | Direction |
| --- | --- | --- |
| `/joystick_cartesian_command` | `geometry_msgs/msg/TwistStamped` | backend -> cartesian_manager |
| `/mode_request` | `std_msgs/msg/String` | backend -> cartesian_manager |

`command_backend: teleop_command` (legacy rollback):

| ROS topic | Message | Direction |
| --- | --- | --- |
| `/teleop_cmd` | `extender_msgs/msg/TeleopCommand` | backend -> sandbox_controller |

Only the publisher for the selected backend is created, so a dead `/teleop_cmd`
publisher never appears on the graph while running against the manager.

> **Frames matter.** `cartesian_manager` accepts its configured base,
> end-effector, and hybrid command frame IDs; an empty `header.frame_id` uses
> `default_input_frame_id`. Linear velocity follows the base-frame convention.
> Angular velocity stamped in the end-effector or hybrid frame is rotated into
> base from live robot context. Commands with unknown frame IDs are ignored.
> `base_link` remains the conservative Explorer default.

The frontend widget `topic` fields such as `/cmd/joystick` or `/cmd/mode` are
UI configuration metadata. They do not change the backend teleop output topic.

### Mode Requests

`cartesian_manager` selects its shapers from a plain string. The backend
validates the grammar before publishing, so an invalid mode is reported to the
operator as a `MODE_REQUEST_REJECTED` event instead of being silently ignored by
the robot.

| Mode request | Effect |
| --- | --- |
| `geometric/both` | No geometric shaping. Neutral state. |
| `geometric/jaco` | Jaco shaper. |
| `geometric/snake` | Snake shaper. |
| `behaviour/passthrough` | No behaviour shaping. |
| `behaviour/joint_target/<name>` | One-shot named joint target, such as `home`. |

Requests are normalized the way the manager normalizes them: lowercased, with
`-` replaced by `_`. `behaviour/joint_target/...` is one-shot, so it is not
recorded as sticky state: the manager returns to passthrough by itself.

B1/B2 are **not** manager modes. They only select which joystick axes drive
translation and rotation, and the tablet resolves them locally before sending a
twist. This matches how `joystick_mapper` treats its own local B1/B2 modes.

The manager **sums all activated inputs** rather than arbitrating between them.
The tablet is one input peer among joystick, head, hand, visual servoing, and
shared control, so a non-zero tablet twist adds to whatever else is active. Send
zeros on release, which the frontend already does through `stopAndZero`.

The target architecture replaces `geometric/jaco` with `translation` and
`orientation` behaviours. When that lands, the tablet's TRANSLATION and ROTATION
modes should become real mode requests instead of local axis masks; the accepted
list lives in `GEOMETRIC_MODES` in `tablet_interface/mode_request.py`.

### Robot Feedback

| Parameter | Default topic | Message | Direction |
| --- | --- | --- | --- |
| `sandbox_ee_pose_topic` | `/ee_pose` | `geometry_msgs/msg/PoseStamped` | qontrol -> backend -> UI |
| `sandbox_velocity_command_topic` | `/ee_velocity` | `geometry_msgs/msg/TwistStamped` | qontrol -> backend -> UI |
| `sandbox_joint_pose_topic` | `/joint_states` | `sensor_msgs/msg/JointState` | robot -> backend -> UI |

Set `sandbox_joint_pose_message_type` to `std_msgs/msg/Float64MultiArray` to read
joint feedback from a legacy `sandbox_controller` stack instead.

### Snake Control

| ROS topic | Message | Direction |
| --- | --- | --- |
| `/activate_snake` | `std_msgs/msg/Bool` | UI -> backend -> controller |

Momentary button contract:

```text
press   -> /activate_snake std_msgs/msg/Bool {data: true}
release -> /activate_snake std_msgs/msg/Bool {data: false}
```

The same joystick velocity is sent in B1 and B2. The frontend changes only the
teleop mode (`B1 -> mode: 0`, `B2 -> mode: 3`); the backend/controller
interprets the mode.

### Visual Servoing

| Topic | Message | Direction |
| --- | --- | --- |
| `/ui/visual_servoing/on` | `std_msgs/msg/Bool` | UI -> ROS |
| `/ui/visual_servoing/save` | `std_msgs/msg/String` | UI -> ROS |
| `/tag_detections` | `extender_msgs/msg/SharedControlGoalArray` | AprilTag detector -> UI / visual servoing |
| `/visual_servoing/velocity_command` | `geometry_msgs/msg/TwistStamped` | visual servoing -> UI |
| `/visual_servoing/error_TAGtoTAGd` | `geometry_msgs/msg/TwistStamped` | visual servoing -> UI |

The UI should display video through stream widgets and monitor only the compact
ROS messages above through `topic_subscribe`.

### Camera Frames

The backend accepts browser-captured `camera_frame` messages and republishes the
image as `sensor_msgs/msg/CompressedImage`.

Legacy/default Petanque parameters:

| Parameter | Default topic |
| --- | --- |
| `measure_request_image_topic` | `/petanque/measure/request_image/compressed` |
| `measure_result_image_topic` | `/petanque/measure/result_image/compressed` |
| `measure_result_vectors_topic` | `/petanque/measure/result_vectors` |

For ROS camera pipelines such as AprilTag detection, prefer direct ROS camera
topics for image transport:

```text
/image_raw
/camera_info
```

## Configuration

Default parameters are declared in `tablet_interface/config.py` and can be
overridden with YAML files in `config/`.

Included profiles:

| File | Purpose |
| --- | --- |
| `config/tablet_interface_parameters.yaml` | Default profile. |
| `config/tablet_interface_parameters_explorer.yaml` | Explorer/sandbox-oriented profile used by `make run-node`. |
| `config/tablet_interface_parameters_kinova.yaml` | Kinova-oriented profile. |

Important parameters:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `command_backend` | `cartesian_manager` | `cartesian_manager` or `teleop_command`. |
| `cartesian_command_topic` | `/joystick_cartesian_command` | Cartesian command topic for `cartesian_manager`. |
| `mode_request_topic` | `/mode_request` | Structured mode request topic. |
| `command_frame_id` | `base_link` | Frame stamped on outgoing commands. Use empty for the manager default, or one of its configured base, end-effector, or hybrid frame IDs. |
| `teleop_cmd_topic` | `/teleop_cmd` | Legacy output topic, only used by the `teleop_command` backend. |
| `publish_rate_hz` | `30.0` in code, profile-specific in YAML | Teleop publish timer rate. |
| `linear_scale` | `0.2` in code, profile-specific in YAML | Linear command scaling. |
| `angular_scale` | `0.5` in code, profile-specific in YAML | Angular command scaling. |
| `default_mode` | `0` | Mode used when client mode is not accepted. |
| `accept_mode_from_client` | `true` | Whether UI can select teleop mode. |
| `state_publish_hz` | `5.0` in code, profile-specific in YAML | Backend state broadcast rate. |
| `bind_host` | `0.0.0.0` | WebSocket bind host. |
| `bind_port` | `8765` | WebSocket bind port. |
| `ws_path` | `/ws/control` | WebSocket path. |
| `topic_snapshot_hz` | `10.0` | Topic monitor snapshot rate. |
| `topic_monitor_specs` | visual-servoing topics | Startup topic monitor subscriptions. |
| `param_call_timeout_sec` | `1.5` | ROS parameter service timeout. |

Example launch with a specific profile:

```bash
ros2 run tablet_interface tablet_interface_node --ros-args \
  --params-file src/input_interfaces/tablet_interface/config/tablet_interface_parameters_explorer.yaml
```

## Development

### Required Extender Packages

For normal development and sandbox testing, build these packages in the Extender
ROS workspace:

| Package / repo | Required for |
| --- | --- |
| [`robot_interfaces/extender_msgs`](https://github.com/ISIR-EXTENDER/robot_interfaces) | `TeleopCommand` and shared Extender messages. |
| [`cartesian_manager`](https://github.com/ISIR-EXTENDER/cartesian_manager) | Routes Cartesian commands and mode requests to `qontrol_controller`. |
| [`qontrol_controller`](https://github.com/ISIR-EXTENDER/qontrol_controller) | QP robot control and `/ee_*` feedback. Branch `topic/isir_manager`. |
| [`sandbox_controller`](https://github.com/ISIR-EXTENDER/sandbox_controller) | Legacy sandbox teleop and feedback loop, rollback only. |
| [`tools/apriltag_detector`](https://github.com/ISIR-EXTENDER/tools/tree/main/apriltag_detector) | AprilTag detections for visual servoing. |
| [`visual_servoing`](https://github.com/ISIR-EXTENDER/visual_servoing) | Visual-servoing controller and telemetry topics. |
| [`extender_ui`](https://github.com/ISIR-EXTENDER/extender_ui) | Frontend runtime and screen builder. |

### Install Workspace Dependencies

From the Extender ROS workspace:

```bash
uv sync
colcon build --symlink-install --packages-select tablet_interface
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Run The Backend

From this package:

```bash
make run-node
```

Equivalent explicit command:

```bash
source /opt/ros/jazzy/setup.bash
source ../../../install/setup.bash
uv run python -m tablet_interface.main --ros-args \
  --params-file config/tablet_interface_parameters_explorer.yaml
```

### Test The WebSocket

With the backend running:

```bash
make run-ws-client
```

The frontend expects:

```text
ws://127.0.0.1:8765/ws/control
```

### Run Tests

```bash
make test
```

The Makefile disables external pytest plugin autoloading so a sourced ROS
environment does not leak unrelated plugins into the package tests.

## Working With The Frontend

Recommended local loop:

1. Start `tablet_interface` with `make run-node`.
2. Start `extender_ui` with `npm run dev`.
3. Open Sandbox V0.0 in the frontend.
4. Use `control_panel`, `snake_control`, `visual_servoing`, or
   `visual_servoing_monitor` depending on the workflow.
5. Check backend `event` messages and UI topic monitors before debugging lower
   layers.

Backend state messages include:

- `connected`
- `cmd_age_ms`
- `watchdog_timeout_ms`
- `last_seq`
- `publishing_rate_hz`
- `current_mode`
- `gripper_state`
- `ee_pose`
- `tcp_speed_mps`
- `joint_positions`

## Compatibility

This backend README is aligned with the current `extender_ui` README update and
the active Extender workspace split.

| Component | Repository | Commit checked | Notes |
| --- | --- | --- | --- |
| Frontend | [`extender_ui`](https://github.com/ISIR-EXTENDER/extender_ui) | `9c3d0db docs: update project readme (#29)` | Documents Sandbox V0.0 and backend contracts. |
| Backend/input interfaces | [`input_interfaces`](https://github.com/ISIR-EXTENDER/input_interfaces) | Current branch | This README update. |
| Controllers | [`controllers`](https://github.com/ISIR-EXTENDER/controllers) | `c6bbebc feat: add snake mode to cartesian_velocity controller (#7)` | Shared robot controllers. |
| Sandbox controller | [`sandbox_controller`](https://github.com/ISIR-EXTENDER/sandbox_controller) | `0411619 fix: use synced joint positions for feedback (#4)` | Reference controller for new backend/UI smoke tests. |
| Robot messages | [`robot_interfaces`](https://github.com/ISIR-EXTENDER/robot_interfaces) | `1543180 Merge pull request #5 from ssrpo/fix/remove-stale-joint-pose-helper` | Provides shared ROS messages. |
| Tools | [`tools`](https://github.com/ISIR-EXTENDER/tools) | `800bed7 Merge pull request #4 from MegMll/topic/add_snake` | Provides `apriltag_detector`. |
| Visual servoing | [`visual_servoing`](https://github.com/ISIR-EXTENDER/visual_servoing) | `bc6a33a first commit` | Robin's current visual-servoing package. |

## Bloom

[`Bloom`](https://github.com/ISIR-EXTENDER/bloom) is the active Extender operator
interface. It talks to `cartesian_manager` through its own FastAPI backend and ROS
adapters, so it does not use this package.

`tablet_interface` is legacy. It stays buildable as the backend for the
`extender_ui` rollback until Bloom's live sessions are accepted:

1. Put new operator workflows in Bloom, not here.
2. Keep the command contract here in step with `cartesian_manager` while the
   rollback is kept (topics, frames, gripper values).
3. Retire this backend once the matching Bloom workflow is accepted on the robot.

## Contributing

- Write README content, comments, PR descriptions, and shared docs in English.
- Keep generic websocket behavior in the core transport/handler layer.
- Put domain-specific ROS behavior in bridge modules.
- Prefer generic messages over new custom websocket message types when possible.
- Preserve legacy Petanque compatibility unless the team explicitly removes it.
- Keep video transport separate from topic monitoring.
- Document any new ROS topic contract in both this README and the frontend
  README.
