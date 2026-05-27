SHELL := /bin/bash

.PHONY: sync
sync: ## Install/update workspace uv dependencies
	@uv sync

.PHONY: run-node
run-node: ## Run tablet_interface node with workspace uv
	@source /opt/ros/humble/setup.bash && \
	source /home/susana/workspace/extender_workspace/install/setup.bash && \
	uv run python -m tablet_interface.main --ros-args --params-file config/tablet_interface_parameters_explorer.yaml

.PHONY: kill-port
kill-port: ## Kill any process listening on port 8765
	@pid=$$(ss -lntp | awk '/:8765/ {print $$NF}' | sed -n 's/.*pid=\([0-9]\+\).*/\1/p' | head -n1); \
	if [ -n "$$pid" ]; then echo "Killing PID $$pid on :8765"; kill $$pid; else echo "No listener on :8765"; fi

.PHONY: restart-node
restart-node: ## Kill existing listener on 8765 then run node
	@$(MAKE) -C . kill-port
	@$(MAKE) -C . run-node

.PHONY: run-ws-client
run-ws-client: ## Run WS client test with workspace uv
	@uv run python scripts/ws_client_test.py --host 127.0.0.1 --port 8765 --path /ws/control

.PHONY: test
test: ## Run unit tests with workspace uv
	@PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q test/test_ws_models.py test/test_config.py test/test_safety_gate.py test/test_teleop_mapping.py test/test_runtime_state.py test/test_petanque_bridge.py test/test_measure_bridge.py test/test_measure_codec.py test/test_sandbox_bridge.py test/test_actuator_bridge.py test/test_camera_bridge.py test/test_generic_publishers.py test/test_typed_message_publishers.py test/test_teleop_command_service.py test/test_ui_topic_publisher.py test/test_config_storage.py test/test_storage_api.py test/test_main_module.py test/test_tablet_interface_node_module.py test/test_ws_server.py test/test_ws_handlers.py

.PHONY: help
help:
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help
