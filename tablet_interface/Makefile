SHELL := /bin/bash

.PHONY: run-node
run-node: ## Run tablet_interface node with uv
	@source /opt/ros/humble/setup.bash && \
	source /home/susana/workspace/extender_workspace/install/extender_msgs/share/extender_msgs/package.bash && \
	uv run --with numpy --with pydantic --with fastapi --with uvicorn python -m tablet_interface.main --ros-args --params-file config/tablet_interface_parameters_explorer.yaml

.PHONY: kill-port
kill-port: ## Kill any process listening on port 8765
	@pid=$$(ss -lntp | awk '/:8765/ {print $$NF}' | sed -n 's/.*pid=\([0-9]\+\).*/\1/p' | head -n1); \
	if [ -n "$$pid" ]; then echo "Killing PID $$pid on :8765"; kill $$pid; else echo "No listener on :8765"; fi

.PHONY: restart-node
restart-node: ## Kill existing listener on 8765 then run node
	@$(MAKE) -C . kill-port
	@$(MAKE) -C . run-node

.PHONY: run-ws-client
run-ws-client: ## Run WS client test (50 Hz)
	@uv run --with websockets python scripts/ws_client_test.py --host 127.0.0.1 --port 8765 --path /ws/control

.PHONY: test
test: ## Run unit tests
	@uv run pytest -q test/test_ws_models.py test/test_safety_gate.py test/test_teleop_mapping.py

.PHONY: help
help:
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help
