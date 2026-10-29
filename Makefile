SHELL := /bin/bash
COMPOSE ?= docker compose
SERVICE ?= notebook
MODE ?= single
# Command-line assignments (make ... LAUNCH_ARGS='...') override .env.
ifdef LAUNCH_ARGS
export LAUNCH_ARGS
endif
.PHONY: help build up-single up-dual notebook shell test down logs show-args config
help:
	@printf '%s\n' 'build: build ROS + notebook image' \
	  'up-single / up-dual: start exactly one stack (defaults: mock, motion disabled)' \
	  'notebook: start Jupyter on http://127.0.0.1:8888 (token in logs)' \
	  'shell: interactive ROS/Python shell; test: offline tests, no robot connection' \
	  'smoke: single/dual mock graph check, no motion goals' \
	  'deps / notebook-smoke: dependency and actual notebook kernel checks' \
	  'smoke-execution: single/dual mock execution and RRTConnect (no hardware)' \
	  'verify: dependencies, tests, notebook kernel, mock graph and execution' \
	  'logs SERVICE=notebook|single|dual; down: stop this project' \
	  'show-args MODE=single|dual; config: rendered Compose configuration' \
	  "Override launch arguments: make up-dual LAUNCH_ARGS='workspace_margin:=0.6'" \
	  'For mock movement explicitly use LAUNCH_ARGS="enable_motion:=true".'
build:
	$(COMPOSE) build tools
up-single:
	$(COMPOSE) up -d single
up-dual:
	$(COMPOSE) up -d dual
notebook:
	$(COMPOSE) up -d notebook
shell:
	$(COMPOSE) run --rm tools bash
test:
	$(COMPOSE) run --rm -T tools /opt/iimt/scripts/test.sh
down:
	$(COMPOSE) --profile '*' down --remove-orphans
logs:
	$(COMPOSE) logs -f $(SERVICE)
show-args:
	$(COMPOSE) run --rm -T tools python /opt/iimt/docker/launch.py $(MODE) --show-args
config:
	$(COMPOSE) --profile '*' config

.PHONY: smoke
smoke:
	$(COMPOSE) run --rm -T tools python /opt/iimt/scripts/mock_smoke.py

.PHONY: deps notebook-smoke smoke-execution verify
deps:
	$(COMPOSE) run --rm -T tools python /opt/iimt/scripts/check_dependencies.py
notebook-smoke:
	$(COMPOSE) run --rm -T tools python /opt/iimt/scripts/notebook_smoke.py
smoke-execution:
	$(COMPOSE) run --rm -T tools python /opt/iimt/scripts/mock_execution.py
# Sequential even under make -j: legacy timing tests must not compete with launches.
verify:
	$(MAKE) deps
	$(MAKE) test
	$(MAKE) notebook-smoke
	$(MAKE) smoke
	$(MAKE) smoke-execution
