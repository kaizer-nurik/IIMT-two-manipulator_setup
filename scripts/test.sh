#!/usr/bin/env bash
set -euo pipefail
# Tests may construct ROS nodes with fake SDKs; isolate them from robot deployments.
export ROS_DOMAIN_ID="${IIMT_TEST_DOMAIN_ID:-218}"
export ROS_LOCALHOST_ONLY=1
cd /opt/iimt/ros2_ws
colcon test --executor sequential --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
python /opt/iimt/scripts/validate_project.py
