#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /opt/iimt/ros2_ws/install/setup.bash
source /opt/iimt/venv/bin/activate
exec "$@"
