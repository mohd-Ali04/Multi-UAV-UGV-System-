#!/usr/bin/env bash
# All project ROS 2 nodes (agents, carousel, fleet, YOLOX, tracker, pursuit, mission, scenario, evaluator).
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
mkdir -p "$TINS8_LOG_DIR/pids"; echo $$ > "$TINS8_LOG_DIR/pids/nodes.pid"
exec ros2 launch tins8_bringup nodes.launch.py "$@"
