#!/usr/bin/env bash
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
mkdir -p "$TINS8_LOG_DIR/pids"; echo $$ > "$TINS8_LOG_DIR/pids/rviz.pid"
exec ros2 launch tins8_bringup rviz.launch.py
