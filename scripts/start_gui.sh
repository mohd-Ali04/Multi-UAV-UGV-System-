#!/usr/bin/env bash
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
mkdir -p "$TINS8_LOG_DIR/pids"; echo $$ > "$TINS8_LOG_DIR/pids/gui.pid"
exec ros2 run tins8_mission tins8_gui --ros-args -p use_sim_time:=true
