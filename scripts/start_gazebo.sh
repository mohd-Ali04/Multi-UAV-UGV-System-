#!/usr/bin/env bash
# Gazebo server + GUI + ROS<->Gazebo bridge.   scripts/start_gazebo.sh [fieldlab|minimal] [gui:=true|false]
# Ready when 'ros2 topic echo /clock --once' returns and the GUI shows the UGV with eight aircraft.
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
mkdir -p "$TINS8_LOG_DIR/pids"; echo $$ > "$TINS8_LOG_DIR/pids/gazebo.pid"
world="${1:-fieldlab}"; shift || true
exec ros2 launch tins8_bringup gazebo.launch.py world:="$world" "$@"
