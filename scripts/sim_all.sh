#!/usr/bin/env bash
# One-command start of the full simulation in this terminal (logs of PX4/agent go to ~/.ros/log).
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
mkdir -p "$TINS8_LOG_DIR/pids"; echo $$ > "$TINS8_LOG_DIR/pids/sim_all.pid"
exec ros2 launch tins8_bringup sim_all.launch.py "$@"
