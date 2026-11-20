#!/usr/bin/env bash
# Keyboard teleoperation of the UGV: i/, forward/back, j/l turn, k stop. Publishes /ugv/cmd_vel.
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
mkdir -p "$TINS8_LOG_DIR/pids"; echo $$ > "$TINS8_LOG_DIR/pids/teleop.pid"
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/ugv/cmd_vel -p speed:=0.5 -p turn:=0.5
