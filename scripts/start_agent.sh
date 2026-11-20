#!/usr/bin/env bash
# Micro XRCE-DDS Agent: bridges both PX4 instances to ROS 2 (/uav01/fmu/..., /uav02/fmu/...).
# Ready when it prints "session established" once per PX4 instance.
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
mkdir -p "$TINS8_LOG_DIR/pids"; echo $$ > "$TINS8_LOG_DIR/pids/xrce_agent.pid"
[ -n "$TINS8_XRCE_AGENT" ] || { echo "MicroXRCEAgent not found - run scripts/setup_deps.sh"; exit 1; }
exec "$TINS8_XRCE_AGENT" udp4 -p "$TINS8_XRCE_PORT"
