#!/usr/bin/env bash
# Micro XRCE-DDS Agent: bridges both PX4 instances to ROS 2 (/uav01/fmu/..., /uav02/fmu/...).
# The agent prints nothing while it runs.  It is connected when each PX4 tab shows
# "INFO  [uxrce_dds_client] successfully created rt/uavXX/fmu/out/..." lines.
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
mkdir -p "$TINS8_LOG_DIR/pids"; echo $$ > "$TINS8_LOG_DIR/pids/xrce_agent.pid"
[ -n "$TINS8_XRCE_AGENT" ] || { echo "MicroXRCEAgent not found - run scripts/setup_deps.sh"; exit 1; }
echo "Micro XRCE-DDS Agent listening on UDP port $TINS8_XRCE_PORT (quiet while running)."
echo "Connected when the PX4 panes print: [uxrce_dds_client] successfully created rt/uavXX/fmu/..."
exec "$TINS8_XRCE_AGENT" udp4 -p "$TINS8_XRCE_PORT"
