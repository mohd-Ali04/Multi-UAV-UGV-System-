#!/usr/bin/env bash
# Source me:  source scripts/env.sh
# Sets up ROS 2 Jazzy, this workspace, Gazebo resource paths and the PX4 dependency paths.
# Override TINS8_DEPS if your PX4 / Micro XRCE-DDS Agent / px4_msgs live elsewhere.
_tins8_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export TINS8_WS="${_tins8_ws}"
# Optional machine-specific settings (not in git), e.g.  TINS8_DEPS=/path/to/px4_deps
[ -f "$TINS8_WS/scripts/local.env" ] && source "$TINS8_WS/scripts/local.env"
export TINS8_DEPS="${TINS8_DEPS:-$HOME/tins8_deps}"
export TINS8_PX4_DIR="${TINS8_PX4_DIR:-$TINS8_DEPS/PX4-Autopilot}"
export TINS8_LOG_DIR="${TINS8_LOG_DIR:-$TINS8_WS/log/runs}"
source /opt/ros/jazzy/setup.bash
if [ -f "$TINS8_DEPS/px4_msgs_ws/install/setup.bash" ]; then
  source "$TINS8_DEPS/px4_msgs_ws/install/setup.bash"
elif [ -f "$TINS8_DEPS/msgs-install/setup.bash" ]; then
  source "$TINS8_DEPS/msgs-install/setup.bash"
fi
if [ -f "$TINS8_WS/install/setup.bash" ]; then
  source "$TINS8_WS/install/setup.bash"
  _desc_share="$TINS8_WS/install/tins_description/share"
  case ":${GZ_SIM_RESOURCE_PATH}:" in *":${_desc_share}:"*) ;; *) export GZ_SIM_RESOURCE_PATH="${_desc_share}:${GZ_SIM_RESOURCE_PATH}";; esac
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-51}"
# Keep DDS discovery on this machine (ROS_LOCALHOST_ONLY is deprecated in Jazzy).
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
unset ROS_LOCALHOST_ONLY
if [ -x "$TINS8_DEPS/agent-build/MicroXRCEAgent" ]; then
  export TINS8_XRCE_AGENT="$TINS8_DEPS/agent-build/MicroXRCEAgent"
else
  export TINS8_XRCE_AGENT="$(command -v MicroXRCEAgent || true)"
fi
mkdir -p "$TINS8_LOG_DIR"
unset _tins8_ws _desc_share
# Keep this project's Gazebo transport traffic separate from any other Gazebo on the machine.
export GZ_PARTITION="${GZ_PARTITION:-tins8}"
export GZ_IP="${GZ_IP:-127.0.0.1}"          # Gazebo transport on the loopback interface only
export TINS8_XRCE_PORT="${TINS8_XRCE_PORT:-8888}"
export TINS8_PX4_INSTANCE_BASE="${TINS8_PX4_INSTANCE_BASE:-0}"
# Put tins8ctl and tins8_gui on PATH (they are also available via 'ros2 run tins8_mission ...').
if [ -d "$TINS8_WS/install/tins8_mission/lib/tins8_mission" ]; then
  case ":$PATH:" in *":$TINS8_WS/install/tins8_mission/lib/tins8_mission:"*) ;; *) export PATH="$TINS8_WS/install/tins8_mission/lib/tins8_mission:$PATH";; esac
fi
