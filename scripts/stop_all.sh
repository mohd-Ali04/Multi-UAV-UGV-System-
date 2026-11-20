#!/usr/bin/env bash
# Clean shutdown of every component this project started (and nothing else on the machine).
# Uses the PID files written by the start scripts; Gazebo/PX4/agent processes that belong to
# other projects are left alone.
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
piddir="$TINS8_LOG_DIR/pids"
stop_tree() {   # SIGINT, then SIGTERM, then SIGKILL a process and its children
  local pid=$1 kids
  kids=$(pgrep -P "$pid" 2>/dev/null || true)
  for k in $kids; do stop_tree "$k"; done
  kill -INT "$pid" 2>/dev/null || return 0
  for _ in 1 2 3 4 5 6 7 8 9 10; do kill -0 "$pid" 2>/dev/null || return 0; sleep 0.3; done
  kill -TERM "$pid" 2>/dev/null; sleep 1; kill -KILL "$pid" 2>/dev/null || true
}
order="gui rviz teleop nodes px4_uav02 px4_uav01 xrce_agent gazebo sim_all"
for name in $order; do
  f="$piddir/$name.pid"
  [ -f "$f" ] || continue
  pid=$(cat "$f")
  if kill -0 "$pid" 2>/dev/null; then echo "stopping $name (pid $pid)"; stop_tree "$pid"; fi
  rm -f "$f"
done
# Leftover Gazebo server (this workspace's world files) and GUI (this workspace's gui.config)
for pid in $(pgrep -f "^gz sim .*tins8_gazebo/(worlds/|config/gui.config)" || true); do echo "stopping gz server $pid"; stop_tree "$pid"; done
for pid in $(pgrep -x px4 || true); do
  case "$(readlink -f /proc/$pid/cwd 2>/dev/null)" in "$TINS8_LOG_DIR"/px4/*) echo "stopping px4 $pid"; stop_tree "$pid";; esac
done
echo "tins8 simulation stopped."
