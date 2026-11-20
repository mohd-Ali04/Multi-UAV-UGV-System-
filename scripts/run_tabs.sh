#!/usr/bin/env bash
# Tabbed launcher: one named tab (gnome-terminal) or window (tmux) per component, in startup order.
#   scripts/run_tabs.sh            -> gnome-terminal tabs (default when available)
#   scripts/run_tabs.sh tmux       -> tmux session "tins8" with named windows
# Order and readiness:  Gazebo (clock ticking) -> DDS agent -> PX4 uav01, uav02 ("Ready for takeoff")
#   -> ROS nodes (agents report connected) -> RViz -> GUI -> teleop and operator controls.
set -e
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/env.sh"
mode="${1:-auto}"
tabs=(
  "Gazebo|$here/start_gazebo.sh fieldlab|0"
  "DDS agent|$here/start_agent.sh|2"
  "PX4 uav01|$here/start_px4.sh uav01 fieldlab|6"
  "PX4 uav02|$here/start_px4.sh uav02 fieldlab|2"
  "ROS nodes|$here/start_nodes.sh|2"
  "RViz|$here/start_rviz.sh|4"
  "GUI|$here/start_gui.sh|1"
  "Teleop UGV|$here/start_teleop.sh|0"
  "Controls|$here/controls.sh|0"
)
if [ "$mode" = auto ]; then
  if command -v gnome-terminal >/dev/null && [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then mode=gnome
  elif command -v tmux >/dev/null; then mode=tmux
  else echo "Need gnome-terminal (desktop) or tmux. Or use: ros2 launch tins8_bringup sim_all.launch.py"; exit 1; fi
fi
if [ "$mode" = tmux ]; then
  tmux has-session -t tins8 2>/dev/null && { echo "tmux session tins8 exists; attach with: tmux attach -t tins8"; exit 1; }
  first=1
  for t in "${tabs[@]}"; do
    IFS='|' read -r title cmd delay <<<"$t"; sleep "$delay"
    if [ $first = 1 ]; then tmux new-session -d -s tins8 -n "$title" "bash -c '$cmd; exec bash'"; first=0
    else tmux new-window -t tins8 -n "$title" "bash -c '$cmd; exec bash'"; fi
  done
  echo "started; attach with: tmux attach -t tins8   (stop everything: scripts/stop_all.sh)"
  exit 0
fi
# gnome-terminal: one window, one tab per component, opened with the startup delays above
first=1
for t in "${tabs[@]}"; do
  IFS='|' read -r title cmd delay <<<"$t"; sleep "$delay"
  if [ $first = 1 ]; then
    gnome-terminal --window --title="tins8: $title" -- bash -c "printf '\033]0;tins8: $title\007'; $cmd; exec bash"; first=0; sleep 1
  else
    gnome-terminal --tab --title="tins8: $title" -- bash -c "printf '\033]0;tins8: $title\007'; $cmd; exec bash"
  fi
done
echo "started $((${#tabs[@]})) tabs. Stop everything with: scripts/stop_all.sh"
