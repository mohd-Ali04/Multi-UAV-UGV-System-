#!/usr/bin/env bash
# Tabbed launcher: every component in its own named terminal, in startup order.
#   scripts/run_tabs.sh              -> Terminator if installed, else gnome-terminal, else tmux
#   scripts/run_tabs.sh terminator   -> ONE Terminator window, 3 tabs split into named panes:
#                                        Simulation (Gazebo | DDS agent / PX4 uav01 | PX4 uav02)
#                                        ROS (ROS nodes / RViz | GUI)   Operate (Controls / Teleop UGV)
#   scripts/run_tabs.sh gnome        -> gnome-terminal, one tab per component
#   scripts/run_tabs.sh tmux         -> tmux session "tins8" with named windows
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
  if command -v terminator >/dev/null && [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then mode=terminator
  elif command -v gnome-terminal >/dev/null && [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then mode=gnome
  elif command -v tmux >/dev/null; then mode=tmux
  else echo "Need terminator or gnome-terminal (desktop), or tmux. Or use: ros2 launch tins8_bringup sim_all.launch.py"; exit 1; fi
fi
if [ "$mode" = terminator ]; then
  # A project-only Terminator config (passed with -g), so your own ~/.config/terminator is untouched.
  conf="$TINS8_LOG_DIR/terminator.conf"
  pane() {  # name  delay_s  command  -> one Terminal entry that sleeps, runs, then keeps a shell open
    printf '    [[[%s]]]\n      type = Terminal\n      parent = %s\n      order = %s\n      title = tins8: %s\n' "$1" "$2" "$3" "$4"
    printf '      command = "printf \x27\\033]0;tins8: %s\\007\x27; sleep %s; %s; exec bash"\n' "$4" "$5" "$6"
  }
  {
    printf '[global_config]\n  suppress_multiple_term_dialog = True\n[keybindings]\n[profiles]\n  [[default]]\n    scrollback_lines = 10000\n    show_titlebar = True\n'
    printf '[layouts]\n  [[tins8]]\n'
    printf '    [[[window0]]]\n      type = Window\n      parent = ""\n      size = 1500, 900\n      title = tins8 simulation\n'
    printf '    [[[tabs]]]\n      type = Notebook\n      parent = window0\n      labels = Simulation, ROS, Operate\n'
    # tab 1: Simulation  (2 x 2)
    printf '    [[[sim]]]\n      type = VPaned\n      parent = tabs\n      order = 0\n      ratio = 0.5\n'
    printf '    [[[sim_top]]]\n      type = HPaned\n      parent = sim\n      order = 0\n      ratio = 0.5\n'
    printf '    [[[sim_bot]]]\n      type = HPaned\n      parent = sim\n      order = 1\n      ratio = 0.5\n'
    pane gazebo sim_top 0 "Gazebo" 0  "'$here/start_gazebo.sh' fieldlab"
    pane agent  sim_top 1 "DDS agent" 2 "'$here/start_agent.sh'"
    pane px4a   sim_bot 0 "PX4 uav01" 8 "'$here/start_px4.sh' uav01 fieldlab"
    pane px4b   sim_bot 1 "PX4 uav02" 10 "'$here/start_px4.sh' uav02 fieldlab"
    # tab 2: ROS  (nodes on top, RViz | GUI below)
    printf '    [[[ros]]]\n      type = VPaned\n      parent = tabs\n      order = 1\n      ratio = 0.6\n'
    printf '    [[[ros_bot]]]\n      type = HPaned\n      parent = ros\n      order = 1\n      ratio = 0.5\n'
    pane nodes ros 0 "ROS nodes" 12 "'$here/start_nodes.sh'"
    pane rviz ros_bot 0 "RViz" 16 "'$here/start_rviz.sh'"
    pane gui  ros_bot 1 "GUI" 17 "'$here/start_gui.sh'"
    # tab 3: Operate  (Controls on top, Teleop below)
    printf '    [[[ops]]]\n      type = VPaned\n      parent = tabs\n      order = 2\n      ratio = 0.65\n'
    pane controls ops 0 "Controls" 17 "'$here/controls.sh'"
    pane teleop   ops 1 "Teleop UGV" 17 "'$here/start_teleop.sh'"
    printf '[plugins]\n'
  } > "$conf"
  setsid terminator -g "$conf" -l tins8 >/dev/null 2>&1 &
  echo "started one Terminator window (tabs: Simulation, ROS, Operate). Type commands in Operate > Controls."
  echo "Stop everything with: scripts/stop_all.sh"
  exit 0
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
