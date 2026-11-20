# Changelog

## 1.0.1 (2026-11-20)

- `scripts/run_tabs.sh` opens one Terminator window when Terminator is installed. The window has three tabs split into named panes:
  - **Simulation:** Gazebo, DDS agent, PX4 uav01, PX4 uav02.
  - **ROS:** ROS nodes, RViz, GUI.
  - **Operate:** Controls, Teleop UGV.
- The launcher mode can be forced with `scripts/run_tabs.sh terminator|gnome|tmux`. Without Terminator it falls back to gnome-terminal tabs, then to tmux.
- The DDS agent prints a banner when it starts. The docs now state the correct readiness signs: the agent is quiet while it runs, and the PX4 panes show `successfully created` lines once connected.
- README and build guide: the new launcher, and a note to start either `run_tabs.sh` or `sim_all.sh`, not both.

## 1.0.0 (2026-11-20)

- First release: TinS-12E UGV with the eight-aircraft carousel in a native Gazebo copy of the Fieldlab world, two PX4 SITL aircraft, YOLOX tracking with a magnified inset, high-speed visual pursuit, mission action, terminal tool and GUI, launchers, build guide and recorded results.
