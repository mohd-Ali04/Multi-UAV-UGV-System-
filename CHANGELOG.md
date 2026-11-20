# Changelog

## Unreleased

- `scripts/setup_deps.sh check`: a read-only report of every dependency, showing OK (with version), MISSING, NOT-BUILT or WRONG. It covers Ubuntu 24.04, ROS 2 Jazzy, the apt packages, Gazebo, PX4, the DDS agent, px4_msgs and the optional launchers. It hints when PX4 is already built elsewhere in the home folder.
- `scripts/setup_deps.sh all` now checks first and installs only what is missing:
  - only the missing apt packages;
  - PX4, the agent and px4_msgs skipped when already built at the pinned version;
  - Terminator offered as an optional extra;
  - a second check at the end.
  `FORCE=1` rebuilds a step.
- `setup_deps.sh` reads `scripts/local.env`, so it uses the same dependency folder as the rest of the scripts. It also understands the older px4_msgs layout (`msgs-install`).
- `ros-jazzy-ros-gz` (metapackage) is no longer required; `ros-gz-sim` and `ros-gz-bridge` are what the project uses.
- README: a WSL2 section (untested guidance), and the check / install-only-missing steps.

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
