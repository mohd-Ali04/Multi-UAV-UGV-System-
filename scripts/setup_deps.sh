#!/usr/bin/env bash
# Reproducible setup of the external dependencies (pinned) on Ubuntu 24.04 + ROS 2 Jazzy.
#
#   scripts/setup_deps.sh apt      # ROS/Gazebo/Python packages via apt   (needs sudo; asks first)
#   scripts/setup_deps.sh px4      # PX4-Autopilot v1.17.0 SITL build      (~3 GB, 15-30 min)
#   scripts/setup_deps.sh agent    # Micro XRCE-DDS Agent v2.4.3
#   scripts/setup_deps.sh msgs     # px4_msgs release/1.17 (ROS 2 package)
#   scripts/setup_deps.sh all      # everything above, in order
#
# Everything except 'apt' installs under $TINS8_DEPS (default ~/tins8_deps); nothing is
# written into system directories.  Pinned revisions (the ones used for the recorded runs):
PX4_TAG=v1.17.0
PX4_COMMIT=d6f12ad1c4f70ad3230afd7d86e971421e02fef4
AGENT_TAG=v2.4.3
AGENT_COMMIT=73622810d984349b80bbac0ef55fc0b694d62222
MSGS_COMMIT=86d8239e962f6939e05c3737784f60c02fa884db   # PX4/px4_msgs release/1.17
set -euo pipefail
DEPS="${TINS8_DEPS:-$HOME/tins8_deps}"
mkdir -p "$DEPS"
step="${1:-all}"

apt_step() {
  pkgs=(ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim ros-jazzy-vision-msgs
        ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard ros-jazzy-tf2-ros
        python3-colcon-common-extensions python3-numpy python3-scipy python3-opencv python3-yaml
        python3-pyqt5 python3-matplotlib python3-pytest git cmake build-essential gnome-terminal)
  echo "The following apt packages are needed (runtime unless noted):"
  printf '  %s\n' "${pkgs[@]}"
  read -r -p "Install them now with sudo apt? [y/N] " ans
  [[ "$ans" =~ ^[Yy]$ ]] || { echo "skipped apt step"; return; }
  sudo apt-get update && sudo apt-get install -y "${pkgs[@]}"
}

px4_step() {
  if [ ! -d "$DEPS/PX4-Autopilot/.git" ]; then
    git clone --branch "$PX4_TAG" --depth 1 --recurse-submodules --shallow-submodules \
      https://github.com/PX4/PX4-Autopilot.git "$DEPS/PX4-Autopilot"
  fi
  cd "$DEPS/PX4-Autopilot"
  [ "$(git rev-parse HEAD)" = "$PX4_COMMIT" ] || { echo "PX4 is not at $PX4_COMMIT"; exit 1; }
  echo "PX4 needs its build toolchain. Its own installer (sudo) is:"
  echo "  bash $DEPS/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx"
  read -r -p "Run it now? [y/N] " ans
  [[ "$ans" =~ ^[Yy]$ ]] && bash Tools/setup/ubuntu.sh --no-nuttx
  make px4_sitl_default        # builds SITL + the gz_bridge; no simulator is launched
}

agent_step() {
  if [ ! -d "$DEPS/Micro-XRCE-DDS-Agent/.git" ]; then
    git clone --branch "$AGENT_TAG" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$DEPS/Micro-XRCE-DDS-Agent"
  fi
  (cd "$DEPS/Micro-XRCE-DDS-Agent" && [ "$(git rev-parse HEAD)" = "$AGENT_COMMIT" ]) || { echo "agent not at $AGENT_COMMIT"; exit 1; }
  cmake -S "$DEPS/Micro-XRCE-DDS-Agent" -B "$DEPS/agent-build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$DEPS/agent-build" -j"$(nproc)"
  echo "agent binary: $DEPS/agent-build/MicroXRCEAgent"
}

msgs_step() {
  mkdir -p "$DEPS/px4_msgs_ws/src"
  if [ ! -d "$DEPS/px4_msgs_ws/src/px4_msgs/.git" ]; then
    git clone https://github.com/PX4/px4_msgs.git "$DEPS/px4_msgs_ws/src/px4_msgs"
  fi
  (cd "$DEPS/px4_msgs_ws/src/px4_msgs" && git checkout -q "$MSGS_COMMIT")
  (set +u; source /opt/ros/jazzy/setup.bash && cd "$DEPS/px4_msgs_ws" && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release)
}

case "$step" in
  apt) apt_step ;;
  px4) px4_step ;;
  agent) agent_step ;;
  msgs) msgs_step ;;
  all) apt_step; px4_step; agent_step; msgs_step ;;
  *) echo "unknown step $step"; exit 1 ;;
esac
echo "done ($step). Dependencies live in $DEPS; scripts/env.sh finds them there."
