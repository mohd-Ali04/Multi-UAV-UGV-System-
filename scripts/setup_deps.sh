#!/usr/bin/env bash
# Reproducible setup of the external dependencies (pinned) on Ubuntu 24.04 + ROS 2 Jazzy.
#
#   scripts/setup_deps.sh check    # read-only report: what is installed (with version) and what is missing
#   scripts/setup_deps.sh all      # check first, then install ONLY what is missing, then check again
#   scripts/setup_deps.sh apt      # missing ROS/Gazebo/Python apt packages      (needs sudo; asks first)
#   scripts/setup_deps.sh px4      # PX4-Autopilot v1.17.0 SITL build             (~3 GB, 15-30 min)
#   scripts/setup_deps.sh agent    # Micro XRCE-DDS Agent v2.4.3
#   scripts/setup_deps.sh msgs     # px4_msgs release/1.17 (ROS 2 package)
#   scripts/setup_deps.sh optional # Terminator, for the single-window launcher (asks first)
#
# Every step skips work that is already done at the pinned version.  FORCE=1 rebuilds anyway.
# PX4, the agent and px4_msgs go under $TINS8_DEPS (default ~/tins8_deps, or the value set in
# scripts/local.env); nothing except the apt packages is written into system directories.
# ROS 2 Jazzy itself is a prerequisite: 'check' reports it, but this script does not add the ROS
# apt repository to your system.
#
# Pinned revisions (the ones used for the recorded runs):
PX4_TAG=v1.17.0
PX4_COMMIT=d6f12ad1c4f70ad3230afd7d86e971421e02fef4
AGENT_TAG=v2.4.3
AGENT_COMMIT=73622810d984349b80bbac0ef55fc0b694d62222
MSGS_COMMIT=86d8239e962f6939e05c3737784f60c02fa884db   # PX4/px4_msgs release/1.17
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -f "$HERE/local.env" ] && source "$HERE/local.env"
DEPS="${TINS8_DEPS:-$HOME/tins8_deps}"
FORCE="${FORCE:-0}"
step="${1:-all}"

APT_PKGS=(ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim ros-jazzy-vision-msgs
          ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard ros-jazzy-tf2-ros
          python3-colcon-common-extensions python3-numpy python3-scipy python3-opencv python3-yaml
          python3-pyqt5 python3-matplotlib python3-pytest git cmake build-essential gnome-terminal)

# ------------------------------------------------------------------ detection (read-only)
have_pkg() { [ "$(dpkg-query -W -f='${Status}' "$1" 2>/dev/null)" = "install ok installed" ]; }
pkg_ver() { dpkg-query -W -f='${Version}' "$1" 2>/dev/null | cut -d- -f1; }
git_head() { git -C "$1" rev-parse HEAD 2>/dev/null || true; }

px4_dir="$DEPS/PX4-Autopilot"
px4_bin="$px4_dir/build/px4_sitl_default/bin/px4"
agent_src="$DEPS/Micro-XRCE-DDS-Agent"
agent_bin="$DEPS/agent-build/MicroXRCEAgent"
msgs_install() {   # same search order as scripts/env.sh
  if [ -f "$DEPS/px4_msgs_ws/install/setup.bash" ]; then echo "$DEPS/px4_msgs_ws/install"
  elif [ -f "$DEPS/msgs-install/setup.bash" ]; then echo "$DEPS/msgs-install"; fi
}
msgs_src() {
  local d
  for d in "$DEPS/px4_msgs_ws/src/px4_msgs" "$DEPS/px4_msgs"; do [ -d "$d/.git" ] && { echo "$d"; return; }; done
  return 0
}

# state of each item: ok | missing | build | wrong
state_ros()   { [ -f /opt/ros/jazzy/setup.bash ] && echo ok || echo missing; }
missing_apt() { local p; for p in "${APT_PKGS[@]}"; do have_pkg "$p" || echo "$p"; done; }
state_px4() {
  [ -d "$px4_dir/.git" ] || { echo missing; return; }
  [ "$(git_head "$px4_dir")" = "$PX4_COMMIT" ] || { echo wrong; return; }
  [ -x "$px4_bin" ] && echo ok || echo build
}
state_agent() {
  if [ -x "$agent_bin" ]; then
    if [ -d "$agent_src/.git" ] && [ "$(git_head "$agent_src")" != "$AGENT_COMMIT" ]; then echo wrong; else echo ok; fi
  elif command -v MicroXRCEAgent >/dev/null; then echo ok-system
  elif [ -d "$agent_src/.git" ]; then echo build
  else echo missing; fi
}
state_msgs() {
  local src; src="$(msgs_src)"
  if [ -n "$(msgs_install)" ]; then
    if [ -n "$src" ] && [ "$(git_head "$src")" != "$MSGS_COMMIT" ]; then echo wrong; else echo ok; fi
  elif [ -n "$src" ]; then echo build
  else echo missing; fi
}

line() { printf '  %-30s %-9s %s\n' "$1" "$2" "$3"; }
check() {
  local nmiss=0 s miss
  echo "Dependency check (dependencies folder: $DEPS)"
  if command -v lsb_release >/dev/null; then
    local os; os="$(lsb_release -ds 2>/dev/null)"
    case "$os" in *"24.04"*) line "Ubuntu 24.04" OK "$os";; *) line "Ubuntu 24.04" WARN "found $os (only 24.04 is tested)";; esac
  fi
  if grep -qi microsoft /proc/version 2>/dev/null; then line "environment" INFO "WSL detected - see the WSL2 section of the README"; fi
  s="$(state_ros)"
  if [ "$s" = ok ]; then line "ROS 2 Jazzy" OK "/opt/ros/jazzy"
  else line "ROS 2 Jazzy" MISSING "install ros-jazzy-desktop first: https://docs.ros.org/en/jazzy/Installation.html"; nmiss=$((nmiss+1)); fi
  miss="$(missing_apt)"
  if [ -z "$miss" ]; then
    line "apt packages (${#APT_PKGS[@]})" OK "ros_gz_bridge $(pkg_ver ros-jazzy-ros-gz-bridge), rviz2 $(pkg_ver ros-jazzy-rviz2), vision_msgs $(pkg_ver ros-jazzy-vision-msgs)"
  else
    line "apt packages" MISSING "$(echo "$miss" | wc -l) missing: $(echo $miss)"; nmiss=$((nmiss+1))
  fi
  if [ "$s" = ok ]; then
    local gzv; gzv="$( (set +u; source /opt/ros/jazzy/setup.bash; gz sim --versions 2>/dev/null) | head -1 || true)"
    if [ -n "$gzv" ]; then line "Gazebo Sim" OK "$gzv (tested: 8.15.0)"; else line "Gazebo Sim" MISSING "comes with ros-jazzy-ros-gz-sim"; fi
  fi
  case "$(state_px4)" in
    ok)      line "PX4-Autopilot $PX4_TAG" OK "$px4_bin";;
    build)   line "PX4-Autopilot $PX4_TAG" NOT-BUILT "source present at the pinned commit; the 'px4' step builds it"; nmiss=$((nmiss+1));;
    wrong)   line "PX4-Autopilot $PX4_TAG" WRONG "$px4_dir is at $(git_head "$px4_dir" | cut -c1-8), expected ${PX4_COMMIT:0:8}"; nmiss=$((nmiss+1));;
    missing) line "PX4-Autopilot $PX4_TAG" MISSING "$px4_dir"; nmiss=$((nmiss+1))
             local d other=""
             while IFS= read -r d; do [ -x "$d/build/px4_sitl_default/bin/px4" ] && { other="${d%/PX4-Autopilot}"; break; }; done \
               < <(find "$HOME" -maxdepth 4 -type d -name PX4-Autopilot -not -path "$DEPS/*" 2>/dev/null)
             if [ -n "$other" ]; then line "" HINT "a PX4 build exists in $other; set TINS8_DEPS to that folder in scripts/local.env"; fi;;
  esac
  case "$(state_agent)" in
    ok)        line "Micro XRCE-DDS Agent $AGENT_TAG" OK "$agent_bin";;
    ok-system) line "Micro XRCE-DDS Agent" OK "$(command -v MicroXRCEAgent) (system install, version not checked)";;
    build)     line "Micro XRCE-DDS Agent $AGENT_TAG" NOT-BUILT "source present; the 'agent' step builds it"; nmiss=$((nmiss+1));;
    wrong)     line "Micro XRCE-DDS Agent $AGENT_TAG" WRONG "source is not at ${AGENT_COMMIT:0:8}"; nmiss=$((nmiss+1));;
    missing)   line "Micro XRCE-DDS Agent $AGENT_TAG" MISSING "$agent_bin"; nmiss=$((nmiss+1));;
  esac
  case "$(state_msgs)" in
    ok)      line "px4_msgs release/1.17" OK "$(msgs_install)";;
    build)   line "px4_msgs release/1.17" NOT-BUILT "source present; the 'msgs' step builds it"; nmiss=$((nmiss+1));;
    wrong)   line "px4_msgs release/1.17" WRONG "source is not at ${MSGS_COMMIT:0:8}"; nmiss=$((nmiss+1));;
    missing) line "px4_msgs release/1.17" MISSING "$DEPS/px4_msgs_ws"; nmiss=$((nmiss+1));;
  esac
  echo "Optional:"
  local t
  for t in terminator gnome-terminal tmux; do
    if command -v "$t" >/dev/null; then line "$t" OK "launcher: scripts/run_tabs.sh $([ "$t" = gnome-terminal ] && echo gnome || echo "$t")"
    else line "$t" absent "$([ "$t" = terminator ] && echo "single-window launcher; the 'optional' step installs it" || true)"; fi
  done
  if command -v google-chrome >/dev/null || command -v chromium >/dev/null; then line "Chrome or Chromium" OK "only needed to re-render the PDF guide"
  else line "Chrome or Chromium" absent "only needed to re-render the PDF guide"; fi
  if [ "$nmiss" -eq 0 ]; then echo "Result: all required dependencies are present."
  else echo "Result: $nmiss required item(s) missing or not built. Run: scripts/setup_deps.sh all"; fi
  return "$nmiss"
}

# ------------------------------------------------------------------ install steps (skip when done)
ask() { local a; read -r -p "$1 [y/N] " a; [[ "$a" =~ ^[Yy]$ ]]; }

apt_step() {
  local miss; miss="$(missing_apt)"
  if [ -z "$miss" ]; then echo "apt: all ${#APT_PKGS[@]} packages already installed, skipping."; return; fi
  echo "apt: these packages are missing:"; printf '  %s\n' $miss
  if ask "Install them now with sudo apt?"; then sudo apt-get update && sudo apt-get install -y $miss; else echo "skipped apt step"; fi
}

px4_step() {
  local s; s="$(state_px4)"
  if [ "$s" = ok ] && [ "$FORCE" != 1 ]; then echo "px4: already built at $PX4_TAG, skipping."; return; fi
  if [ "$s" = wrong ]; then echo "px4: $px4_dir is not at $PX4_COMMIT; move it away or set another TINS8_DEPS."; exit 1; fi
  mkdir -p "$DEPS"
  if [ "$s" = missing ]; then
    git clone --branch "$PX4_TAG" --depth 1 --recurse-submodules --shallow-submodules \
      https://github.com/PX4/PX4-Autopilot.git "$px4_dir"
    [ "$(git_head "$px4_dir")" = "$PX4_COMMIT" ] || { echo "PX4 is not at $PX4_COMMIT"; exit 1; }
  fi
  echo "PX4 needs its build toolchain. Its own installer (sudo) is:"
  echo "  bash $px4_dir/Tools/setup/ubuntu.sh --no-nuttx"
  if ask "Run it now? (answer N if the PX4 toolchain is already installed)"; then (cd "$px4_dir" && bash Tools/setup/ubuntu.sh --no-nuttx); fi
  (cd "$px4_dir" && make px4_sitl_default)      # builds SITL + the gz_bridge; no simulator is launched
}

agent_step() {
  local s; s="$(state_agent)"
  if [[ "$s" == ok* ]] && [ "$FORCE" != 1 ]; then echo "agent: already built, skipping."; return; fi
  if [ "$s" = wrong ]; then echo "agent: $agent_src is not at $AGENT_COMMIT"; exit 1; fi
  mkdir -p "$DEPS"
  [ -d "$agent_src/.git" ] || git clone --branch "$AGENT_TAG" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$agent_src"
  [ "$(git_head "$agent_src")" = "$AGENT_COMMIT" ] || { echo "agent not at $AGENT_COMMIT"; exit 1; }
  cmake -S "$agent_src" -B "$DEPS/agent-build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$DEPS/agent-build" -j"$(nproc)"
  echo "agent binary: $agent_bin"
}

msgs_step() {
  local s; s="$(state_msgs)"
  if [ "$s" = ok ] && [ "$FORCE" != 1 ]; then echo "msgs: px4_msgs already built ($(msgs_install)), skipping."; return; fi
  if [ "$s" = wrong ]; then echo "msgs: px4_msgs source is not at $MSGS_COMMIT"; exit 1; fi
  [ "$(state_ros)" = ok ] || { echo "msgs: ROS 2 Jazzy is required to build px4_msgs"; exit 1; }
  local src; src="$(msgs_src)"
  if [ -z "$src" ]; then
    mkdir -p "$DEPS/px4_msgs_ws/src"
    src="$DEPS/px4_msgs_ws/src/px4_msgs"
    git clone https://github.com/PX4/px4_msgs.git "$src"
  fi
  (cd "$src" && git checkout -q "$MSGS_COMMIT")
  if [ "$src" = "$DEPS/px4_msgs_ws/src/px4_msgs" ]; then
    (set +u; source /opt/ros/jazzy/setup.bash && cd "$DEPS/px4_msgs_ws" && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release)
  else   # older layout: source in $DEPS/px4_msgs, install into $DEPS/msgs-install
    (set +u; source /opt/ros/jazzy/setup.bash && cd "$DEPS" && colcon build --base-paths px4_msgs \
       --build-base msgs-build --install-base msgs-install --cmake-args -DCMAKE_BUILD_TYPE=Release)
  fi
}

optional_step() {
  if command -v terminator >/dev/null; then echo "optional: Terminator already installed."; return; fi
  echo "optional: Terminator gives the single-window launcher (scripts/run_tabs.sh). Without it,"
  echo "          the launcher uses gnome-terminal tabs or tmux."
  if ask "Install Terminator with sudo apt?"; then sudo apt-get install -y terminator; else echo "skipped Terminator"; fi
}

case "$step" in
  check) set +e; check; exit $? ;;
  apt) apt_step ;;
  px4) px4_step ;;
  agent) agent_step ;;
  msgs) msgs_step ;;
  optional) optional_step ;;
  all)
    set +e; check; set -e
    if [ "$(state_ros)" != ok ]; then
      echo; echo "Install ROS 2 Jazzy (ros-jazzy-desktop) first, then run this again:"
      echo "  https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"
      exit 1
    fi
    echo; echo "Installing only what is missing:"
    apt_step; px4_step; agent_step; msgs_step; optional_step
    echo; set +e; check; set -e ;;
  *) echo "unknown step $step (use: check | all | apt | px4 | agent | msgs | optional)"; exit 1 ;;
esac
echo "done ($step). Dependencies live in $DEPS; scripts/env.sh finds them there."
