#!/usr/bin/env bash
# Build the workspace (px4_msgs must already be built by scripts/setup_deps.sh msgs).
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/env.sh
colcon build "$@"
echo "built. In every new terminal: source scripts/env.sh"
