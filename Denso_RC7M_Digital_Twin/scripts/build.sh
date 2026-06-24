#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/humble/setup.bash
cd "${ROOT}/ros2_ws"
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
echo "Run: source ${ROOT}/ros2_ws/install/setup.bash"
