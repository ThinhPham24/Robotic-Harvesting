#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is not installed at /opt/ros/humble." >&2
  echo "Follow docs/UBUNTU_22_04_SETUP.md first." >&2
  exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/humble/setup.bash
cd "${WORKSPACE}"
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install

echo
echo "Build complete."
echo "Run: source ${WORKSPACE}/install/setup.bash"
