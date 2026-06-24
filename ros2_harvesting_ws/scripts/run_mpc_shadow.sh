#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
ros2 launch harvest_bringup mpc_shadow.launch.py
