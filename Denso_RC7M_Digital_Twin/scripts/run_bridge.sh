#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${ROOT}/ros2_ws/install/setup.bash"
ros2 launch denso_rc7m_bringup telemetry_bridge.launch.py
