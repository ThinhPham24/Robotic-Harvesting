# DENSO VS-6556E / RC7M Digital Twin Gateway

Standalone project for integrating a legacy DENSO VS-6556E with RC7M into a
ROS 2 Humble and Unity digital-twin architecture.

The RC7M does not run ROS. An external gateway translates a controller-supported
legacy interface into a small, versioned telemetry protocol consumed by ROS 2.

## Recommended architecture

```text
DENSO VS-6556E
      |
    RC7M
      |
      | controller-supported interface
      | ORiN/CAO/b-CAP if licensed and supported, otherwise PLC/fieldbus/I/O
      v
Windows Legacy Gateway
      |
      | telemetry-only UDP v1
      v
Ubuntu 22.04 / ROS 2 Humble
      |
      +--> /denso/joint_states
      +--> /denso/controller_state
      +--> health and maintenance pipeline
      +--> Unity / external simulator
```

## Current implementation

- Buildable ROS 2 Humble interface package.
- UDP telemetry bridge.
- Cross-platform fake RC7M telemetry sender.
- Windows gateway architecture and ORiN integration boundary.
- Protocol specification.
- Safety and staged commissioning guide.

No physical robot command path is implemented.

## Main code

- `ros2_ws/src/denso_rc7m_gateway/denso_rc7m_gateway/telemetry_bridge.py`
- `ros2_ws/src/denso_rc7m_gateway/denso_rc7m_gateway/serial_protocol.py`
- `ros2_ws/src/denso_rc7m_gateway/denso_rc7m_gateway/serial_gateway.py`
- `gateway_simulator/fake_rc7m_gateway.py`
- `protocol/telemetry_v1.schema.json`

## Quick local test

Terminal 1:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
ros2 launch denso_rc7m_bringup telemetry_bridge.launch.py
```

Terminal 2:

```bash
cd gateway_simulator
python3 fake_rc7m_gateway.py
```

Terminal 3:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic echo /denso/joint_states
```

Read `docs/STEP_BY_STEP.md` before connecting any controller network.

For direct communication diagnostics, start with:

```bash
python3 communication_tests/ubuntu_network_probe.py <RC7M_IP>
```

Then follow `communication_tests/README.md`.

For the existing RS-232 protocol and MoveIt adapter, see:

```text
docs/SERIAL_ROS_MOVEIT.md
```
