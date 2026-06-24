# FANUC ROS 2 Robotic Harvesting Workspace

This workspace is the integration layer between the existing `Scan_app`,
the FANUC cobot, perception, motion planning, end-effector control, and
high-level AI.

## Supported development platform

- Ubuntu 22.04 LTS
- ROS 2 Humble
- MoveIt 2
- `ros2_control`
- Unity 2022.3 LTS for the visual digital twin
- Gazebo or another supported dynamics simulator before hardware testing

Do not select a FANUC driver until the robot model, controller model,
installed FANUC software options, and available communication protocol have
been confirmed.

## Architecture

```text
Camera / Scan_app / Sensors
             |
             v
    harvest_perception
             |
      fruit poses + scene
             v
    harvest_task_manager <----- harvest_ai_manager
             |                   intent/proposals only
             v
   harvest_motion_planning
             |
       validated trajectory
             v
       harvest_safety
             |
             v
      harvest_control
             |
             v
   fanuc_hardware_interface
             |
             v
         FANUC cobot
```

The LLM/VLA layer must never publish joint commands or bypass trajectory,
workspace, collision, speed, and operator-approval checks.

## Workspace tree

```text
ros2_harvesting_ws/
├── docs/
│   ├── ARCHITECTURE.md
│   ├── INTERFACES.md
│   ├── ROADMAP.md
│   └── SAFETY.md
├── src/
│   ├── harvest_interfaces/
│   ├── fanuc_hardware_interface/
│   ├── harvest_description/
│   ├── harvest_bringup/
│   ├── harvest_perception/
│   ├── harvest_motion_planning/
│   ├── harvest_control/
│   ├── harvest_mpc_controller/
│   ├── harvest_safety/
│   ├── harvest_task_manager/
│   ├── harvest_digital_twin/
│   ├── harvest_unity_bridge/
│   ├── harvest_condition_monitor/
│   ├── harvest_predictive_maintenance/
│   └── harvest_ai_manager/
└── third_party.repos
```

## Ownership rules

- `fanuc_hardware_interface`: vendor communication and robot-state transport.
- `harvest_control`: deterministic controller and end-effector commands.
- `harvest_motion_planning`: collision-aware robot motion generation.
- `harvest_task_manager`: harvesting state machine and recovery behavior.
- `harvest_ai_manager`: natural-language/VLA inference and structured proposals.
- `harvest_safety`: command validation and independent execution permission.
- `harvest_perception`: fruit, stem, obstacle, and grasp-target estimation.
- `harvest_digital_twin`: simulation and live real/twin state synchronization.
- `harvest_unity_bridge`: ROS contracts and synchronization rules for the Unity twin.
- `harvest_condition_monitor`: residual, load, temperature, cycle, and anomaly monitoring.
- `harvest_predictive_maintenance`: maintenance-risk and RUL estimation from historical data.
- `harvest_mpc_controller`: constrained predictive reference generation.

## Initial build workflow

After package manifests and implementations are added:

```bash
cd ros2_harvesting_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Starter monitoring commands:

```bash
ros2 launch harvest_bringup health_monitoring.launch.py
ros2 launch harvest_bringup mpc_shadow.launch.py
```

For complete Ubuntu 22.04 installation and run instructions, see
`docs/UBUNTU_22_04_SETUP.md`.

Start with fake hardware and simulation. Do not connect motion execution to the
physical cobot until joint naming, limits, frames, controller behavior, stop
paths, and trajectory cancellation have been verified.
