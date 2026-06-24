# harvest_bringup

Launch files and deployment configuration.

Planned directories:

```text
launch/
config/
rviz/
```

Keep separate parameter files for simulation, lab hardware, and production.

Implemented starter launches:

```bash
ros2 launch harvest_bringup health_monitoring.launch.py
ros2 launch harvest_bringup mpc_shadow.launch.py
```

The health launch expects both real and twin joint-state topics. The MPC launch
is shadow mode and publishes recommendations only.
