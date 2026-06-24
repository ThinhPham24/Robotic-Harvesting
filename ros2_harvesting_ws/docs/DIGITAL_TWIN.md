# Digital Twin and Predictive Maintenance

## Twin levels

1. **Geometric twin** — URDF/SDF, TF, collision models, cameras, plants, and cell.
2. **Behavioral twin** — controller delay, joint dynamics, friction, payload, and tool.
3. **Operational twin** — live synchronization, residuals, health state, and history.
4. **Predictive twin** — fault scenarios, degradation models, maintenance risk, and RUL.

Build them in that order.

Unity is the preferred visual and operator-facing twin for this project. It
should consume the ROS-fused semantic scene and may publish synthetic sensor
data. See `UNITY_INTEGRATION.md`.

## Asset hierarchy

```text
harvesting_cell
├── fanuc_robot
│   ├── joint_1 ... joint_6
│   ├── controller
│   └── network_link
├── harvesting_tool
│   ├── gripper
│   ├── cutter
│   └── force_sensor
├── perception
│   ├── left_camera
│   ├── right_camera
│   ├── calibration
│   └── compute_gpu
├── scanner
│   ├── turntable
│   └── stereo_reconstruction
└── cell_environment
```

## Telemetry to retain

- commanded and measured joint position/velocity
- trajectory tracking error and following-error events
- motor current/torque and temperature, when exposed by FANUC
- controller mode, alarms, safety state, and communication latency
- tool current, force, cycle time, success/failure, and jams
- camera exposure, dropped frames, image sharpness, and calibration residual
- CPU/GPU temperature, utilization, memory, disk, and inference latency
- scanner point count, registration fitness/RMSE, and calibration age
- maintenance, part replacement, lubrication, and fault labels

Use a time-series database for numeric telemetry and object storage for images,
bags, reports, and model artifacts. Preserve ROS timestamps and asset IDs.

## Health computation

Start with transparent signals:

- exponentially weighted tracking-error baseline
- command-to-feedback latency
- temperature/current threshold and trend
- cycle-time drift
- repeated controller/tool faults
- camera calibration and image-quality drift
- point-cloud registration-quality drift

Only introduce learned anomaly models after healthy operating envelopes are
well sampled across payload, speed, temperature, and harvesting conditions.

## Predictive maintenance limitations

Remaining useful life cannot be validated from normal data alone. It needs
run-to-failure data, known degradation experiments, or reliable maintenance
labels. Until then, publish risk and inspection recommendations with low
confidence rather than false precise hour estimates.

## Digital thread

Every scan and harvest result should link:

- fruit/operation ID
- robot and tool configuration version
- calibration versions
- software and model versions
- trajectory and controller configuration
- health snapshot
- operator and maintenance events
- outcome and failure code
