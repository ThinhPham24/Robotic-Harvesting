# Telemetry Protocol v1

The Windows gateway sends one UTF-8 JSON object per UDP datagram.

Default destination:

```text
Ubuntu gateway IP: configured by operator
UDP port: 15000
Maximum recommended rate: 50 Hz
```

All six joint positions use degrees at the legacy boundary. The ROS bridge
converts them to radians.

Example:

```json
{
  "protocol": "denso_rc7m.telemetry.v1",
  "sequence": 42,
  "source_time_unix_ns": 1730000000000000000,
  "controller": "RC7M",
  "robot": "VS-6556E",
  "joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
  "joint_position_deg": [0, -20, 45, 0, 30, 0],
  "joint_velocity_deg_s": [0, 0, 0, 0, 0, 0],
  "mode": "MANUAL",
  "servo_on": false,
  "emergency_stop": false,
  "protective_stop": false,
  "alarm_code": 0,
  "alarm_text": "",
  "native_task": "IDLE",
  "cycle_count": 120
}
```

Malformed, stale, out-of-order, oversized, or wrong-version packets are rejected.
