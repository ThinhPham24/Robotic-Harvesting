# Step-by-Step Development Guide

## Stage 1: Run the bridge without the robot

Install ROS 2 Humble on Ubuntu 22.04.

Build:

```bash
cd /home/airlab/Desktop/Robotic-Harvesting/Denso_RC7M_Digital_Twin/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

Run the ROS receiver:

```bash
ros2 launch denso_rc7m_bringup telemetry_bridge.launch.py
```

In a second terminal, run fake controller telemetry:

```bash
cd /home/airlab/Desktop/Robotic-Harvesting/Denso_RC7M_Digital_Twin/gateway_simulator
python3 fake_rc7m_gateway.py
```

Inspect:

```bash
ros2 topic echo /denso/joint_states
ros2 topic echo /denso/controller_state
```

## Stage 2: Prepare the digital model

1. Obtain or create a VS-6556E URDF.
2. Confirm joint order, zero pose, limits, and axis signs from the real robot.
3. Import the same geometry into Unity or the selected external simulator.
4. Subscribe the simulator to `/denso/joint_states`.
5. Compare at least five asymmetric poses.

## Stage 3: Audit RC7M

Complete `CAPABILITY_AUDIT.md`.

The first real connection must be read-only:

- servo off
- automatic program stopped
- no controller variable writes
- isolated robot network
- operator at teach pendant

## Stage 4: Implement the Windows adapter

On a Windows PC with the licensed DENSO/ORiN tools:

1. Confirm that the RC7 provider can connect.
2. Read controller name/version.
3. Read robot joint position.
4. Read mode, servo, E-stop, alarm, task, and I/O state.
5. Convert units to degrees and SI-friendly metadata.
6. Publish `telemetry_v1` UDP packets to the Ubuntu PC.
7. Add heartbeat and sequence counters.
8. Log every provider error and reconnect.

Do not add write methods during this stage.

## Stage 5: Connect Unity

Unity consumes ROS state, not RC7M directly:

```text
RC7M -> Windows gateway -> ROS 2 -> Unity
```

Display:

- robot joint pose
- controller mode and alarm
- packet age and loss
- native task/program state
- digital-twin joint residual
- tool and I/O state

## Stage 6: Collect maintenance data

Store:

- timestamped joint motion
- cycle ID and task
- alarm codes
- command/feedback delay
- tool state
- camera/perception quality
- maintenance and replacement events

Predictive maintenance requires labeled history. Begin with trend and anomaly
monitoring, not precise remaining-life claims.

## Stage 7: Add VLA

First use VLA for:

- scene understanding
- target ranking
- operator explanations
- choosing one approved skill
- recovery proposals from a finite list

VLA output must pass:

```text
schema validation
 -> task policy
 -> digital-twin prediction
 -> collision/workspace validation
 -> operator/safety approval
 -> named RC7M skill request
```

## Stage 8: Add commands only if justified

Prefer named native programs with handshake variables. Keep trajectory planning
and MPC in shadow/simulation unless RC7M documentation proves a safe,
deterministic external-control interface.
