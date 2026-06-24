# Existing Serial Protocol to ROS 2 and MoveIt

## Corrections made to the supplied code

- `pyserial.Serial.read()` does not accept a `timeout=` argument.
- `msg_timeout_` and `pos_timeout_` were undefined.
- serial writes now use `write_timeout`
- responses are ASCII and CR framed
- `R` and `F` are read with explicit deadlines
- stale bytes are cleared before a transaction
- only one object owns the serial port
- all numeric values are checked for count and finiteness
- joint units are converted from controller degrees to ROS radians
- hardware execution defaults to disabled

## Read-only communication test

Install:

```bash
sudo apt install python3-serial
```

Find the serial port:

```bash
ls -l /dev/serial/by-id/
```

Give the current user serial permission:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing the group.

Build:

```bash
cd /home/airlab/Desktop/Robotic-Harvesting/Denso_RC7M_Digital_Twin
./scripts/build.sh
source ros2_ws/install/setup.bash
```

Read joints and Cartesian position without moving:

```bash
ros2 run denso_rc7m_gateway serial_read_test \
  --port /dev/serial/by-id/YOUR_USB_SERIAL_DEVICE \
  --baud 19200
```

## Run joint-state publishing

Edit:

```text
ros2_ws/src/denso_rc7m_gateway/config/serial_gateway.yaml
```

Set the correct serial port. Keep:

```yaml
execution_enabled: false
```

Run:

```bash
ros2 launch denso_rc7m_bringup serial_gateway.launch.py
```

Inspect:

```bash
ros2 topic echo /joint_states
```

## Connect MoveIt in shadow mode

Copy or merge `moveit_config/moveit_controllers.yaml` into the generated
VS-6556E MoveIt configuration.

MoveIt can then detect the action server, but goals are rejected while
`execution_enabled` is false. This is intentional.

## Enabling motion for supervised testing

Only after completing the validation checklist, set:

```yaml
execution_enabled: true
execution_speed_percent: 5
maximum_joint_step_rad: 0.05
```

Start with:

- pendant/manual supervision
- reduced speed
- empty workspace
- one very small joint movement
- no gripper action
- an operator ready at physical E-stop

The current action adapter sends each accepted waypoint using command `0`.
This is development code, not a certified production trajectory controller.
