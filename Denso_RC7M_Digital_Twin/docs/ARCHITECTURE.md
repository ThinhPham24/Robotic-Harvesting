# RC7M Integration Architecture

## Why a gateway is required

The official DENSO ROS 2 driver currently documents real-controller support for
RC8/RC8A, with RC9 under construction. RC7M is not listed. The controller must
therefore remain responsible for its native motion and safety while an external
computer adapts whatever legacy interface is actually installed.

## Gateway choices

Evaluate in this order:

1. **ORiN2/CAO provider for RC7**
   - Preferred when a licensed RC7 provider is installed and can read robot,
     variables, mode, alarms, and I/O.
   - Usually implies a Windows gateway because ORiN tooling and providers are
     commonly Windows/COM based.

2. **b-CAP endpoint**
   - Use only if RC7M documentation and a direct test confirm that the controller
     or ORiN gateway exposes the required b-CAP services.
   - Do not assume RC8 b-CAP configuration applies to RC7M.

3. **PLC or fieldbus gateway**
   - RC7M exchanges position/status/handshake variables with a PLC.
   - The Ubuntu gateway reads those registers using the supported industrial
     protocol.
   - Appropriate for monitoring and discrete task requests, not high-rate servo.

4. **Digital I/O plus controller program**
   - Lowest capability fallback.
   - Useful for cycle state, faults, heartbeat, and approved program selection.

## Separation of authority

```text
RC7M native program
  owns motion, interpolation, servo, limits, and tool sequence

ROS 2 task layer
  observes state and may later request an approved named skill

Unity / VLA
  visualizes, predicts, ranks, and proposes
  never commands joints or raw motion
```

## Future command model

Do not stream joint positions to RC7M from ROS unless DENSO explicitly supports
a suitable real-time external-control mode for this controller.

A safer future interface is a finite skill set:

```text
HOME
MOVE_TO_INSPECTION
PICK_SLOT_01
PLACE_BIN_A
STOP_REQUEST
RESET_REQUEST
```

Each request needs:

- monotonically increasing request ID
- requested skill and bounded parameters
- controller-ready precondition
- positive acknowledgement
- execution timeout
- completion/failure code
- independent stop path

## Digital twin

The twin compares:

- native RC7M feedback
- commanded native program state
- simulated joint state
- cycle and tool state
- perception-derived scene

This supports monitoring and model identification even when the controller
cannot accept modern ROS trajectory commands.
