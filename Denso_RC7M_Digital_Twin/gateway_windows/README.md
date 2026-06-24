# Windows Legacy Gateway

Use a Windows industrial PC when the RC7M interface depends on DENSO ORiN/CAO
providers or other Windows-only software.

## Components

```text
Rc7ControllerAdapter
  owns provider/COM calls and unit conversion

TelemetrySnapshot
  immutable normalized state

UdpTelemetryPublisher
  serializes telemetry_v1 and sends it to Ubuntu

GatewaySupervisor
  heartbeat, reconnect, logging, and read-only policy
```

## First implementation

Implement only:

- connect/disconnect
- read controller and robot identity
- read six joint positions
- read mode, servo, E-stop, alarms, native task, cycle counter
- publish UDP telemetry

Do not implement:

- motor power changes
- task start
- variable writes
- motion commands
- controller reset

The exact CAO provider name, object hierarchy, and variable names must come from
the installed RC7M ORiN provider documentation. RC8 examples are not proof of
RC7M compatibility.
