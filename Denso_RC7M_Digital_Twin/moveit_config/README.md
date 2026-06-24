# MoveIt Adapter Configuration

`moveit_controllers.yaml` points MoveIt at:

```text
/denso_joint_trajectory_controller/follow_joint_trajectory
```

This action is implemented by `serial_gateway.py`.

## Critical limitation

The RC7M serial program accepts blocking point commands, not a timed streaming
trajectory. Therefore:

- waypoint timing from MoveIt is not followed
- velocity/acceleration profiles are not followed
- cancellation is not available during one active point command
- feedback is unavailable while waiting for `F`
- native PTP interpolation can differ from the planned interpolated path

For these reasons, hardware execution defaults to:

```yaml
execution_enabled: false
```

Use MoveIt first for:

- robot-state visualization
- collision checking
- planning
- shadow trajectory comparison

Before enabling execution:

1. create an accurate VS-6556E URDF/SRDF
2. verify joint order, signs, zeros, and limits
3. validate every serial command in manual/reduced-speed mode
4. determine whether controller-side buffered trajectory execution can be added
5. implement a controller-side stop/cancel command
6. compare native paths against MoveIt paths
7. add independent workspace and joint-limit validation

For production, a controller-side PACScript buffer/handshake protocol is
preferable to sending every MoveIt point as a separate PTP move.
