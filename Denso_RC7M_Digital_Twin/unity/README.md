# Unity / External Simulator Integration

Use ROS 2 as the normalized source:

```text
/denso/joint_states
/denso/controller_state
```

Unity should:

- map the six ROS joints to the VS-6556E hierarchy
- show controller mode, servo, E-stop, alarm, task, packet age, and packet loss
- render real and simulated robot poses simultaneously
- show joint residuals and stale-data warnings
- replay recorded cycles

For future VLA:

- consume camera and scene state
- propose a target or approved skill
- simulate the proposal first
- send the proposal to a deterministic policy/task manager

Never connect VLA output directly to RC7M.

If an external ASIC/physics simulator is used, implement the same ROS topic
contract so Unity and the simulator can be exchanged without changing the
legacy controller gateway.
