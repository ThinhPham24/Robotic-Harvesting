# Safety Boundary

ROS is not the safety-rated controller. FANUC safety functions, emergency
stops, enabling devices, fences/scanners, payload configuration, and speed
limits remain authoritative.

## Software safety gates

Before execution, `harvest_safety` should verify:

- hardware heartbeat is healthy
- robot mode permits the requested operation
- no active controller fault
- operator enable is valid
- target observation is fresh
- transform chain is available and current
- target lies inside the configured harvesting workspace
- trajectory respects joint, velocity, acceleration, and Cartesian limits
- collision checking succeeded
- tool state is compatible with the next action

## Execution monitoring

Stop or cancel on:

- stale robot state
- stale target or lost visual tracking
- excessive trajectory tracking error
- unexpected contact or tool load
- controller fault
- communication timeout
- workspace boundary violation
- operator stop

Validate in this order:

1. unit tests
2. recorded sensor data
3. RViz with no controller
4. fake hardware
5. simulation
6. physical robot in reduced-speed/manual supervision
7. production-like autonomous trials

Never use an LLM response as evidence that an action is safe.
