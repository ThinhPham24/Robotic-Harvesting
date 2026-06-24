# Development Roadmap

## Phase 0: Hardware definition

- Record FANUC robot and controller model.
- Record controller software version and installed communication options.
- Define end effector, payload, I/O, cameras, and network topology.
- Obtain authoritative joint limits and tool frames.

Exit condition: the chosen driver can read state and cancel motion reliably.

## Phase 1: Robot model and simulation

- Build URDF/Xacro and SRDF.
- Configure MoveIt 2 and `ros2_control`.
- Validate joint names, directions, limits, and collision geometry.
- Plan and execute simple trajectories in fake hardware/simulation.

## Phase 2: Hardware integration

- Implement or integrate the FANUC driver adapter.
- Verify state frequency, latency, trajectory execution, cancellation, faults,
  and reconnection.
- Keep speed reduced and workspace empty.

## Phase 3: Perception

- Publish calibrated images and point clouds.
- Detect and track fruit, stem, leaves, and obstacles.
- Estimate 6D target poses with uncertainty.
- Build hand-eye calibration and repeatability tests.

## Phase 4: Harvest planning

- Generate candidate approach and harvest poses.
- Add collision-aware staged planning.
- Add constrained final approach and retreat.
- Test unreachable, occluded, and stale targets.

## Phase 5: End-effector control

- Add gripper/cutter I/O.
- Detect grasp/cut success.
- Define safe tool states during motion and faults.

## Phase 6: Task autonomy

- Implement the harvesting behavior tree/state machine.
- Add retry limits and recovery.
- Add rosbag logging and per-fruit result records.

## Phase 7: VLA/LLM

- Add structured task proposals.
- Start with offline log analysis and target ranking.
- Require schema validation and policy/safety approval.
- Compare against a deterministic baseline before enabling online use.

## Phase 8: Digital twin and maintenance

- Reuse the production URDF, limits, controller parameters, and sensor frames.
- Calibrate simulated inertia, friction, backlash, delay, payload, and tool behavior.
- Record synchronized commands, states, temperatures, currents, faults, cycle
  counts, image-quality metrics, network latency, and maintenance events.
- Establish healthy baselines before training anomaly or RUL models.
- Inject sensor, network, actuator, and calibration drift faults in simulation.
- Validate predictions against real maintenance records.

## Phase 9: MPC

- Start with a joint-space linear model and reference governor in simulation.
- Identify delay and dynamics from reduced-speed experiments.
- Add state estimation and disturbance modeling.
- Add nonlinear kinematics and collision constraints only after timing is measured.
- Compare tracking, solve time, constraint violations, and stop behavior against
  the standard trajectory-controller baseline.
- Deploy only through the safety gate and a standard controller interface.

## Suggested first milestone

In simulation:

1. load the FANUC model
2. publish one synthetic fruit pose
3. plan to a pre-grasp pose
4. execute approach and retreat
5. record the result

Do this before integrating YOLO, a real camera, or an LLM.
