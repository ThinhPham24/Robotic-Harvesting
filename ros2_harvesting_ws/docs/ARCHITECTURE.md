# System Architecture

## 1. Hardware and driver layer

`fanuc_hardware_interface` isolates all FANUC-specific communication.

Required outputs:

- `/joint_states`
- robot mode and fault state
- controller heartbeat
- digital and analog I/O state

Required command path:

- FollowJointTrajectory-compatible execution, preferably through
  `joint_trajectory_controller`
- explicit enable, stop, cancel, reset, and recovery operations

Keep controller communication out of perception and planning packages.

## 2. Robot model and transforms

`harvest_description` owns:

- FANUC URDF/Xacro
- joint limits
- tool and gripper geometry
- camera mounts
- collision geometry
- SRDF planning groups

Minimum TF tree:

```text
world
└── robot_base
    └── ... robot joints ...
        └── tool0
            ├── end_effector
            └── wrist_camera_optical_frame
```

Fixed external cameras should be calibrated into `world` or `robot_base`.
Object poses must always include a timestamp and frame ID.

## 3. Perception

`harvest_perception` converts sensor data into a planning scene and harvest
targets.

Pipeline:

```text
RGB/depth/stereo
  -> synchronization and calibration
  -> fruit/stem/leaf segmentation
  -> depth or stereo reconstruction
  -> 3D fruit and stem pose estimation
  -> temporal tracking and confidence filtering
  -> planning-scene obstacles and HarvestTarget messages
```

Separate detection from pose estimation. A 2D box or mask is not a robot pose.
Each target should contain:

- stable target ID
- fruit pose
- approach direction
- estimated stem/cut pose when applicable
- dimensions
- confidence
- freshness timestamp
- reachability status

The existing `Scan_app` should initially be integrated through saved PLY/images
or a small ROS bridge. Avoid importing its PyQt GUI into ROS nodes.

## 4. Motion planning

`harvest_motion_planning` owns MoveIt 2 integration.

Recommended stages:

1. target validation
2. candidate grasp/cut pose generation
3. inverse-kinematics filtering
4. collision checking
5. pre-grasp plan
6. constrained Cartesian approach
7. harvesting action
8. retreat
9. placement into collection container

Use MoveIt Task Constructor when the workflow needs multiple dependent stages.
Use Servo only for bounded visual correction near the target. Global motion
should remain trajectory planned.

## 5. Control

`harvest_control` contains deterministic execution:

- arm trajectory controller configuration
- gripper/cutter controller
- tool I/O adapter
- optional force or compliance controller
- trajectory monitoring

Planning decides where to move. Controllers decide how to track an accepted
reference. AI components do neither at the low level.

## 6. Task manager

`harvest_task_manager` is the application coordinator. A state-machine or
behavior-tree implementation is appropriate.

```text
IDLE
 -> ACQUIRE_SCENE
 -> SELECT_TARGET
 -> VALIDATE_TARGET
 -> PLAN_APPROACH
 -> MOVE_PREGRASP
 -> VISUAL_REFINE
 -> HARVEST
 -> VERIFY_PICK
 -> RETREAT
 -> PLACE
 -> RECORD_RESULT
 -> IDLE
```

Every movement state needs timeout, cancellation, failure classification, and
a bounded recovery path.

## 7. VLA/LLM manager

`harvest_ai_manager` is optional and asynchronous.

Good responsibilities:

- convert operator language into a structured task request
- rank valid targets using semantic criteria
- explain failures
- suggest recovery from an approved action set
- analyze logs and performance

Forbidden responsibilities:

- raw joint velocity or torque publication
- safety-state modification
- collision-check bypass
- direct controller calls
- arbitrary shell or network actions during robot operation

AI output should be schema validated, converted into a finite command set, and
approved by `harvest_task_manager` and `harvest_safety`.

## 8. Bringup

`harvest_bringup` owns launch composition and environment-specific parameters.

Recommended launch modes:

- `simulation.launch.py`
- `fake_hardware.launch.py`
- `perception_only.launch.py`
- `planning_demo.launch.py`
- `hardware_bringup.launch.py`
- `full_harvest.launch.py`

Hardware launch must default to motion disabled.

## 9. Digital twin and asset health

The operational twin receives the same joint references and environment state
as the real system, then compares predicted and measured behavior:

```text
commands + environment + payload
              |
              v
       simulation/model twin
              |
       predicted telemetry
              |
              v
real telemetry -> residual monitor -> condition monitor
                                      |
                                      v
                              maintenance predictor
```

The twin must model the complete harvesting cell, not only the arm:

- FANUC arm and controller communication
- end effector and cutter
- cameras and illumination
- turntable and scanner
- network and compute health
- plants, fruit, supports, collection bin, and collision geometry

Use Gazebo for commissioning and fault injection. Use measured telemetry and an
identified reduced-order model for online prediction. A visually accurate
simulation without calibrated dynamics is not a maintenance model.

## 10. MPC

MPC belongs between a validated task-space/joint reference and the deterministic
controller. Its initial role is reference governance and bounded local tracking.

```text
MoveIt trajectory / visual target
             |
             v
      state estimator
             |
             v
    MPC reference generator
             |
      safety validation
             |
             v
 ros2_control / FANUC controller
```

The MPC optimization should include:

- joint position, velocity, and acceleration limits
- Cartesian workspace limits
- collision-distance constraints
- tracking and control-effort costs
- input-rate penalties
- optional force/contact constraints near the fruit

The production solver should execute in C++ or generated C code with measured
worst-case solve time. Python is suitable for model development and offline
validation, not the final hardware control loop.
