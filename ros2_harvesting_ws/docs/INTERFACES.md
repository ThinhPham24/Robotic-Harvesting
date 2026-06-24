# ROS Interface Contract

Create the concrete definitions in `harvest_interfaces` before implementing
nodes. Names below are proposed contracts.

## Topics

| Name | Suggested type | Publisher | Consumer |
|---|---|---|---|
| `/harvest/perception/targets` | `HarvestTargetArray` | perception | task manager |
| `/harvest/perception/scene_entities` | `SceneEntityArray` | perception | Unity/planning scene |
| `/harvest/perception/scene_cloud` | `sensor_msgs/PointCloud2` | perception | MoveIt scene updater |
| `/harvest/selected_target` | `HarvestTarget` | task manager | planner/UI |
| `/harvest/system_state` | `SystemState` | task manager | UI/logger/AI |
| `/harvest/safety/status` | `SafetyStatus` | safety | all managers |
| `/harvest/tool/state` | `ToolState` | control | task manager |
| `/harvest/ai/proposal` | `TaskProposal` | AI manager | task manager |
| `/harvest/twin/residual` | `TwinResidual` | digital twin | condition monitor |
| `/harvest/health/state` | `AssetHealth` | condition monitor | maintenance/UI |
| `/harvest/maintenance/prediction` | `MaintenancePrediction` | maintenance | UI/task manager |
| `/harvest/mpc/reference` | `sensor_msgs/JointState` | MPC | safety/reference adapter |

## Actions

| Name | Purpose |
|---|---|
| `/harvest/execute_harvest` | Execute one supervised harvesting workflow |
| `/harvest/plan_to_target` | Produce a collision-checked staged plan |
| `/harvest/tool/command` | Open, close, cut, release, or stop tool |
| `/harvest/acquire_scene` | Capture and process a fresh observation |

## Services

| Name | Purpose |
|---|---|
| `/harvest/safety/request_enable` | Request execution permission |
| `/harvest/safety/stop` | Stop and invalidate current permission |
| `/harvest/validate_target` | Check age, confidence, workspace, and reachability |
| `/harvest/reset_fault` | Controlled recovery request |

## Message design rules

- Use SI units: metres, radians, seconds, newtons.
- Every spatial message includes `std_msgs/Header`.
- Never infer a TF frame from a topic name.
- Use UUIDs or stable IDs for tracked fruit.
- Include confidence and observation age.
- Actions must support cancellation.
- Errors use machine-readable codes plus human-readable text.
- AI proposals contain symbolic goals, never actuator commands.
