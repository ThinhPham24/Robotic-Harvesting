# Unity Digital Twin Integration

## Authority model

Unity is responsible for:

- rendering the robot, cell, plants, fruit, sensors, and health overlays
- operator monitoring and replay
- synthetic-camera generation and fault scenarios
- optional physics experiments

ROS 2 remains authoritative for:

- robot timestamps and joint state
- TF and calibrated coordinate frames
- perception output and object identity
- planning, MPC, safety, and controller commands
- maintenance records and health decisions

Unity must not become a second uncontrolled command source.

## Data flow

```text
real cameras/sensors
        |
        v
ROS perception and sensor fusion
        |
        +--> /harvest/perception/scene_entities
        +--> /harvest/perception/scene_cloud
        +--> /tf
        |
        v
Unity scene updater
        |
        +--> robot and scene visualization
        +--> health and maintenance overlays
        +--> predicted MPC trajectory
        |
synthetic observations / twin state
        |
        v
/harvest/twin/* topics
```

Do not send full point clouds or camera frames through a low-rate UI channel
unless required. Use:

- reliable, moderate-rate messages for joint state, health, and scene entities
- sensor-data QoS or a dedicated stream for images and point clouds
- mesh/asset caching for static geometry
- sequence IDs and timestamps for dynamic scene updates

## Perception-driven scene reconstruction

The sensor pipeline should produce a fused world model before Unity consumes it:

1. calibrate camera intrinsics and camera-to-robot extrinsics
2. synchronize image, depth/stereo, robot state, and tool state
3. detect fruit, stem, leaves, supports, people, and obstacles
4. estimate 3D poses and uncertainty
5. transform all entities into the configured `world` frame
6. track entities over time with stable IDs
7. publish semantic entities and collision geometry
8. update or remove Unity GameObjects by stable ID

Unity should not estimate robot-world transforms independently from ROS.

## Coordinate conversion

ROS commonly uses right-handed FLU coordinates. Unity uses a left-handed
coordinate system. Use the ROS-TCP-Connector geometry conversion utilities or
one centrally tested conversion layer. Never scatter manual sign swaps across
multiple scripts.

Test at least:

- robot base origin
- positive X/Y/Z axes
- one asymmetric joint pose
- camera optical frame
- one known fruit point measured in the real cell

## Unity object lifecycle

Maintain a dictionary keyed by `SceneEntity.id`.

```text
new ID       -> instantiate prefab
existing ID  -> filtered pose/size update
missing ID   -> mark stale, then remove after timeout
low confidence -> render warning/transparent state
collision_enabled -> update planning/twin collision representation
```

Do not destroy objects after one missed frame. Use observation age and a
configurable stale timeout.

## Topics consumed by Unity

- `/joint_states`
- `/tf` and `/tf_static`
- `/harvest/perception/scene_entities`
- `/harvest/health/state`
- `/harvest/maintenance/prediction`
- `/harvest/mpc/reference`
- `/harvest/mpc/status`
- `/harvest/system_state`

## Topics optionally produced by Unity

- `/harvest/twin/joint_states`
- `/harvest/twin/synthetic_camera/*`
- `/harvest/twin/contact_events`
- `/harvest/twin/fault_injection/events`

Unity-originated topics must be namespaced under `/harvest/twin` unless an
explicitly reviewed bridge promotes them.

## Networking

For the Unity ROS-TCP approach:

- install Unity's ROS-TCP-Connector package
- run a compatible ROS-TCP-Endpoint on the ROS machine
- pin reviewed versions instead of tracking a moving branch
- isolate the robot control network from general office/cloud traffic
- monitor bridge heartbeat, queue growth, message age, and reconnect events

The Unity bridge is not suitable as a safety-rated or hard-real-time control
transport.
