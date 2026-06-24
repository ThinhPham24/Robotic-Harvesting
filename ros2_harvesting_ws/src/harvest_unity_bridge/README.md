# harvest_unity_bridge

Integration contract for the existing Unity3D digital twin.

Unity subscribes to authoritative ROS state and scene topics and publishes only
namespaced simulated data under `/harvest/twin`.

Recommended Unity components:

```text
RosConnectionManager
RobotJointStateSubscriber
TfSceneSynchronizer
SemanticSceneSubscriber
HealthOverlaySubscriber
MpcPredictionSubscriber
TwinSensorPublisher
BridgeHeartbeatMonitor
```

Generate C# classes for `harvest_interfaces` using the ROS-TCP-Connector message
generation tool. Centralize ROS/Unity coordinate conversion.

See `docs/UNITY_INTEGRATION.md`.
