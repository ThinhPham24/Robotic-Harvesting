# Unity Robotic Harvesting Digital Twin

Standalone Unity project for the FANUC robotic-harvesting digital twin and
operator dashboard. This project is separate from `Scan_app`.

## Recommended editor

- Ubuntu 22.04 LTS
- Unity 2022.3 LTS or newer
- UI Toolkit
- Universal Render Pipeline may be added for the final 3D scene

## Open and run

1. Open `Unity_Digital_Twin` from Unity Hub.
2. Open any scene or use the default empty scene.
3. Press Play.

`AppBootstrapper` automatically creates the runtime UI. It currently uses demo
telemetry so the interface can be developed before ROS is connected.

The UI demo does not require ROS. ROS integration targets ROS 2 Humble on
Ubuntu 22.04 and is a separate setup step.

## Main code

- `Assets/HarvestingUI/Scripts/AppBootstrapper.cs`
- `Assets/HarvestingUI/Scripts/DashboardController.cs`
- `Assets/HarvestingUI/Resources/UI/HarvestDashboard.uxml`
- `Assets/HarvestingUI/Resources/UI/HarvestDashboard.uss`

## ROS integration

`RosStateAdapter.cs` defines the boundary between Unity and ROS. Replace the
demo source with an adapter based on Unity ROS-TCP-Connector after generating
the custom ROS message classes.

Unity should subscribe to authoritative robot/perception state. Motion and tool
commands remain disabled until an explicitly reviewed command workflow exists.

See `../ros2_harvesting_ws/docs/UBUNTU_22_04_SETUP.md` for the exact commands.

## Design reference

`Assets/HarvestingUI/Design/robotic_harvesting_dashboard_concept.png`
