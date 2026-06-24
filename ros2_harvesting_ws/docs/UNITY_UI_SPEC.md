# Unity UI Specification

Visual reference: `ui/robotic_harvesting_dashboard_concept-v2.png`

## Screen hierarchy

```text
AppShell
├── TopStatusBar
├── LeftNavigation
├── MainWorkspace
│   ├── DigitalTwinViewport
│   └── ContextInspector
└── BottomTelemetryDrawer
```

## Primary screens

- Overview
- Digital Twin
- Perception
- Mission
- Motion and MPC
- Health
- Predictive Maintenance
- Logs
- Settings

## Digital Twin screen

The main viewport should retain the largest area. It displays:

- real-time robot joint state
- semantic entities reconstructed by perception
- selected fruit and grasp/cut pose
- uncertainty volume
- safety zones and collision objects
- planned and MPC-predicted trajectories
- stale-data and communication warnings

The viewport is observational by default. Motion execution requires a separate,
explicit mission workflow and safety permission.

## Persistent top bar

Always display:

- ROS bridge state and data age
- robot controller mode
- safety state
- active mission
- operator
- UTC/system time
- alarm count

## Context inspector

The right panel changes with selection:

- target identity, class, confidence, age, and pose
- current behavior-tree/task state
- plan validity and predicted duration
- tool state
- approved actions such as plan, hold, cancel, or acknowledge

Avoid raw joint-command sliders in the operational UI.

## Telemetry and maintenance

The bottom drawer contains synchronized plots:

- joint tracking residual
- motor load/current
- MPC solve time and prediction error
- camera and perception latency
- network latency
- tool force/current

The maintenance panel shows health score, active indicators, confidence,
inspection recommendation, and evidence links. Do not present RUL as precise
when the model reports insufficient history.

## Visual language

- Background: `#0B1220`
- Primary panel: `#111C2E`
- Secondary panel: `#17243A`
- Border: `#2A3A52`
- Primary text: `#E6EDF7`
- Secondary text: `#91A4BE`
- Information: `#38BDF8`
- Healthy: `#22C55E`
- Warning: `#F59E0B`
- Critical: `#EF4444`
- Robot/accent: `#F4C430`

Use green only for confirmed healthy/ready states, amber for degraded states,
and red only for faults, stops, or unsafe conditions.

## Unity implementation

Use UI Toolkit rather than world-space Canvas for the main desktop HMI:

```text
Assets/HarvestingUI/
├── UXML/
├── USS/
├── Scripts/
│   ├── ViewModels/
│   ├── RosSubscribers/
│   ├── Widgets/
│   └── Navigation/
├── Icons/
├── Fonts/
└── Themes/
```

Apply a view-model layer between ROS messages and UI elements. ROS callbacks
must not directly manipulate arbitrary visual elements; normalize state,
timestamps, quality, and alarms first.
