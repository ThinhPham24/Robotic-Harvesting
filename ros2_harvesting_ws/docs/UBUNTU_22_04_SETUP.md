# Ubuntu 22.04 Setup and Run Guide

This project targets Ubuntu 22.04 LTS with ROS 2 Humble.

## What can run now

| Component | Current state |
|---|---|
| Unity dashboard | Runs with built-in demo telemetry |
| ROS custom interfaces | Implemented |
| Digital-twin residual monitor | Implemented |
| Condition monitor | Implemented |
| Maintenance-risk baseline | Implemented |
| MPC shadow reference | Implemented |
| Unity-to-ROS transport | Adapter boundary only |
| FANUC hardware motion | Not implemented; robot/controller details required |
| Real 3D robot scene | Import your existing Unity models into the viewport |

## 1. Install ROS 2 Humble

Open a terminal:

```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y curl
```

Install the official ROS apt-source package:

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F "tag_name" | awk -F'"' '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
```

Install ROS, build tools, MoveIt, and ros2_control:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

Initialize rosdep once:

```bash
sudo rosdep init
rosdep update
```

If `sudo rosdep init` reports that the sources file already exists, continue
with `rosdep update`.

Source ROS:

```bash
source /opt/ros/humble/setup.bash
```

Optional automatic sourcing:

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

## 2. Build this workspace

```bash
cd /home/airlab/Desktop/Robotic-Harvesting/ros2_harvesting_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

Confirm that packages are visible:

```bash
ros2 pkg list | grep harvest
```

## 3. Run the monitoring pipeline

Terminal 1:

```bash
cd /home/airlab/Desktop/Robotic-Harvesting/ros2_harvesting_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch harvest_bringup health_monitoring.launch.py
```

The nodes will wait for these inputs:

```text
/joint_states
/harvest/twin/joint_states
```

Without both topics, no twin residual or health output is expected.

Inspect outputs in Terminal 2:

```bash
cd /home/airlab/Desktop/Robotic-Harvesting/ros2_harvesting_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /harvest/twin/residual
```

Other output topics:

```bash
ros2 topic echo /harvest/health/state
ros2 topic echo /harvest/maintenance/prediction
```

## 4. Run the MPC prototype

Terminal 1:

```bash
cd /home/airlab/Desktop/Robotic-Harvesting/ros2_harvesting_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch harvest_bringup mpc_shadow.launch.py
```

The MPC node requires:

```text
/joint_states
/harvest/mpc/target
```

Publish a six-joint example state in Terminal 2:

```bash
ros2 topic pub -r 20 /joint_states sensor_msgs/msg/JointState \
"{header: {frame_id: 'robot_base'}, name: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'], position: [0.0,0.0,0.0,0.0,0.0,0.0], velocity: [0.0,0.0,0.0,0.0,0.0,0.0]}"
```

Publish a target in Terminal 3:

```bash
ros2 topic pub -r 2 /harvest/mpc/target sensor_msgs/msg/JointState \
"{header: {frame_id: 'robot_base'}, name: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'], position: [0.20,-0.30,0.25,0.0,0.15,0.0]}"
```

Inspect the shadow output in Terminal 4:

```bash
ros2 topic echo /harvest/mpc/reference
```

And solver status:

```bash
ros2 topic echo /harvest/mpc/status
```

This output is not authorized for the physical FANUC robot.

## 5. Run the Unity dashboard

Install Unity Hub for Linux, then install Unity Editor 2022.3 LTS with Linux
Build Support.

In Unity Hub:

1. Select **Add/Open project**.
2. Choose:

   ```text
   /home/airlab/Desktop/Robotic-Harvesting/Unity_Digital_Twin
   ```

3. Allow package import and script compilation to finish.
4. Open or create an empty 3D scene.
5. Press **Play**.

No GameObject setup is required. `AppBootstrapper.cs` creates the dashboard
automatically. The dashboard uses `DemoRobotStateSource.cs`.

Main Unity files:

```text
Unity_Digital_Twin/Assets/HarvestingUI/Scripts/AppBootstrapper.cs
Unity_Digital_Twin/Assets/HarvestingUI/Scripts/DashboardController.cs
Unity_Digital_Twin/Assets/HarvestingUI/Resources/UI/HarvestDashboard.uxml
Unity_Digital_Twin/Assets/HarvestingUI/Resources/UI/HarvestDashboard.uss
```

## 6. Put your 3D scene behind the UI

In your Unity scene:

1. Import the FANUC arm, tool, camera, greenhouse, and plant models.
2. Keep the main camera rendering the 3D cell.
3. Press Play; the dashboard overlays the rendered scene.
4. Replace the placeholder viewport area with a camera `RenderTexture` in the
   next implementation step.
5. Match Unity joint object names to ROS joint names.

Do not manually estimate ROS-to-Unity coordinate transforms in each script.
Use one tested conversion layer.

## 7. Add Unity ROS-TCP-Connector

In Unity:

1. Open **Window → Package Manager**.
2. Select **+ → Add package from git URL**.
3. Enter:

   ```text
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   ```

The official repository currently advertises ROS 2 Foxy/Galactic rather than
Humble. Treat Humble support as unverified. Pin a tested version/commit and
validate message generation, reconnect behavior, and load before deployment.

`RosStateAdapter.cs` is intentionally incomplete until the connector and
generated message classes are available.

## 8. Current run order

For UI work only:

```text
Open Unity_Digital_Twin in Unity Hub → press Play
```

For ROS monitoring:

```text
build workspace → launch health_monitoring.launch.py
```

For MPC development:

```text
launch mpc_shadow.launch.py → publish test state and target
```

For a connected Unity/ROS twin:

```text
install and validate ROS-TCP transport → implement RosStateAdapter.cs
```

For FANUC operation:

```text
select and validate the exact FANUC driver → add safety and hardware testing
```

Do not connect MPC or Unity UI outputs directly to FANUC motion commands.
