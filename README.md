# External Vision-Assisted Multi-Robot Navigation
## ROS2 Humble Simulation Package

**Architecture:** Ceiling-mounted overhead camera → AprilTag detection → Robot pose → Nav2 navigation  
**No LIDAR. No AMCL. No onboard sensors.** Pure external vision localization.

---

## 1. Workspace Structure

```
sim_ws/
└── src/
    ├── robot_description/          # URDF/Xacro AMR robot with AprilTag
    │   ├── urdf/robot.urdf.xacro
    │   ├── meshes/apriltag_36h11_id0.png
    │   └── launch/display.launch.py
    │
    ├── simulation_pkg/             # Gazebo world, maps, sim launch
    │   ├── worlds/shopfloor.world
    │   ├── maps/shopfloor.pgm  +  shopfloor.yaml
    │   └── launch/simulation.launch.py
    │
    ├── apriltag_detection_pkg/     # Overhead camera → pose estimation
    │   ├── apriltag_detection_pkg/apriltag_localizer.py
    │   ├── config/apriltag_config.yaml
    │   └── launch/apriltag.launch.py
    │
    ├── camera_localization_pkg/    # Pose → TF + /odom relay (replaces AMCL)
    │   ├── camera_localization_pkg/pose_relay.py
    │   └── launch/
    │
    ├── nav2_config/                # Nav2 YAML params (no AMCL)
    │   ├── config/nav2_params.yaml
    │   └── launch/nav2.launch.py
    │
    └── bringup_pkg/                # Master launch + RViz config
        ├── launch/bringup.launch.py
        ├── launch/send_goal.py
        └── rviz/nav2_view.rviz
```

---

## 2. System Requirements

- **Ubuntu 22.04 LTS**
- **ROS2 Humble Hawksbill**
- **Gazebo Classic 11**
- Python 3.10+

---

## 3. Install Dependencies

### 3.1 ROS2 Humble (if not installed)
```bash
# Follow official docs: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
sudo apt install ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.2 Gazebo + ROS2 bridge
```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros \
  gazebo11
```

### 3.3 Nav2 full stack
```bash
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-common \
  ros-humble-nav2-core \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-controller \
  ros-humble-nav2-planner \
  ros-humble-nav2-behaviors \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-navfn-planner \
  ros-humble-dwb-core \
  ros-humble-dwb-plugins \
  ros-humble-nav2-smoother \
  ros-humble-nav2-waypoint-follower \
  ros-humble-nav2-velocity-smoother \
  ros-humble-nav2-msgs
```

### 3.4 Robot description tools
```bash
sudo apt install -y \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools
```

### 3.5 Vision / AprilTag dependencies
```bash
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  python3-opencv

# Option A: pupil-apriltags (recommended, faster)
pip3 install pupil-apriltags

# Option B: python-opencv ArUco fallback (no extra install needed)
# The node auto-detects which library is available
```

### 3.6 Python utilities
```bash
pip3 install scipy Pillow
```

---

## 4. Build the Workspace

```bash
cd ~/sim_ws

# Install any missing ROS deps
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Add to ~/.bashrc for convenience
echo "source ~/sim_ws/install/setup.bash" >> ~/.bashrc
```

---

## 5. Regenerate the Map (optional)

The PGM map is already included. To regenerate from scratch:

```bash
cd ~/sim_ws/src/simulation_pkg/maps
python3 generate_map.py
# Outputs: shopfloor.pgm (120×120 pixels, 0.05 m/px resolution)
```

---

## 6. Launch the Simulation

### Option A: All-in-one launch (recommended)

```bash
source ~/sim_ws/install/setup.bash
ros2 launch bringup_pkg bringup.launch.py
```

This starts: Gazebo → Robot spawn → AprilTag localizer → Pose relay → Map server → Nav2 → RViz

### Option B: Step-by-step (for debugging)

**Terminal 1 – Gazebo simulation:**
```bash
source ~/sim_ws/install/setup.bash
ros2 launch simulation_pkg simulation.launch.py
```

Wait until you see `Spawn status: SpawnEntity: Successfully spawned entity [amr_robot]`

**Terminal 2 – AprilTag localization:**
```bash
source ~/sim_ws/install/setup.bash
ros2 launch apriltag_detection_pkg apriltag.launch.py
```

Verify pose is being published:
```bash
ros2 topic echo /robot_pose --once
```

**Terminal 3 – Map server + Nav2:**
```bash
source ~/sim_ws/install/setup.bash
ros2 launch nav2_config nav2.launch.py
```

**Terminal 4 – RViz:**
```bash
source ~/sim_ws/install/setup.bash
rviz2 -d ~/sim_ws/src/bringup_pkg/rviz/nav2_view.rviz
```

---

## 7. Send Navigation Goals

### Method A: RViz 2D Goal Pose (interactive)
1. In RViz, click the **"Nav2 Goal"** button in the toolbar (arrow icon)
2. Click and drag anywhere on the map to set position + orientation
3. Robot will plan and navigate automatically

### Method B: CLI goal sender
```bash
source ~/sim_ws/install/setup.bash

# Navigate to (4.0, 2.0) facing east
python3 ~/sim_ws/src/bringup_pkg/launch/send_goal.py --x 4.0 --y 2.0 --yaw 0.0

# Navigate to (1.5, 1.5) facing north
python3 ~/sim_ws/src/bringup_pkg/launch/send_goal.py --x 1.5 --y 1.5 --yaw 1.57

# Navigate near top-right area (avoiding tall obstacle)
python3 ~/sim_ws/src/bringup_pkg/launch/send_goal.py --x 3.5 --y 2.5 --yaw 3.14
```

### Method C: ros2 topic publish
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

## 8. Verify Everything is Working

### Check TF tree
```bash
ros2 run tf2_tools view_frames
# Should show: map → odom → base_footprint → base_link
```

### Check active topics
```bash
ros2 topic list
# Required topics:
# /overhead_camera/image_raw
# /robot_pose
# /odom
# /cmd_vel
# /map
# /plan
```

### Check Nav2 node status
```bash
ros2 node list | grep nav2
```

### Monitor camera detection
```bash
ros2 topic hz /robot_pose
# Should show ~30 Hz when tag is visible
```

### View overhead camera feed
```bash
ros2 run rqt_image_view rqt_image_view /overhead_camera/image_raw
```

---

## 9. System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                        GAZEBO SIMULATION                         │
│                                                                  │
│   ┌──────────────────┐          ┌─────────────────────────┐     │
│   │  Overhead Camera  │          │      AMR Robot           │     │
│   │  (2.55m height)   │          │  ┌─────────────────┐    │     │
│   │  FOV: 110°        │──────────│  │  AprilTag ID=0  │    │     │
│   │  1280×720 @ 30fps │ image    │  │  (on top)       │    │     │
│   └──────────────────┘          │  └─────────────────┘    │     │
│                                 │  Diff-drive (cmd_vel)    │     │
│                                 └─────────────────────────┘     │
└─────────────────────────────────────────────────────────────────┘
         │
         │  /overhead_camera/image_raw
         ▼
┌─────────────────────┐
│  apriltag_localizer │  (apriltag_detection_pkg)
│  ┌────────────────┐ │
│  │ Detect tag ID0 │ │
│  │ Estimate pose  │ │
│  │ (x, y, theta)  │ │
│  └────────────────┘ │
└─────────────────────┘
         │
         │  /robot_pose  (PoseStamped in map frame)
         ▼
┌─────────────────────┐
│    pose_relay       │  (camera_localization_pkg)
│  ┌────────────────┐ │
│  │ TF broadcaster │ │  ──►  map → odom → base_footprint
│  │ /odom publisher│ │  ──►  /odom (Odometry)
│  └────────────────┘ │
└─────────────────────┘
         │
         │  TF tree + /odom
         ▼
┌─────────────────────────────────────────────────────┐
│                    NAV2 STACK                        │
│                                                      │
│  map_server ──► /map                                 │
│  planner_server ──► global path planning             │
│  controller_server ──► DWB local planner             │
│  bt_navigator ──► behaviour trees                    │
│                                                      │
│  Output: /cmd_vel ──► Robot moves!                   │
└─────────────────────────────────────────────────────┘
```

---

## 10. Troubleshooting

### Problem: Robot not detected / no /robot_pose published

**Check 1:** Is the camera publishing?
```bash
ros2 topic hz /overhead_camera/image_raw
# Should be ~30 Hz
```

**Check 2:** Is the tag visible in the image?
```bash
ros2 run rqt_image_view rqt_image_view /overhead_camera/image_raw
# You should see the robot with a white square (AprilTag) on top
```

**Check 3:** AprilTag library installed?
```bash
python3 -c "import apriltag; print('pupil-apriltags OK')"
# OR
python3 -c "import cv2; print(cv2.__version__)"
```

**Fix:** If pupil-apriltags not found, the node falls back to cv2.aruco automatically.

---

### Problem: TF tree broken / Nav2 fails to start

**Check TF:**
```bash
ros2 run tf2_ros tf2_echo map base_link
```

**Expected output:**
```
At time 0.0
- Translation: [x, y, 0.045]
- Rotation: [0, 0, sin(θ/2), cos(θ/2)]
```

**If map→odom is missing:** The pose_relay node is not running. Check:
```bash
ros2 node list | grep pose_relay
```

---

### Problem: Nav2 keeps printing "Waiting for costmap to become valid"

The global costmap depends on the map. Ensure map_server is active:
```bash
ros2 lifecycle list map_server
# Should show: map_server [active]
```

Force activate:
```bash
ros2 lifecycle set map_server activate
```

---

### Problem: Robot spins in place / oscillates

Reduce max angular velocity in `nav2_params.yaml`:
```yaml
FollowPath:
  max_vel_theta: 0.5    # reduce from 1.0
  RotateToGoal.scale: 16.0  # reduce from 32.0
```

---

### Problem: Gazebo very slow / low FPS

Add `--verbose` to see errors, or reduce camera resolution in `shopfloor.world`:
```xml
<width>640</width>
<height>480</height>
```

---

### Problem: "No module named 'apriltag_detection_pkg'"

Rebuild and resource:
```bash
cd ~/sim_ws
colcon build --packages-select apriltag_detection_pkg
source install/setup.bash
```

---

## 11. Customisation Guide

### Change robot spawn position
```bash
ros2 launch simulation_pkg simulation.launch.py robot_x:=2.0 robot_y:=2.0 robot_yaw:=1.57
```

### Use a different camera FOV
Edit `shopfloor.world`, change `<horizontal_fov>`:
- 90°  = 1.5708 radians
- 110° = 1.9199 radians  ← current
- 120° = 2.0944 radians
- 160° = 2.7925 radians

### Change navigation speed limits
Edit `nav2_params.yaml`:
```yaml
FollowPath:
  max_vel_x: 0.6      # increase for faster navigation
  max_vel_theta: 1.5  # increase for faster turning
```

### Add second robot
1. Spawn with unique entity name and namespace:
```bash
ros2 run gazebo_ros spawn_entity.py -topic robot_description \
  -entity amr_robot_2 -x 5.0 -y 1.0 -z 0.05
```
2. Run a second apriltag_localizer with `tag_id: 1` in a namespace `/robot2`
3. The AprilTag on robot 2 should be tag ID 1 (tag36h11 family)

---

## 12. Key ROS2 Topics Reference

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/overhead_camera/image_raw` | `sensor_msgs/Image` | Camera → | Raw camera image |
| `/overhead_camera/camera_info` | `sensor_msgs/CameraInfo` | Camera → | Camera intrinsics |
| `/robot_pose` | `geometry_msgs/PoseStamped` | localizer → | Robot pose in map frame |
| `/odom` | `nav_msgs/Odometry` | relay → | Odometry for Nav2 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | localizer → | Initial pose for Nav2 |
| `/map` | `nav_msgs/OccupancyGrid` | map_server → | Static map |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 → robot | Velocity commands |
| `/plan` | `nav_msgs/Path` | Nav2 → | Global planned path |
| `/local_plan` | `nav_msgs/Path` | Nav2 → | Local planned path |
| `/goal_pose` | `geometry_msgs/PoseStamped` | user → Nav2 | Navigation goal |

---

## 13. Extension to Real Hardware

When moving from simulation to real ESP32-CAM hardware:

1. **Replace** the Gazebo camera topic with your real camera's ROS2 image topic
2. **Calibrate** the camera and update intrinsics via `/camera_info`
3. **Update** `camera_x`, `camera_y`, `camera_z` in `apriltag_config.yaml` to match real mounting position
4. **Print** a physical AprilTag36h11 ID=0 at 10cm size and attach to robot top
5. **Set** `use_sim_time: false` in all launch files for real hardware

```bash
# Real hardware launch (no Gazebo)
ros2 launch apriltag_detection_pkg apriltag.launch.py use_sim_time:=false
ros2 launch nav2_config nav2.launch.py use_sim_time:=false
```
