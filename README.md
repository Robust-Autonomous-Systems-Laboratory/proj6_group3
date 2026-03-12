# Project 6: Naive Mapping by Waypoints

**EE5531 Introduction to Robotics**

**Team Members:** Anders Smitterberg, Victor Ameh

---

## 1. Navigation Strategy Summary (5 pts)

### Environment Description



### Environment Sketch



### Waypoint Summary


---

## 2. System Architecture (5 pts)

### Data Flow Diagram


### EKF Configuration Summary


### Project 5 Sensor Characterization Integration


---

## 3. Map Accuracy Results (15 pts)

### Distance Accuracy Table


### Orientation Assessment

**Waypoint 1:**
- Expected landmark direction:
- Observed in map:
- Rotational misalignment:

**Waypoint 2:**
- Expected landmark direction:
- Observed in map:
- Rotational misalignment:

**Waypoint 3:**
- Expected landmark direction:
- Observed in map:
- Rotational misalignment:

**Waypoint 4:**
- Expected landmark direction:
- Observed in map:
- Rotational misalignment:

**Waypoint 5:**
- Expected landmark direction:
- Observed in map:
- Rotational misalignment:

### RViz Screenshots

**Individual scan captures at each waypoint:**

**Measurement tool usage:**

**Overall map with all scans visualized:**

---

## 4. Discussion (10 pts)

### Mapping Accuracy Analysis

### Sources of Error

- **Localization drift:**
- **Measurement uncertainty:** 
- **Sensor limitations:** 

### Map Consistency Assessment


### Recommendations for Improvement

---

## 5. Usage Instructions (5 pts)

### Clone and Build

This repository is the `src/` directory of a colcon workspace. Set it up like this:

```bash
mkdir -p proj6_ws
cd proj6_ws
git clone https://github.com/Robust-Autonomous-Systems-Laboratory/proj6_group3 src
source /opt/ros/jazzy/setup.bash
colcon build
```

### Terminal Setup (every new terminal)

```bash
source /opt/ros/jazzy/setup.bash
source src/turtlebot_connect.sh
source install/setup.bash
```

`turtlebot_connect.sh` sets `TURTLEBOT3_MODEL=burger`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, and `ROS_DOMAIN_ID=7`.

### Launch the TurtleBot3 Bringup

On the robot (SSH):
```bash
source /opt/ros/jazzy/setup.bash
source turtlebot_connect.sh
ros2 launch turtlebot3_bringup robot.launch.py
```

### Launch Localization Node

Runs the EKF, fusing wheel encoders (`/joint_states`) and IMU (`/imu`).
Publishes pose to `/localization/pose` and path to `/localization/path`.

```bash
ros2 launch scan_capture_pkg localization.launch.py
```

If localization is unavailable, `scan_capture_node` automatically falls back to raw odometry from `/odom` directly fron the Turtlebot.

### Teleoperate the Robot

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### Run the Scan Capture System

```bash
# Default — saves to data/captures/, reads pose from /localization/pose
ros2 launch scan_capture_pkg scan_capture.launch.py

# Override output directory
ros2 launch scan_capture_pkg scan_capture.launch.py output_dir:=/tmp/my_captures
```

### Capture Scans at Waypoints

**Keyboard interface**:
```bash
ros2 run scan_capture_pkg keyboard_capture.py
# 1–9 : capture with that waypoint ID
# s   : capture with auto-incrementing ID
# q   : quit
```

**Manual service call**:
```bash
ros2 service call /scan_capture/capture scan_capture_pkg/srv/CaptureScan \
  "{waypoint_id: 1, description: 'north_wall'}"
```

Each successful capture writes two files to `data/captures/`:
- `wp_01_<timestamp>.yaml` — pose (x, y, yaw) and scan metadata
- `wp_01_<timestamp>.npy`  — raw range array (float32)

### Record a Bag File

```bash
ros2 bag record -o data/mapping_run \
  /scan /odom /imu /localization/pose /scan_capture/pointcloud
```

### Visualize the Captured Map

```bash
# Open RViz with provided config
rviz2 -d src/scan_capture_pkg/config/mapping.rviz

# Or replay a recorded bag (keeps all scan captures visible in RViz)
ros2 bag play data/mapping_run --clock
```

In RViz, set the `/scan_capture/pointcloud` display history depth to at least the number of waypoints.

---

## 6. Acknowledgements