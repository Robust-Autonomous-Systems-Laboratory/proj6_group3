# Navigation and Measurement Strategy

## Team Members
- Anders Smitterberg
- Victor Ameh

---

## 1. Environment Description

### Location

EERC 727 -- a lab room with a recycling bin placed approximately in the center. The north wall runs along the top of the room. A door is located in the east wall. The floor is smooth tile suitable for wheeled odometry.

### Environment Sketch

![Environment sketch](../figures/environment_sketch.jpg)

### Ground Truth Measurements (Leica Laser Rangefinder)

| Waypoint | Heading | North Wall (m) | East Door (m) | Bin Corner (m) |
|----------|---------|----------------|---------------|----------------|
| 1        | East    | 1.506          | 3.980         | 1.655          |
| 2        | East    | 1.417          | 1.999         | 1.778          |
| 3        | South   | 3.014          | 1.865         | 1.168          |
| 4        | West    | 4.743          | 4.309         | 1.324          |
| 5        | North   | 3.752          | 5.514         | 1.662          |
| 6 (return)| East  | 1.376          | 3.958         | 1.757          |

## 2. Procedure Description

The robot was manually teleoperated clockwise around the recycling bin across five waypoints, with a sixth capture taken upon returning to the start position for loop closure evaluation.

At each waypoint the following steps were performed in order:

1. Drive the robot to a suitable position and stop completely
2. Take an RViz screenshot showing the current scan and pose
3. Trigger a scan capture using the keyboard interface (`keyboard_capture.py`)
4. Measure the distance from the robot (LDS center) to three landmarks using the Leica laser rangefinder:
   - Nearest visible corner of the recycling bin
   - North wall (shortest perpendicular distance)
   - East door (shortest distance to door surface)

The robot's position at each waypoint is recorded by the EKF localization node (`/localization/pose`) at the moment of capture -- no pre-marked positions were used.

After data collection, map distances were measured using `analysis/visualize_captures.py --measure`, which transforms each scan into the odom frame using the saved pose and provides an interactive click-to-measure tool.