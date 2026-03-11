# Map Evaluation Analysis

## Overview

Map accuracy is evaluated by comparing physical measurements (tape measure) against
distances measured in the assembled map (RViz Measure tool).

This is a **manual process** — you will use RViz interactively to measure distances
and record results in `config/measurements.yaml`.

---

## Step 1: Load Captured Point Clouds

Each scan capture saves a YAML file with pose information and a .npy file with
range data. To visualize in RViz:

Option A: Replay your bag file
```bash
ros2 bag play data/mapping_run --clock
# In RViz, subscribe to /scan_capture/pointcloud
```

Option B: Load point clouds individually (requires conversion script)

---

## Step 2: Measure Distances in RViz

### Using the Measure Tool

1. In RViz toolbar, click the **Measure** button (ruler icon)
2. Click on the waypoint position (where the robot was)
3. Click on the landmark (wall surface, corner, etc.)
4. Read the distance from the status bar or tool panel

### Tips for Accurate Measurement

- Zoom in closely on the points you're measuring between
- For walls: click on the nearest point cloud points on the wall surface
- For corners: click on the corner vertex
- Measure the same feature you measured physically

---

## Step 3: Record Results

For each waypoint, record in `config/measurements.yaml`:

```yaml
map_evaluation:
  results:
    - waypoint: 1
      landmark: "North wall"
      measured_m: 2.35      # Your tape measure value
      rviz_m: 2.31          # Distance from RViz Measure tool
      error_m: 0.04         # measured - rviz
      error_percent: 1.7    # (error / measured) * 100
```

---

## Step 4: Assess Orientation

For each waypoint, qualitatively assess:

1. Is the measured landmark in the expected direction?
   - If you were facing north when you captured the scan, does the north wall
     appear ahead in the point cloud?

2. Do features align across scans?
   - If two scans capture the same wall, do the point clouds overlap cleanly?
   - Misalignment indicates localization error.

Take screenshots showing:
- Good alignment (walls match up)
- Any misalignment or "double walls"

---

## Step 5: Compute Summary Statistics

After recording all waypoint results:

```
Mean distance error = (sum of |error_m|) / (number of waypoints)
Max distance error = max(|error_m|)
```

Record in `measurements.yaml`:
```yaml
map_evaluation:
  mean_distance_error_m: 0.XX
  max_distance_error_m: 0.XX
  notes: "Summary observations..."
```

---

## Step 6: Document in README

Include in your project README:

1. **Distance Accuracy Table** — all waypoints with measured vs RViz values
2. **Orientation Assessment** — qualitative description for each waypoint
3. **Screenshots** — showing RViz measurements and map quality
4. **Discussion** — error sources, consistency assessment, improvements
