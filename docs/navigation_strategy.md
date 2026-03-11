# Navigation and Measurement Strategy

## Team Members
- Member 1: [Name]
- Member 2: [Name]

---

## 1. Environment Description

### Location
[Describe the room/area: dimensions, floor type, notable features]

### Environment Sketch
```
[Draw ASCII sketch of environment with walls, landmarks, and planned waypoints]

Example:
    +------------------+
    |                  |
    |   [P]            |  P = Pillar
    |                  |
    |  1 ----> 2       |  1-5 = Waypoints
    |          |       |  Arrows = Path
    |          v       |
    |  5 <---- 3 -> 4  |
    |                  |
    +------------------+
         North Wall
```

### Identified Landmarks
| ID | Landmark | Location | Notes |
|----|----------|----------|-------|
| A  | North wall | North side | Flat surface, good for distance measurement |
| B  | Pillar | Center-left | Clear corner visible to LiDAR |
| C  | ... | ... | ... |

---

## 2. Waypoint Plan

### Waypoint Layout

| Waypoint | Position (x, y) | Landmark to Measure | Measurement Direction |
|----------|-----------------|---------------------|----------------------|
| 1 (Start)| (0.0, 0.0) | North wall | Straight ahead (0°) |
| 2 | | | |
| 3 | | | |
| 4 | | | |
| 5 | | | |

### Path Statistics
- Number of waypoints: 
- Total path length: 
- Loop closure planned: Yes / No
- Estimated navigation time: 

---

## 3. Orientation Strategy

### Heading Reference System
[Describe how you will define and track orientation]

Options:
- [ ] Align robot parallel to a specific wall at each waypoint
- [ ] Use floor tape lines to define heading
- [ ] Use compass directions (if walls align N/S/E/W)
- [ ] Other: _______________

### At Each Waypoint
| Waypoint | Orientation Reference | Expected Landmark Direction |
|----------|----------------------|----------------------------|
| 1 | Parallel to south wall | North wall at 0° (ahead) |
| 2 | | |
| 3 | | |
| 4 | | |
| 5 | | |

---

## 4. Measurement Protocol

### Before Data Collection
- [ ] Mark all waypoints with tape
- [ ] Measure waypoint positions from start
- [ ] Identify measurement landmark at each waypoint
- [ ] Measure distance to each landmark
- [ ] Record all measurements in `config/measurements.yaml`

### At Each Waypoint During Collection
1. [ ] Stop robot completely at tape mark
2. [ ] Align robot to planned orientation
3. [ ] Wait 2 seconds for settling
4. [ ] Verify landmark visible in RViz scan
5. [ ] Capture scan with waypoint ID
6. [ ] Take RViz screenshot
7. [ ] Note any observations

### Measurement Technique
- Measuring from: [LiDAR center / robot center / tape mark]
- Measuring to: [Wall surface / corner point / pillar edge]
- Tool: [Tape measure / laser measure]
- Estimated measurement uncertainty: ± ___ m

---

## 5. Expected Challenges

### Localization Error Sources
1. **Odometry drift**: [Expected impact, mitigation]
2. **IMU bias**: [Expected impact, mitigation]
3. **Wheel slip**: [Floor conditions]

### Mapping Challenges
1. **Scan alignment**: [How might scans misalign?]
2. **Landmark visibility**: [Any occlusion concerns?]
3. **Environmental factors**: [People, lighting, reflections]

### Mitigation Strategies
- 
- 
- 

---

## 6. Roles During Data Collection

| Task | Team Member |
|------|-------------|
| Robot pilot | |
| RViz monitoring / screenshots | |
| Scan capture triggering | |
| Observation notes | |

---

## 7. Post-Collection Analysis Plan

### Map Evaluation Steps
1. [ ] Load all point clouds in RViz
2. [ ] Verify scans appear in `odom` frame
3. [ ] For each waypoint, measure distance to landmark using RViz Measure tool
4. [ ] Record RViz measurements in `measurements.yaml`
5. [ ] Compute errors (measured - RViz)
6. [ ] Take screenshots showing measurements
7. [ ] Assess orientation consistency at each waypoint

### Quality Checks
- [ ] Do walls from different scans align?
- [ ] Is loop closure consistent (if applicable)?
- [ ] Are there any obvious mapping artifacts?

---

## 8. Pre-Run Checklist

- [ ] Robot battery charged
- [ ] All waypoints marked with tape
- [ ] All positions measured and recorded
- [ ] All landmark distances measured and recorded
- [ ] `measurements.yaml` filled with ground truth
- [ ] Localization node tested
- [ ] Scan capture service tested
- [ ] RViz configured
- [ ] Bag recording tested
- [ ] Camera/screenshot tool ready
- [ ] This strategy document complete
