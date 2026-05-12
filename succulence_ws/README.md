# Succulence Rover: Autonomous SLAM and Navigation

A ROS2-based robotics system that enables autonomous navigation using SLAM (Simultaneous Localization and Mapping). Built for the Algorithmic Robotics Lab, this project implements a complete autonomy stack: motion modeling, occupancy grid mapping, scan matching, pose graph optimization, path planning, and real-time navigation.

## Vision

Kevin is lost on the Mars surface. A rover equipped with wheel encoders and 2D lidar must:
1. **Map** obstacles in an unknown environment
2. **Localise** itself accurately despite wheel slip
3. **Plan** a collision-free path to Kevin's position
4. **Navigate** autonomously, replanning as new obstacles are discovered

## Quick Start

### Prerequisites

- ROS2 (Humble or later)
- Python 3.10+
- Dependencies: `rclpy`, `numpy`, `scipy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros`

### Build

```bash
cd ~/succulence_ws
colcon build
source install/setup.bash
```

### Launch SLAM Pipeline

```bash
ros2 launch succulence_rover_ros slam.launch.py
```

### Launch Dead Reckoning (Early Weeks)

```bash
ros2 launch succulence_rover_ros dead_reckoning.launch.py
```

## System Architecture

The autonomy stack implements a **Sense-Think-Act feedback loop**:

```
SENSORS                 SLAM                          PLANNING & CONTROL
[Wheel Encoders]  ──┐                                       
                   ├──> [Motion Model]              
[2D Lidar Scan]   ──┤         ↓
                   ├──> [Occupancy Grid]              
                   │         ↓
                   └──> [Scan Matcher]                
                         ↓
                   [Pose Graph Optimizer]            
                         ↓
              ┌─ [Corrected Pose]
              │    [Corrected Map]
              ▼                        
          [A* Path Planner] ──> [Waypoints]
                │                      ↓
                └──> [Path Follower] ──> [Wheel Commands]
```

### Core Modules

| Module | Week | Purpose |
|--------|------|---------|
| **Motion Model** | 5-6 | Processes wheel odometry, estimates pose trajectories with uncertainty |
| **Occupancy Grid** | 6 | Bayesian probabilistic grid mapping from pose + lidar scans |
| **Scan Matcher** | 7 | Lidar scan correlation for pose correction and loop closure |
| **Pose Graph SLAM** | 8 | Fuses all constraints, optimizes pose trajectories, rebuilds corrected map |
| **A* Path Planner** | 10 | Plans collision-free path to goal through learned map |
| **Path Follower** | 10 | Real-time steering control to follow waypoints |

## Project Structure

```
succulence_ws/
├── README.md                          # This file
├── src/succulence_rover_ros/          # Main ROS2 package
│   ├── succulence_rover_ros/          # Python source code
│   │   ├── motion_model.py            # Odometry processing
│   │   ├── occupancy_grid_mapper.py   # Grid mapping
│   │   ├── scan_matcher.py            # Scan alignment
│   │   ├── pose_graph.py              # Pose graph structure
│   │   ├── graph_optimizer.py         # Optimization solver
│   │   ├── slam_node.py               # SLAM orchestrator
│   │   └── utils.py                   # SE(2) math utilities
│   ├── launch/                        # ROS2 launch files
│   │   ├── dead_reckoning.launch.py   # Weeks 5-6
│   │   └── slam.launch.py             # Weeks 7-8+
│   ├── config/                        # Configuration
│   │   ├── params.yaml                # Tunable parameters
│   │   └── succulance.rviz            # RViz visualization config
│   ├── package.xml                    # Package manifest
│   ├── setup.py                       # Entry points
│   ├── setup.cfg                      # Installation config
│   └── SYSTEMS_OVERVIEW.md            # Full architecture reference
├── build/                             # Build artifacts (generated)
├── install/                           # Installation directory (generated)
└── log/                               # Build logs (generated)
```

## ROS2 Topics & Services

### Published Topics

- `/motion_model/pose` – Estimated robot pose (Point + orientation)
- `/motion_model/covariance` – Pose uncertainty (2×2 covariance matrix)
- `/occupancy_grid_mapper/grid` – Occupancy grid map
- `/pose_graph/corrected_pose` – SLAM-corrected pose
- `/pose_graph/corrected_map` – SLAM-optimized occupancy grid
- `/path_planner/path` – Planned path to goal
- `/path_follower/cmd_vel` – Velocity commands to wheels

### Subscribed Topics

- `/odom` – Raw wheel encoder odometry
- `/scan` – 2D LaserScan from lidar
- `/goal_position` – Target location for navigation

## Configuration

Edit `config/params.yaml` to tune:

- **Motion Model:** Process noise, covariance scaling
- **Occupancy Grid:** Cell size, probability thresholds, ray tracing parameters
- **Scan Matcher:** Search radius, grid resolution, correlation threshold
- **Pose Graph:** Keyframe spacing, optimization iterations
- **Path Planner:** A* search parameters, safety margins
- **Path Follower:** Control gains, speed limits

## RViz Visualization

View the mapping and navigation in real-time:

```bash
ros2 run rviz2 rviz2 -d src/succulence_rover_ros/config/succulance.rviz
```

Displays:
- Robot pose and uncertainty ellipse
- Occupancy grid (free/occupied/unknown cells)
- Lidar scans
- Planned path
- Pose graph nodes and edges (SLAM)

## Development Guide

### Week-by-Week Build Strategy

Each week fixes issues from the previous:

1. **Weeks 5-6:** Implement motion model and occupancy grid
   - Problem: Wheel odometry drifts, map is skewed
   - Solution: Accurate pose tracking with uncertainty bounds

2. **Week 7:** Add scan matcher
   - Problem: Growing pose uncertainty compounds mapping errors
   - Solution: Use lidar scans to correct pose estimates

3. **Week 8:** Add pose graph optimization
   - Problem: Accumulating small errors produce misaligned maps
   - Solution: Globally optimize all poses to satisfy constraints

4. **Week 10:** Path planning and navigation
   - Problem: Have a good map, now need to reach goal
   - Solution: A* planner + path follower controller

### Running Tests

```bash
colcon test --packages-select succulence_rover_ros
```

### Building Documentation

Source code includes detailed comments and docstrings. Key equations are in SYSTEMS_OVERVIEW.md.

## Key References

- **SYSTEMS_OVERVIEW.md** – Complete system architecture with detailed block diagrams and algorithms
- **ROS2 Humble Documentation** – https://docs.ros.org/en/humble/
- **Probabilistic Robotics** – Thrun, Burgard, Fox (textbook reference)

## Dependencies

- **rclpy** – ROS2 Python client library
- **numpy, scipy** – Numerical computing
- **sensor_msgs, nav_msgs, geometry_msgs** – ROS2 message types
- **tf2_ros** – ROS2 transform handling

## Maintainers

**Maleen Jayasuriya** – Algorithmic Robotics Lab, University of Canberra

## License

MIT License – See LICENSE file for details

---

## Troubleshooting

### Build fails with missing dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### RViz doesn't display anything
- Ensure `/tf` topic is published (robot frame transforms)
- Check that `/scan` and `/odom` topics have data
- Reload the RViz config: `File → Load Config → config/succulance.rviz`

### Robot doesn't move
- Verify `cmd_vel` messages are being published: `ros2 topic echo /cmd_vel`
- Check motor controller is running and responsive
- Review path planner node logs: `ros2 logs [node_name]`

### Map has ghosted/double walls
- This indicates pose drift; wait for scan matching (Week 7) to activate
- Or reduce motion model noise in `params.yaml`

---

**Happy mapping! 🌵🚀**
