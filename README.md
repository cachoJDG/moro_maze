# moro_maze

Manual documentation for the `moro_maze` ROS 2 package.

This package solves a maze-navigation assignment with a TurtleBot3 Burger in
Gazebo. The robot must localize itself, compute a path to an exit, and drive
through the maze using the implemented software.

The solution combines:

- **SOAR-style initial localization** from the map and the first `LaserScan`.
- **SOAR-style global planning** with a visibility graph and Breadth-First
  Search.
- **MORO-style local planning** with forward simulation and direct velocity
  control.
- **Nav2 map server and AMCL** for map publication and continuous pose tracking
  while the robot moves.

## Package Contents

| Path | Purpose |
|---|---|
| `launch/simulation_launch.py` | Starts Gazebo, TurtleBot3, Nav2, RViz, and the custom nodes. |
| `moro_maze/localisation_node.py` | Estimates the initial robot pose from the first laser scan. |
| `moro_maze/global_planner_node.py` | Builds the graph, detects maze exits, runs BFS, and publishes the global path. |
| `moro_maze/local_planner_node.py` | Follows the global path with a forward-simulation local controller. |
| `moro_maze/map_utils.py` | Occupancy-grid helper functions and exit detection. |
| `moro_maze/search_utils.py` | Graph, Bresenham line checks, BFS, and path expansion utilities. |
| `moro_maze/control_utils.py` | MORO control utilities: transforms, control generation, forward kinematics, PT2 model, and cost function. |
| `maps/map.yaml`, `maps/map.pgm` | Static maze map used by Nav2 and the custom planner. |
| `worlds/default_gzsim.world` | Gazebo simulation world. |
| `rviz/config.rviz` | RViz configuration. |
| `params/nav2_params.yaml` | Nav2 parameter file kept with the package. |

## Runtime Architecture

The launch file starts Gazebo, spawns the TurtleBot3, starts Nav2, and runs the
custom localization, global planning, and local planning nodes.

```text
/map_server/map service        /scan
          |                      |
          v                      v
  localisation_node  ---->  /estimated_pose
          |                      |
          |                      v
          |              global_planner_node
          |                      |
          v                      v
      /initialpose          /global_path
          |                      |
          v                      v
        AMCL  -------->  local_planner_node
          |                      |
          |                      v
   tf map->base_link         /cmd_vel
                                 |
                                 v
                              Gazebo
```

Nav2 is used for:

- `map_server`, which provides the static occupancy map.
- `AMCL`, which provides continuous localization through the `map -> base_link`
  transform after the custom node publishes the initial pose.

The custom code performs the main assignment logic:

- Initial localization.
- Global path planning.
- Local trajectory generation and velocity control.

The local planner publishes directly to `/cmd_vel`. It does not send navigation
goals to Nav2.

## Main Topics

| Topic | Message type | Publisher | Consumer / purpose |
|---|---|---|---|
| `/estimated_pose` | `geometry_msgs/PoseStamped` | `localisation_node` | Used by `global_planner_node` as the planning start. |
| `/estimated_pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | `localisation_node` | Debug/diagnostic estimated pose with covariance. |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | `localisation_node` | Seeds AMCL. |
| `/global_path` | `nav_msgs/Path` | `global_planner_node` | Consumed by `local_planner_node`. |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | `local_planner_node` | Velocity command for the robot through the simulation bridge. |
| `/local_trajectory` | `nav_msgs/Path` | `local_planner_node` | Best simulated local trajectory for visualization. |
| `/current_goal` | `geometry_msgs/PoseStamped` | `local_planner_node` | Current local goal expressed in `base_link`. |

## How to Run in JupyterLab

In the JupyterLab terminal, run:

```bash
colcon build --packages-select moro_maze
source install/setup.bash
ros2 launch moro_maze simulation_launch.py
```


## Implemented Software

### Initial Localization

Implemented in `moro_maze/localisation_node.py`.

The localization node performs a one-shot estimate of the robot start pose:

1. It requests the static map from `/map_server/map`.
2. It converts the occupancy grid into free-cell and wall-cell world
   coordinates.
3. It trains a `k=1` nearest-neighbor classifier that labels coordinates as
   free or occupied.
4. It waits for the first `/scan`.
5. It converts valid laser ranges from polar to Cartesian coordinates.
6. It tests all possible integer start poses with yaw `0`.
7. It selects the pose whose translated scan points best match wall cells in
   the map.
8. It publishes the result to `/estimated_pose`, `/estimated_pose_cov`, and
   `/initialpose`.

The scan points are shifted outward by half a map cell before scoring. This
helps the laser-hit points align with wall cells in the occupancy grid.

Important constants:

| Constant | Value | Meaning |
|---|---:|---|
| `OCCUPIED_THRESHOLD` | `65` | Occupancy value considered a wall. |
| `SCAN_DOWNSAMPLE_STEP` | `8` | Uses every eighth laser beam for speed. |
| `KNN_NEIGHBORS` | `1` | Nearest-neighbor classifier size. |
| `MAP_SERVICE_TIMEOUT_SEC` | `5.0` | Map service wait timeout. |

Limitation: this is an initial localization step only. It assumes yaw `0` and
does not continuously localize the robot after it starts moving. Continuous
pose tracking during motion is provided by AMCL.

### Global Planning

Implemented in `moro_maze/global_planner_node.py`,
`moro_maze/map_utils.py`, and `moro_maze/search_utils.py`.

The global planner:

1. Subscribes to the static map.
2. Subscribes to `/estimated_pose`.
3. Detects real maze exits by finding openings in the occupied wall shell.
4. Samples free map cells on a regular lattice.
5. Builds a visibility graph from sampled free cells.
6. Connects graph nodes that share a clear row or column line of sight.
7. Anchors the robot start and exit candidates to visible graph nodes.
8. Runs Breadth-First Search to find a path to an exit.
9. Densifies the path so the local planner receives closer waypoints.
10. Publishes the result as `/global_path`.

The graph node names follow the cookbook convention `gx.gy`, for example
`4.4` or `22.16`.

Important constants:

| Constant | Value | Meaning |
|---|---:|---|
| `GRAPH_STEP` | `6` | Main graph lattice spacing in grid cells. |
| `EXECUTION_STEP_CELLS` | `2` | Densified waypoint spacing. |
| `MINIMUM_EXIT_RUN` | `2` | Minimum opening size to count as an exit. |
| `EXIT_INSET_CELLS` | `1` | Final doorway waypoint offset from the border. |

If the graph cannot produce a path with the default spacing, the planner tries
fallback graph steps `3` and `1`.

### Local Planning and Control

Implemented in `moro_maze/local_planner_node.py` and
`moro_maze/control_utils.py`.

The local planner is a forward-simulation controller similar to a Dynamic
Window Approach:

1. It receives `/global_path`.
2. It reads the current robot pose from the `map -> base_link` transform.
3. It selects the current waypoint.
4. It expresses that waypoint relative to the robot frame with homogeneous
   transforms.
5. It generates valid `(v, w)` velocity candidates around the previous command.
6. It forward-simulates each candidate over a short horizon.
7. It scores each simulated trajectory with a quadratic cost function.
8. It publishes the lowest-cost command to `/cmd_vel`.
9. It publishes `/local_trajectory` and `/current_goal` for visualization.

The controller stops when the final waypoint is reached, or after the final
goal grace period has elapsed.

Important constants:

| Constant | Value | Meaning |
|---|---:|---|
| `CONTROL_RATE` | `5.0 Hz` | Main controller frequency. |
| `SIM_TS` | `0.2 s` | Simulation time step. |
| `HORIZON` | `12` | Number of forward-simulation steps. |
| `GOAL_TOLERANCE` | `0.25 m` | Intermediate waypoint tolerance. |
| `FINAL_GOAL_TOLERANCE` | `0.10 m` | Final waypoint stopping tolerance. |
| `FINAL_GOAL_GRACE_SEC` | `10.0 s` | Maximum extra time on the final leg. |
| `V_MAX` | `0.22 m/s` | Maximum linear speed. |
| `W_MAX` | `1.4 rad/s` | Maximum angular speed. |
| `Q_POS` | `1.0` | Position error weight. |
| `Q_THETA` | `0.0` | Heading error weight. |
| `R_V` | `0.05` | Linear velocity penalty. |
| `R_W` | `0.02` | Angular velocity penalty. |

The heading weight is intentionally set to zero. For this maze task, reaching
the waypoint position is more important than matching the waypoint orientation.
Using a heading penalty caused the robot to rotate in place at corners instead
of continuing through the maze.

The local planner also advances to the next waypoint if it detects that the
robot has already passed the current one. This avoids getting stuck while
trying to return to a waypoint that is now behind the robot.

### Utility Modules

`map_utils.py` defines `GridMap`, a wrapper around `nav_msgs/OccupancyGrid`.
It handles:

- World/grid coordinate conversion.
- Free, occupied, and unknown cell queries.
- Nearest free-cell search.
- Wall and free-cell extraction.
- Border-exit detection.

`search_utils.py` contains:

- Graph node encoding/decoding.
- Bresenham line traversal.
- Line-of-sight collision checks.
- Visibility-graph construction.
- Breadth-First Search.
- Path reconstruction.
- Sparse-path expansion.

`control_utils.py` contains:

- Pose-to-transform and transform-to-pose helpers.
- Goal transformation into the robot frame.
- Control candidate generation.
- Forward kinematics.
- PT2 dynamic model for linear velocity.
- Quadratic cost calculation.
- Candidate trajectory evaluation.

`forwardKinematics` and `PT2Block` are based on the MORO Project Cookbook
material referenced in the assignment.

## Design Decisions

| Decision | Reason |
|---|---|
| Use custom `/cmd_vel` control instead of Nav2 goals | The assignment requires implemented local planning/control logic. |
| Use AMCL after initial localization | The implemented scan matching gives a good initial pose; AMCL provides robust continuous pose tracking while moving. |
| Detect openings in the actual wall shell | The map has a free outer padding ring, so scanning only the outer border gives false exits. |
| Densify the global path | Sparse graph waypoints are too far apart for smooth local control. |
| Set `Q_THETA = 0` | The robot should follow waypoint positions through the maze instead of rotating to match every intermediate heading. |

## Known Limitations

- Initial localization assumes integer start coordinates and yaw `0`.
- The custom localization node only estimates the initial pose once.
- Continuous localization depends on AMCL.
- The local planner cost function does not include an obstacle-distance term.
  It assumes the global path is collision-free.
- The launch file starts more of Nav2 than strictly necessary. The custom code
  mainly uses Nav2's map server and AMCL.
- Simulation speed depends on the machine running Gazebo.

## Expected Behavior

For valid assignment spawn positions, the expected behavior is:

1. The map is received.
2. The first scan is processed.
3. The initial pose is estimated.
4. AMCL is seeded through `/initialpose`.
5. The global planner finds one of the real maze exits.
6. The global path is published.
7. The local planner follows the path by publishing `/cmd_vel`.
8. The robot exits or reaches the final doorway waypoint.
9. The local planner publishes a zero command and reports that the robot stopped.
