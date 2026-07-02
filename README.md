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

## Function Reference

This section lists every function and class implemented in `moro_maze/` and
what it does. Order follows the module layout of the package.

### `moro_maze/map_utils.py`

Occupancy-grid helper used by both the localization node and the global
planner. Wraps a `nav_msgs/OccupancyGrid` and offers world/grid conversion,
free/occupied queries, and border-exit detection.

**`class GridMap`** — thin wrapper around a `nav_msgs/OccupancyGrid`.

| Function | Purpose |
|---|---|
| `__init__(data, width, height, resolution, origin_x, origin_y, occupied_threshold=65)` | Store the occupancy data as a `[gy, gx]` 2-D array and remember the map metadata. |
| `from_msg(msg, occupied_threshold=65)` *(classmethod)* | Build a `GridMap` directly from a `nav_msgs/OccupancyGrid` message. |
| `world_to_grid(x, y)` | Convert world coordinates (m) to the grid cell that contains them. |
| `grid_to_world(gx, gy)` | Convert a grid cell to the world coordinates of its center. |
| `in_bounds(gx, gy)` | Return `True` if the cell lies inside the map. |
| `occupancy_at(gx, gy)` | Return the raw ROS occupancy value (`0..100`, `-1` unknown, `None` out of bounds). |
| `is_unknown(gx, gy)` | Return `True` for unknown cells or out-of-bounds cells. |
| `is_occupied(gx, gy)` | Return `True` when the cell's occupancy is at or above the wall threshold. |
| `is_free(gx, gy)` | Return `True` when the cell is inside the map and below the wall threshold. |
| `summary()` | Return a dict with size, resolution, origin, free/occupied/unknown counts, and mean known occupancy. |
| `occupied_world_points()` | Cache and return every occupied cell's world center as an `(N, 2)` array. |
| `free_world_points()` | Cache and return every free cell's world center as an `(N, 2)` array. |
| `wall_world_points()` | Alias for `occupied_world_points()`. |
| `knn_dataset()` | Return `(X, y)` for a free/wall classifier: `X` = world points, `y` = 0 free / 1 wall. |
| `nearest_obstacle_distance(x, y)` | Euclidean distance from `(x, y)` to the closest occupied cell. |
| `nearest_free_cell(gx, gy, max_radius=4)` | Snap a cell to the closest free cell, searching outward in growing square rings. |
| `free_neighbor_cells(gx, gy, connectivity=8)` | Return the free 4- or 8-connected neighbors of a cell. |
| `detect_border_exits(minimum_run=2)` | Locate real maze exits: find the bounding box of the wall shell, scan each side for free runs of at least `minimum_run` cells, and return the outer-border cells aligned with each opening. |

### `moro_maze/search_utils.py`

Graph-search utilities for the SOAR global planner: build a visibility graph
over sampled free cells, run BFS, reconstruct the path, and densify it.
Nodes are identified by strings `'gx.gy'` (cookbook convention); edges are
dicts `{'parent', 'child', 'cost'}`.

| Function | Purpose |
|---|---|
| `to_node_name(gx, gy)` | Encode a grid cell as the node name `'gx.gy'`. |
| `from_node_name(name)` | Decode a node name back into `(gx, gy)` integers. |
| `edge_cost(parent_name, child_name)` | Euclidean distance in cells between two nodes. |
| `line_cells(start_cell, end_cell)` | Bresenham's line algorithm — every grid cell touched by the segment. |
| `line_is_free(grid_map, start_cell, end_cell)` | Line-of-sight test: `True` only if every Bresenham cell is in bounds and free. |
| `nearest_visible_node(target_cell, node_cells, grid_map)` | Closest graph node with clear line of sight to `target_cell`. Used to anchor the robot start and the exit goals to the graph. |
| `sampling_anchor_cell(origin, resolution)` | Grid cell whose center sits at world coordinate 0 — used to phase-align the sampling lattice to integer world coords. |
| `sampled_free_cells(grid_map, graph_step)` | Free cells on a regular lattice: offset from the anchor is a multiple of `graph_step` on both axes. |
| `connect_visible_neighbors(edges, seen_pairs, node_a, node_b)` | Add an undirected edge between two nodes with Euclidean cost, skipping duplicates. |
| `build_graph(grid_map, required_cells=None, graph_step=6)` | Build the visibility graph: sample free cells on the lattice, group by row and column, connect consecutive nodes when line of sight is clear. Returns `(sorted_nodes, edges)`. |
| `get_children(edges, node_name)` | Neighbors of a node in the undirected graph. |
| `bfs_search(start_name, edges, goal_name)` | Breadth-First Search over the graph. Returns the discovered edge list (search tree). |
| `reconstruct_path(discovered_edges, start_name, goal_name)` | Walk the BFS search tree backwards from the goal to the start; return the node-name list. |
| `path_name_cost(path_names)` | Sum of Euclidean edge costs along a node-name path. |
| `path_names_to_cells(path_names)` | Convert a list of node names to `(gx, gy)` cells. |
| `expand_path_cells(path_cells)` | Return every Bresenham cell along the whole path (dense, per-cell). |
| `expand_sparse_path(grid_map, path_cells, step_cells=2)` | Insert intermediate waypoints every `step_cells` cells along each long edge, validating that each falls on a free cell. Used to turn the sparse graph path into something the local planner can follow smoothly. |

### `moro_maze/control_utils.py`

Local-planning utilities for the MORO cookbook's DWA-style controller:
pose/transform conversions, control-candidate generation, forward
kinematics, PT2 dynamics, cost function, and trajectory scoring.
`forwardKinematics` and `PT2Block` come from the MORO Project Cookbook and
are reproduced as cited sources.

| Function | Purpose |
|---|---|
| `pose2tf_mat(pose)` | `(x, y, theta)` → 2-D homogeneous transform matrix. |
| `tf_mat2pose(tf_mat)` | 2-D homogeneous transform matrix → `(x, y, theta)`. |
| `goal_relative_to_robot(robot_pose, goal_pose)` | Express `goal_pose` in the robot frame via `inv(T_robot) @ T_goal`. |
| `generateControls(last_control, v_min, v_max, w_min, w_max, v_acc, w_acc, v_step, w_step)` | Discrete `(v, w)` candidates in `[last ± acc]`, clipped to absolute limits — respects the robot's inertia. |
| `forwardKinematics(control, lastPose, dt, dtype=np.float64)` | Closed-form unicycle integration for a constant `(v, w)` over `dt`. *(MORO cookbook.)* |
| `class PT2Block` | Discrete PT2 (Tustin) block modelling linear-velocity dynamics. *(MORO cookbook.)* |
| &nbsp;&nbsp;`__init__(T=0, D=0, kp=1, ts=0, bufferLength=3)` | Allocate input/output ring buffers; optionally set the constants. |
| &nbsp;&nbsp;`setConstants(T, D, kp, ts)` | Compute the six Tustin coefficients from time constant `T`, damping `D`, gain `kp`, sampling time `ts`. |
| &nbsp;&nbsp;`update(e)` | Advance one step: shift the buffers, compute the new output, return it. |
| `default_state_weight()` | Default `Q = diag(1, 1, 0.5)` state weight for the quadratic cost. |
| `default_control_weight()` | Default `R = diag(0.1, 0.1)` control-effort weight. |
| `_wrap_angle(angle)` | Wrap an angle to `[-pi, pi]` using `atan2(sin, cos)`. |
| `costFn(pose, goalpose, control, Q=None, R=None)` | Weighted quadratic cost `eᵀ Q e + uᵀ R u`; heading error is wrapped before use. |
| `evaluateControls(controls, robotModelPT2, goalpose, horizon, ts, Q=None, R=None)` | Forward-simulate each control candidate over `horizon` steps starting from the robot-frame origin; each simulation uses a deep copy of the PT2 state. Returns `(costs, trajectories)`. |

### `moro_maze/localisation_node.py`

Node that computes a one-shot initial pose from the first `LaserScan` and
publishes it to `/estimated_pose`, `/estimated_pose_cov`, and `/initialpose`
(seeding AMCL).

**`class LocalisationNode(Node)`**

| Function | Purpose |
|---|---|
| `__init__()` | Configure publishers, the `GetMap` client, the map-request timer, and the latched-QoS profile for pose outputs. |
| `try_request_map()` | Timer callback that keeps polling `/map_server/map` until the service answers. |
| `map_response_callback(future)` | On map response: build `GridMap`, extract free/wall points, fit the `k=1` kNN classifier, generate candidate integer poses, and subscribe to `/scan`. |
| `scan_callback(msg)` | On the first scan: project it to Cartesian, estimate the best pose, publish it, and unsubscribe. |
| `occupancy_grid_to_numpy(map_msg)` | Reshape the flat occupancy array into a `[gy, gx]` numpy array. |
| `extract_map_positions(map_msg)` | Walk every map cell and return two arrays of world coordinates: free cells and wall cells. |
| `build_knn_dataset(free_positions, wall_positions)` | Stack the free and wall points into `(X, y)` for the `k=1` classifier (0 free / 1 wall). |
| `scan_to_cartesian(msg)` | Convert polar `LaserScan` ranges to `(x, y)` points in the robot frame — downsampled, valid-range-filtered, and shifted outward by half a map cell. |
| `estimate_initial_pose(scan_points)` | Score every candidate integer pose and return the winner (pose, score). |
| `score_pose(pose, scan_points)` | For a candidate pose at yaw 0 on a free cell, translate the scan and return the fraction of laser hits classified as walls by the kNN. |
| `generate_candidate_integer_poses()` | Enumerate every integer `(x, y)` inside the map that lands on a free cell (cookbook assumption: yaw 0). |
| `publish_estimated_pose(scan_msg, pose)` | Publish the estimate as `PoseStamped`, `PoseWithCovarianceStamped`, and `/initialpose` for AMCL. |
| `main(args=None)` | Entry point: init rclpy, spin the node, shut it down cleanly. |

### `moro_maze/global_planner_node.py`

Node that turns the map + estimated pose into a global path published to
`/global_path`. Uses `search_utils.py` and `map_utils.py`.

**`class GlobalPlannerNode(Node)`**

| Function | Purpose |
|---|---|
| `__init__()` | Configure `/map` and `/estimated_pose` subscriptions plus the latched `/global_path` publisher. |
| `map_callback(msg)` | Convert the map once into a `GridMap` and trigger planning. |
| `pose_callback(msg)` | Cache the estimated pose and trigger planning. |
| `try_plan_test_path()` | Main planning routine: detect exits, build the visibility graph (retrying `graph_step` = `6 → 3 → 1`), anchor start and exits, run BFS to every candidate exit, keep the cheapest, correct overshoot, extend through the doorway, densify, and publish. |
| `publish_path(path_cells)` | Convert grid cells to world poses (map frame) and publish as `nav_msgs/Path`. |
| `interior_goal_cell(exit_cell, graph_step)` | For a border-exit cell, step `graph_step` cells inward so the goal sits inside the maze and can be anchored to the graph, then snap to the nearest free cell. |
| `main(args=None)` | Entry point: init rclpy, spin the node, shut it down cleanly. |

### `moro_maze/local_planner_node.py`

Node that drives the robot along `/global_path` with a forward-simulation
(DWA-style) controller, publishing `/cmd_vel`.

**`class LocalPlannerNode(Node)`**

| Function | Purpose |
|---|---|
| `__init__()` | Configure publishers/subscribers, the tf listener, the PT2 dynamic model, the `Q`/`R` weight matrices, and the control-loop timer. |
| `path_callback(msg)` | Convert `nav_msgs/Path` into `(x, y, theta)` goal poses (theta = heading to the next waypoint) and reset the follow-state. |
| `localise_robot()` | Look up the `map → base_link` tf and return the robot pose `(x, y, yaw)` or `None` if unavailable. |
| `control_loop()` | One controller tick: locate the robot, advance past reached/overshot waypoints, handle the final-goal tolerance and grace period, generate candidate controls, forward-simulate and score them, publish the lowest-cost command, and update the PT2 state. |
| `publish_cmd(control)` | Publish `(v, w)` as a `TwistStamped` on `/cmd_vel`. |
| `stop_robot()` | Zero the last control and publish a stop command. |
| `publish_trajectory(trajectory)` | Publish the best simulated trajectory as `nav_msgs/Path` in `base_link`. |
| `publish_goal(rel_goal)` | Publish the current goal relative to the robot as `PoseStamped` in `base_link`. |
| `main(args=None)` | Entry point: init rclpy, spin the node, shut it down cleanly. |

### Summary — All Functions I Implemented in `src/moro_maze`

Full flat list for the assignment write-up.

**`moro_maze/map_utils.py`**
- `GridMap.__init__`
- `GridMap.from_msg`
- `GridMap.world_to_grid`
- `GridMap.grid_to_world`
- `GridMap.in_bounds`
- `GridMap.occupancy_at`
- `GridMap.is_unknown`
- `GridMap.is_occupied`
- `GridMap.is_free`
- `GridMap.summary`
- `GridMap.occupied_world_points`
- `GridMap.free_world_points`
- `GridMap.wall_world_points`
- `GridMap.knn_dataset`
- `GridMap.nearest_obstacle_distance`
- `GridMap.nearest_free_cell`
- `GridMap.free_neighbor_cells`
- `GridMap.detect_border_exits`

**`moro_maze/search_utils.py`**
- `to_node_name`
- `from_node_name`
- `edge_cost`
- `line_cells`
- `line_is_free`
- `nearest_visible_node`
- `sampling_anchor_cell`
- `sampled_free_cells`
- `connect_visible_neighbors`
- `build_graph`
- `get_children`
- `bfs_search`
- `reconstruct_path`
- `path_name_cost`
- `path_names_to_cells`
- `expand_path_cells`
- `expand_sparse_path`

**`moro_maze/control_utils.py`**
- `pose2tf_mat`
- `tf_mat2pose`
- `goal_relative_to_robot`
- `generateControls`
- `forwardKinematics` *(from MORO cookbook)*
- `PT2Block.__init__` *(from MORO cookbook)*
- `PT2Block.setConstants` *(from MORO cookbook)*
- `PT2Block.update` *(from MORO cookbook)*
- `default_state_weight`
- `default_control_weight`
- `_wrap_angle`
- `costFn`
- `evaluateControls`

**`moro_maze/localisation_node.py`**
- `LocalisationNode.__init__`
- `LocalisationNode.try_request_map`
- `LocalisationNode.map_response_callback`
- `LocalisationNode.scan_callback`
- `LocalisationNode.occupancy_grid_to_numpy`
- `LocalisationNode.extract_map_positions`
- `LocalisationNode.build_knn_dataset`
- `LocalisationNode.scan_to_cartesian`
- `LocalisationNode.estimate_initial_pose`
- `LocalisationNode.score_pose`
- `LocalisationNode.generate_candidate_integer_poses`
- `LocalisationNode.publish_estimated_pose`
- `main`

**`moro_maze/global_planner_node.py`**
- `GlobalPlannerNode.__init__`
- `GlobalPlannerNode.map_callback`
- `GlobalPlannerNode.pose_callback`
- `GlobalPlannerNode.try_plan_test_path`
- `GlobalPlannerNode.publish_path`
- `GlobalPlannerNode.interior_goal_cell`
- `main`

**`moro_maze/local_planner_node.py`**
- `LocalPlannerNode.__init__`
- `LocalPlannerNode.path_callback`
- `LocalPlannerNode.localise_robot`
- `LocalPlannerNode.control_loop`
- `LocalPlannerNode.publish_cmd`
- `LocalPlannerNode.stop_robot`
- `LocalPlannerNode.publish_trajectory`
- `LocalPlannerNode.publish_goal`
- `main`
