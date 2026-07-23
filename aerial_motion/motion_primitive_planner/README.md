# Motion Primitive Planner

`motion_primitive_planner` provides two online local-planning chains for DRAGON: a root-link planner for `gcopter/traj_server` with an optional `multilink_copilot`, and a whole-body planner that publishes `aerial_robot_msgs/FullStateTarget` directly.

## Design

Both nodes use the same `PlanningEnvironment` for accumulated-map maintenance, goal clamping, route search, horizon truncation, root primitive generation, and immutable binary occupancy snapshots. They also share DRAGON joint metadata, bounded trajectory history, nominal follow-the-leader prediction, stability configuration, candidate diagnostics, and one instantaneous whole-body collision checker. The root node retains GCOPTER trajectory handover and publication, while the whole-body node retains joint-space planning, activation/hold state, publisher ownership checks, and direct full-state output.

### Shared Collision Model

For each instantaneous DRAGON configuration, the shared checker reconstructs the four link centerlines, samples each at intervals no greater than `VoxelWidth / 2`, and returns `true` on the first sample inside the binary obstacle map dilated by `DilateRadius`. The default `VoxelWidth=0.10 m` and `DilateRadius=0.20 m` approximate each link as a radius-`0.20 m` voxelized capsule. Both planners adaptively subdivide time until the conservative maximum whole-body displacement between collision checks is no greater than `VoxelWidth / 2`; the root-link planner checks interpolated nominal configurations, while the whole-body planner checks its final joint trajectory.

### Root-Link Planner

The planner searches toward the global goal, selects a local target, and generates polynomial motion primitives for that segment. The default candidate set contains a nominal trajectory and two four-direction offset rings for moderate and large detours.

Each candidate predicts the full body with the shared follow-the-leader predictor, checks swept-body collisions and the same joint-limit, feasible-control, thrust, rotor-clearance, and baselink-tilt constraints used by the whole-body planner, then selects the shortest and smoothest feasible trajectory. If no candidate is feasible, the active trajectory is retained and planning is retried later.

`PredictionDt` controls only nominal stability evaluation; collision sampling is independently determined from command-rate nominal intervals and the adaptive whole-body displacement bound.

`AllowCopilotStabilityProjectionFallback` is disabled by default. When enabled, a candidate rejected only for insufficient nominal `fc_rp_min` may be projected to a stable configuration by Copilot. Because the projected body can differ from the collision-checked shape, use this option only in free space or when projected-shape collision checking is provided separately.

Both launch files first load `config/common_motion_primitive_planner.yaml`, then their mode-specific configuration. Shared parameter names and ROS topics remain identical between the two nodes; `MaxBaselinkTilt` defaults to `1.20 rad` in both modes.

### Whole-Body Planner

For every root primitive, the whole-body node plans and revalidates a continuous joint trajectory. It first uses nominal follow-the-leader motion, connects infeasible intervals with OMPL RRT-Connect, projects an infeasible terminal target to the nearest stable fold when necessary, and slows the root trajectory to meet joint-velocity and 40 Hz command-step limits.

Only candidates with a fully feasible joint trajectory and collision-free swept body may execute. Candidates are ranked by minimum duration, minimum joint motion, and minimum root jerk. The node publishes root and joint commands at 40 Hz and holds the latest validated command at zero velocity after a planning failure.

The whole-body planner exclusively owns `full_state_target`; do not run it with output from `traj_server` or `multilink_copilot`.

## RViz Candidate Markers

`/candidate_markers` publishes all root-link candidates as a `visualization_msgs/MarkerArray`. Colors indicate the evaluation result:

- **Green:** selected feasible candidate.
- **Cyan:** feasible but not selected.
- **Orange:** requires downstream `fc_rp_min` projection or fails another flight-feasibility check.
- **Magenta:** rejected because the nominal joint configuration violates a joint limit.
- **Red:** rejected because of predicted whole-body collision, trajectory-generation failure, or another non-feasible status.

Both nodes publish `/selected_candidate`, `/selected_min_fc_rp`, and `/selected_joint_motion`. Collision results are binary and no clearance diagnostic is published.

## Build

```bash
cd motion_planning_ws
catkin build motion_primitive_planner
source devel/setup.bash
```

## Run

Ensure that the DRAGON robot description and required input topics are available before launching either chain.

### Root-Link Planner

```bash
roslaunch motion_primitive_planner motion_primitive_planner.launch
```

`gcopter/traj_server` executes the root-link trajectory. Add `multilink_copilot` only when stability projection is required.

### Whole-Body Planner

The whole-body planner requires FLU odometry on `/dragon/root/flu_odom`. Start `root_state_to_flu_odom.launch` first; it converts the root-tail pose from `/dragon/root/tail_pose` to the required odometry topic.

```bash
roslaunch naraha_center root_state_to_flu_odom.launch
```

```bash
roslaunch motion_primitive_planner whole_body_motion_primitive_planner.launch
```

Do not start `gcopter/traj_server` or `multilink_copilot` full-state output with this chain.
