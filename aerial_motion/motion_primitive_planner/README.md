# Motion Primitive Planner

`motion_primitive_planner` provides two online local-planning chains for DRAGON: a root-link planner for `gcopter/traj_server` with an optional `multilink_copilot`, and a whole-body planner that publishes `aerial_robot_msgs/FullStateTarget` directly.

## Design

### Root-Link Planner

The planner searches toward the global goal, selects a local target, and generates polynomial motion primitives for that segment. The default candidate set contains a nominal trajectory and two four-direction offset rings for moderate and large detours.

Each candidate predicts the full body with a follow-the-leader configuration, checks swept-body collisions and flight-feasibility margin, then selects the shortest and smoothest feasible trajectory. If no candidate is feasible, the active trajectory is retained and planning is retried later.

`AllowCopilotStabilityProjectionFallback` is disabled by default. When enabled, a candidate rejected only for insufficient nominal `fc_rp_min` may be projected to a stable configuration by Copilot. Because the projected body can differ from the collision-checked shape, use this option only in free space or when projected-shape collision checking is provided separately.

### Whole-Body Planner

For every root primitive, the whole-body node plans and revalidates a continuous joint trajectory. It first uses nominal follow-the-leader motion, connects infeasible intervals with OMPL RRT-Connect, projects an infeasible terminal target to the nearest stable fold when necessary, and slows the root trajectory to meet joint-velocity and 40 Hz command-step limits.

Only candidates with a fully feasible joint trajectory and collision-free swept body may execute. Candidates are ranked by maximum minimum clearance, minimum duration, minimum joint motion, and minimum root jerk. The node publishes root and joint commands at 40 Hz and holds the latest validated command at zero velocity after a planning failure.

The whole-body planner exclusively owns `full_state_target`; do not run it with output from `traj_server` or `multilink_copilot`.

## RViz Candidate Markers

`/candidate_markers` publishes all root-link candidates as a `visualization_msgs/MarkerArray`. Colors indicate the evaluation result:

- **Green:** selected feasible candidate.
- **Cyan:** feasible but not selected.
- **Orange:** requires downstream `fc_rp_min` projection or fails another flight-feasibility check.
- **Magenta:** rejected because the nominal joint configuration violates a joint limit.
- **Red:** rejected because of predicted whole-body collision, trajectory-generation failure, or another non-feasible status.

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
