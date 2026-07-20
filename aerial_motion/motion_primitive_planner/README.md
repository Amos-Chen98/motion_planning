# Motion Primitive Planner

`motion_primitive_planner` is a ROS package for whole-body-aware local navigation of the DRAGON root link. It generates multiple motion primitives, rejects candidates that violate collision or stability constraints, and sends the best feasible trajectory to `gcopter/traj_server`.

## Design

The planner follows an online replanning architecture: it searches a route toward the global goal, extracts a local target, and generates a set of geometrically different polynomial motion primitives for that local segment. The default nine candidates contain a nominal path plus two four-direction offset rings, so both moderate and maximum detours are available. Each candidate is evaluated by predicting the follow-the-leader configuration of the complete DRAGON body, checking its swept links for collisions, and evaluating its feasible-control margin with the DRAGON robot model. The shortest nominally feasible candidate is executed, with trajectory smoothness used as a tie-breaker. When `AllowCopilotStabilityProjectionFallback` is enabled and every otherwise-valid candidate violates only the nominal `fc_rp_min` constraint, the planner instead selects the shortest recoverable candidate and uses nominal joint motion, margin, and smoothness as tie-breakers, then relies on downstream `multilink_copilot` to project its joint target to a stable configuration. This joint-motion term favors keeping the current fold instead of crossing a near-straight configuration. If every candidate has a hard failure, the currently active trajectory is preserved and planning is attempted again later.

The Copilot projection fallback is disabled by default because the projected whole-body shape can differ from the nominal shape used by the planner's collision check. Enable it only when a downstream Copilot is present and the environment is free space, or when collision checking of the projected shape is handled separately.

## RViz Candidate Colors

The `/candidate_markers` `visualization_msgs/MarkerArray` displays every generated root-link trajectory using a common line width. The trajectory colors represent the candidate evaluation result:

- **Green:** the feasible candidate selected for execution.
- **Cyan:** a feasible candidate that was not selected.
- **Orange:** a candidate that needs downstream `fc_rp_min` projection or is rejected by another flight-feasibility check.
- **Magenta:** a candidate rejected because its predicted nominal joint configuration violates a joint limit.
- **Red:** a candidate rejected because of a predicted whole-body collision, trajectory-generation failure, or another non-feasible status.

## Build

```bash
cd motion_planning_ws
catkin build motion_primitive_planner
source devel/setup.bash
```

## Test

```bash
catkin run_tests motion_primitive_planner
catkin_test_results build/motion_primitive_planner
```

## Run

Make sure the DRAGON robot description and required input topics are available, then run:

```bash
roslaunch motion_primitive_planner motion_primitive_planner.launch
```
