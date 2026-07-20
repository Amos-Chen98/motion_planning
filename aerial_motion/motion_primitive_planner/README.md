# Motion Primitive Planner

`motion_primitive_planner` is a ROS package for whole-body-aware local navigation of the DRAGON root link. It generates multiple motion primitives, rejects candidates that violate collision or stability constraints, and sends the best feasible trajectory to `gcopter/traj_server`.

## Design

The planner follows an online replanning architecture: it searches a route toward the global goal, extracts a local target, and generates a set of geometrically different polynomial motion primitives for that local segment. Each candidate is evaluated by predicting the follow-the-leader configuration of the complete DRAGON body, checking its swept links for collisions, and evaluating its feasible-control margin with the DRAGON robot model. The shortest feasible candidate is executed, with trajectory smoothness used as a tie-breaker; if every candidate is rejected, the currently active trajectory is preserved and planning is attempted again later.

## RViz Candidate Colors

The `/candidate_markers` `visualization_msgs/MarkerArray` displays every generated root-link trajectory using a common line width. The trajectory colors represent the candidate evaluation result:

- **Green:** the feasible candidate selected for execution.
- **Cyan:** a feasible candidate that was not selected.
- **Orange:** a candidate rejected by the flight-feasibility checks, including the roll/pitch feasible-control margin and, when applicable, the translational feasible-control margin, static-thrust bounds, or rotor-clearance constraint.
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
