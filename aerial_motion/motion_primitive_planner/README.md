# Motion Primitive Planner

`motion_primitive_planner` plans synchronized root and joint motion for DRAGON and publishes `aerial_robot_msgs/FullStateTarget` directly. It checks the final planned whole-body motion for environmental collision and flight feasibility.

For algorithm details, see [Algorithm Design](doc/algorithm_design.md).

## Source Organization

The package has six production C++ source files. The five planning and geometry files form `motion_primitive_planner_core`; the node file builds `whole_body_motion_primitive_planner_node` and contains `main()`.

| Source | Responsibility |
| --- | --- |
| [whole_body_planner_node.cpp](src/whole_body_planner_node.cpp) | ROS IO, robot model lifecycle, planning worker, trajectory activation and execution, replanning, hold commands, and diagnostics. |
| [whole_body_planner.cpp](src/whole_body_planner.cpp) | Batch joint planning, root time scaling, full-trajectory collision checks, and candidate selection. |
| [root_primitive_generator.cpp](src/root_primitive_generator.cpp) | Map replacement and snapshots, route search, local targets, and MINCO root primitives. |
| [joint_trajectory_planner.cpp](src/joint_trajectory_planner.cpp) | Trajectory history, nominal follow-the-leader prediction, joint search, attitude allocation, timing, and interpolation. |
| [dragon_geometry.cpp](src/dragon_geometry.cpp) | Robot metadata, attitude and frame transforms, link geometry, and instantaneous body collision checks. |
| [planner_config.cpp](src/planner_config.cpp) | Parameter loading and validation. |

Headers follow the same responsibilities. `WholeBodyPlanner::plan()` receives a root-candidate batch and its occupancy snapshot, predicted start joints and attitude, nominal context, and a ROS-time deadline; it returns candidate results and the selected index. The node generates the batch and captures the snapshot under the existing map lock, then evaluates candidates outside that lock. Execution state remains in the node; its small replanning and diagnostic helpers are defined in `whole_body_planner_node.h` for independent testing.

## Build

```bash
cd motion_planning_ws
catkin build motion_primitive_planner
source devel/setup.bash
```

## Run

Ensure that the DRAGON robot description and required input topics are available before launching the planner.

The whole-body planner starts `root_state_to_flu_odom.launch` automatically. It converts the root-tail pose from `/dragon/root/tail_pose` to FLU odometry on `/dragon/root/flu_odom`.

```bash
roslaunch motion_primitive_planner whole_body_motion_primitive_planner.launch
```

Do not start `gcopter/traj_server` or `multilink_copilot` full-state output with this chain.

## Configuration

[whole_body_motion_primitive_planner.yaml](config/whole_body_motion_primitive_planner.yaml) contains the map/path, primitive, follow-the-leader, flight-feasibility, joint-planning, and execution settings. The launch file loads this configuration and then applies its launch-argument overrides. The default replanning ratio is `0.3`, the activation lead time is `0.75 s`, and commands are published at `40 Hz`.
