# Motion Primitive Planner

`motion_primitive_planner` plans synchronized root and joint motion for DRAGON and publishes `aerial_robot_msgs/FullStateTarget` directly. It checks the final planned whole-body motion for environmental collision and flight feasibility.

For algorithm details, see [Algorithm Design](doc/algorithm_design.md).

Candidate evaluation uses a persistent thread pool with one independent robot model per candidate. `PlanningThreads=0` (the default) uses the available CPUs, capped by `CandidateCount`; `1` selects the serial path, and a positive value sets the thread limit. The launch argument is `planning_threads`, for example `roslaunch motion_primitive_planner whole_body_motion_primitive_planner.launch planning_threads:=1`. Results retain candidate order and the existing ranking rules. Under a shared deadline, parallel execution may finish additional candidates and consequently select a different trajectory. See [Parallel Performance](doc/parallel_performance.md) for measurements and reproduction instructions.

## Source Organization

The package has six production C++ source files. The five planning and geometry files form `motion_primitive_planner_core`; the node file builds `whole_body_motion_primitive_planner_node` and contains `main()`.

| Source | Responsibility |
| --- | --- |
| [whole_body_planner_node.cpp](src/whole_body_planner_node.cpp) | ROS IO, robot model lifecycle, planning worker, trajectory activation and execution, replanning, hold commands, and diagnostics. |
| [whole_body_planner.cpp](src/whole_body_planner.cpp) | Batch joint planning, root time scaling, full-trajectory collision checks, and candidate selection. |
| [root_primitive_generator.cpp](src/root_primitive_generator.cpp) | Map replacement and snapshots, route search, local targets, and MINCO root primitives. |
| [joint_trajectory_planner.cpp](src/joint_trajectory_planner.cpp) | Trajectory history for scoring, root-attitude prediction, analytic terminal joint targets, joint search, attitude allocation, timing, and interpolation. |
| [dragon_geometry.cpp](src/dragon_geometry.cpp) | Robot metadata, attitude and frame transforms, link geometry, and instantaneous body collision checks. |
| [planner_config.cpp](src/planner_config.cpp) | Parameter loading and validation. |

Headers follow the same responsibilities. `WholeBodyPlanner::plan()` receives a root-candidate batch and its occupancy snapshot, predicted start joints and attitude, nominal context, and a ROS-time deadline; it returns candidate results and the selected index. The node generates the batch and captures the snapshot under the existing map lock, then evaluates candidates outside that lock. Execution state remains in the node; its small replanning and diagnostic helpers are defined in `whole_body_planner_node.h` for independent testing.

## Build

Install the [self-filter dependencies](../pcd_self_filter/README.md#dependencies-and-build) first; the default launch starts both the filter and mapper.

```bash
cd motion_planning_ws
source ../jsk_aerial_robot_ws/devel/setup.bash
catkin build pcd_self_filter voxel_mapping motion_primitive_planner
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

The mapper enables [single-frame noise filtering](../voxel_mapping/README.md#noise-filtering) by default, requiring two other points within 0.20 m before a point can mark a voxel occupied. The planner launch forwards `enable_noise_filter`, `noise_filter_radius`, and `noise_filter_min_neighbors`; for example, append `noise_filter_min_neighbors:=3` to require three neighbors or `enable_noise_filter:=false` to restore single-point occupancy. The neighbor radius is independent of `voxel_width` and the planner's obstacle dilation radius.

[whole_body_motion_primitive_planner.yaml](config/whole_body_motion_primitive_planner.yaml) contains the map/path, primitive, root-attitude, terminal IK, flight-feasibility, joint-planning, and execution settings. `JointReferenceDt` controls root-attitude prediction; terminal tail targets are computed directly on the remaining MINCO curve and aligned initial-body segments, independently of trajectory-history sampling. The launch file loads this configuration and then applies its launch-argument overrides. The default replanning ratio is `0.3`, the activation lead time is `0.75 s`, and commands are published at `40 Hz`.
