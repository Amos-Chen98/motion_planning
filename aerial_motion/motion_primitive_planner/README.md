# Motion Primitive Planner

`motion_primitive_planner` plans synchronized root and joint motion for DRAGON and publishes `aerial_robot_msgs/FullStateTarget` directly. It checks the final planned whole-body motion for environmental collision and flight feasibility.

For algorithm details, see [Algorithm Design](doc/algorithm_design.md).

Candidate evaluation uses a persistent thread pool with one independent robot model per candidate. `PlanningThreads=0` (the default) uses the available CPUs, capped by `CandidateCount`; `1` selects the serial path, and a positive value sets the thread limit. The launch argument is `planning_threads`, for example `roslaunch motion_primitive_planner whole_body_motion_primitive_planner.launch planning_threads:=1`. Results retain candidate order and the ranking rules. Under a shared deadline, parallel execution may finish additional candidates and consequently select a different trajectory. Serial/parallel equivalence is covered by the joint bridge regression tests.

## Source Organization

The planning and geometry sources form `motion_primitive_planner_core`; the node source builds `whole_body_motion_primitive_planner_node` and contains `main()`.

Public headers live in `include/motion_primitive_planner/`. Implementation files and private headers (`candidate_executor.h` and `trajectory_collision.h`) live directly in `src/`; tests live in `test/`. Tests that use private headers add `src/` to their include paths; the install ships the public headers.

| Source | Responsibility |
| --- | --- |
| [whole_body_planner_node.cpp](src/whole_body_planner_node.cpp) | ROS IO, robot model lifecycle, planning worker, trajectory activation and execution, replanning, hold commands, and diagnostics. |
| [whole_body_planner.cpp](src/whole_body_planner.cpp) | Batch joint planning, root time scaling, full-trajectory collision checks, and candidate selection. |
| [root_primitive_generator.cpp](src/root_primitive_generator.cpp) | Map replacement and snapshots, route search, local targets, and MINCO root primitives. |
| [octomap_message.cpp](src/octomap_message.cpp) | Full probability-tree message validation and decoding. |
| [joint_trajectory_planner.cpp](src/joint_trajectory_planner.cpp) | Trajectory history for scoring, root-attitude prediction, analytic terminal joint targets, joint search, attitude allocation, timing, and interpolation. |
| [dragon_geometry.cpp](src/dragon_geometry.cpp) | Robot metadata, attitude and frame transforms, and nominal link geometry. |
| [dragon_collision_checker.cpp](src/dragon_collision_checker.cpp) | Eight URDF primitives, collision-only KDL workspaces, and immutable OctoMap environments. |
| [planner_config.cpp](src/planner_config.cpp) | Parameter loading and validation. |

Headers follow the same responsibilities. `WholeBodyPlanner::plan()` receives a root-candidate batch and its `PlanningSceneSnapshot` (route backend and raw collision map), predicted start joints and attitude, nominal context, and a ROS-time deadline; it returns candidate results and the selected index. The node generates the batch and captures the snapshot under the existing map lock, then evaluates candidates outside that lock. Execution state remains in the node; its small replanning and diagnostic helpers are defined in `whole_body_planner_node.h` for independent testing.

## Build

Install Coal 3 with OctoMap support, OctoMap, and the URDF/KDL development dependencies. Make the Coal installation discoverable through `CMAKE_PREFIX_PATH` or `coal_DIR`; CMake links the `coal::coal` imported target.

Install the [self-filter dependencies](../pcd_filter/README.md#dependencies-and-build) first; the default launch starts both the filter and mapper.

```bash
cd motion_planning_ws
source ../jsk_aerial_robot_ws/devel/setup.bash
catkin build pcd_filter octomap_mapper motion_primitive_planner
source devel/setup.bash
```

## Run

The DRAGON robot description and its input topics must be available before launch.

The whole-body planner starts `root_state_to_flu_odom.launch` automatically. It converts the root-tail pose from `/dragon/root/tail_pose` to FLU odometry on `/dragon/root/flu_odom`.

```bash
roslaunch motion_primitive_planner whole_body_motion_primitive_planner.launch
```

This chain is the sole full-state output source, replacing `gcopter/traj_server` and `multilink_copilot` full-state output.

## Configuration

Environmental collision uses only the four `link1…4` URDF boxes and four `gimbal1…4_roll_module` cylinders. Their dimensions and collision origins come from the robot description, and KDL supplies their poses, including joint offsets. Collision queries use the six body-joint angles and always set gimbal roll/pitch to zero because the cylinders represent swept envelopes.

The planner receives full probability trees on `octomap/full` from the official `octomap_server` through the small `octomap_mapper` integration package; occupied leaves (`p >= 0.5`) enter collision queries directly, with zero extra margin. The upstream server ray-traces timestamped scans from the physical sensor origin and incrementally fuses free and occupied observations. `DilateRadius` (`dilate_radius`, default 0.20 m) applies only to root-route guidance. Each map message is decoded once and atomically replaces the route backend and collision tree in one immutable scene snapshot. Pruned leaves cover their complete volume in both representations. Missing cells inside the map are free; the entire box/cylinder volume must remain inside the grid bounds. `CommandHz` sets discrete trajectory collision sampling, including the first and final configurations.

The `pcd_filter` launch enables [single-frame noise filtering](../pcd_filter/README.md#usage) by default, requiring two other points within 0.20 m before a point can contribute a hit or free ray. The planner launch forwards `enable_noise_filter`, `noise_filter_radius`, and `noise_filter_min_neighbors`; for example, append `noise_filter_min_neighbors:=3` to require three neighbors or `enable_noise_filter:=false` to admit isolated returns.

A standard TF publisher locates `dragon/octomap_grid`, and the official server publishes occupancy markers on scan updates at `/dragon/octomap/occupied_cells`. Run `roslaunch motion_primitive_planner rviz.launch` to visualize the occupied leaf volumes. `sensor_origin_frame_id` (default `dragon/lidar_origin`) selects the physical ray origin: the filter outputs XYZ in that frame so the server casts rays from the physical LiDAR origin. The frame is a link in the robot description, published by `robot_state_publisher`. MapBound constrains planning only; the tree and RViz can include outside obstacles. Empty scans retain history; use `/dragon/octomap/reset` to clear it. See the [mapper interfaces and coordinate conventions](../octomap_mapper/README.md).

[whole_body_motion_primitive_planner.yaml](config/whole_body_motion_primitive_planner.yaml) contains the map/path, primitive, root-attitude, terminal IK, flight-feasibility, joint-planning, and execution settings. `JointReferenceDt` controls root-attitude prediction; terminal tail targets are computed directly on the remaining MINCO curve and aligned initial-body segments, independently of trajectory-history sampling. The launch file loads this configuration and then applies its launch-argument overrides. The default replanning ratio is `0.3`, the activation lead time is `0.75 s`, and commands are published at `40 Hz`. Root-route guidance uses `gcopter_planner::RoutePlannerConfig` and `RoutePlannerBackend`; GCOPTER trajectory-optimizer parameters are not loaded or required.
