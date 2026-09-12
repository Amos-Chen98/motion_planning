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
| [local_map.cpp](src/local_map.cpp) | Local bitmap validation, route import, and collision OcTree construction. |
| [joint_trajectory_planner.cpp](src/joint_trajectory_planner.cpp) | Trajectory history for scoring, root-attitude prediction, analytic terminal joint targets, joint search, attitude allocation, timing, and interpolation. |
| [dragon_geometry.cpp](src/dragon_geometry.cpp) | Robot metadata, attitude and frame transforms, and nominal link geometry. |
| [dragon_collision_checker.cpp](src/dragon_collision_checker.cpp) | Eight URDF primitives, collision-only KDL workspaces, and immutable OctoMap environments. |
| [planner_config.cpp](src/planner_config.cpp) | Parameter loading and validation. |

Headers follow the same responsibilities. `WholeBodyPlanner::plan()` receives a root-candidate batch and its `PlanningSceneSnapshot` (route backend and raw collision map), predicted start joints and attitude, nominal context, and a ROS-time deadline; it returns candidate results and the selected index. A dedicated map worker constructs and atomically publishes complete scenes. The planning worker captures one scene before generating the root batch and uses that same version throughout candidate evaluation; it does not lock the live mapper. Execution state remains in the node; its small replanning and diagnostic helpers are defined in `whole_body_planner_node.h` for independent testing.

## Build

Install Coal 3 with OctoMap support, OctoMap, and the URDF/KDL development dependencies. Make the Coal installation discoverable through `CMAKE_PREFIX_PATH` or `coal_DIR`; CMake links the `coal::coal` imported target.

Install the [self-filter dependencies](../pcd_filter/README.md#dependencies-and-build) first; the default launch starts both the filter and mapper.

```bash
cd motion_planning_ws
source ../jsk_aerial_robot_ws/devel/setup.bash
catkin build pcd_filter rog_map_msgs rog_map motion_primitive_planner
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

The planner receives `rog_map_msgs/LocalMap` snapshots on `rog_map/local_map` from a separate ROG node. Each snapshot carries raw and incrementally inflated occupancy, its observation time, version, and world-aligned sliding-window bounds. A map worker builds an occupied-only local OcTree for the unchanged Coal backend and imports the already inflated route bitmap without repeating dilation. No OctomapServer, global map, or tree serialization is used in this workflow. The requested window defaults to 12 × 12 × 8 m at 0.10 m resolution, rounded upward to ROG's odd-cell layout. `DilateRadius` defaults to 0.20 m and applies only to root guidance; Coal keeps zero extra margin. Unknown cells inside the window are traversable, while every complete box/cylinder must fit within its half-open bounds. Planning waits for the first valid snapshot.

The `pcd_filter` launch enables [single-frame noise filtering](../pcd_filter/README.md#usage) by default, requiring two other points within 0.20 m before a point can contribute a hit or free ray. The planner launch forwards `enable_noise_filter`, `noise_filter_radius`, and `noise_filter_min_neighbors`; for example, append `noise_filter_min_neighbors:=3` to require three neighbors or `enable_noise_filter:=false` to admit isolated returns.

`sensor_origin_frame_id` (default `dragon/lidar_origin`) selects the physical ray origin. The mapper resolves cloud and sensor transforms at the scan timestamp; no moving grid TF is required. There is no fixed MapBound activity restriction. The original world goal is retained across window shifts, with a local projected endpoint and bounded nearby alternatives selected on each planning attempt. Reaching a window endpoint does not mark the real goal reached. Empty scans retain local history; `/dragon/rog_map/reset` clears it. Local occupancy and bounds are visualized on `/dragon/rog_map/occupied_cells` at 2 Hz when subscribed. See the [ROG node and snapshot contract](../../../ROG-Map/rog_map/README.md). Desktop validation, replay conditions and reproduction commands are recorded in [local map benchmarks](doc/local_map_benchmark.md).

[whole_body_motion_primitive_planner.yaml](config/whole_body_motion_primitive_planner.yaml) contains the map/path, primitive, root-attitude, terminal IK, flight-feasibility, joint-planning, and execution settings. `JointReferenceDt` controls root-attitude prediction; terminal tail targets are computed directly on the remaining MINCO curve and aligned initial-body segments, independently of trajectory-history sampling. The launch file loads this configuration and then applies its launch-argument overrides. The default replanning ratio is `0.3`, the activation lead time is `0.75 s`, and commands are published at `40 Hz`. Root-route guidance uses `gcopter_planner::RoutePlannerConfig` and `RoutePlannerBackend`; GCOPTER trajectory-optimizer parameters are not loaded or required.

## Mapping diagnostics

`/dragon/rog_map/diagnostics` reports fusion and export timing; `/dragon/planning/map_diagnostics` reports scene construction, update frequency and map age. `/dragon/planning/map_ready` sends the processed observation header only when subscribed, allowing a benchmark observer to match filtered scans to available scenes without retransmitting maps. `rosrun rog_map measure_local_map.py --duration 90 --output /tmp/local_map_metrics.json` measures this chain. New maps do not automatically revalidate executing trajectories; the existing execution/replanning policy remains in force.
