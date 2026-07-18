# GCOPTER

## Project Configuration

This is a ROS catkin package. The main configuration files are:

- `CMakeLists.txt`: builds the planning nodes and links Eigen3, OMPL, and ROS components.
- `package.xml`: declares ROS package dependencies.
- `config/gcopter_planner_config.yaml`: stores GCOPTER planner parameters.
- `../geo_robot_model/config/livox_mid360_simulator_config.yaml`: stores Livox Mid-360 simulator range/FOV parameters.
- `include/gcopter/planner_common.hpp` and `src/planner_common.cpp`: shared configuration, voxel-map/path/corridor/optimization backend, TF and odometry handling, point-cloud decoding, visualization access, and trajectory publishing for both planner modes.
- `launch/demo.launch`: starts the mock map generator, point-robot model, planner mode, and RViz.
- `launch/gcopter_planner.launch`: reusable headless launch that starts either the global planner or the online local-perception planner.
- `../geo_robot_model/launch/point_robot_bringup.launch`: reusable point-robot `nav_msgs/Odometry` model used by the standalone demo.
- `../geo_robot_model/launch/livox_mid360_simulator.launch`: geometric Livox Mid-360 point-cloud range/FOV simulator.

### System Dependencies

```bash
sudo apt update
sudo apt install python3-catkin-tools libompl-dev
```

Optional CPU performance mode:

```bash
sudo apt install cpufrequtils
sudo cpufreq-set -g performance
```

### ROS Dependencies

The demo launch file starts:

- `gcopter/online_replanning` by default, or `gcopter/global_planning` with `planner_mode:=global`
- `geo_robot_model/livox_mid360_simulator` in online mode
- `geo_robot_model/point_robot_model.py`
- `geo_robot_model/pose_to_flight_nav.py`
- `mockamap/mockamap_node`
- `rviz`

Make sure the `mockamap` package is available in the same catkin workspace. In this repository, it is located at:

```text
src/motion_planning/aerial_motion/map_gen/mockamap
```

### Main Parameters

Parameter files:

```text
config/gcopter_planner_config.yaml
../geo_robot_model/config/livox_mid360_simulator_config.yaml
```

Topics (set via `<remap>`, see `launch/gcopter_planner.launch`):

- `pcl_topic`: input point cloud for the active planner. In `global` mode it is the world-frame obstacle map consumed by `global_planning` (e.g. `/voxel_map`); in `online` mode it is the LiDAR-frame local point cloud consumed by `online_replanning` (e.g. `/simulated_livox_points`).
- `target`: RViz goal topic. Default: `/move_base_simple/goal`.
- `odom`: odometry used as the planning start position. Planning always starts from the latest odometry; a target is ignored until a valid odometry message is received. Default: `quadrotor/uav/cog/odom`.
- `command`: `geometry_msgs/PoseStamped` position commands are always published (streamed at `CommandHz` along the planned trajectory). Default: `quadrotor/target_pose`.

In the standalone demo, `geo_robot_model/pose_to_flight_nav.py` converts the `PoseStamped` command stream to `aerial_robot_msgs/FlightNav` on `quadrotor/uav/nav`; `geo_robot_model/point_robot_model.py` consumes that command and publishes `nav_msgs/Odometry` on `quadrotor/uav/cog/odom`.

Livox Mid-360 simulator topics (online mode, see `../geo_robot_model/launch/livox_mid360_simulator.launch`):

- `global_pcl_topic`: world-frame obstacle cloud consumed by `livox_mid360_simulator`. Default: `/voxel_map`.
- `local_pcl_topic`: LiDAR-frame local point cloud produced by `livox_mid360_simulator` and fed to `online_replanning` through `pcl_topic`. Default: `/simulated_livox_points`.

Frame convention in online mode:

- `world`: global map, target, odometry parent, visualization, and command frame.
- `quadrotor/cog`: moving robot frame and odometry child frame.
- `quadrotor/lidar_imu`: lidar IMU frame and local point-cloud frame, rigidly attached below `quadrotor/cog`.

The demo TF chain is `world -> quadrotor/cog -> quadrotor/lidar_imu`. The point-robot model updates the first transform from robot odometry, and point-robot bringup publishes the mini-quadrotor-compatible fixed extrinsic for the second transform. The Livox simulator only consumes this TF chain and never publishes `/tf` or `/tf_static`.

Starting the Livox simulator by itself requires another robot bringup to already publish odometry and `RobotFrameId -> LidarImuFrameId`. While either input is missing, the simulator waits and does not publish a local point cloud; a missing TF additionally produces throttled warnings.

Common parameters:

- `WorldFrameId`: frame used for planning and visualization in both modes, and for command headers in `traj_server`. Default: `world`.
- `VoxelWidth`: voxel resolution.
- `DilateRadius`: obstacle inflation radius.
- `MapBound`: planning map bounds, formatted as `[xmin, xmax, ymin, ymax, zmin, zmax]`.
- `UseFixedTargetHeight`, `TargetHeight`: fix goal height to `TargetHeight`.
- `UseTargetZ`: use the target message's `position.z` as the goal height (for a 3D goal source such as an interactive marker). When neither `UseFixedTargetHeight` nor `UseTargetZ` is set, the height is derived from the RViz goal orientation.
- `TimeoutRRT`: RRT search timeout.
- `MaxVelMag`, `MaxBdrMag`, `MaxTiltAngle`: velocity, body-rate, and tilt constraints. Both modes verify the continuous optimized trajectory against `MaxVelMag` and time-scale any violating trajectory before publishing it.
- `GravAcc`: gravitational acceleration.
- `WeightT`, `ChiVec`, `SmoothingEps`, `IntegralIntervs`, `RelCostTol`: optimizer parameters.

Trajectory execution parameters are consumed only by `traj_server`:

- `CommandHz`: streaming rate of position commands.
- `PublishYawCommand`: align command yaw with horizontal trajectory velocity; otherwise preserve measured yaw. The standalone demo forwards this setting to the Pose-to-FlightNav bridge.

Both planners validate map bounds, voxel resolution, dynamic limits, and optimizer vectors during startup. Invalid configurations terminate with a descriptive fatal error. Point clouds are decoded by XYZ field name, so field reordering and padding are supported while non-finite points are ignored.

Online parameters:

- `RobotFrameId`: robot frame represented by the odometry pose. Default: `quadrotor/cog`.
- `LidarImuFrameId`: existing robot TF frame used for the local point-cloud header and LiDAR pose lookup. Default: `quadrotor/lidar_imu`.
- `ReplanHz`: online replanning rate. Default: `2.0`.
- `UseAccumulatedMap`: keep previously observed local obstacles. Default: `true`.
- `PublishRate`, `LivoxMinRange`, `LivoxMaxRange`, `LivoxHorizontalFovDeg`, `LivoxVerticalMinDeg`, `LivoxVerticalMaxDeg`: Livox Mid-360 simulator rate and geometric limits. Defaults: `10.0`, `0.1`, `2.0`, `360.0`, `-7.0`, `52.0`.

### Reusable Headless Launch

Use `gcopter_planner.launch` when another package provides the map, visualization, and robot stack. This launch starts only one planner node; it does not start the mock map generator, Livox simulator, point robot, command bridge, or RViz.

Set `planner_mode:=global` or `planner_mode:=online`. Both planners publish `gcopter/PolyTraj` trajectories for `traj_server`, which streams `geometry_msgs/PoseStamped` position commands. Both planners ignore targets until valid world-frame odometry has been received.

Example closed-loop integration:

```bash
roslaunch gcopter gcopter_planner.launch \
  planner_mode:=online \
  world_frame_id:=world \
  pcl_topic:=/simulated_livox_points \
  target_topic:=/move_base_simple/goal \
  odom_topic:=quadrotor/uav/cog/odom \
  command_topic:=quadrotor/target_pose
```

## Minimal Demo

Run the following commands from the catkin workspace root:

```bash
catkin build gcopter
source devel/setup.bash
roslaunch gcopter demo.launch
```

This launch starts the mock map generator, geo point-robot model, Pose-to-FlightNav bridge, selected planner, and RViz for the visual demo. Online mode also starts the Livox Mid-360 simulator. The command loop is `PoseStamped -> FlightNav -> Odometry`; the point robot is identical in global and online modes and publishes odometry on `/quadrotor/uav/cog/odom`. The `spawn_x/y/z/yaw` values define its initial world pose, `ideal_robot_publish_rate` controls only odometry/TF publication, and the translational velocity limit is `max_vel_mag`.

To change the demo initial pose or run the global planner:

```bash
roslaunch gcopter demo.launch spawn_x:=1.0 spawn_y:=0.0 spawn_z:=1.0 spawn_yaw:=1.57
roslaunch gcopter demo.launch planner_mode:=global
```

Then, in RViz, use `2D Nav Goal`:

1. Click to set the goal point and trigger planning. The start point is taken from the latest robot odometry.
2. Continue clicking to set new goals.

The `2D Nav Goal` arrow direction determines the relative target height. The planned trajectory is displayed in RViz. Speed, tilt angle, and body rate are published on `/visualizer/speed`, `/visualizer/tilt_angle`, and `/visualizer/body_rate`, and can be plotted with `rqt_plot`.
