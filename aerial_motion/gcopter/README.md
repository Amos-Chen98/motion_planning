# GCOPTER

## Project Configuration

This is a ROS catkin package. The main configuration files are:

- `CMakeLists.txt`: builds the planning and simulation nodes and links Eigen3, OMPL, and ROS components.
- `package.xml`: declares ROS package dependencies.
- `config/gcopter_planner_config.yaml`: stores GCOPTER planner parameters.
- `config/livox_mid360_simulator_config.yaml`: stores Livox Mid-360 simulator range/FOV parameters.
- `launch/demo.launch`: starts the mock map generator, point-robot model, planner mode, and RViz.
- `launch/point_robot_model.launch`: reusable point-robot `nav_msgs/Odometry` model for the standalone demo.
- `launch/gcopter_planner.launch`: reusable headless launch that starts either the global planner or the online local-perception planner.
- `launch/livox_mid360_simulator.launch`: geometric Livox Mid-360 point-cloud range/FOV simulator.

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
- `gcopter/livox_mid360_simulator` in online mode
- `mockamap/mockamap_node`
- `gcopter/point_robot_model.py`
- `rviz`

Make sure the `mockamap` package is available in the same catkin workspace. In this repository, it is located at:

```text
src/motion_planning/aerial_motion/map_gen/mockamap
```

### Main Parameters

Parameter files:

```text
config/gcopter_planner_config.yaml
config/livox_mid360_simulator_config.yaml
```

Topics (set via `<remap>`, see `launch/gcopter_planner.launch`):

- `pcl_topic`: input point cloud for the active planner. In `global` mode it is the world-frame obstacle map consumed by `global_planning` (e.g. `/voxel_map`); in `online` mode it is the body-frame local point cloud consumed by `online_replanning` (e.g. `/simulated_livox_points`).
- `target`: RViz goal topic. Default: `/move_base_simple/goal`.
- `odom`: odometry used as the planning start position. Planning always starts from the latest odometry; a target is ignored until a valid odometry message is received. Default: `quadrotor/uav/cog/odom`.
- `command`: `geometry_msgs/PoseStamped` position commands are always published (streamed at `CommandHz` along the planned trajectory). Default: `quadrotor/target_pose`.

Livox Mid-360 simulator topics (online mode, see `launch/livox_mid360_simulator.launch`):

- `global_pcl_topic`: world-frame obstacle cloud consumed by `livox_mid360_simulator`. Default: `/voxel_map`.
- `local_pcl_topic`: body-frame local point cloud produced by `livox_mid360_simulator` and fed to `online_replanning` through `pcl_topic`. Default: `/simulated_livox_points`.

Frame convention in online mode:

- `world`: global map, target, odometry parent, visualization, and command frame.
- `camera_init`: fixed reference frame published by the standalone point-robot demo from `spawn_x/y/z/yaw`.
- `quadrotor/cog`: moving robot frame and odometry child frame.
- `body`: lidar IMU frame and local point-cloud frame, rigidly attached below `quadrotor/cog`.

The demo TF chain is `world -> quadrotor/cog -> body`. The first transform is
updated from robot odometry; the second is the fixed lidar extrinsic. The
separate `world -> camera_init` transform is a fixed reference and is not a
parent of the lidar frame.

Common parameters:

- `FrameId`: frame used for visualization and command headers in global mode. Default: `world`.
- `VoxelWidth`: voxel resolution.
- `DilateRadius`: obstacle inflation radius.
- `MapBound`: planning map bounds, formatted as `[xmin, xmax, ymin, ymax, zmin, zmax]`.
- `CommandHz`: streaming rate of the position commands along the planned trajectory.
- `UseFixedTargetHeight`, `TargetHeight`: fix goal height to `TargetHeight`.
- `UseTargetZ`: use the target message's `position.z` as the goal height (for a 3D goal source such as an interactive marker). When neither `UseFixedTargetHeight` nor `UseTargetZ` is set, the height is derived from the RViz goal orientation.
- `TimeoutRRT`: RRT search timeout.
- `MaxVelMag`, `MaxBdrMag`, `MaxTiltAngle`: velocity, body-rate, and tilt constraints.
  Online mode verifies the continuous optimized trajectory against `MaxVelMag`
  and time-scales any violating trajectory before publishing commands.
- `GravAcc`: gravitational acceleration.
- `WeightT`, `ChiVec`, `SmoothingEps`, `IntegralIntervs`, `RelCostTol`: optimizer parameters.

Online parameters:

- `WorldFrameId`: frame used for online visualization and command headers. Default: `world`.
- `RobotFrameId`: robot frame represented by the odometry pose. Default: `quadrotor/cog`.
- `LidarImuFrameId`: local point-cloud frame published by the Livox Mid-360 simulator. Default: `body`.
- `camera_init_frame_id`: fixed reference frame published by the standalone point-robot model. Default: `camera_init`.
- `ReplanHz`: online replanning rate. Default: `2.0`.
- `UseAccumulatedMap`: keep previously observed local obstacles. Default: `true`.
- `PublishRate`, `LivoxMinRange`, `LivoxMaxRange`, `LivoxHorizontalFovDeg`, `LivoxVerticalMinDeg`, `LivoxVerticalMaxDeg`: Livox Mid-360 simulator rate and geometric limits. Defaults: `10.0`, `0.1`, `2.0`, `360.0`, `-7.0`, `52.0`.

### Reusable Headless Launch

Use `gcopter_planner.launch` when another package provides the map,
visualization, and robot stack. This launch starts only one planner node; it
does not start the mock map generator, Livox simulator, point robot, or RViz.

Set `planner_mode:=global` or `planner_mode:=online`. Both planners publish
`geometry_msgs/PoseStamped` position commands, and both ignore targets until a
valid world-frame odometry message has been received.

Example closed-loop integration:

```bash
roslaunch gcopter gcopter_planner.launch \
  planner_mode:=online \
  frame_id:=world \
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

This launch starts the mock map generator, point-robot model, selected planner,
and RViz for the visual demo. Online mode also starts the Livox Mid-360
simulator. The point robot is identical in global and online modes: it publishes
`world -> body` odometry on `quadrotor/uav/cog/odom`. The same
`spawn_x/y/z/yaw` values define the robot's initial world pose and the fixed
`world -> camera_init` transform used for TF visualization.
`ideal_robot_publish_rate` controls only the odometry/TF publication rate; the
translational velocity limit is `max_vel_mag`.

To change the demo initial pose or run the global planner:

```bash
roslaunch gcopter demo.launch spawn_x:=1.0 spawn_y:=0.0 spawn_z:=1.0 spawn_yaw:=1.57
roslaunch gcopter demo.launch planner_mode:=global
```

Then, in RViz, use `2D Nav Goal`:

1. Click to set the goal point and trigger planning. The start point is taken
   from the latest robot odometry.
2. Continue clicking to set new goals.

The `2D Nav Goal` arrow direction determines the relative target height. The
planned trajectory is displayed in RViz. Speed, tilt angle, and body rate are
published on `/visualizer/speed`, `/visualizer/tilt_angle`, and
`/visualizer/body_rate`, and can be plotted with `rqt_plot`.
