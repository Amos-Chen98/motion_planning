# GCOPTER

## Project Configuration

This is a ROS catkin package. The main configuration files are:

- `CMakeLists.txt`: builds the `global_planning` node and links Eigen3, OMPL, and ROS components.
- `package.xml`: declares ROS package dependencies.
- `config/global_planning.yaml`: stores global planning parameters.
- `launch/global_planning.launch`: starts the mock map generator, point-robot model, planner node, and RViz.
- `launch/point_robot_model.launch`: reusable point-robot `nav_msgs/Odometry` model for the standalone demo.
- `launch/gcopter_planner.launch`: reusable headless launch that starts only the planner node.

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

- `gcopter/global_planning`
- `mockamap/mockamap_node`
- `gcopter/point_robot_model.py`
- `rviz`

Make sure the `mockamap` package is available in the same catkin workspace. In this repository, it is located at:

```text
src/motion_planning/aerial_motion/map_gen/mockamap
```

### Main Parameters

Parameter file:

```text
config/global_planning.yaml
```

Topics (set via `<remap>`, see `launch/gcopter_planner.launch`):

- `map`: input point cloud map topic. Default: `/voxel_map`.
- `target`: RViz goal topic. Default: `/move_base_simple/goal`.
- `odom`: odometry used as the planning start position. Planning always starts from the latest odometry; a target is ignored until a valid odometry message is received. Default: `quadrotor/uav/cog/odom`.
- `command`: `geometry_msgs/PoseStamped` position commands are always published (streamed at `CommandHz` along the planned trajectory). Default: `quadrotor/target_pose`.

Common parameters:

- `FrameId`: frame used for visualization and command headers. Default: `odom`.
- `VoxelWidth`: voxel resolution.
- `DilateRadius`: obstacle inflation radius.
- `MapBound`: planning map bounds, formatted as `[xmin, xmax, ymin, ymax, zmin, zmax]`.
- `CommandHz`: streaming rate of the position commands along the planned trajectory.
- `UseFixedTargetHeight`, `TargetHeight`: fix goal height to `TargetHeight`.
- `UseTargetZ`: use the target message's `position.z` as the goal height (for a 3D goal source such as an interactive marker). When neither `UseFixedTargetHeight` nor `UseTargetZ` is set, the height is derived from the RViz goal orientation.
- `TimeoutRRT`: RRT search timeout.
- `MaxVelMag`, `MaxBdrMag`, `MaxTiltAngle`: velocity, body-rate, and tilt constraints.
- `GravAcc`: gravitational acceleration.
- `WeightT`, `ChiVec`, `SmoothingEps`, `IntegralIntervs`, `RelCostTol`: optimizer parameters.

### Reusable Headless Launch

Use `gcopter_planner.launch` when another package provides the map,
visualization, and robot stack. This launch starts only `gcopter/global_planning`;
it does not start the mock map generator or RViz.

The planner always starts planning from the latest robot odometry and always
publishes `geometry_msgs/PoseStamped` position commands. A target is ignored
until a valid odometry message has been received.

Example closed-loop integration:

```bash
roslaunch gcopter gcopter_planner.launch \
  frame_id:=world \
  map_topic:=/pointcloud/output \
  target_topic:=/move_base_simple/goal \
  odom_topic:=quadrotor/uav/cog/odom \
  command_topic:=quadrotor/target_pose
```

## Minimal Demo

Run the following commands from the catkin workspace root:

```bash
catkin build gcopter
source devel/setup.bash
roslaunch gcopter global_planning.launch
```

This launch starts the mock map generator, point-robot model,
`gcopter_planner.launch`, and RViz for the visual demo. The point robot
publishes its spawn pose at `(0.0, 0.0, 1.0)` on `quadrotor/uav/cog/odom` by
default, subscribes to `quadrotor/target_pose`, and immediately mirrors each
command pose back into odometry. This creates a closed-loop demo where each new
plan starts from the latest commanded pose.

To change the point robot spawn pose or disable the point-robot model:

```bash
roslaunch gcopter global_planning.launch spawn_x:=1.0 spawn_y:=0.0 spawn_z:=1.0 spawn_yaw:=1.57
roslaunch gcopter global_planning.launch use_ideal_robot_model:=false
```

Then, in RViz, use `2D Nav Goal`:

1. Click to set the goal point and trigger planning. The start point is taken
   from the latest robot odometry.
2. Continue clicking to set new goals.

The `2D Nav Goal` arrow direction determines the relative target height. The
planned trajectory is displayed in RViz. Speed, tilt angle, and body rate are
published on `/visualizer/speed`, `/visualizer/tilt_angle`, and
`/visualizer/body_rate`, and can be plotted with `rqt_plot`.
