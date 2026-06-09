# GCOPTER

## Project Configuration

This is a ROS catkin package. The main configuration files are:

- `CMakeLists.txt`: builds the `global_planning` node and links Eigen3, OMPL, and ROS components.
- `package.xml`: declares ROS package dependencies.
- `config/global_planning.yaml`: stores global planning parameters.
- `launch/global_planning.launch`: starts the random map generator, planner node, RViz, and `rqt_plot`.

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
- `rviz`
- `rqt_plot`

Make sure the `mockamap` package is available in the same catkin workspace. In this repository, it is located at:

```text
src/motion_planning/aerial_motion/map_gen/mockamap
```

### Main Parameters

Parameter file:

```text
config/global_planning.yaml
```

Common parameters:

- `MapTopic`: input point cloud map topic. Default: `/voxel_map`.
- `TargetTopic`: RViz goal topic. Default: `/move_base_simple/goal`.
- `VoxelWidth`: voxel resolution.
- `DilateRadius`: obstacle inflation radius.
- `MapBound`: planning map bounds, formatted as `[xmin, xmax, ymin, ymax, zmin, zmax]`.
- `TimeoutRRT`: RRT search timeout.
- `MaxVelMag`, `MaxBdrMag`, `MaxTiltAngle`: velocity, body-rate, and tilt constraints.
- `MinThrust`, `MaxThrust`, `VehicleMass`, `GravAcc`: vehicle dynamics parameters.
- `HorizDrag`, `VertDrag`, `ParasDrag`: drag parameters.
- `WeightT`, `ChiVec`, `SmoothingEps`, `IntegralIntervs`, `RelCostTol`: optimizer parameters.

## Minimal Demo

Run the following commands from the catkin workspace root:

```bash
catkin build gcopter
source devel/setup.bash
roslaunch gcopter global_planning.launch
```

RViz and `rqt_plot` will open after launch. In RViz, use `2D Nav Goal`:

1. Click once to set the start point.
2. Click again to set the goal point and trigger planning.
3. Continue clicking to set new start and goal pairs.

The `2D Nav Goal` arrow direction determines the relative target height. The planned trajectory is displayed in RViz. Speed, total thrust, tilt angle, and body rate are published for display in `rqt_plot`.
