# Motion Primitive Planner

`motion_primitive_planner` provides two online local-planning chains for DRAGON: a root-link planner for `gcopter/traj_server` with an optional `multilink_copilot`, and a whole-body planner that publishes `aerial_robot_msgs/FullStateTarget` directly.

For algorithm details, see [Algorithm Design](doc/algorithm_design.md).

## Build

```bash
cd motion_planning_ws
catkin build motion_primitive_planner
source devel/setup.bash
```

## Run

Ensure that the DRAGON robot description and required input topics are available before launching either chain.

### Root-Link Planner

```bash
roslaunch motion_primitive_planner motion_primitive_planner.launch
```

`gcopter/traj_server` executes the root-link trajectory. Add `multilink_copilot` only when stability projection is required.

### Whole-Body Planner

The whole-body planner starts `root_state_to_flu_odom.launch` automatically. It converts the root-tail pose from `/dragon/root/tail_pose` to FLU odometry on `/dragon/root/flu_odom`.

```bash
roslaunch motion_primitive_planner whole_body_motion_primitive_planner.launch
```

Do not start `gcopter/traj_server` or `multilink_copilot` full-state output with this chain.
