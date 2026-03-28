# mono_planner
Motion planner for monolithic (rigid-body) robots. Plans from point cloud input.

## Dependencies

```
sudo apt install ros-one-octomap* # change the ROS version according to your system
pip install octomap-python
```

If you encounter an error while installing octomap-python, run the following command instead:

```
CXXFLAGS="-std=c++11" pip install octomap-python
```

If problem still exists, refer to https://github.com/wkentaro/octomap-python for more solutions.

If it reports missing `libdynamicedt3d.so.1.8` while running, add the following line to `.bashrc`:

```
export LD_LIBRARY_PATH=~/.local/lib:$LD_LIBRARY_PATH
```

## Waypoint-Conditioned Planners

`waypoint_conditioned_planner.launch` supports two planner variants that share the
same ROS interface:

- Holonomic planner: intended for rigid-body robots that can track arbitrary pose
  trajectories.
- Nonholonomic planner: constrains the published orientation to follow the
  trajectory tangent and enforces forward-motion feasibility at each knot.

### Common Interface

Both variants use the same topics and launch arguments:

- Input `root/tail_pose` (`geometry_msgs/PoseStamped`): robot root pose input.
- Input `waypoint/pose_x` (`geometry_msgs/PoseStamped`): waypoint topics discovered
  automatically from the ROS graph.
- Output `root/target_pose` (`geometry_msgs/PoseStamped`): sampled target pose
  along the planned trajectory.
- Launch args:
  `nonholo`, `publish_rate_hz`, `total_trajectory_time`, `robot_frame_type`,
  `robot_pose_topic`, `target_pose_topic`, and `ns`.

### Common Behavior

Both variants share the same replanning and root-pose semantics:

- The first received `root/tail_pose` is latched as the fixed terminal pose for all
  future plans.
- The latest received `root/tail_pose` is used as the initial pose when a new plan
  is generated.
- The first plan is attempted automatically once the required root and waypoint
  inputs are available.
- Replans are triggered only by waypoint content changes or waypoint topic-set changes. The replan takes the latest `root/tail_pose` as the initial state.
- In `LINK` mode, the incoming `root/tail_pose` orientation is rotated by 180
  degrees about the robot's local `Z` axis before planning. This is specially for multi-link robots such as DRAGON.

### Quick Start

1. Launch the waypoint pose publisher:

```bash
roslaunch mono_planner waypoint_pose_publisher.launch
```

2. Publish the current root tail pose:

```bash
rostopic pub -r 10 dragon/root/tail_pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'world'
pose:
  position:
    x: 0.0
    y: 0.0
    z: 1.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

3. Launch one of the planner variants:

```bash
# Holonomic planner
roslaunch mono_planner waypoint_conditioned_planner.launch robot_frame_type:=LINK

# Nonholonomic planner
roslaunch mono_planner waypoint_conditioned_planner.launch nonholo:=true robot_frame_type:=LINK
```

### Holonomic Planner

This variant is intended for holonomic robots. It tracks the pose sequence

`current root -> waypoint_0 -> waypoint_1 -> ... -> startup root`

without imposing a tangent-following body-frame constraint on the output
orientation.

### Nonholonomic Planner

This variant keeps the same ROS interface, but the published `root/target_pose`
orientation is constrained by the trajectory tangent so that the robot body `X`
axis points along the motion direction.

Unlike the holonomic planner, the nonholonomic planner also treats the
orientations in `root/tail_pose` and `waypoint/pose_x` as hard knot constraints.
If any waypoint or root orientation is incompatible with a forward-moving
trajectory, the planner rejects the plan instead of publishing a command sequence.

When you prepare your own waypoint file, each waypoint orientation must be
compatible with the ordered path
`current root -> waypoint_0 -> waypoint_1 -> ... -> startup root`. In practice,
the body `X` axis defined by each waypoint orientation must point generally along
the local forward travel direction. If a waypoint orientation points sideways or
backwards relative to either adjacent path segment, the planner reports that the
segment is incompatible and no trajectory is published.
