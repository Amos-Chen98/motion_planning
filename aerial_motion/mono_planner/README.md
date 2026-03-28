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

## Waypoint-Conditioned Demo

This demo is intended for holonomic robots.

First, launch the waypoint pose publisher:

```bash
roslaunch mono_planner waypoint_pose_publisher.launch
```

Then publish the current root tail pose:

```bash
rostopic pub -r 10 /root/tail_pose geometry_msgs/PoseStamped "header:
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

Finally, launch the waypoint-conditioned planner:

```bash
roslaunch mono_planner waypoint_conditioned_planner.launch
```

## Nonholonomic Waypoint-Conditioned Demo

This variant keeps the same ROS interface as the holonomic demo, but the published
`root/target_pose` orientation is constrained by the trajectory tangent so that the
robot body `X` axis always points along the motion direction. Unlike the holonomic
variant, the nonholonomic planner also treats the orientations in `root/tail_pose`
and `waypoint/pose_x` as hard constraints at those knots. If any waypoint or root
orientation is incompatible with a forward-moving nonholonomic trajectory, the node
will reject the plan instead of publishing a command sequence.

To run the nonholonomic demo:

1. Launch the waypoint publisher:

```bash
roslaunch mono_planner waypoint_pose_publisher.launch
```

2. Publish the root pose shown above on `/root/tail_pose`.

3. Launch the nonholonomic planner:

```bash
roslaunch mono_planner waypoint_conditioned_planner.launch nonholo:=true
```

When you prepare your own waypoint file, each waypoint orientation must be
compatible with the closed-loop path order
`root -> waypoint_0 -> waypoint_1 -> ... -> root`. In practice, the body `X` axis
defined by each waypoint orientation must point generally along the local forward
travel direction. If a waypoint orientation points sideways or backwards relative to
either adjacent path segment, the planner will report that the segment is
incompatible and no trajectory will be published.

The same launch file now supports both planners:

```bash
# Holonomic planner
roslaunch mono_planner waypoint_conditioned_planner.launch

# Nonholonomic planner
roslaunch mono_planner waypoint_conditioned_planner.launch nonholo:=true
```
