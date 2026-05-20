# Geometry-Only Waypoint Simulation

This demo runs the geometry-only nominal multilink copilot in RViz. It does not
start Gazebo, does not require TF from a robot model, and does not require a
URDF display. The upstream `mono_planner` nonholonomic waypoint-conditioned
planner publishes the root-link tail reference on `/dragon/root/target_pose`,
and `geometry_only_copilot_planner` converts that reference into nominal
multilink body motion.

The launch file forces `/use_sim_time` to `false` so that it works in a
no-Gazebo session without a `/clock` publisher.

## Run

From the workspace root:

```bash
source devel/setup.bash
roslaunch multilink_copilot geometry_only_waypoint_sim.launch link_num:=4
```

Any positive integer link count is accepted. Run any nominal link-count case by setting `link_num`:

```bash
roslaunch multilink_copilot geometry_only_waypoint_sim.launch link_num:=3
roslaunch multilink_copilot geometry_only_waypoint_sim.launch link_num:=4
roslaunch multilink_copilot geometry_only_waypoint_sim.launch link_num:=8
roslaunch multilink_copilot geometry_only_waypoint_sim.launch link_num:=10
```

For a command-line smoke test without RViz:

```bash
roslaunch multilink_copilot geometry_only_waypoint_sim.launch link_num:=4 launch_rviz:=false
```

## Useful Arguments

- `link_num`: nominal serial-link count. Any positive integer is supported;
  use `3` through `10` to reproduce the geometry-only link-count sweep in the
  paper.
- `link_length`: nominal link length in meters. Default: `0.455`.
- `waypoint_config`: waypoint YAML path relative to the `mono_planner`
  package. Default: `test_data/case_0001.yaml`.
- `total_trajectory_time`: duration of the waypoint-conditioned root-link tail
  trajectory. Default: `60.0`.
- `launch_rviz`: whether to open RViz with the geometry-only marker config.
  Default: `true`.

Example with a different waypoint case:

```bash
roslaunch multilink_copilot geometry_only_waypoint_sim.launch \
  link_num:=5 \
  waypoint_config:=test_data/case_0008.yaml
```

## Main Topics

- `/dragon/root/target_pose`: root-link tail reference from
  `nonholo_wpt_cond_planner`.
- `/dragon/full_state_target`: nominal full-state target from the
  geometry-only copilot. The joint count is `2 * (link_num - 1)`.
- `/dragon/geometry/root_link_tail_pose`: current root-link tail pose used by
  the geometry-only copilot.
- `/dragon/geometry/last_link_tail_pose`: current last-link tail pose.
- `/dragon/geometry/trajectory_markers`: RViz markers for the full
  time-colored root-link tail trajectory, thin black nominal link cylinders,
  and follower tail points.
- `/dragon/mono_planner/traj_marker`: upstream root-link trajectory and
  waypoint markers from `mono_planner`. The demo RViz view hides these raw
  markers and uses the trajectory line as the source for the time-colored
  root-link visualization.

## Expected Startup

The launch starts:

- `root_tail_pose_seed_publisher.py`, which provides the initial
  `/dragon/root/tail_pose` seed.
- `waypoint_pose_publisher.py`, which publishes the waypoint set.
- `nonholo_wpt_cond_planner.py`, which publishes `/dragon/root/target_pose`.
- `geometry_only_copilot_planner`, which publishes nominal geometry-only body
  motion and RViz markers.

If RViz opens but no body trajectory appears, check that `/dragon/root/target_pose`
and `/dragon/geometry/trajectory_markers` are publishing.
