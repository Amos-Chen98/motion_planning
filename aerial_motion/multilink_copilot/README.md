# multilink_copilot

Generate full-state targets for multilink aerial robots from a first-link-tail motion reference.

## Dependencies

[jsk_aerial_robot integration](https://github.com/jsk-ros-pkg/jsk_aerial_robot/pull/758).

## Copilot Planner

```bash
roslaunch multilink_copilot copilot_planner.launch
```

The launch loads [config/copilot_planner.yaml](config/copilot_planner.yaml). Set `robot_ns` to select the robot namespace (default: `dragon`) and `target_pose_frame_type` to select the target orientation convention (default: `FLU`). `LINK` uses the link frame; `FLU` uses Forward-Left-Up and applies a 180-degree rotation about the local Z axis internally.

Topics are relative to the robot namespace:

| Topic | Direction | Message type | Purpose |
| --- | --- | --- | --- |
| `root/target_pose` | Input | `geometry_msgs/PoseStamped` | First-link tail pose reference |
| `joint_states` | Input | `sensor_msgs/JointState` | Measured joint configuration |
| `full_state_target` | Output | `aerial_robot_msgs/FullStateTarget` | Robot root and joint targets |
| `trajectory_visualization` | Output | `visualization_msgs/MarkerArray` | Tracked trajectory |

The planner derives the robot root pose from the tail reference and begins full-state publication after receiving a complete joint state. During snake-mode startup, it combines the available tail trajectory with measured body geometry to reconstruct nominal joints. Root translation and rotation thresholds in the YAML control when full-state updates are published.

## Demo Workflows

Each workflow supplies `root/target_pose` to the copilot planner. Dragon bringup provides arming, takeoff, landing, and teleoperation.

### Dragon Dance

The dance launch starts the planner with the `FLU` convention and coordinates entry motion, repeated dance cycles, and recovery.

1. Start Dragon in simulation:

   ```bash
   roslaunch dragon bringup.launch demo:=false rm:=false sim:=true headless:=false
   ```

2. Use bringup teleoperation to take off and enter `HOVER`.

3. Start the dance from an interactive terminal:

   ```bash
   roslaunch multilink_copilot dragon_dance.launch
   ```

The demo waits for robot feedback, control connections, and `HOVER`. Entry begins when measured linear speed and yaw rate are within 10% of their configured entry limits. A smooth quintic reference moves the robot to the entry pose, with duration calculated from displacement, yaw change, and speed limits. The robot holds that pose until tracking errors satisfy their tolerances and the first valid planner output arrives, then repeats the configured dance cycle (60 seconds at 40 Hz by default). Motion timing follows ROS time, and each stage advances when its completion conditions are met.

Press `q` or call the stop service to begin recovery:

```bash
rosservice call /dragon/dragon_dance/stop "{}"
```

Recovery holds position until the speed conditions are met, restores the configured planar joint shape, and levels baselink roll and pitch while preserving yaw. Once the leveling reference finishes and joint and attitude errors satisfy their tolerances, the dance launch exits. Land through bringup teleoperation. `Ctrl+C` terminates the launch immediately; `q` and the stop service run the recovery sequence.

The demo publishes one status message and log entry on startup and each stage transition. The latched status topic retains the latest stage message:

```bash
rostopic echo /dragon/dragon_dance/status
```

Set trajectory geometry, entry speeds, convergence tolerances, keyboard input, and recovery targets in [config/dragon_dance.yaml](config/dragon_dance.yaml). For a custom configuration or namespace:

```bash
roslaunch multilink_copilot dragon_dance.launch robot_ns:=dragon config_file:=/absolute/path/to/dragon_dance.yaml
```

Use the selected namespace in the service and topic commands above.

### Waypoint-conditioned Maneuvering

Start Dragon through bringup and enter `HOVER`, then run these commands in separate terminals, in order:

```bash
roslaunch mono_planner waypoint_pose_publisher.launch
roslaunch multilink_copilot copilot_planner.launch target_pose_frame_type:=FLU
roslaunch mono_planner waypoint_conditioned_planner.launch nonholo:=true robot_frame_type:=LINK total_trajectory_time:=50
```

The waypoint publisher loads `mono_planner/config/demo_waypoints.yaml`. The waypoint-conditioned planner generates the `FLU` tail reference, and copilot converts it into full-state targets.

### Human-guided Shared Control

Use the [copilot_joystick branch of jsk_aerial_robot](https://github.com/Amos-Chen98/jsk_aerial_robot/tree/copilot_joystick). Launch these commands in separate terminals:

```bash
roslaunch dragon bringup.launch
roslaunch aerial_robot_base joy_stick.launch robot_name:=dragon
roslaunch multilink_copilot copilot_planner.launch target_pose_frame_type:=FLU
```

Arm, take off, and enter `HOVER` through bringup teleoperation. `DragonRootTargetNavigator` then publishes joystick-driven tail references in the `FLU` convention.

| Control | Action |
| --- | --- |
| `R2` | Move forward along body `+X` |
| Left stick horizontal | Control yaw |
| Left stick vertical | Control pitch |

Use the standard bringup buttons for arming, takeoff, landing, and stopping.

## Evaluation

### Replay Rosbag

Select a recording from `data_manager/dragon_copilot/data/rosbag` and open it in RViz:

```bash
rosrun multilink_copilot replay_rosbag.py
```

Use `--latest` to select the newest bag or `--pause` to start playback paused. To launch a specific recording:

```bash
roslaunch multilink_copilot replay_rosbag.launch bag:=/absolute/path/to/recording.bag
```

Replay uses `--clock`, loads `dragon/robots/quad/v1_5_202601.urdf.xacro`, and opens `mono_planner/rviz/wpt_cond_manv.rviz`. The TF bridge forwards the recorded `world -> dragon/root` transform, while `robot_state_publisher` reconstructs the link tree from joint states.

### Envelope Width

Run evaluation during a live waypoint maneuver or rosbag replay:

```bash
roslaunch multilink_copilot run_evaluation.launch
```

The launch publishes `last_link/tail_pose` from `root/pose` and the root-to-last-link TF relation, then measures root and tail crossings of waypoint ring planes. JSON results are saved to `data_manager/dragon_copilot/data/envelope_width`.

Set `robot_ns` and `output_dir` to customize the run:

```bash
roslaunch multilink_copilot run_evaluation.launch robot_ns:=dragon output_dir:=/tmp/envelope_width
```

To start the planner and evaluator together:

```bash
roslaunch multilink_copilot copilot_planner.launch evaluation:=true
```
