# multilink_copilot

This package provides a copilot planner that generates full-state targets for multilink aerial robots with reference to the movement of the head.

## 1. Dependencies

jsk_aerial_robot: [https://github.com/jsk-ros-pkg/jsk_aerial_robot/pull/758](https://github.com/jsk-ros-pkg/jsk_aerial_robot/pull/758)

## 2. Copilot Planner

The main entrypoint is:

```bash
roslaunch multilink_copilot copilot_planner.launch
```

### 2.1 Common Interface

The copilot planner exposes a shared interface regardless of how the head-motion
reference is generated:

- Input `root/target_pose` (`geometry_msgs/PoseStamped`): target pose of the
  first-link tail.
- Input `joint_states` (`sensor_msgs/JointState`): measured multilink joint
  state used to anchor startup and warmup behavior to the robot's actual shape.
- Output `full_state_target` (`aerial_robot_msgs/FullStateTarget`): full-state
  target for the multilink robot.
- Output `trajectory_visualization` (`visualization_msgs/MarkerArray`):
  visualization of the internally tracked trajectory.
- Launch args:
  `robot_ns` and `target_pose_frame_type`.

### 2.2 Common Behavior

The copilot planner shares the same behavior across all demos:

- It interprets `root/target_pose` as the first-link tail pose and derives the
  robot root pose internally.
- `target_pose_frame_type` controls how the incoming target pose is interpreted:
  `LINK` means the link frame, while `FLU` means Forward-Left-Up and is converted
  internally with a 180-degree rotation about the local/body `Z` axis.
- The launch file loads `multilink_copilot/config/copilot_planner.yaml` by
  default, and `target_pose_frame_type` can still be overridden from the launch
  arg.
- The planner waits for the first complete `joint_states` message before
  publishing `full_state_target`; until then it still tracks
  `root/target_pose` and accumulates the internal trajectory buffer.
- In snake-mode startup and warmup, the planner appends the current measured body geometry behind the available first-link-tail history, then reconstructs the nominal joints directly from that continuous geometric chain. It does not extrapolate the oldest trajectory tangent or blend joints according to accumulated history length.
- Full-state publishing can be gated by significant root motion using the
  thresholds defined in the YAML configuration.

Evaluation is disabled by default in `copilot_planner.launch`. Set `evaluation:=true` to include the evaluation helpers, or launch them separately with `roslaunch multilink_copilot run_evaluation.launch`. The helpers publish `last_link/tail_pose` from `root/pose` and the TF relation `dragon/root -> dragon/link4`, and write the envelope-width JSON result.

## 3. Demo Workflows

Both demo flows use the same copilot planner. The main difference is which node
publishes `root/target_pose`:

- `dragon_dance.launch`: starts the copilot planner and interactive dance demo together.
- `mono_planner waypoint_conditioned_planner.launch`: publishes waypoint-based
  target poses that the copilot planner converts into multilink full-state
  targets.
- `DragonRootTargetNavigator`: publishes joystick-driven target poses for
  human-guided shared control.

### 3.1 Dragon Dance

`dragon_dance.launch` starts both the dance node and the copilot planner with the `FLU` target pose convention; evaluation is disabled. Dragon bringup and manual takeoff/landing remain separate. Use only one dance launch, and stop other publishers of `root/target_pose`, `full_state_target`, `uav/nav`, and joint/rotation control commands before starting it.

Start the demo in the following order:

1. Launch Dragon in simulation:

   ```bash
   roslaunch dragon bringup.launch demo:=false rm:=false sim:=true headless:=false
   ```

2. Use the standard bringup teleoperation interface to make the robot take off and hover.

3. Start the unified demo from an interactive terminal:

   ```bash
   roslaunch multilink_copilot dragon_dance.launch
   ```

The demo can also be started before takeoff: it waits for valid robot inputs, control connections, and `HOVER`. Once the measured speed conditions are satisfied, it moves to the dance entry point automatically and repeats the original 60-second dance cycle at 40 Hz. The bringup option `demo:=false` disables Dragon's built-in demo command publisher.

Entry duration is calculated from the measured CoG displacement and shortest yaw difference, with limits `entry_max_linear_speed: 0.3` (m/s) and `entry_max_yaw_rate: 0.3` (rad/s). A smooth quintic reference includes the raw measured initial velocity and ends at rest; the limits constrain the generated reference. Readiness checks the current measured linear speed and yaw rate directly against 10% of those limits (default 0.03 m/s and 0.03 rad/s). The demo starts entry on the first qualifying update, without a low-pass filter or a settling interval.

After the entry reference finishes, the demo holds its endpoint until measured position/yaw errors satisfy their tolerances, then immediately requests the first planner output. A robot already within entry tolerance and satisfying the speed conditions skips the entry motion. The entry pose remains held until the first valid planner output arrives; forwarding and dancing then begin immediately. Reference motion uses ROS time. Waiting for inputs, convergence, or planner output has no timeout.

To finish normally, press `q` in the launch terminal. If terminal input is unavailable, use the stop service from another terminal:

```bash
rosservice call /dragon/dragon_dance/stop "{}"
```

Normal stopping first closes the copilot output gate and commands a position hold. As soon as measured linear speed and yaw rate satisfy the same 10% thresholds used for entry readiness, joint recovery begins. Joint errors within `shutdown_joint_tolerance` immediately advance to leveling the physical body. Following `DragonNavigator::targetRotationMotionCallback` and `landingProcess` in `jsk_aerial_robot`, the demo sends `target_rotation_motion` with `header.frame_id: baselink`, smoothly brings physical baselink roll/pitch to zero, and preserves its measured yaw. Feedback comes from `uav/baselink/odom`; CoG attitude is a mutable control-frame attitude and is not used to declare the body level. Pitch joint targets must be zero to make the body planar; yaw joint targets retain the configured recovery shape. Recovery has no fixed waiting interval, settling interval, progress timeout, or total timeout. Repeated stop requests are idempotent. After successful completion, the launch stops its dance and copilot nodes and prompts you to land manually through the original bringup teleoperation interface. `Ctrl+C` terminates the launch immediately and does not complete this normal stopping sequence; external bringup keeps running.

The terminal reports the current stage, waiting reason, computed entry duration, tracking errors, cycle count, and faults. The latched status topic is also available while the demo is running:

```bash
rostopic echo /dragon/dragon_dance/status
```

No elapsed-time watchdog is applied to feedback age, planner output gaps, convergence, or a paused ROS clock. The latest valid feedback remains available until replaced; paused ROS time pauses reference progress while keyboard/service handling and status reporting remain responsive. If a condition never becomes satisfied, the experiment operator terminates the process and makes the necessary adjustments. Explicit invalid feedback, loss of `HOVER` during motion, disconnected control/planner interfaces, and backward ROS time still stop automatic commands with a specific error. After an interrupted run, land/reset through the existing bringup interface, correct the reported cause, and start the next demo after a fresh takeoff; the navigator can retain orientation-control state from the previous dance. During operation, the planner publishes to `dragon_dance/planned_full_state_target`; only the dance node forwards accepted output to `full_state_target`, preventing planner output from competing with the stopping sequence.

Dance settings are in [`config/dragon_dance.yaml`](config/dragon_dance.yaml). To choose a namespace or your own configuration:

```bash
roslaunch multilink_copilot dragon_dance.launch robot_ns:=dragon config_file:=/absolute/path/to/dragon_dance.yaml
```

The YAML groups trajectory shape/timing, entry speed limits, convergence tolerances, status reporting, and stopping targets. Keep the original ellipse defaults (semi-axes 1.8/1.2 m, height offset 1.25 m, amplitude 0.25 m) for the existing dance. Change the speed limits to adjust entry motion. Removed timeout parameters, `settle_duration`, `shutdown_wait_duration`, and the old `startup_transition_duration` are ignored with migration warnings when supplied by an older configuration. Copilot settings remain in `config/copilot_planner.yaml`. With a different `robot_ns`, use that namespace in the service/topic commands above.

During joint recovery, all target joint errors must be within `shutdown_joint_tolerance` (default 0.05 rad); the first qualifying update starts leveling. `shutdown_cog_hold_duration` (default 0.5 seconds) and `shutdown_leveling_publish_duration` (default 6 seconds) remain as reference-motion durations in ROS time. Once the leveling reference reaches its endpoint, measured physical baselink roll/pitch are within `shutdown_leveling_tolerance` (default 0.1 rad), and all joint errors are within `shutdown_joint_tolerance`, recovery completes immediately. Throughout leveling, the demo continues sending the joint targets so the planar shape is maintained. A leveling reference that has finished continues publishing its final target until both attitude and joint tolerances are satisfied.

### 3.2 Waypoint-conditioned maneuvering

This demo uses the waypoint-conditioned planner from `mono_planner` as the
upstream source of `root/target_pose`. The copilot planner then converts that
target stream into full-state targets for Dragon.

Start the demo in the following order:

1. Launch Dragon with the standard bringup flow, then make the robot take off and hover.

2. Launch the waypoint pose publisher:

   ```bash
   roslaunch mono_planner waypoint_pose_publisher.launch
   ```

3. Launch the copilot planner with the FLU target pose frame:

   ```bash
   roslaunch multilink_copilot copilot_planner.launch target_pose_frame_type:=FLU
   ```

4. Launch the nonholonomic waypoint-conditioned planner:

   ```bash
   roslaunch mono_planner waypoint_conditioned_planner.launch nonholo:=true robot_frame_type:=LINK total_trajectory_time:=50
   ```

The waypoint pose publisher loads `mono_planner/config/demo_waypoints.yaml` by
default.

### 3.3 Human-guided shared control

This demo uses the Dragon joystick navigation plugin as the upstream publisher
of `root/target_pose`. `multilink_copilot` then converts that target stream
into `full_state_target` for shared control of the multilink robot.

Demo-specific prerequisite:

- `jsk_aerial_robot` branch:
  [https://github.com/Amos-Chen98/jsk_aerial_robot/tree/copilot_joystick](https://github.com/Amos-Chen98/jsk_aerial_robot/tree/copilot_joystick)

Start the demo in the following order:

1. Launch Dragon bringup:

   ```bash
   roslaunch dragon bringup.launch
   ```

2. Launch the joystick driver:

   ```bash
   roslaunch aerial_robot_base joy_stick.launch robot_name:=dragon
   ```

3. Launch the copilot planner with the FLU target pose frame:

   ```bash
   roslaunch multilink_copilot copilot_planner.launch target_pose_frame_type:=FLU
   ```

Use the normal Dragon bringup teleoperation flow to arm, take off, and enter
`HOVER`. After the robot is hovering, the joystick-driven navigation plugin
publishes `root/target_pose`, and the copilot planner consumes it as the shared
control reference.

Controller mapping for this demo:

- `R2`: move forward along the robot body `+X` direction.
- Left stick horizontal: control yaw.
- Left stick vertical: control pitch.
- `L2`: ignored in this shared-control mode.
- Right stick: ignored in this shared-control mode.
- Other Dragon joystick buttons keep their original bringup behavior, including
  the standard arm, takeoff, landing, and stop flow.

`target_pose_frame_type:=FLU` is required because this joystick-driven demo
publishes `root/target_pose` in the FLU convention.

## 4. Configuration

The planner parameters are stored in:

```bash
multilink_copilot/config/copilot_planner.yaml
```

The launch file loads this YAML by default. `target_pose_frame_type` can still be overridden from the launch arg.

## 5. Evaluation

The evaluation tools support two common workflows:

- Replay a recorded rosbag for visual inspection or offline analysis.
- Run the envelope-width evaluator during a live experiment or rosbag replay.

### 5.1 Replay Rosbag

Recorded bags are stored under:

```bash
data_manager/dragon_copilot/data/rosbag
```

Use the replay helper to choose a bag interactively and visualize it in RViz:

```bash
rosrun multilink_copilot replay_rosbag.py
```

The helper launches `launch/evaluation/replay_rosbag.launch`, loads the Dragon URDF from
`dragon/robots/quad/v1_5_202601.urdf.xacro`, starts `rosbag play` with
`--clock`, and opens `mono_planner/rviz/wpt_cond_manv.rviz`. Bag `/tf` is
replayed through a small bridge that forwards only `world -> dragon/root`, so
`robot_state_publisher` can regenerate the complete Dragon link tree from
`dragon/joint_states` without duplicate TF publishers.

Common noninteractive forms:

```bash
rosrun multilink_copilot replay_rosbag.py --latest
rosrun multilink_copilot replay_rosbag.py --pause
```

You can also launch a specific bag directly. The launch file lives under
`launch/evaluation/`; `roslaunch` resolves it by filename:

```bash
roslaunch multilink_copilot replay_rosbag.launch bag:=$(rospack find data_manager)/dragon_copilot/data/rosbag/2026-05-04-11-38-33.bag
```

### 5.2 Run Evaluation

Run the evaluation launch during a live waypoint-conditioned maneuver or while
replaying a recorded bag:

```bash
roslaunch multilink_copilot run_evaluation.launch
```

This launch starts:

- `last_link_tail_pose_publisher.py`: publishes `last_link/tail_pose` from
  `root/pose` and the TF relation `dragon/root -> dragon/link4`.
- `envelope_width_evaluator.py`: detects when `dragon/root/pose` and
  `dragon/last_link/tail_pose` cross the waypoint ring planes, then writes an
  envelope-width JSON result.

By default, results are written to:

```bash
data_manager/dragon_copilot/data/envelope_width
```

Common launch arguments:

```bash
roslaunch multilink_copilot run_evaluation.launch robot_ns:=dragon
roslaunch multilink_copilot run_evaluation.launch output_dir:=/tmp/envelope_width
```

`copilot_planner.launch` leaves evaluation disabled by default. To start the planner and evaluation together, use:

```bash
roslaunch multilink_copilot copilot_planner.launch evaluation:=true
```
