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
- Full-state publishing can be gated by significant root motion using the
  thresholds defined in the YAML configuration.

## 3. Demo Workflows

Both demo flows use the same copilot planner. The main difference is which node
publishes `root/target_pose`:

- `dragon_dance.launch`: publishes an interactive demo target stream.
- `mono_planner waypoint_conditioned_planner.launch`: publishes waypoint-based
  target poses that the copilot planner converts into multilink full-state
  targets.
- `DragonRootTargetNavigator`: publishes joystick-driven target poses for
  human-guided shared control.

### 3.1 Dragon Dance

Start the demo in the following order:

1. Launch Dragon in simulation:

   ```bash
   roslaunch dragon bringup.launch rm:=false sim:=true headless:=false
   ```

2. Launch the copilot planner with the FLU target pose frame:

   ```bash
   roslaunch multilink_copilot copilot_planner.launch target_pose_frame_type:=FLU
   ```

3. Use the standard bringup teleoperation interface to make the robot take off and hover.

4. After the robot is hovering, launch the dragon dance target publisher from an interactive terminal:

   ```bash
   roslaunch multilink_copilot dragon_dance.launch
   ```

To stop tracking, press any key in the terminal that launched `dragon_dance.launch`. The demo node will stop publishing the root tail target and execute its shutdown leveling sequence automatically.

After the shutdown sequence finishes and the `dragon_dance` node exits, send the normal landing command through the standard teleoperation interface.

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
