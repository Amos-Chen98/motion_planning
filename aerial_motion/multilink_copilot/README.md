# multilink_copilot

This package provides a copilot planner that generates full-state targets for multilink aerial robots with reference to the movement of the head.

## Dependencies

jsk_aerial_robot: https://github.com/jsk-ros-pkg/jsk_aerial_robot/pull/746

## Usage

```bash
roslaunch multilink_copilot copilot_planner.launch
```

## Configuration

The planner parameters are stored in:

```bash
multilink_copilot/config/copilot_planner.yaml
```

The launch file loads this YAML by default. `target_pose_frame_type` can still be overridden from the launch arg.



