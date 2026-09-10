# pcd_filter

ROS 1 point-cloud preprocessing using `robot_body_filter` for self-filtering and `pcl/RadiusOutlierRemoval` for noise filtering.

## Dependencies and Build

With `jsk_aerial_robot_ws` already built, install dependencies and build:

```bash
cd /path/to/motion_planning_ws
source ../jsk_aerial_robot_ws/devel/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin build pcd_filter
source devel/setup.bash
```

## Usage

Start the robot description and state publishers first; robot TF must cover each cloud's timestamp.

For standalone DRAGON preprocessing with output at the LiDAR origin:

```bash
roslaunch pcd_filter pcd_filter.launch output_frame_id:=dragon/lidar_origin
```

The node names are fixed to `pcd_self_filter` and `pcd_denoise` within `robot_ns`. Input: `/dragon/cloud_registered_body`. Self-filtered output: `/dragon/cloud_self_filtered`. Denoised output: `/dragon/cloud_denoised` (all `sensor_msgs/PointCloud2`). Override the raw input with `input_topic`, the intermediate self-filtered topic with `body_output_topic`, and the final output with `output_topic`. With noise filtering disabled, the final output defaults to `cloud_self_filtered`; the planner and replay mapper follow that topic automatically. View either output in RViz using a PointCloud2 display.

The [whole-body planner launch](../motion_primitive_planner/launch/whole_body_motion_primitive_planner.launch) starts this filter and the OctoMap mapper automatically.
