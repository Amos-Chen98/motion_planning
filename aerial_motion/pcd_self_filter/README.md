# pcd_self_filter

ROS 1 point-cloud preprocessing using `robot_body_filter` for self-filtering and `pcl/RadiusOutlierRemoval` for noise filtering.

## Dependencies and Build

With `jsk_aerial_robot_ws` already built, install dependencies and build:

```bash
cd /path/to/motion_planning_ws
source ../jsk_aerial_robot_ws/devel/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin build pcd_self_filter
source devel/setup.bash
```

## Usage

Start the robot description and state publishers first. Robot TF must be available at each cloud's timestamp.

For standalone DRAGON preprocessing with output at the LiDAR origin:

```bash
roslaunch pcd_self_filter pcd_self_filter.launch output_frame_id:=dragon/lidar_origin publish_sensor_origin_tf:=true
```

Input: `/dragon/cloud_registered_body`. Filtered output: `/dragon/cloud_self_filtered` (`sensor_msgs/PointCloud2`). Change these with `input_topic` and `output_topic`. View the output in RViz using a PointCloud2 display.

The [whole-body planner launch](../motion_primitive_planner/launch/whole_body_motion_primitive_planner.launch) starts this filter and the OctoMap mapper automatically; when using it, skip the standalone command above.

## Radius noise filtering

- `noise_filter_radius`: `0.20` m.
- `noise_filter_min_neighbors`: `2` other points required within the radius; replay uses `6`.
- `enable_noise_filter:=false`: disable noise filtering.
