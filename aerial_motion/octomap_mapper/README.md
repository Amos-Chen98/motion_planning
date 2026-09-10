# octomap_mapper

ROS 1 integration of the official `octomap_server` for incremental mapping and RViz visualization.

## Dependencies and Build

With `jsk_aerial_robot_ws` already built, install dependencies (including `octomap_server`) and build:

```bash
cd /path/to/motion_planning_ws
source ../jsk_aerial_robot_ws/devel/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin build octomap_mapper
source devel/setup.bash
```

## Usage

Start [point-cloud preprocessing](../pcd_filter/README.md#usage) first. Input must be individual scans expressed in the physical LiDAR-origin frame, with TF available at each scan timestamp.

```bash
roslaunch octomap_mapper octomap_mapper.launch
```

The default input is `/dragon/cloud_denoised`; change it with `pcl_topic`. Set `voxel_width` for resolution (default `0.10` m) and `max_range` for ray distance (default `-1`, unlimited).

The [whole-body planner launch](../motion_primitive_planner/launch/whole_body_motion_primitive_planner.launch) starts preprocessing and mapping automatically; when using it, skip the standalone command above.

## Map Output and RViz

| Topic | Type | Contents |
| --- | --- | --- |
| `/dragon/octomap/full` | `octomap_msgs/Octomap` | Full probability map for the planner |
| `/dragon/octomap/occupied_cells` | `visualization_msgs/MarkerArray` | Occupied cells for RViz |

In RViz, set Fixed Frame to `world` and add a MarkerArray display for `/dragon/octomap/occupied_cells`. No additional RViz plugin is required.

`map_bound` sets the grid origin from its lower corner. Mapping and visualization may extend outside these bounds; the planner enforces its planning bounds.

Empty scans retain the accumulated map. To clear it and its visualization:

```bash
rosservice call /dragon/octomap/reset '{}'
```
