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

Start [point-cloud preprocessing](../pcd_filter/README.md#usage) first. Input is individual scans in the physical LiDAR-origin frame, with TF available at each scan timestamp.

```bash
roslaunch octomap_mapper octomap_mapper.launch
```

The default input is `/dragon/cloud_denoised`; change it with `pcl_topic`. Set `voxel_width` for resolution (default `0.10` m) and `max_range` for ray distance (default `12.0` m). Points beyond this distance contribute free-space rays up to the limit, but their endpoints are not inserted as obstacles. This limits each observation, not the accumulated map size; existing distant cells are not deleted. Use `max_range:=-1` to restore unlimited range.

The performance defaults are `latch:=false`, `height_map:=false`, and `compress_map:=true`. The server prepares outputs only when they have subscribers, avoids per-voxel height colors in RViz markers, and retains tree compression. Resolution and hit/miss probabilities are unchanged. With latching disabled, new subscribers wait for the next scan update; after a paused replay they will not immediately receive the previous map. Use `latch:=true` for retained outputs, or `height_map:=true` for height coloring. The whole-body planner and Naraha `replay_nav.launch` expose these options as `octomap_latch`, `octomap_height_map`, and `octomap_compress_map`, alongside `max_range`.

For an A/B comparison against the previous mapper settings, run `roslaunch octomap_mapper octomap_mapper.launch max_range:=-1 latch:=true height_map:=true`. Measure both `rostopic hz /dragon/octomap/full` and `rostopic delay /dragon/octomap/full` with the same input and subscribers. Full-map publication still scales with accumulated tree size; these settings do not guarantee 10 Hz for every scene.

The [whole-body planner launch](../motion_primitive_planner/launch/whole_body_motion_primitive_planner.launch) starts preprocessing and mapping automatically.

## Map Output and RViz

| Topic | Type | Contents |
| --- | --- | --- |
| `/dragon/octomap/full` | `octomap_msgs/Octomap` | Full probability map for the planner |
| `/dragon/octomap/occupied_cells` | `visualization_msgs/MarkerArray` | Occupied cells for RViz |

In RViz, set Fixed Frame to `world` and add a MarkerArray display for `/dragon/octomap/occupied_cells`.

`map_bound` sets the grid origin from its lower corner. Mapping and visualization extend past these bounds; the planner enforces its own planning bounds.

Empty scans retain the accumulated map. To clear it and its visualization:

```bash
rosservice call /dragon/octomap/reset '{}'
```
