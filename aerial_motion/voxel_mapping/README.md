# Voxel Mapping

`voxel_mapping` converts registered point clouds into occupied voxel centers for collision checking. It transforms each cloud into `WorldFrameId` at the cloud timestamp, removes spatially isolated points, and publishes the occupied map as a latched `sensor_msgs/PointCloud2`.

## Build and Run

The package requires C++17, Eigen, PCL (`common`, `filters`, and `search`), and the ROS dependencies declared in `package.xml`.

```bash
cd motion_planning_ws
source ../jsk_aerial_robot_ws/devel/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin build voxel_mapping
source devel/setup.bash
roslaunch voxel_mapping voxel_mapping.launch pcl_topic:=/dragon/cloud_self_filtered
```

The standalone launch defaults to `/dragon/cloud_registered_body`; use `/dragon/cloud_self_filtered` when running `pcd_self_filter`. The mapper subscribes to `cloud` and publishes `occupied`, remapped by default to `/dragon/voxelmap/occupied`. Each output point is an occupied voxel's center, with the map frame and input timestamp. If an already world-frame cloud has a zero timestamp, the output uses the current time.

## Noise Filtering

The mapper first excludes nonfinite coordinates and points outside the effective voxel grid. It then applies PCL `RadiusOutlierRemoval` once to the remaining points of the current observation, before deduplicating voxel hits. By default, a point is retained only if at least **two other points** are within **0.20 m**. Neighbor support can cross voxel boundaries; multiple samples inside one voxel also count as neighbors. Historical observations and previously occupied voxels do not contribute support.

An isolated point or isolated pair therefore creates no occupied voxels, even when repeated in later frames. A compact cluster of three mutually neighboring points can be retained immediately. Filtering uses point coordinates, not voxel centers; voxel membership is calculated from the original double-precision world coordinates so PCL's float conversion does not move hits across voxel boundaries.

| Private ROS parameter | Launch argument | Default | Meaning |
| --- | --- | --- | --- |
| `EnableNoiseFilter` | `enable_noise_filter` | `true` | Enable single-frame radius outlier removal. |
| `NoiseFilterRadius` | `noise_filter_radius` | `0.20` | Neighbor radius in meters; finite and positive. |
| `NoiseFilterMinNeighbors` | `noise_filter_min_neighbors` | `2` | Minimum other points inside the radius; integer of at least 1, excluding the query point. |
| `VoxelWidth` | `voxel_width` | `0.10` | Voxel edge length in meters. |
| `UseAccumulatedMap` | `use_accumulated_map` | `true` | Append accepted voxel hits instead of replacing the map per observation. |
| `WorldFrameId` | `world_frame_id` | `world` | Coordinate frame for filtering and output. |
| `MapBound` | `map_bound` | `[-10, 10, -10, 10, 0.2, 8]` | Bounds ordered as xmin, xmax, ymin, ymax, zmin, zmax. |

The effective grid has `floor((max - min) / VoxelWidth)` cells on each axis, with an inclusive lower boundary and exclusive upper boundary. Out-of-grid points do not support neighbors at the map boundary. Parameters are loaded at startup, and the startup log reports the filter state, radius, and neighbor threshold. Radius and neighbor parameters must be valid even when filtering is disabled.

To require three other neighbors within the default radius:

```bash
roslaunch voxel_mapping voxel_mapping.launch \
  pcl_topic:=/dragon/cloud_self_filtered noise_filter_min_neighbors:=3
```

To allow more widely spaced returns to support one another:

```bash
roslaunch voxel_mapping voxel_mapping.launch \
  pcl_topic:=/dragon/cloud_self_filtered noise_filter_radius:=0.30
```

Increasing the neighbor threshold or decreasing the radius rejects more points. Decreasing the threshold or increasing the radius retains sparser structures, but also allows more noise through. The radius is independent of voxel width. The defaults are an initial setting for the 0.10 m grid, not a calibration against recorded scene data; check retained thin structures and distant surfaces with your sensor before choosing stronger settings.

To restore single-point occupancy without noise filtering:

```bash
roslaunch voxel_mapping voxel_mapping.launch enable_noise_filter:=false
```

`motion_primitive_planner/whole_body_motion_primitive_planner.launch` exposes the same three noise-filter arguments and passes them to its mapper.

## Map Lifetime and Limitations

With accumulation enabled, accepted voxels persist for the node's lifetime. Empty clouds and observations whose points are all rejected leave that accumulated map unchanged. With `use_accumulated_map:=false`, each successfully processed observation replaces the map, and an empty or fully rejected observation publishes an empty map. Missing TF or point-cloud parsing failures skip the entire update and retain the previous map in both modes.

There is no multi-frame confirmation, ray clearing, or time decay. Dense noise clusters can pass the spatial filter, and real obstacles represented only by isolated returns can be removed. Restart the mapper to clear its accumulated map when changing filter settings; stricter settings do not retroactively remove previously accepted voxels.

## Tests

```bash
catkin run_tests voxel_mapping
catkin_test_results build/voxel_mapping/test_results
```

Core tests cover neighbor thresholds, radius overrides, voxel boundaries, planes and supported thin rods, invalid points, precision, and map lifetime. The ROS test checks default filtering, parameter overrides, disabling the filter, and published occupied centers in accumulation and replacement modes.
