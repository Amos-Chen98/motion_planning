#pragma once

#include <sensor_msgs/PointCloud2.h>

namespace pcd_filter
{
// Validate storage before handing it to upstream iterators, pack padded rows,
// and remove nonfinite XYZ while preserving every other byte of each point.
sensor_msgs::PointCloud2 prepareCloud(const sensor_msgs::PointCloud2& input);
}  // namespace pcd_filter
