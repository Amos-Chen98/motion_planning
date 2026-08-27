#include <voxel_mapping/occupied_voxel_map.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace voxel_mapping
{
namespace
{
bool allFinite(const std::vector<double>& values)
{
  return std::all_of(values.begin(), values.end(),
                     [](const double value) { return std::isfinite(value); });
}
}  // namespace

std::string VoxelMapConfig::validationError() const
{
  if (world_frame_id.empty())
  {
    return "WorldFrameId must not be empty";
  }
  if (!std::isfinite(voxel_width) || voxel_width <= 0.0)
  {
    return "VoxelWidth must be finite and positive";
  }
  if (map_bound.size() != 6 || !allFinite(map_bound))
  {
    return "MapBound must contain six finite values";
  }
  for (int axis = 0; axis < 3; ++axis)
  {
    const double extent = map_bound[2 * axis + 1] - map_bound[2 * axis];
    if (extent <= 0.0)
    {
      return "MapBound must satisfy min < max on every axis";
    }
    if (extent / voxel_width < 1.0)
    {
      return "MapBound must contain at least one voxel on every axis";
    }
  }
  return std::string();
}

void VoxelMapConfig::validateOrThrow() const
{
  const std::string error = validationError();
  if (!error.empty())
  {
    throw std::invalid_argument(error);
  }
}

OccupiedVoxelMap::OccupiedVoxelMap(const VoxelMapConfig& config)
  : config_(config)
{
  config_.validateOrThrow();
  origin_ = Eigen::Vector3d(config_.map_bound[0], config_.map_bound[2],
                            config_.map_bound[4]);
  size_ = Eigen::Vector3i(
      static_cast<int>((config_.map_bound[1] - config_.map_bound[0]) /
                       config_.voxel_width),
      static_cast<int>((config_.map_bound[3] - config_.map_bound[2]) /
                       config_.voxel_width),
      static_cast<int>((config_.map_bound[5] - config_.map_bound[4]) /
                       config_.voxel_width));
}

void OccupiedVoxelMap::update(const std::vector<Eigen::Vector3d>& points)
{
  if (!config_.use_accumulated_map)
  {
    occupied_voxel_keys_.clear();
  }
  for (const Eigen::Vector3d& point : points)
  {
    if (!point.allFinite())
    {
      continue;
    }
    const int64_t key = voxelKey(point);
    if (key >= 0)
    {
      occupied_voxel_keys_.insert(key);
    }
  }
}

std::vector<Eigen::Vector3d> OccupiedVoxelMap::occupiedVoxelCenters() const
{
  std::vector<Eigen::Vector3d> centers;
  centers.reserve(occupied_voxel_keys_.size());
  for (const int64_t key : occupied_voxel_keys_)
  {
    const Eigen::Vector3i id = voxelIdFromKey(key);
    centers.push_back(origin_ +
                      config_.voxel_width *
                          (id.cast<double>() + Eigen::Vector3d::Constant(0.5)));
  }
  return centers;
}

size_t OccupiedVoxelMap::occupiedVoxelCount() const
{
  return occupied_voxel_keys_.size();
}

int64_t OccupiedVoxelMap::voxelKey(const Eigen::Vector3d& point) const
{
  const Eigen::Vector3i id = ((point - origin_) / config_.voxel_width)
                                 .array()
                                 .floor()
                                 .cast<int>();
  if ((id.array() < 0).any() || (id.array() >= size_.array()).any())
  {
    return -1;
  }
  return static_cast<int64_t>(id.x()) +
         static_cast<int64_t>(size_.x()) *
             (static_cast<int64_t>(id.y()) +
              static_cast<int64_t>(size_.y()) * static_cast<int64_t>(id.z()));
}

Eigen::Vector3i OccupiedVoxelMap::voxelIdFromKey(const int64_t key) const
{
  const int64_t xy = static_cast<int64_t>(size_.x()) * size_.y();
  const int z = static_cast<int>(key / xy);
  const int64_t remainder = key - static_cast<int64_t>(z) * xy;
  const int y = static_cast<int>(remainder / size_.x());
  const int x = static_cast<int>(remainder - static_cast<int64_t>(y) * size_.x());
  return Eigen::Vector3i(x, y, z);
}
}  // namespace voxel_mapping
