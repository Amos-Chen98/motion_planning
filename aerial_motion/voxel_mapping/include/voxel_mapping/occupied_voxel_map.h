#ifndef VOXEL_MAPPING_OCCUPIED_VOXEL_MAP_H
#define VOXEL_MAPPING_OCCUPIED_VOXEL_MAP_H

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

namespace voxel_mapping
{
struct VoxelMapConfig
{
  std::string world_frame_id = "world";
  double voxel_width = 0.0;
  std::vector<double> map_bound;
  bool use_accumulated_map = true;

  std::string validationError() const;
  void validateOrThrow() const;
};

class OccupiedVoxelMap
{
public:
  explicit OccupiedVoxelMap(const VoxelMapConfig& config);

  void update(const std::vector<Eigen::Vector3d>& points);
  std::vector<Eigen::Vector3d> occupiedVoxelCenters() const;
  size_t occupiedVoxelCount() const;

private:
  int64_t voxelKey(const Eigen::Vector3d& point) const;
  Eigen::Vector3i voxelIdFromKey(int64_t key) const;

  VoxelMapConfig config_;
  Eigen::Vector3d origin_;
  Eigen::Vector3i size_;
  std::unordered_set<int64_t> occupied_voxel_keys_;
};
}  // namespace voxel_mapping

#endif  // VOXEL_MAPPING_OCCUPIED_VOXEL_MAP_H
