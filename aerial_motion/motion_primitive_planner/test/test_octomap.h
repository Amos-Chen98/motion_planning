#pragma once
#include <motion_primitive_planner/dragon_collision_checker.h>
#include <motion_primitive_planner/root_primitive_generator.h>

namespace motion_primitive_planner
{
// Fixtures create sensor-independent maps explicitly; production only accepts full trees.
inline std::shared_ptr<const octomap::OcTree> testTree(
    const std::vector<Eigen::Vector3d>& points, double width,
    const Eigen::Vector3d& origin, const Eigen::Vector3i& size)
{
  auto tree = std::make_shared<octomap::OcTree>(width);
  for (const auto& point : points)
  {
    if (!point.allFinite()) continue;
    const Eigen::Array3d index = ((point - origin) / width).array().floor();
    if ((index < 0).any() || (index >= size.cast<double>().array()).any()) continue;
    const Eigen::Vector3d center = ((index + .5) * width).matrix();
    tree->setNodeValue(octomap::point3d(center.x(), center.y(), center.z()), tree->getClampingThresMaxLog(), true);
  }
  tree->updateInnerOccupancy();
  return tree;
}
inline Eigen::Isometry3d gridTransform(const Eigen::Vector3d& origin)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity(); transform.translation() = origin; return transform;
}
class TestCollisionEnvironment : public CollisionEnvironment
{
public:
  TestCollisionEnvironment(const std::vector<Eigen::Vector3d>& points, double width,
                           const Eigen::Vector3d& origin, const Eigen::Vector3i& size)
    : CollisionEnvironment(testTree(points, width, origin, size), gridTransform(origin),
                           origin, origin + size.cast<double>() * width) {}
};
inline void replaceTestMap(PlanningEnvironment& environment, const std::vector<Eigen::Vector3d>& points)
{
  const auto backend = environment.snapshot()->route;
  environment.replaceMap(testTree(points, backend->voxelScale(), backend->mapOrigin(), backend->mapSize()),
                         gridTransform(backend->mapOrigin()), backend->mapOrigin(), backend->mapCorner());
}
}  // namespace motion_primitive_planner
