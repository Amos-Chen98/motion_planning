#include <motion_primitive_planner/local_map.h>
#include <motion_primitive_planner/dragon_collision_checker.h>
#include <rog_map_msgs/validation.h>
#include <octomap/OcTree.h>

namespace motion_primitive_planner {
void validateLocalMap(
    const rog_map_msgs::LocalMap& map, const gcopter_planner::RoutePlannerConfig& config)
{
  rog_map_msgs::validate(map);
  if (map.header.frame_id != config.worldFrameId ||
      std::abs(map.resolution - config.voxelWidth) > 1e-9 ||
      map.inflation_steps != uint32_t(std::ceil(config.dilateRadius/config.voxelWidth)))
    throw std::invalid_argument("Local map frame, resolution or inflation does not match planner");
  const Eigen::Vector3d origin(map.origin.x, map.origin.y, map.origin.z);
  const Eigen::Vector3i size(map.size[0], map.size[1], map.size[2]);
  if ((size.array() > 32768).any() ||
      (size.cast<double>().array() * map.resolution <= 2*(config.dilateRadius+map.resolution)).any() ||
      (origin / map.resolution - (origin / map.resolution).array().round().matrix()).cwiseAbs().maxCoeff() > 1e-6)
    throw std::invalid_argument("Unaligned or excessive local OctoMap grid");
}

std::shared_ptr<const PlanningSceneSnapshot> buildLocalScene(
    const rog_map_msgs::LocalMap& map, const gcopter_planner::RoutePlannerConfig& config)
{
  validateLocalMap(map, config);
  const Eigen::Vector3d origin(map.origin.x, map.origin.y, map.origin.z);
  const Eigen::Vector3i size(map.size[0], map.size[1], map.size[2]);
  auto route = std::make_shared<gcopter_planner::RoutePlannerBackend>(config, origin, size);
  route->setInflatedBits(map.inflated_bits);
  auto tree = std::make_shared<octomap::OcTree>(map.resolution);
  // Only occupied cells need allocation: unknown/free are both traversable.
  for (size_t byte = 0; byte < map.occupied_bits.size(); ++byte) {
    unsigned mask = map.occupied_bits[byte];
    while (mask) {
      const size_t i = 8*byte + __builtin_ctz(mask);
      mask &= mask-1;
      const size_t x = i % map.size[0], yz = i / map.size[0];
      const size_t y = yz % map.size[1], z = yz / map.size[1];
      tree->setNodeValue(octomap::point3d((x+.5)*map.resolution, (y+.5)*map.resolution,
                                        (z+.5)*map.resolution), tree->getClampingThresMaxLog(), true);
    }
  }
  tree->updateInnerOccupancy();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = origin;
  auto scene = std::make_shared<PlanningSceneSnapshot>();
  scene->route = route;
  scene->collision = std::make_shared<CollisionEnvironment>(tree, transform, origin, route->mapCorner());
  scene->map_stamp = map.header.stamp;
  scene->epoch = map.epoch;
  scene->version = map.version;
  return scene;
}
}
