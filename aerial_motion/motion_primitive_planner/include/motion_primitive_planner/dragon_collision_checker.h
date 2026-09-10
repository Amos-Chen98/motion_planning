// -*- mode: c++ -*-
#ifndef MOTION_PRIMITIVE_PLANNER_DRAGON_COLLISION_CHECKER_H
#define MOTION_PRIMITIVE_PLANNER_DRAGON_COLLISION_CHECKER_H

#include <motion_primitive_planner/dragon_geometry.h>

#include <memory>
#include <octomap/OcTree.h>

namespace motion_primitive_planner
{

//! Immutable full OctoMap in a translated grid. Free and unknown space are traversable.
class CollisionEnvironment
{
public:
  CollisionEnvironment(std::shared_ptr<const octomap::OcTree> tree,
                       const Eigen::Isometry3d& world_from_grid,
                       const Eigen::Vector3d& origin, const Eigen::Vector3d& corner);
  ~CollisionEnvironment();

  const Eigen::Vector3d& origin() const;
  const Eigen::Vector3d& corner() const;
  size_t occupiedVoxelCount() const;

private:
  struct Data;
  std::shared_ptr<const Data> data_;
  friend class DragonCollisionChecker;
};

//! Eight URDF primitives and the existing KDL tree, detached from mutable dynamics.
class DragonCollisionModel
{
public:
  explicit DragonCollisionModel(const Dragon::HydrusLikeRobotModel& model);
  DragonCollisionModel(const urdf::Model& urdf, const KDL::Tree& tree,
                       const std::vector<std::string>& joint_names,
                       const std::vector<int>& joint_indices, double link_length);
  ~DragonCollisionModel();

  size_t geometryCount() const;

private:
  struct Data;
  std::shared_ptr<const Data> data_;
  friend class DragonCollisionChecker;
};

//! One workspace per candidate. The model and environment may be shared read-only.
class DragonCollisionChecker
{
public:
  explicit DragonCollisionChecker(std::shared_ptr<const DragonCollisionModel> model);
  ~DragonCollisionChecker();
  DragonCollisionChecker(const DragonCollisionChecker&) = delete;
  DragonCollisionChecker& operator=(const DragonCollisionChecker&) = delete;

  //! Invalid configurations or failed FK are rejected as collisions.
  bool collides(const WholeBodyConfiguration& configuration,
                const CollisionEnvironment& environment);

private:
  struct Workspace;
  std::unique_ptr<Workspace> workspace_;
};

}  // namespace motion_primitive_planner

#endif  // MOTION_PRIMITIVE_PLANNER_DRAGON_COLLISION_CHECKER_H
