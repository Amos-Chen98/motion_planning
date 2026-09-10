#include <motion_primitive_planner/dragon_collision_checker.h>

#include <coal/collision.h>
#include <coal/octree.h>
#include <coal/shape/geometric_shapes.h>
#include <kdl/treefksolverpos_recursive.hpp>
#include <octomap/OcTree.h>

#include <array>
#include <cmath>
#include <map>
#include <limits>
#include <set>
#include <stdexcept>

#ifndef COAL_HAS_OCTOMAP
#error "motion_primitive_planner requires Coal built with OctoMap support"
#endif

namespace motion_primitive_planner
{
namespace
{
Eigen::Isometry3d eigenFrame(const KDL::Frame& frame)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  for (int row = 0; row < 3; ++row)
  {
    result.translation()(row) = frame.p(row);
    for (int col = 0; col < 3; ++col) result.linear()(row, col) = frame.M(row, col);
  }
  return result;
}

Eigen::Isometry3d collisionOrigin(const urdf::Pose& pose)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  const Eigen::Quaterniond rotation(pose.rotation.w, pose.rotation.x,
                                   pose.rotation.y, pose.rotation.z);
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  if (!result.translation().allFinite() || !rotation.coeffs().allFinite() ||
      rotation.norm() < 1e-12)
    throw std::invalid_argument("Invalid URDF collision origin");
  result.linear() = rotation.normalized().toRotationMatrix();
  return result;
}
}  // namespace

struct CollisionEnvironment::Data
{
  Eigen::Vector3d origin;
  Eigen::Vector3d corner;
  std::shared_ptr<const octomap::OcTree> tree;
  std::shared_ptr<const coal::OcTree> geometry;
  coal::Transform3s transform;
};

CollisionEnvironment::CollisionEnvironment(
    std::shared_ptr<const octomap::OcTree> tree, const Eigen::Isometry3d& world_from_grid,
    const Eigen::Vector3d& origin, const Eigen::Vector3d& corner)
{
  if (!tree || !std::isfinite(tree->getResolution()) || tree->getResolution() <= 0 ||
      !origin.allFinite() || !corner.allFinite() || (corner.array() <= origin.array()).any() ||
      !world_from_grid.matrix().allFinite() ||
      !world_from_grid.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-9) ||
      !world_from_grid.translation().isApprox(origin, 1e-9))
    throw std::invalid_argument("Invalid collision OctoMap or grid transform");
  auto data = std::make_shared<Data>();
  data->origin = origin;
  data->corner = corner;
  data->tree = std::move(tree);
  auto geometry = std::make_shared<coal::OcTree>(data->tree);
  geometry->setOccupancyThres(0.5);
  geometry->setFreeThres(std::nextafter(0.5, 0.0));
  geometry->setCellDefaultOccupancy(0.0);
  if (data->tree->size() != 0)
  {
    // Coal's generic implementation pads leaf centers by only the base resolution.
    // Include each pruned leaf's full extent instead.
    Eigen::Vector3d low = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d high = -low;
    for (auto it = data->tree->begin_leafs(); it != data->tree->end_leafs(); ++it)
    {
      const Eigen::Vector3d center(it.getX(), it.getY(), it.getZ());
      low = low.cwiseMin(center.array().matrix() - Eigen::Vector3d::Constant(it.getSize() / 2));
      high = high.cwiseMax(center.array().matrix() + Eigen::Vector3d::Constant(it.getSize() / 2));
    }
    geometry->aabb_local = coal::AABB(low, high);
    geometry->aabb_center = (low + high) / 2;
    geometry->aabb_radius = (high - low).norm() / 2;
  }
  data->geometry = geometry;
  data->transform = coal::Transform3s(world_from_grid.linear(), world_from_grid.translation());
  data_ = std::move(data);
}

CollisionEnvironment::~CollisionEnvironment() = default;
const Eigen::Vector3d& CollisionEnvironment::origin() const { return data_->origin; }
const Eigen::Vector3d& CollisionEnvironment::corner() const { return data_->corner; }
size_t CollisionEnvironment::occupiedVoxelCount() const
{
  size_t count = 0;
  for (auto it = data_->tree->begin_leafs(); it != data_->tree->end_leafs(); ++it)
    if (data_->tree->isNodeOccupied(*it))
      count += size_t(1) << (3 * (data_->tree->getTreeDepth() - it.getDepth()));
  return count;
}

struct DragonCollisionModel::Data
{
  struct Primitive
  {
    std::string link;
    Eigen::Isometry3d origin;
    std::shared_ptr<const coal::CollisionGeometry> geometry;
    bool cylinder = false;
    Eigen::Vector3d half_size = Eigen::Vector3d::Zero();
    double radius = 0.0;
    double half_length = 0.0;
  };
  KDL::Tree tree;
  std::vector<int> joint_indices;
  double link_length;
  std::array<Primitive, 8> primitives;
};

DragonCollisionModel::DragonCollisionModel(const Dragon::HydrusLikeRobotModel& model)
  : DragonCollisionModel(model.getUrdfModel(), model.getTree(), model.getLinkJointNames(),
                         model.getLinkJointIndices(), model.getLinkLength())
{
  if (model.getRotorNum() != 4)
    throw std::invalid_argument("DRAGON collision model requires four flying links");
}

DragonCollisionModel::DragonCollisionModel(
    const urdf::Model& urdf, const KDL::Tree& tree,
    const std::vector<std::string>& joint_names, const std::vector<int>& joint_indices,
    double link_length)
{
  if (!std::isfinite(link_length) || link_length <= 0.0 || joint_names.size() != 6 ||
      joint_indices.size() != joint_names.size())
    throw std::invalid_argument("Invalid DRAGON collision kinematics");
  auto data = std::make_shared<Data>();
  data->tree = tree;
  data->joint_indices = joint_indices;
  data->link_length = link_length;

  std::set<std::string> required_body_joints;
  for (int link = 1; link < 4; ++link)
    for (const auto* axis : {"pitch", "yaw"})
      required_body_joints.insert("joint" + std::to_string(link) + "_" + axis);
  std::map<std::string, unsigned int> kdl_joints;
  for (const auto& entry : tree.getSegments())
  {
    const auto& segment = GetTreeElementSegment(entry.second);
    if (segment.getJoint().getType() != KDL::Joint::None)
      kdl_joints.emplace(segment.getJoint().getName(), GetTreeElementQNr(entry.second));
  }
  std::set<int> used_indices;
  for (size_t index = 0; index < joint_names.size(); ++index)
  {
    const auto found = kdl_joints.find(joint_names[index]);
    if (required_body_joints.erase(joint_names[index]) != 1 ||
        found == kdl_joints.end() || joint_indices[index] < 0 ||
        static_cast<unsigned int>(joint_indices[index]) != found->second ||
        found->second >= tree.getNrOfJoints() || !used_indices.insert(joint_indices[index]).second)
      throw std::invalid_argument("Incomplete DRAGON collision joint mapping: " + joint_names[index]);
  }
  for (int link = 1; link <= 4; ++link)
  {
    for (const auto* axis : {"roll", "pitch"})
      if (kdl_joints.count("gimbal" + std::to_string(link) + "_" + axis) == 0)
        throw std::invalid_argument("Missing DRAGON gimbal KDL joint");
    for (int kind = 0; kind < 2; ++kind)
    {
      auto& primitive = data->primitives[2 * (link - 1) + kind];
      primitive.cylinder = kind == 1;
      primitive.link = kind == 0 ? "link" + std::to_string(link) :
                                  "gimbal" + std::to_string(link) + "_roll_module";
      const auto urdf_link = urdf.getLink(primitive.link);
      if (!urdf_link || tree.getSegment(primitive.link) == tree.getSegments().end())
        throw std::invalid_argument("Missing DRAGON collision link: " + primitive.link);
      const int type = primitive.cylinder ? urdf::Geometry::CYLINDER : urdf::Geometry::BOX;
      urdf::CollisionSharedPtr collision;
      for (const auto& candidate : urdf_link->collision_array)
      {
        if (!candidate || !candidate->geometry || candidate->geometry->type != type) continue;
        if (collision) throw std::invalid_argument("Duplicate DRAGON collision primitive: " + primitive.link);
        collision = candidate;
      }
      if (!collision) throw std::invalid_argument("Missing required URDF primitive: " + primitive.link);
      primitive.origin = collisionOrigin(collision->origin);
      std::shared_ptr<coal::CollisionGeometry> geometry;
      if (primitive.cylinder)
      {
        const auto& cylinder = static_cast<const urdf::Cylinder&>(*collision->geometry);
        if (!std::isfinite(cylinder.radius) || cylinder.radius <= 0.0 ||
            !std::isfinite(cylinder.length) || cylinder.length <= 0.0)
          throw std::invalid_argument("Invalid URDF cylinder: " + primitive.link);
        primitive.radius = cylinder.radius;
        primitive.half_length = 0.5 * cylinder.length;
        geometry = std::make_shared<coal::Cylinder>(cylinder.radius, cylinder.length);
      }
      else
      {
        const auto& box = static_cast<const urdf::Box&>(*collision->geometry);
        const Eigen::Vector3d size(box.dim.x, box.dim.y, box.dim.z);
        if (!size.allFinite() || (size.array() <= 0.0).any())
          throw std::invalid_argument("Invalid URDF box: " + primitive.link);
        primitive.half_size = 0.5 * size;
        geometry = std::make_shared<coal::Box>(size);
      }
      geometry->computeLocalAABB();
      primitive.geometry = std::move(geometry);
    }
  }
  data_ = std::move(data);
}

DragonCollisionModel::~DragonCollisionModel() = default;
size_t DragonCollisionModel::geometryCount() const { return data_->primitives.size(); }

struct DragonCollisionChecker::Workspace
{
  explicit Workspace(std::shared_ptr<const DragonCollisionModel::Data> data)
    : model(std::move(data)), solver(model->tree), joints(model->tree.getNrOfJoints())
  {
    request.num_max_contacts = 1;
    request.enable_contact = false;
    request.security_margin = 0.0;
  }
  std::shared_ptr<const DragonCollisionModel::Data> model;
  KDL::TreeFkSolverPos_recursive solver;
  KDL::JntArray joints;
  coal::CollisionRequest request;
  coal::CollisionResult result;
};

DragonCollisionChecker::DragonCollisionChecker(std::shared_ptr<const DragonCollisionModel> model)
{
  if (!model) throw std::invalid_argument("Missing DRAGON collision model");
  workspace_.reset(new Workspace(model->data_));
}
DragonCollisionChecker::~DragonCollisionChecker() = default;

bool DragonCollisionChecker::collides(const WholeBodyConfiguration& configuration,
                                     const CollisionEnvironment& environment)
{
  auto& work = *workspace_;
  const auto& model = *work.model;
  const auto& map = *environment.data_;
  const auto& rotation = configuration.root_link_rotation;
  if (!configuration.link1_tail.allFinite() || !rotation.allFinite() ||
      !rotation.isUnitary(1e-6) || std::abs(rotation.determinant() - 1.0) > 1e-6 ||
      configuration.joint_positions.size() != static_cast<int>(model.joint_indices.size()) ||
      !configuration.joint_positions.allFinite()) return true;

  // Always canonicalize gimbals, including when a dynamics model has solved them.
  work.joints.data.setZero();
  for (size_t index = 0; index < model.joint_indices.size(); ++index)
    work.joints(model.joint_indices[index]) = configuration.joint_positions(index);
  KDL::Frame frame;
  if (work.solver.JntToCart(work.joints, frame, "link1") < 0) return true;
  Eigen::Isometry3d world_link1 = Eigen::Isometry3d::Identity();
  world_link1.linear() = rotation;
  world_link1.translation() = configuration.link1_tail - model.link_length * rotation.col(0);
  const Eigen::Isometry3d world_root = world_link1 * eigenFrame(frame).inverse();
  for (const auto& primitive : model.primitives)
  {
    if (work.solver.JntToCart(work.joints, frame, primitive.link) < 0) return true;
    const Eigen::Isometry3d pose = world_root * eigenFrame(frame) * primitive.origin;
    if (!pose.matrix().allFinite()) return true;
    // Exact support extents; Coal's general rotated-object AABB can be looser.
    Eigen::Vector3d extent;
    if (primitive.cylinder)
    {
      for (int axis = 0; axis < 3; ++axis)
        extent(axis) = primitive.radius * std::hypot(pose.linear()(axis, 0), pose.linear()(axis, 1)) +
                       primitive.half_length * std::abs(pose.linear()(axis, 2));
    }
    else extent = pose.linear().cwiseAbs() * primitive.half_size;
    if (((pose.translation() - extent).array() < map.origin.array()).any() ||
        ((pose.translation() + extent).array() >= map.corner.array()).any()) return true;
    if (map.tree->size() == 0) continue;
    work.result.clear();
    coal::collide(primitive.geometry.get(), coal::Transform3s(pose.linear(), pose.translation()),
                  map.geometry.get(), map.transform, work.request, work.result);
    if (work.result.isCollision()) return true;
  }
  return false;
}

}  // namespace motion_primitive_planner
