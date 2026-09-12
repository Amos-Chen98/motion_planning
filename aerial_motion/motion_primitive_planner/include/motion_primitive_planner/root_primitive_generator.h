// -*- mode: c++ -*-
#ifndef MOTION_PRIMITIVE_PLANNER_ROOT_PRIMITIVE_GENERATOR_H
#define MOTION_PRIMITIVE_PLANNER_ROOT_PRIMITIVE_GENERATOR_H

#include <motion_primitive_planner/planner_config.h>

#include <gcopter/trajectory.hpp>

#include <memory>
#include <octomap/OcTree.h>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace motion_primitive_planner
{

enum class CandidateStatus
{
  kUnevaluated,
  kGenerationFailed,
  kCollision,
  kJointPlanningFailed,
  kFeasible,
  kSelected
};

struct Candidate
{
  Trajectory<5> trajectory;
  CandidateStatus status = CandidateStatus::kUnevaluated;
  double path_length = 0.0;
  double jerk_energy = 0.0;
  std::string detail;
};

class PrimitiveGenerator
{
public:
  explicit PrimitiveGenerator(const PrimitiveConfig& config);

  std::vector<Candidate> generate(const Eigen::Matrix3d& initial_state,
                                  const Eigen::Matrix3d& final_state) const;

  static double sampledLength(const Trajectory<5>& trajectory);

private:
  std::vector<Eigen::Vector3d> candidateRoute(const Eigen::Vector3d& start,
                                              const Eigen::Vector3d& target,
                                              int candidate_index) const;
  bool buildTrajectory(const std::vector<Eigen::Vector3d>& route,
                       const Eigen::Matrix3d& initial_state,
                       const Eigen::Matrix3d& final_state,
                       Candidate& candidate) const;

  PrimitiveConfig config_;
};

struct RootState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
};

enum class PrimitiveBatchFailure
{
  kNone,
  kStartCollision,
  kRouteSearchFailed,
  kLocalRouteEmpty
};

struct PrimitiveBatch
{
  PrimitiveBatchFailure failure = PrimitiveBatchFailure::kNone;
  std::string detail;
  std::vector<Eigen::Vector3d> local_route;
  Eigen::Vector3d local_target = Eigen::Vector3d::Zero();
  bool terminal = false;
  std::vector<Candidate> candidates;
  gcopter_planner::RouteSearchTiming route_search_timing;

  bool success() const { return failure == PrimitiveBatchFailure::kNone; }
};

class CollisionEnvironment;

//! Published atomically: route guidance and exact body collision use the same local snapshot version.
struct PlanningSceneSnapshot
{
  std::shared_ptr<const gcopter_planner::RoutePlannerBackend> route;
  std::shared_ptr<const CollisionEnvironment> collision;
  ros::Time map_stamp;
  uint64_t epoch = 0;
  uint64_t version = 0;
};

class PlanningEnvironment
{
public:
  explicit PlanningEnvironment(const SharedPlannerConfig& config);

  void replaceMap(std::shared_ptr<const octomap::OcTree> tree,
                  const Eigen::Isometry3d& world_from_grid,
                  const Eigen::Vector3d& origin, const Eigen::Vector3d& corner,
                  const ros::Time& stamp = ros::Time());
  Eigen::Vector3d clampTarget(const Eigen::Vector3d& requested, double clearance) const;
  PrimitiveBatch generate(const RootState& start, const Eigen::Vector3d& target);

  PrimitiveBatch generate(const RootState& start, const Eigen::Vector3d& target,
                          std::shared_ptr<const PlanningSceneSnapshot> scene);
  void replaceScene(std::shared_ptr<const PlanningSceneSnapshot> scene);

  bool occupied(const Eigen::Vector3d& point) const;
  double voxelScale() const;
  Eigen::Vector3d mapOrigin() const;
  Eigen::Vector3d mapCorner() const;
  std::shared_ptr<const PlanningSceneSnapshot> snapshot() const;

  static Eigen::Vector3d truncateRoute(const std::vector<Eigen::Vector3d>& full_route,
                                       double horizon,
                                       std::vector<Eigen::Vector3d>& local_route);

private:
  SharedPlannerConfig config_;
  std::shared_ptr<const PlanningSceneSnapshot> scene_;
  PrimitiveGenerator generator_;
};

}  // namespace motion_primitive_planner

#endif  // MOTION_PRIMITIVE_PLANNER_ROOT_PRIMITIVE_GENERATOR_H
