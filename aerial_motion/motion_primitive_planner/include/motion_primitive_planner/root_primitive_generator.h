// -*- mode: c++ -*-
#ifndef MOTION_PRIMITIVE_PLANNER_ROOT_PRIMITIVE_GENERATOR_H
#define MOTION_PRIMITIVE_PLANNER_ROOT_PRIMITIVE_GENERATOR_H

#include <motion_primitive_planner/planner_config.h>

#include <gcopter/trajectory.hpp>

#include <memory>
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

  bool success() const { return failure == PrimitiveBatchFailure::kNone; }
};

class PlanningEnvironment
{
public:
  explicit PlanningEnvironment(const SharedPlannerConfig& config);

  void replaceMap(const std::vector<Eigen::Vector3d>& occupied_voxel_centers);
  Eigen::Vector3d clampTarget(const Eigen::Vector3d& requested, double clearance) const;
  PrimitiveBatch generate(const RootState& start, const Eigen::Vector3d& target);

  bool occupied(const Eigen::Vector3d& point) const;
  double voxelScale() const;
  Eigen::Vector3d mapOrigin() const;
  Eigen::Vector3d mapCorner() const;
  std::shared_ptr<const gcopter_planner::PlannerBackend> occupancySnapshot() const;

  static Eigen::Vector3d truncateRoute(const std::vector<Eigen::Vector3d>& full_route,
                                       double horizon,
                                       std::vector<Eigen::Vector3d>& local_route);

private:
  SharedPlannerConfig config_;
  std::shared_ptr<gcopter_planner::PlannerBackend> backend_;
  PrimitiveGenerator generator_;
};

}  // namespace motion_primitive_planner

#endif  // MOTION_PRIMITIVE_PLANNER_ROOT_PRIMITIVE_GENERATOR_H
