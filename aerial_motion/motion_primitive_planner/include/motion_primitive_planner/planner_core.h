// -*- mode: c++ -*-
#ifndef MOTION_PRIMITIVE_PLANNER_PLANNER_CORE_H
#define MOTION_PRIMITIVE_PLANNER_PLANNER_CORE_H

#include <gcopter/trajectory.hpp>

#include <Eigen/Dense>

#include <functional>
#include <string>
#include <vector>

namespace motion_primitive_planner
{

enum class CandidateStatus
{
  kUnevaluated,
  kGenerationFailed,
  kCollision,
  kJointLimit,
  kStability,
  kStabilityProjection,
  kFeasible,
  kSelected
};

struct PrimitiveConfig
{
  int candidate_count = 9;
  double max_offset = 0.8;
  double max_velocity = 0.2;
  double cruise_velocity = 0.16;
  double minimum_piece_duration = 0.2;
};

struct Candidate
{
  Trajectory<5> trajectory;
  CandidateStatus status = CandidateStatus::kUnevaluated;
  double path_length = 0.0;
  double jerk_energy = 0.0;
  double min_fc_rp = 0.0;
  double joint_motion = 0.0;
  bool requires_stability_projection = false;
  std::string detail;

  bool feasible() const
  {
    return status == CandidateStatus::kFeasible || status == CandidateStatus::kSelected;
  }
};

class PrimitiveGenerator
{
public:
  explicit PrimitiveGenerator(const PrimitiveConfig& config);

  std::vector<Candidate> generate(const std::vector<Eigen::Vector3d>& route,
                                  const Eigen::Matrix3d& initial_state,
                                  const Eigen::Matrix3d& final_state) const;

  static std::vector<Eigen::Vector3d> prepareRoute(const std::vector<Eigen::Vector3d>& route);
  static double sampledLength(const Trajectory<5>& trajectory);

private:
  std::vector<Eigen::Vector3d> offsetRoute(const std::vector<Eigen::Vector3d>& route,
                                           int candidate_index) const;
  bool buildTrajectory(const std::vector<Eigen::Vector3d>& route,
                       const Eigen::Matrix3d& initial_state,
                       const Eigen::Matrix3d& final_state,
                       Candidate& candidate) const;

  PrimitiveConfig config_;
};

int selectBestCandidate(const std::vector<Candidate>& candidates,
                        bool allow_stability_projection_fallback = false);

std::vector<Eigen::Vector3d> linkEndpoints(const Eigen::Vector3d& link1_tail,
                                           const Eigen::Matrix3d& root_link_rotation,
                                           const Eigen::VectorXd& joint_positions,
                                           const std::vector<int>& pitch_joint_indices,
                                           const std::vector<int>& yaw_joint_indices,
                                           int link_num,
                                           double link_length);

bool bodyCollides(const std::vector<Eigen::Vector3d>& endpoints,
                  double sample_spacing,
                  const std::function<bool(const Eigen::Vector3d&)>& occupied);

const char* candidateStatusName(CandidateStatus status);

}  // namespace motion_primitive_planner

#endif  // MOTION_PRIMITIVE_PLANNER_PLANNER_CORE_H
