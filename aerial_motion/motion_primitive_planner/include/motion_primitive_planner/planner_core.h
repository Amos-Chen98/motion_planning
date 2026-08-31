// -*- mode: c++ -*-
#ifndef MOTION_PRIMITIVE_PLANNER_PLANNER_CORE_H
#define MOTION_PRIMITIVE_PLANNER_PLANNER_CORE_H

#include <gcopter/trajectory.hpp>

#include <Eigen/Dense>

#include <functional>
#include <limits>
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
  kJointPlanningFailed,
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

struct WholeBodyCandidateScore
{
  bool feasible = false;
  double duration = std::numeric_limits<double>::infinity();
  double joint_motion = std::numeric_limits<double>::infinity();
  double root_jerk = std::numeric_limits<double>::infinity();
  double tracking_error_rms = 0.0;
};

struct WholeBodyConfiguration
{
  Eigen::Vector3d link1_tail = Eigen::Vector3d::Zero();
  Eigen::Matrix3d root_link_rotation = Eigen::Matrix3d::Identity();
  Eigen::VectorXd joint_positions;
};

struct DragonCollisionGeometry
{
  int link_num = 0;
  double link_length = 0.0;
  std::vector<int> pitch_joint_indices;
  std::vector<int> yaw_joint_indices;
};

struct RootCommandKinematics
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

class TrajectoryReplanTrigger
{
public:
  explicit TrajectoryReplanTrigger(double trigger_ratio = 0.5);

  void arm(double start_time, double duration, bool terminal);
  void reset();
  bool shouldTrigger(double current_time);

  double triggerRatio() const { return trigger_ratio_; }
  bool armed() const { return armed_; }
  bool triggered() const { return triggered_; }
  bool terminal() const { return terminal_; }

private:
  double trigger_ratio_ = 0.5;
  double start_time_ = 0.0;
  double duration_ = 0.0;
  bool armed_ = false;
  bool triggered_ = false;
  bool terminal_ = false;
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

int selectBestCandidate(const std::vector<Candidate>& candidates,
                        bool allow_stability_projection_fallback = false);

//! The two weights convert joint-space path length [rad] and downstream-link
//! tracking RMS [m] into equivalent execution-time penalties [s].
int selectBestWholeBodyCandidate(const std::vector<WholeBodyCandidateScore>& candidates,
                                 double joint_motion_cost_weight = 0.25,
                                 double tracking_error_cost_weight = 6.0);

RootCommandKinematics tailFluToRootLinkCommand(const Eigen::Vector3d& tail_position,
                                                const Eigen::Vector3d& tail_velocity,
                                                const Eigen::Matrix3d& tail_flu_rotation,
                                                const Eigen::Vector3d& angular_velocity,
                                                double link_length);

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

bool wholeBodyCollides(const WholeBodyConfiguration& configuration,
                       const DragonCollisionGeometry& geometry,
                       double sample_spacing,
                       const std::function<bool(const Eigen::Vector3d&)>& occupied);

double shortestYawDelta(double start_yaw, double end_yaw);

const char* candidateStatusName(CandidateStatus status);

}  // namespace motion_primitive_planner

#endif  // MOTION_PRIMITIVE_PLANNER_PLANNER_CORE_H
