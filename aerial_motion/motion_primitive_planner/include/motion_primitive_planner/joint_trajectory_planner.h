// -*- mode: c++ -*-
#ifndef MOTION_PRIMITIVE_PLANNER_JOINT_TRAJECTORY_PLANNER_H
#define MOTION_PRIMITIVE_PLANNER_JOINT_TRAJECTORY_PLANNER_H

#include <motion_primitive_planner/planner_common.h>

#include <gcopter/trajectory.hpp>
#include <multilink_copilot/follow_the_leader.h>
#include <multilink_copilot/stability_evaluator.h>

#include <Eigen/Dense>

#include <chrono>
#include <cstddef>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace motion_primitive_planner
{

struct JointPlannerConfig
{
  FollowerConfig follower;
  double reference_dt = 0.10;
  //! Total per-candidate joint-planning budget.
  double planning_timeout = 0.15;
  double validity_resolution = 0.025;
  double max_joint_velocity = 4.0;
  double max_joint_command_step = 0.10;
  unsigned int random_seed = 1;
};

struct TimedJointWaypoint
{
  double time = 0.0;
  Eigen::VectorXd positions;
};

struct TimedYawWaypoint
{
  double time = 0.0;
  double yaw = 0.0;
};

struct JointPlanResult
{
  bool success = false;
  std::vector<TimedJointWaypoint> joint_waypoints;
  std::vector<TimedYawWaypoint> yaw_waypoints;
  double duration = 0.0;
  double time_scale = 1.0;
  double minimum_fc_rp = 0.0;
  double joint_motion = 0.0;
  //! RMS distance over time and downstream link tails from the root-tail trace.
  double tracking_error_rms = 0.0;
  //! Largest downstream link-tail distance from the root-tail trace.
  double tracking_error_max = 0.0;
  std::string detail;

  Eigen::VectorXd jointPositions(double time) const;
  Eigen::VectorXd jointVelocities(double time) const;
  double yaw(double time) const;
  double yawRate(double time) const;
};

class JointTrajectoryPlanner
{
public:
  JointTrajectoryPlanner(const JointPlannerConfig& config,
                         const std::shared_ptr<multilink_copilot::StabilityEvaluator>& stability_evaluator);

  //! `time_budget` overrides `planning_timeout` when positive, which lets the
  //! caller share one wall-clock budget across all root candidates.
  JointPlanResult plan(const Trajectory<5>& root_trajectory,
                       const NominalJointContext& nominal_context,
                       const Eigen::VectorXd& start_joint_positions,
                       double start_yaw,
                       double time_budget = 0.0);

  bool planStableConnection(const Eigen::VectorXd& start_joint_positions,
                            const Eigen::VectorXd& goal_joint_positions,
                            double root_link_yaw,
                            std::vector<Eigen::VectorXd>& path,
                            double& minimum_fc_rp,
                            std::string* failure_reason = nullptr);

private:
  using Clock = std::chrono::steady_clock;

  struct NominalSample
  {
    double time = 0.0;
    double yaw = 0.0;
    Eigen::VectorXd joints;
  };

  std::vector<NominalSample> buildNominalSamples(const Trajectory<5>& root_trajectory,
                                                 const NominalJointContext& context,
                                                 const Eigen::VectorXd& start_joints,
                                                 double start_yaw);

  bool chainIsSafe(const std::vector<Eigen::VectorXd>& chain, double start_yaw, double goal_yaw,
                   double& minimum_fc_rp);
  //! Greedy corner removal on a sampled path; the result is subsequently
  //! re-validated against the complete time-varying root-yaw schedule.
  std::vector<Eigen::VectorXd> shortcutChain(const std::vector<Eigen::VectorXd>& chain,
                                             double root_yaw);

  bool edgeInteriorIsSafe(const Eigen::VectorXd& start,
                          const Eigen::VectorXd& goal,
                          double start_yaw,
                          double goal_yaw,
                          double resolution,
                          double& minimum_fc_rp);
  bool budgetExpired() const;
  bool searchJointPath(const Eigen::VectorXd& start,
                       const Eigen::VectorXd& goal,
                       double yaw,
                       const Clock::time_point& deadline,
                       std::vector<Eigen::VectorXd>& path);
  bool configurationIsSafe(const Eigen::VectorXd& joints, double yaw,
                           multilink_copilot::StabilityMetrics* metrics = nullptr);
  static double nominalYawAt(const std::vector<NominalSample>& samples, double time);
  bool timedConfigurationPathIsSafe(const TimedJointWaypoint& start,
                                    const TimedJointWaypoint& goal,
                                    const std::vector<NominalSample>& nominal,
                                    double& minimum_fc_rp);
  bool computeTrackingError(const Trajectory<5>& root_trajectory,
                            const NominalJointContext& context,
                            const std::vector<NominalSample>& nominal,
                            JointPlanResult& result) const;
  bool repairEndpoint(const Eigen::VectorXd& desired,
                      const Eigen::VectorXd& reference,
                      double yaw,
                      bool allow_unstable_seed,
                      Eigen::VectorXd& repaired);

  JointPlannerConfig config_;
  //! Hard wall-clock limit for the running plan; every validation loop honours it.
  Clock::time_point deadline_ = Clock::time_point::max();
  std::shared_ptr<multilink_copilot::StabilityEvaluator> stability_evaluator_;
  NominalJointPredictor nominal_predictor_;
};

}  // namespace motion_primitive_planner

#endif  // MOTION_PRIMITIVE_PLANNER_JOINT_TRAJECTORY_PLANNER_H
