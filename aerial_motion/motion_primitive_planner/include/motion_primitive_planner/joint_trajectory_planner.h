// -*- mode: c++ -*-
#ifndef MOTION_PRIMITIVE_PLANNER_JOINT_TRAJECTORY_PLANNER_H
#define MOTION_PRIMITIVE_PLANNER_JOINT_TRAJECTORY_PLANNER_H

#include <motion_primitive_planner/dragon_geometry.h>

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

class TrajectoryHistory
{
public:
  TrajectoryHistory() = default;
  explicit TrajectoryHistory(const FollowerConfig& config);

  bool append(const Eigen::Vector3d& position);
  bool append(const Eigen::Vector3d& position, double sample_interval, double maximum_length);

  const std::deque<multilink_copilot::TrajectoryPoint>& points() const { return points_; }
  double arcLength() const { return arc_length_; }
  double sampleInterval() const { return sample_interval_; }
  double maximumLength() const { return maximum_length_; }

private:
  double sample_interval_ = 0.05;
  double maximum_length_ = 10.0;
  std::deque<multilink_copilot::TrajectoryPoint> points_;
  double arc_length_ = 0.0;
};

struct NominalJointContext
{
  TrajectoryHistory executed_history;
  int link_num = 0;
  double link_length = 0.0;
  std::vector<int> pitch_joint_indices;
  std::vector<int> yaw_joint_indices;
};

NominalJointContext makeNominalJointContext(const TrajectoryHistory& history,
                                            const DragonModelInfo& model);

struct NominalJointSample
{
  double time = 0.0;
  double yaw = 0.0;
  double pitch = 0.0;
  Eigen::Vector3d root_position = Eigen::Vector3d::Zero();
  Eigen::VectorXd joints;
  bool history_changed = false;
};

class NominalJointPredictor
{
public:
  explicit NominalJointPredictor(const FollowerConfig& config) : config_(config)
  {
    config_.validateOrThrow();
  }

  std::vector<NominalJointSample> predict(const Trajectory<5>& root_trajectory,
                                          const NominalJointContext& context,
                                          const Eigen::VectorXd& start_joints,
                                          double start_yaw,
                                          double sample_dt) const;

  std::vector<NominalJointSample> predict(const Trajectory<5>& root_trajectory,
                                          const NominalJointContext& context,
                                          const Eigen::VectorXd& start_joints,
                                          const RootAttitude& start_attitude,
                                          double sample_dt,
                                          bool command_pitch,
                                          double trajectory_start_time = 0.0,
                                          double output_time_offset = 0.0) const;

private:
  FollowerConfig config_;
};

struct TimedJointWaypoint
{
  double time = 0.0;
  Eigen::VectorXd positions;
};

struct TimedRootAttitudeWaypoint
{
  double time = 0.0;
  RootAttitude attitude;
};

struct JointPlanResult
{
  bool success = false;
  std::vector<TimedJointWaypoint> joint_waypoints;
  std::vector<TimedRootAttitudeWaypoint> attitude_waypoints;
  double duration = 0.0;
  double time_scale = 1.0;
  //! Duration for which the root-tail translation remains at trajectory time zero.
  double root_translation_delay = 0.0;
  double minimum_fc_rp = 0.0;
  double joint_motion = 0.0;
  //! RMS distance over time and downstream link tails from the root-tail trace.
  double tracking_error_rms = 0.0;
  //! Largest downstream link-tail distance from the root-tail trace.
  double tracking_error_max = 0.0;
  std::string detail;

  Eigen::VectorXd jointPositions(double time) const;
  Eigen::VectorXd jointVelocities(double time) const;
  RootAttitude attitude(double time) const;
  Eigen::Matrix3d rootLinkRotation(double time) const;
  Eigen::Vector3d angularVelocity(double time) const;
  double yaw(double time) const;
  double yawRate(double time) const;
  double pitch(double time) const;
  double pitchRate(double time) const;
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
                       const RootAttitude& start_attitude,
                       double time_budget = 0.0);

  JointPlanResult plan(const Trajectory<5>& root_trajectory,
                       const NominalJointContext& nominal_context,
                       const Eigen::VectorXd& start_joint_positions,
                       double start_yaw,
                       double time_budget = 0.0)
  {
    return plan(root_trajectory, nominal_context, start_joint_positions,
                RootAttitude{start_yaw, 0.0}, time_budget);
  }

  //! Applies the deterministic joint1-priority prefix at a normalized attitude
  //! progress. Only joint1 pitch/yaw may change; model limits are authoritative.
  static Eigen::VectorXd joint1PriorityConfiguration(
      const Eigen::VectorXd& start_joint_positions,
      int joint1_pitch_index,
      int joint1_yaw_index,
      const std::vector<double>& lower_limits,
      const std::vector<double>& upper_limits,
      const RootAttitude& start_attitude,
      const RootAttitude& goal_attitude,
      double progress);

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
    RootAttitude attitude;
    Eigen::VectorXd joints;
  };

  std::vector<NominalSample> buildNominalSamples(const Trajectory<5>& root_trajectory,
                                                 const NominalJointContext& context,
                                                 const Eigen::VectorXd& start_joints,
                                                 const RootAttitude& start_attitude,
                                                 double trajectory_start_time = 0.0,
                                                 double output_time_offset = 0.0);

  bool chainIsSafe(const std::vector<Eigen::VectorXd>& chain, double start_yaw, double goal_yaw,
                   double& minimum_fc_rp);
  //! Greedy corner removal on a sampled path; the result is subsequently
  //! re-validated against the complete time-varying root-attitude schedule.
  std::vector<Eigen::VectorXd> shortcutChain(const std::vector<Eigen::VectorXd>& chain,
                                             const Eigen::Matrix3d& root_link_rotation);

  bool edgeInteriorIsSafe(const Eigen::VectorXd& start,
                          const Eigen::VectorXd& goal,
                          double start_yaw,
                          double goal_yaw,
                          double resolution,
                          double& minimum_fc_rp);
  bool budgetExpired() const;
  bool searchJointPath(const Eigen::VectorXd& start,
                       const Eigen::VectorXd& goal,
                       const Eigen::Matrix3d& root_link_rotation,
                       const Clock::time_point& deadline,
                       std::vector<Eigen::VectorXd>& path);
  bool configurationIsSafe(const Eigen::VectorXd& joints, double yaw,
                           multilink_copilot::StabilityMetrics* metrics = nullptr);
  bool configurationIsSafe(const Eigen::VectorXd& joints,
                           const RootAttitude& attitude,
                           multilink_copilot::StabilityMetrics* metrics = nullptr);
  bool configurationIsSafe(const Eigen::VectorXd& joints,
                           const Eigen::Matrix3d& root_link_rotation,
                           multilink_copilot::StabilityMetrics* metrics = nullptr);
  static RootAttitude nominalAttitudeAt(const std::vector<NominalSample>& samples,
                                        double time);
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
                      const RootAttitude& attitude,
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
