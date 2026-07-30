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
  //! Budget for a single infeasible-interval bridge inside that total.
  double bridge_timeout = 0.12;
  double validity_resolution = 0.025;
  double max_joint_velocity = 4.0;
  double max_joint_command_step = 0.10;
  //! How far the bridge endpoints may be pulled away from an infeasible interval.
  double anchor_backoff_time = 0.50;
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
  //! Number of infeasible nominal intervals repaired by the bridge search.
  int bridge_count = 0;
  //! Number of infeasible nominal intervals the planner could only hold through.
  int hold_count = 0;
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
    bool safe = false;
    //! Normalized worst-case flight-feasibility margin; negative when infeasible.
    double margin = -std::numeric_limits<double>::infinity();
    double fc_rp_min = 0.0;
  };

  //! Inclusive [first, last] index range of consecutive flight-feasible samples.
  struct SafeRun
  {
    size_t first = 0;
    size_t last = 0;
  };

  std::vector<NominalSample> buildNominalSamples(const Trajectory<5>& root_trajectory,
                                                 const NominalJointContext& context,
                                                 const Eigen::VectorXd& start_joints,
                                                 double start_yaw);
  static std::vector<SafeRun> safeRuns(const std::vector<NominalSample>& samples);

  //! Ordered anchor indices for one side of a bridge: the sample closest to the
  //! infeasible interval first, then progressively better-conditioned ones.
  std::vector<size_t> anchorCandidates(const std::vector<NominalSample>& samples,
                                       size_t nominal_index, size_t limit_index) const;

  //! `start` is taken by value because `path` may be the container holding it.
  bool appendBridge(TimedJointWaypoint start,
                    double start_root_yaw,
                    const Eigen::VectorXd& goal,
                    double goal_time,
                    double goal_root_yaw,
                    const Clock::time_point& deadline,
                    std::vector<TimedJointWaypoint>& path,
                    double& minimum_fc_rp,
                    int& bridge_count);

  bool planBridge(const Eigen::VectorXd& start,
                  const Eigen::VectorXd& goal,
                  double start_yaw,
                  double goal_yaw,
                  const Clock::time_point& deadline,
                  std::vector<Eigen::VectorXd>& path,
                  double& minimum_fc_rp);

  std::vector<std::vector<Eigen::VectorXd>> bridgeChains(const Eigen::VectorXd& start,
                                                         const Eigen::VectorXd& goal) const;
  std::vector<std::vector<int>> jointOrders(const Eigen::VectorXd& start,
                                            const Eigen::VectorXd& goal) const;
  static std::vector<Eigen::VectorXd> expandChain(const std::vector<Eigen::VectorXd>& keys,
                                                  const std::vector<int>& order);
  Eigen::VectorXd deepFold(const Eigen::VectorXd& reference,
                           const Eigen::VectorXd& sign_source) const;

  bool chainIsSafe(const std::vector<Eigen::VectorXd>& chain, double start_yaw, double goal_yaw,
                   double& minimum_fc_rp);
  //! Greedy corner removal on a sampled detour; the result is re-validated by
  //! `chainIsSafe`, so a single root yaw is enough for the search itself.
  std::vector<Eigen::VectorXd> shortcutChain(const std::vector<Eigen::VectorXd>& chain,
                                             double root_yaw);

  bool directEdgeIsSafe(const Eigen::VectorXd& start,
                        const Eigen::VectorXd& goal,
                        double start_yaw,
                        double goal_yaw,
                        double& minimum_fc_rp);
  //! Validates only the strict interior of an edge whose endpoints are already
  //! known to be flight-feasible, which halves the cost of dense tracking.
  bool edgeInteriorIsSafe(const Eigen::VectorXd& start,
                          const Eigen::VectorXd& goal,
                          double start_yaw,
                          double goal_yaw,
                          double resolution,
                          double& minimum_fc_rp);
  bool budgetExpired() const;
  bool searchJointDetour(const Eigen::VectorXd& start,
                         const Eigen::VectorXd& goal,
                         double yaw,
                         const Clock::time_point& deadline,
                         std::vector<Eigen::VectorXd>& path);
  bool configurationIsSafe(const Eigen::VectorXd& joints, double yaw,
                           multilink_copilot::StabilityMetrics* metrics = nullptr);
  double stabilityMargin(const Eigen::VectorXd& joints, double yaw, bool& safe,
                         double& fc_rp_min);
  Eigen::VectorXd projectedTerminal(const Eigen::VectorXd& desired,
                                    const Eigen::VectorXd& current,
                                    const Eigen::VectorXd& start,
                                    double yaw,
                                    bool& success);

  JointPlannerConfig config_;
  //! Hard wall-clock limit for the running plan; every validation loop honours it.
  Clock::time_point deadline_ = Clock::time_point::max();
  std::shared_ptr<multilink_copilot::StabilityEvaluator> stability_evaluator_;
  NominalJointPredictor nominal_predictor_;
  //! Per-joint deep-fold magnitude derived from the model joint limits.
  Eigen::VectorXd fold_magnitude_;
  std::vector<int> pitch_joint_indices_;
  std::vector<int> yaw_joint_indices_;
};

}  // namespace motion_primitive_planner

#endif  // MOTION_PRIMITIVE_PLANNER_JOINT_TRAJECTORY_PLANNER_H
