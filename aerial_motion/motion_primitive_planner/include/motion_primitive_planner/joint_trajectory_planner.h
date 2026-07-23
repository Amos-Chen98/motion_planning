// -*- mode: c++ -*-
#ifndef MOTION_PRIMITIVE_PLANNER_JOINT_TRAJECTORY_PLANNER_H
#define MOTION_PRIMITIVE_PLANNER_JOINT_TRAJECTORY_PLANNER_H

#include <motion_primitive_planner/planner_common.h>

#include <gcopter/trajectory.hpp>
#include <multilink_copilot/follow_the_leader.h>
#include <multilink_copilot/stability_evaluator.h>

#include <Eigen/Dense>

#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace motion_primitive_planner
{

struct JointPlannerConfig
{
  FollowerConfig follower;
  double reference_dt = 0.10;
  double planning_timeout = 0.05;
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

  JointPlanResult plan(const Trajectory<5>& root_trajectory,
                       const NominalJointContext& nominal_context,
                       const Eigen::VectorXd& start_joint_positions,
                       double start_yaw);

  bool planStableConnection(const Eigen::VectorXd& start_joint_positions,
                            const Eigen::VectorXd& goal_joint_positions,
                            double root_link_yaw,
                            std::vector<Eigen::VectorXd>& path,
                            double& minimum_fc_rp,
                            std::string* failure_reason = nullptr);

private:
  struct NominalSample
  {
    double time = 0.0;
    double yaw = 0.0;
    Eigen::VectorXd joints;
    bool safe = false;
  };

  std::vector<NominalSample> buildNominalSamples(const Trajectory<5>& root_trajectory,
                                                  const NominalJointContext& context,
                                                  const Eigen::VectorXd& start_joints,
                                                  double start_yaw);
  bool appendConnection(const TimedJointWaypoint& start,
                        const TimedJointWaypoint& goal,
                        double start_yaw,
                        double goal_yaw,
                        const std::chrono::steady_clock::time_point& deadline,
                        std::vector<TimedJointWaypoint>& path,
                        double& minimum_fc_rp);
  bool directEdgeIsSafe(const Eigen::VectorXd& start,
                        const Eigen::VectorXd& goal,
                        double start_yaw,
                        double goal_yaw,
                        double& minimum_fc_rp);
  bool searchJointDetour(const Eigen::VectorXd& start,
                         const Eigen::VectorXd& goal,
                         double yaw,
                         const std::chrono::steady_clock::time_point& deadline,
                         std::vector<Eigen::VectorXd>& path);
  bool configurationIsSafe(const Eigen::VectorXd& joints, double yaw,
                           multilink_copilot::StabilityMetrics* metrics = nullptr);
  Eigen::VectorXd projectedTerminal(const Eigen::VectorXd& desired,
                                    const Eigen::VectorXd& current,
                                    const Eigen::VectorXd& start,
                                    const NominalJointContext& context,
                                    double yaw,
                                    bool& success);
  JointPlannerConfig config_;
  std::shared_ptr<multilink_copilot::StabilityEvaluator> stability_evaluator_;
  NominalJointPredictor nominal_predictor_;
};

}  // namespace motion_primitive_planner

#endif  // MOTION_PRIMITIVE_PLANNER_JOINT_TRAJECTORY_PLANNER_H
