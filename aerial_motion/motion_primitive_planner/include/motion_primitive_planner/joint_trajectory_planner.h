// -*- mode: c++ -*-
#ifndef MOTION_PRIMITIVE_PLANNER_JOINT_TRAJECTORY_PLANNER_H
#define MOTION_PRIMITIVE_PLANNER_JOINT_TRAJECTORY_PLANNER_H

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
  double reference_dt = 0.10;
  double command_hz = 40.0;
  double planning_timeout = 0.05;
  double validity_resolution = 0.025;
  double max_joint_velocity = 4.0;
  double max_joint_command_step = 0.10;
  double trajectory_sample_interval = 0.05;
  double trajectory_buffer_max_length = 10.0;
  double ik_singularity_threshold = 0.10;
  double max_yaw_rate = 1.5;
  bool publish_yaw_command = true;
  unsigned int random_seed = 1;
};

struct NominalJointContext
{
  std::deque<multilink_copilot::TrajectoryPoint> executed_history;
  double executed_arc_length = 0.0;
  int link_num = 0;
  double link_length = 0.0;
  std::vector<int> pitch_joint_indices;
  std::vector<int> yaw_joint_indices;
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
  static bool appendHistory(const Eigen::Vector3d& position,
                            double sample_interval,
                            double maximum_length,
                            std::deque<multilink_copilot::TrajectoryPoint>& history,
                            double& arc_length);

  JointPlannerConfig config_;
  std::shared_ptr<multilink_copilot::StabilityEvaluator> stability_evaluator_;
};

}  // namespace motion_primitive_planner

#endif  // MOTION_PRIMITIVE_PLANNER_JOINT_TRAJECTORY_PLANNER_H
