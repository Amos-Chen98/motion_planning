// -*- mode: c++ -*-
#ifndef MULTILINK_COPILOT_STABILITY_EVALUATOR_H
#define MULTILINK_COPILOT_STABILITY_EVALUATOR_H

#include <dragon/model/hydrus_like_robot_model.h>

#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <string>
#include <vector>

namespace multilink_copilot
{

struct StabilityConfig
{
  int qp_max_iterations = 20;
  double qp_joint_step_limit = 0.1;
  double qp_regularization = 1e-3;
  double qp_convergence_tolerance = 1e-3;
  double feasibility_tolerance = 1e-4;
  bool check_fc_t = false;
  double fc_rp_min_threshold = 3.2;
  double fc_t_min_threshold = 0.01;
  double static_thrust_min = 2.0;
  double static_thrust_max = 30.0;
  double overlap_min_clearance = 0.01;
  double max_baselink_tilt = 1.2;
};

struct StabilityMetrics
{
  bool safe = false;
  double fc_rp_min = 0.0;
  double fc_t_min = 0.0;
  double static_thrust_min = 0.0;
  double static_thrust_max = 0.0;
  double overlap_clearance = 0.0;
  double baselink_tilt = 0.0;
};

class StabilityEvaluator
{
public:
  StabilityEvaluator(const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& robot_model,
                     const StabilityConfig& config);

  void setRootLinkRotation(const KDL::Rotation& root_link_rotation);
  bool evaluate(const Eigen::VectorXd& joint_positions, StabilityMetrics& metrics);
  bool satisfiesSafeStability(const StabilityMetrics& metrics) const;
  std::string describeViolations(const StabilityMetrics& metrics) const;
  double violationScore(const StabilityMetrics& metrics) const;

  bool projectToSafe(const Eigen::VectorXd& desired_joint_positions,
                     const Eigen::VectorXd& reference_joint_positions,
                     Eigen::VectorXd& stable_joint_positions,
                     bool allow_unstable_seed = false);
  bool projectNearestSafe(const Eigen::VectorXd& desired_joint_positions,
                          const std::vector<Eigen::VectorXd>& reference_joint_positions,
                          Eigen::VectorXd& stable_joint_positions);

  bool withinJointLimits(const Eigen::VectorXd& joint_positions) const;
  Eigen::VectorXd clampJointPositions(const Eigen::VectorXd& joint_positions) const;
  KDL::JntArray buildFullJointPositions(const Eigen::VectorXd& joint_positions) const;
  void updateRobotModel(const Eigen::VectorXd& joint_positions);

  int jointCount() const;
  const StabilityConfig& config() const;
  const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& robotModel() const;

private:
  boost::shared_ptr<Dragon::HydrusLikeRobotModel> robot_model_;
  StabilityConfig config_;
  KDL::Rotation root_link_rotation_ = KDL::Rotation::Identity();
  std::vector<int> link_joint_indices_;
  int link_joint_num_ = 0;
  int link_num_ = 0;
};

}  // namespace multilink_copilot

#endif  // MULTILINK_COPILOT_STABILITY_EVALUATOR_H
