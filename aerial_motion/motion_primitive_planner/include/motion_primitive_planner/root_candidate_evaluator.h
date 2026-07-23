// -*- mode: c++ -*-
#ifndef MOTION_PRIMITIVE_PLANNER_ROOT_CANDIDATE_EVALUATOR_H
#define MOTION_PRIMITIVE_PLANNER_ROOT_CANDIDATE_EVALUATOR_H

#include <motion_primitive_planner/planner_common.h>

#include <memory>

namespace motion_primitive_planner
{

bool isOnlyFcRpViolation(const multilink_copilot::StabilityMetrics& metrics,
                         const multilink_copilot::StabilityConfig& config);

class RootCandidateEvaluator
{
public:
  RootCandidateEvaluator(const FollowerConfig& follower_config,
                         double prediction_dt,
                         bool allow_stability_projection_fallback,
                         const DragonModelInfo& model,
                         const std::shared_ptr<multilink_copilot::StabilityEvaluator>& stability_evaluator,
                         const PlanningEnvironment& environment);

  void evaluate(Candidate& candidate,
                const NominalJointContext& nominal_context,
                const Eigen::VectorXd& start_joints,
                double start_yaw);

private:
  FollowerConfig follower_config_;
  double prediction_dt_ = 0.10;
  bool allow_stability_projection_fallback_ = false;
  const DragonModelInfo& model_;
  std::shared_ptr<multilink_copilot::StabilityEvaluator> stability_evaluator_;
  const PlanningEnvironment& environment_;
  NominalJointPredictor predictor_;
};

}  // namespace motion_primitive_planner

#endif  // MOTION_PRIMITIVE_PLANNER_ROOT_CANDIDATE_EVALUATOR_H
