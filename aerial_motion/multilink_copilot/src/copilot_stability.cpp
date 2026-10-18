// copilot_stability.cpp
// Stability evaluation, reference selection, and joint-space helpers

#include <multilink_copilot/copilot.h>
#include <multilink_copilot/joint_command_continuity.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace multilink_copilot
{
bool CopilotPlanner::computeStableJointPositions(const Eigen::VectorXd& nominal_joint_positions,
                                                 Eigen::VectorXd& stable_joint_positions)
{
  const Eigen::VectorXd precheck_joint_positions = clampLinkJointPositions(getCurrentLinkJointPositions());
  const Eigen::VectorXd measured_joint_positions = precheck_joint_positions;
  const Eigen::VectorXd desired_joint_positions = clampLinkJointPositions(nominal_joint_positions);
  const auto evaluate_seed = [this](const Eigen::VectorXd& joints) {
    JointSeedEvaluation evaluation;
    evaluation.valid = evaluateStability(joints, evaluation.metrics);
    if (evaluation.valid)
      evaluation.violation_score = computeStabilityViolationScore(evaluation.metrics);
    return evaluation;
  };
  const JointSeedEvaluation desired_evaluation = evaluate_seed(desired_joint_positions);
  const StabilityMetrics& desired_metrics = desired_evaluation.metrics;

  StableCandidateSource accepted_source = StableCandidateSource::kDesiredSeed;

  if (desired_evaluation.isValid() && desired_metrics.safe)
  {
    stable_joint_positions = desired_joint_positions;
  }
  else
  {
    const Eigen::VectorXd* published = has_latest_published_joint_positions_ ?
        &latest_published_joint_positions_ : nullptr;
    const JointSeedSelection selection = selectJointSeed(
        desired_joint_positions, measured_joint_positions, published, stable_joint_history_,
        desired_evaluation, evaluate_seed);

    if (selection.reference_index)
    {
      const JointSeedCandidate& reference = selection.candidates[*selection.reference_index];
      const Eigen::VectorXd& stable_reference = reference.joint_positions;
      accepted_source = reference.source;
      restoreRobotModelToLinkJointPositions(stable_reference);

      if (!solveStableJointQp(desired_joint_positions, stable_reference, stable_joint_positions))
      {
        used_fallback_ = true;
        stable_joint_positions = stable_reference;
      }

      if (!checkStability(stable_joint_positions, false))
      {
        if (!checkStability(stable_reference, false))
        {
          restoreRobotModelToLinkJointPositions(precheck_joint_positions);
          ROS_ERROR_THROTTLE(1.0, "[CopilotPlanner] Failed to find any conservative stable reference configuration");
          return false;
        }

        used_fallback_ = true;
        stable_joint_positions = stable_reference;
      }

      if (!areSimilarJointPositions(stable_joint_positions, stable_reference))
      {
        accepted_source = StableCandidateSource::kProjected;
      }
    }
    else
    {
      const auto project = [this, &desired_joint_positions](const Eigen::VectorXd& seed,
                                                           Eigen::VectorXd& repaired) {
        return solveStableJointQp(desired_joint_positions, seed, repaired, true);
      };
      if (!repairJointSeeds(desired_joint_positions, selection.candidates, stability_candidate_max_repairs_,
                            project, stable_joint_positions))
      {
        restoreRobotModelToLinkJointPositions(precheck_joint_positions);
        ROS_ERROR_THROTTLE(1.0, "[CopilotPlanner] Failed to find any conservative stable reference configuration");
        return false;
      }
      accepted_source = StableCandidateSource::kRepair;
    }
  }

  stable_joint_positions = clampLinkJointPositions(stable_joint_positions);
  if (!enforceStableJointCommandContinuity(measured_joint_positions, stable_joint_positions, accepted_source))
  {
    restoreRobotModelToLinkJointPositions(precheck_joint_positions);
    return false;
  }

  accepted_source_ = describeStableCandidateSource(accepted_source);
  restoreRobotModelToLinkJointPositions(stable_joint_positions);
  rememberStableJointPositions(stable_joint_positions);

  if ((desired_joint_positions - stable_joint_positions).norm() > stability_qp_convergence_tol_)
  {
    StabilityMetrics stable_metrics;
    evaluateStability(stable_joint_positions, stable_metrics);

    ROS_WARN_STREAM_THROTTLE(0.5,
                             "[CopilotPlanner] Solved a conservative stable joint target"
                                 << ", source: " << describeStableCandidateSource(accepted_source)
                                 << ", desired_violation: " << describeStabilityViolations(desired_metrics)
                                 << ", stable_metrics: (fc_rp_min: " << stable_metrics.fc_rp_min
                                 << ", thrust range: [" << stable_metrics.static_thrust_min << ", "
                                 << stable_metrics.static_thrust_max
                                 << "], overlap clearance: " << stable_metrics.overlap_clearance << ")");
  }

  return true;
}

void CopilotPlanner::updateRobotModelForTargetConfiguration(const KDL::JntArray& joint_positions)
{
  if (latest_target_pose_)
  {
    tf::Transform root_tf;
    tf::poseMsgToTF(convertLink1TailPoseToRootPose(latest_target_pose_->pose), root_tf);

    tf::Transform root_to_baselink_tf;
    tf::transformKDLToTF(
        dragon_robot_model_->forwardKinematics<KDL::Frame>(dragon_robot_model_->getBaselinkName(), joint_positions),
        root_to_baselink_tf);

    KDL::Rotation desired_cog_orientation;
    tf::quaternionTFToKDL(root_tf * root_to_baselink_tf.getRotation(), desired_cog_orientation);
    dragon_robot_model_->setCogDesireOrientation(desired_cog_orientation);
  }

  dragon_robot_model_->updateRobotModel(joint_positions);
}

bool CopilotPlanner::evaluateStability(const Eigen::VectorXd& joint_positions, StabilityMetrics& metrics)
{
  const Eigen::VectorXd clamped_joint_positions = clampLinkJointPositions(joint_positions);
  KDL::Rotation root_rotation = KDL::Rotation::Identity();
  if (latest_target_pose_)
  {
    const geometry_msgs::Pose root_pose = convertLink1TailPoseToRootPose(latest_target_pose_->pose);
    root_rotation = KDL::Rotation::Quaternion(root_pose.orientation.x, root_pose.orientation.y,
                                              root_pose.orientation.z, root_pose.orientation.w);
  }
  stability_evaluator_->setRootLinkRotation(root_rotation);
  return stability_evaluator_->evaluate(clamped_joint_positions, metrics);
}

bool CopilotPlanner::satisfiesSafeStability(const StabilityMetrics& metrics) const
{
  return stability_evaluator_->satisfiesSafeStability(metrics);
}

std::string CopilotPlanner::describeStabilityViolations(const StabilityMetrics& metrics) const
{
  return stability_evaluator_->describeViolations(metrics);
}

double CopilotPlanner::computeStabilityViolationScore(const StabilityMetrics& metrics) const
{
  return stability_evaluator_->violationScore(metrics);
}

void CopilotPlanner::rememberStableJointPositions(const Eigen::VectorXd& stable_joint_positions)
{
  rememberJointSeed(stable_joint_history_, clampLinkJointPositions(stable_joint_positions),
                    static_cast<size_t>(stability_candidate_history_size_), stability_qp_convergence_tol_);
}

bool CopilotPlanner::areSimilarJointPositions(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs) const
{
  if (lhs.size() != rhs.size())
  {
    return false;
  }

  return (lhs - rhs).norm() <= stability_qp_convergence_tol_;
}

const char* CopilotPlanner::describeStableCandidateSource(StableCandidateSource source) const
{
  switch (source)
  {
    case StableCandidateSource::kLatestPublished:
      return "latest_published";
    case StableCandidateSource::kCurrentMeasured:
      return "current_measured";
    case StableCandidateSource::kStableHistory:
      return "stable_history";
    case StableCandidateSource::kDesiredSeed:
      return "desired";
    case StableCandidateSource::kProjected:
      return "projected";
    case StableCandidateSource::kRepair:
      return "repair";
  }

  return "unknown";
}

bool CopilotPlanner::enforceStableJointCommandContinuity(const Eigen::VectorXd& measured_joint_positions,
                                                         Eigen::VectorXd& stable_joint_positions,
                                                         StableCandidateSource& accepted_source)
{
  if (max_joint_step_before_publish_ <= 0.0 || link_joint_num_ <= 0)
  {
    return true;
  }

  Eigen::VectorXd continuity_reference = measured_joint_positions;
  StableCandidateSource reference_source = StableCandidateSource::kCurrentMeasured;
  if (has_latest_published_joint_positions_ && latest_published_joint_positions_.size() == link_joint_num_)
  {
    continuity_reference = clampLinkJointPositions(latest_published_joint_positions_);
    reference_source = StableCandidateSource::kLatestPublished;
  }

  Eigen::VectorXd limited_joint_positions;
  bool was_limited = false;
  if (!limitJointPositionStep(continuity_reference, stable_joint_positions, max_joint_step_before_publish_,
                              limited_joint_positions, was_limited))
  {
    ROS_ERROR_THROTTLE(1.0, "[CopilotPlanner] Cannot enforce joint command continuity due to invalid input");
    return false;
  }

  if (!was_limited)
  {
    return true;
  }

  const double requested_max_step =
      (stable_joint_positions - continuity_reference).cwiseAbs().maxCoeff();
  StabilityMetrics limited_metrics;
  if (evaluateStability(limited_joint_positions, limited_metrics) && limited_metrics.safe)
  {
    stable_joint_positions = limited_joint_positions;
    accepted_source = StableCandidateSource::kProjected;
    ROS_WARN_THROTTLE(0.5,
                      "[CopilotPlanner] Rate-limited stable joint target: requested max step %.3f rad, "
                      "limited to %.3f rad",
                      requested_max_step, max_joint_step_before_publish_);
    return true;
  }

  StabilityMetrics reference_metrics;
  if (evaluateStability(continuity_reference, reference_metrics) && reference_metrics.safe)
  {
    used_fallback_ = true;
    stable_joint_positions = continuity_reference;
    accepted_source = reference_source;
    ROS_WARN_THROTTLE(0.5,
                      "[CopilotPlanner] Holding the previous stable joint target: the bounded transition toward "
                      "a different stability branch is unsafe");
    return true;
  }

  ROS_ERROR_THROTTLE(1.0,
                     "[CopilotPlanner] Neither the bounded joint transition nor its continuity reference is stable");
  return false;
}

bool CopilotPlanner::checkStability(const Eigen::VectorXd& joint_positions, bool report_result)
{
  const Eigen::VectorXd clamped_joint_positions = clampLinkJointPositions(joint_positions);
  StabilityMetrics metrics;
  evaluateStability(clamped_joint_positions, metrics);

  const bool is_stable = metrics.safe;
  if (!report_result)
  {
    return is_stable;
  }

  if (is_stable)
  {
    ROS_INFO_STREAM("[CopilotPlanner] Current target is stable"
                    << " (fc_rp_min: " << metrics.fc_rp_min << ", thrust range: ["
                    << metrics.static_thrust_min << ", " << metrics.static_thrust_max
                    << "], overlap clearance: " << metrics.overlap_clearance << ")");
  }
  else
  {
    ROS_WARN_STREAM("[CopilotPlanner] Current target is unstable"
                    << " (fc_rp_min: " << metrics.fc_rp_min << ", thrust range: ["
                    << metrics.static_thrust_min << ", " << metrics.static_thrust_max
                    << "], overlap clearance: " << metrics.overlap_clearance
                    << ", violation: " << describeStabilityViolations(metrics) << ")");

    dragon_robot_model_->stabilityCheck(true);
  }

  return is_stable;
}

KDL::JntArray CopilotPlanner::buildUpdatedJointPositions(const Eigen::VectorXd& joint_positions) const
{
  KDL::JntArray updated_joint_positions = dragon_robot_model_->getJointPositions();
  if (updated_joint_positions.rows() != dragon_robot_model_->getTree().getNrOfJoints())
  {
    updated_joint_positions.resize(dragon_robot_model_->getTree().getNrOfJoints());
  }

  const int joint_count = std::min<int>(joint_positions.size(), link_joint_indices_.size());
  for (int i = 0; i < joint_count; ++i)
  {
    updated_joint_positions(link_joint_indices_.at(i)) = joint_positions(i);
  }

  return updated_joint_positions;
}

Eigen::VectorXd CopilotPlanner::clampLinkJointPositions(const Eigen::VectorXd& joint_positions) const
{
  Eigen::VectorXd clamped_joint_positions = Eigen::VectorXd::Zero(link_joint_num_);
  const int joint_count = std::min<int>(joint_positions.size(), link_joint_num_);

  for (int i = 0; i < joint_count; ++i)
  {
    double value = joint_positions(i);
    if (!std::isfinite(value))
    {
      ROS_WARN_THROTTLE(1.0, "[CopilotPlanner] Detected non-finite joint target; replacing it with 0");
      value = 0.0;
    }

    if (i < static_cast<int>(dragon_robot_model_->getLinkJointLowerLimits().size()) &&
        i < static_cast<int>(dragon_robot_model_->getLinkJointUpperLimits().size()))
    {
      value = std::max(dragon_robot_model_->getLinkJointLowerLimits().at(i),
                       std::min(value, dragon_robot_model_->getLinkJointUpperLimits().at(i)));
    }

    clamped_joint_positions(i) = value;
  }

  return clamped_joint_positions;
}

Eigen::VectorXd CopilotPlanner::getCurrentLinkJointPositions() const
{
  Eigen::VectorXd current_link_joint_positions = Eigen::VectorXd::Zero(link_joint_num_);
  const KDL::JntArray& full_joint_positions = dragon_robot_model_->getJointPositions();

  for (int i = 0; i < link_joint_num_; ++i)
  {
    if (i < static_cast<int>(link_joint_indices_.size()) &&
        link_joint_indices_.at(i) < static_cast<int>(full_joint_positions.rows()))
    {
      current_link_joint_positions(i) = full_joint_positions(link_joint_indices_.at(i));
    }
  }

  return current_link_joint_positions;
}

Eigen::VectorXd CopilotPlanner::buildDefaultReferenceJointPositions() const
{
  Eigen::VectorXd default_joint_positions = Eigen::VectorXd::Zero(link_joint_num_);

  int valid_yaw_count = 0;
  for (const int local_index : yaw_joint_local_indices_)
  {
    if (local_index < 0 || local_index >= link_joint_num_)
    {
      continue;
    }

    if (valid_yaw_count == 0)
    {
      default_joint_positions(local_index) = -M_PI / 4.0;
    }
    else if (valid_yaw_count <= 2)
    {
      default_joint_positions(local_index) = M_PI / 2.0;
    }

    ++valid_yaw_count;
  }

  return clampLinkJointPositions(default_joint_positions);
}

void CopilotPlanner::restoreRobotModelToLinkJointPositions(const Eigen::VectorXd& joint_positions)
{
  updateRobotModelForTargetConfiguration(buildUpdatedJointPositions(clampLinkJointPositions(joint_positions)));
}

}  // namespace multilink_copilot
