// copilot_stability.cpp
// Stability evaluation, reference selection, and joint-space helpers

#include <multilink_copilot/copilot.h>
#include <multilink_copilot/joint_command_continuity.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace multilink_copilot
{
namespace
{
constexpr double kDirectionNormEpsilon = 1e-6;

double clampUnit(double value)
{
  return std::max(-1.0, std::min(1.0, value));
}

int getSafeCandidatePriority(CopilotPlanner::StableCandidateSource source)
{
  switch (source)
  {
    case CopilotPlanner::StableCandidateSource::kLatestPublished:
      return 0;
    case CopilotPlanner::StableCandidateSource::kCurrentMeasured:
      return 1;
    case CopilotPlanner::StableCandidateSource::kLatestStable:
      return 2;
    case CopilotPlanner::StableCandidateSource::kStableHistory:
      return 3;
    case CopilotPlanner::StableCandidateSource::kDesiredSeed:
      return 4;
    case CopilotPlanner::StableCandidateSource::kProjected:
      return 5;
    case CopilotPlanner::StableCandidateSource::kRepair:
      return 6;
  }

  return 7;
}

int getRepairCandidatePriority(CopilotPlanner::StableCandidateSource source)
{
  switch (source)
  {
    case CopilotPlanner::StableCandidateSource::kLatestPublished:
      return 0;
    case CopilotPlanner::StableCandidateSource::kCurrentMeasured:
      return 1;
    case CopilotPlanner::StableCandidateSource::kDesiredSeed:
      return 2;
    case CopilotPlanner::StableCandidateSource::kLatestStable:
      return 3;
    case CopilotPlanner::StableCandidateSource::kStableHistory:
      return 4;
    case CopilotPlanner::StableCandidateSource::kProjected:
      return 5;
    case CopilotPlanner::StableCandidateSource::kRepair:
      return 6;
  }

  return 7;
}
}  // namespace

bool CopilotPlanner::computeStableJointPositions(const Eigen::VectorXd& nominal_joint_positions,
                                                 Eigen::VectorXd& stable_joint_positions)
{
  const Eigen::VectorXd precheck_joint_positions = clampLinkJointPositions(getCurrentLinkJointPositions());
  const Eigen::VectorXd measured_joint_positions = precheck_joint_positions;
  const Eigen::VectorXd desired_joint_positions = clampLinkJointPositions(nominal_joint_positions);
  StabilityMetrics desired_metrics;
  evaluateStability(desired_joint_positions, desired_metrics);

  StableCandidateSource accepted_source = StableCandidateSource::kDesiredSeed;

  if (desired_metrics.safe)
  {
    stable_joint_positions = desired_joint_positions;
  }
  else
  {
    const Eigen::Vector3d current_target_direction = getCurrentTargetDirection();
    const std::vector<StableCandidate> candidate_pool =
        buildStableCandidates(desired_joint_positions, measured_joint_positions, current_target_direction);

    Eigen::VectorXd stable_reference;
    StableCandidateSource reference_source = StableCandidateSource::kLatestStable;
    if (tryGetStableReferenceJointPositions(candidate_pool, stable_reference, reference_source))
    {
      accepted_source = reference_source;

      if (!solveStableJointQp(desired_joint_positions, stable_reference, stable_joint_positions))
      {
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

        stable_joint_positions = stable_reference;
      }

      if (!areSimilarJointPositions(stable_joint_positions, stable_reference))
      {
        accepted_source = StableCandidateSource::kProjected;
      }
    }
    else if (!tryRepairStableJointPositions(desired_joint_positions, candidate_pool, stable_joint_positions,
                                            accepted_source))
    {
      restoreRobotModelToLinkJointPositions(precheck_joint_positions);
      ROS_ERROR_THROTTLE(1.0, "[CopilotPlanner] Failed to find any conservative stable reference configuration");
      return false;
    }
  }

  stable_joint_positions = clampLinkJointPositions(stable_joint_positions);
  if (!enforceStableJointCommandContinuity(measured_joint_positions, stable_joint_positions, accepted_source))
  {
    restoreRobotModelToLinkJointPositions(precheck_joint_positions);
    return false;
  }

  restoreRobotModelToLinkJointPositions(stable_joint_positions);
  rememberStableJointPositions(stable_joint_positions, accepted_source);

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

std::vector<CopilotPlanner::StableCandidate> CopilotPlanner::buildStableCandidates(
    const Eigen::VectorXd& desired_joint_positions,
    const Eigen::VectorXd& measured_joint_positions,
    const Eigen::Vector3d& current_target_direction)
{
  std::vector<StableCandidate> candidate_pool;
  candidate_pool.reserve(static_cast<size_t>(3 + stability_candidate_top_k_ + 1));

  const auto append_candidate = [&](const Eigen::VectorXd& raw_joint_positions, StableCandidateSource source,
                                    double target_direction_angle = 0.0) {
    const Eigen::VectorXd clamped_joint_positions = clampLinkJointPositions(raw_joint_positions);
    for (const StableCandidate& existing_candidate : candidate_pool)
    {
      if (areSimilarJointPositions(existing_candidate.joint_positions, clamped_joint_positions))
      {
        return;
      }
    }

    candidate_pool.push_back(buildStableCandidate(clamped_joint_positions, source, desired_joint_positions,
                                                  measured_joint_positions, target_direction_angle));
  };

  append_candidate(measured_joint_positions, StableCandidateSource::kCurrentMeasured);

  if (has_latest_published_joint_positions_ && latest_published_joint_positions_.size() == link_joint_num_)
  {
    append_candidate(latest_published_joint_positions_, StableCandidateSource::kLatestPublished);
  }

  if (has_latest_stable_joint_positions_ && latest_stable_joint_positions_.size() == link_joint_num_)
  {
    append_candidate(latest_stable_joint_positions_, StableCandidateSource::kLatestStable);
  }

  for (const StableHistoryEntry& history_entry :
       getNearestStableHistoryEntries(desired_joint_positions, measured_joint_positions, current_target_direction))
  {
    append_candidate(history_entry.joint_positions, StableCandidateSource::kStableHistory,
                     computeTargetDirectionAngle(current_target_direction, history_entry.target_direction));
  }

  append_candidate(desired_joint_positions, StableCandidateSource::kDesiredSeed);
  return candidate_pool;
}

bool CopilotPlanner::tryGetStableReferenceJointPositions(const std::vector<StableCandidate>& candidate_pool,
                                                         Eigen::VectorXd& stable_reference,
                                                         StableCandidateSource& reference_source)
{
  std::vector<StableCandidate> safe_candidates;
  safe_candidates.reserve(candidate_pool.size());
  for (const StableCandidate& candidate : candidate_pool)
  {
    if (candidate.metrics.safe)
    {
      safe_candidates.push_back(candidate);
    }
  }

  if (safe_candidates.empty())
  {
    return false;
  }

  const auto best_candidate = std::min_element(
      safe_candidates.begin(), safe_candidates.end(),
      [](const StableCandidate& lhs, const StableCandidate& rhs) {
        if (getSafeCandidatePriority(lhs.source) != getSafeCandidatePriority(rhs.source))
        {
          return getSafeCandidatePriority(lhs.source) < getSafeCandidatePriority(rhs.source);
        }
        if (lhs.measured_distance != rhs.measured_distance)
        {
          return lhs.measured_distance < rhs.measured_distance;
        }
        return lhs.desired_distance < rhs.desired_distance;
      });

  stable_reference = best_candidate->joint_positions;
  reference_source = best_candidate->source;
  restoreRobotModelToLinkJointPositions(stable_reference);
  return true;
}

bool CopilotPlanner::tryRepairStableJointPositions(const Eigen::VectorXd& desired_joint_positions,
                                                   const std::vector<StableCandidate>& candidate_pool,
                                                   Eigen::VectorXd& stable_joint_positions,
                                                   StableCandidateSource& repaired_source)
{
  if (stability_candidate_max_repairs_ <= 0)
  {
    return false;
  }

  std::vector<StableCandidate> repair_candidates;
  repair_candidates.reserve(candidate_pool.size());
  for (const StableCandidate& candidate : candidate_pool)
  {
    if (!candidate.metrics.safe)
    {
      repair_candidates.push_back(candidate);
    }
  }

  if (repair_candidates.empty())
  {
    return false;
  }

  std::sort(repair_candidates.begin(), repair_candidates.end(),
            [](const StableCandidate& lhs, const StableCandidate& rhs) {
              if (lhs.violation_score != rhs.violation_score)
              {
                return lhs.violation_score < rhs.violation_score;
              }
              if (getRepairCandidatePriority(lhs.source) != getRepairCandidatePriority(rhs.source))
              {
                return getRepairCandidatePriority(lhs.source) < getRepairCandidatePriority(rhs.source);
              }
              if (lhs.desired_distance != rhs.desired_distance)
              {
                return lhs.desired_distance < rhs.desired_distance;
              }
              return lhs.measured_distance < rhs.measured_distance;
            });

  bool found_repaired_candidate = false;
  double best_desired_distance = std::numeric_limits<double>::infinity();
  const int repair_attempts = std::min<int>(stability_candidate_max_repairs_, repair_candidates.size());
  for (int i = 0; i < repair_attempts; ++i)
  {
    Eigen::VectorXd repaired_joint_positions;
    if (!solveStableJointQp(desired_joint_positions, repair_candidates.at(i).joint_positions, repaired_joint_positions,
                            true))
    {
      continue;
    }

    const double desired_distance = (repaired_joint_positions - desired_joint_positions).norm();
    if (!found_repaired_candidate || desired_distance < best_desired_distance)
    {
      stable_joint_positions = repaired_joint_positions;
      best_desired_distance = desired_distance;
      repaired_source = StableCandidateSource::kRepair;
      found_repaired_candidate = true;
    }
  }

  return found_repaired_candidate;
}

CopilotPlanner::StableCandidate CopilotPlanner::buildStableCandidate(const Eigen::VectorXd& raw_joint_positions,
                                                                     StableCandidateSource source,
                                                                     const Eigen::VectorXd& desired_joint_positions,
                                                                     const Eigen::VectorXd& measured_joint_positions,
                                                                     double target_direction_angle)
{
  StableCandidate candidate;
  candidate.joint_positions = clampLinkJointPositions(raw_joint_positions);
  candidate.source = source;
  candidate.target_direction_angle = target_direction_angle;
  candidate.desired_distance = (candidate.joint_positions - desired_joint_positions).norm();
  candidate.measured_distance = (candidate.joint_positions - measured_joint_positions).norm();
  evaluateStability(candidate.joint_positions, candidate.metrics);
  candidate.violation_score = computeStabilityViolationScore(candidate.metrics);
  return candidate;
}

std::vector<CopilotPlanner::StableHistoryEntry> CopilotPlanner::getNearestStableHistoryEntries(
    const Eigen::VectorXd& desired_joint_positions,
    const Eigen::VectorXd& measured_joint_positions,
    const Eigen::Vector3d& current_target_direction) const
{
  if (stability_candidate_top_k_ <= 0 || stable_joint_history_.empty())
  {
    return std::vector<StableHistoryEntry>();
  }

  struct RankedHistoryEntry
  {
    StableHistoryEntry entry;
    double direction_angle = 0.0;
    double desired_distance = 0.0;
    double measured_distance = 0.0;
  };

  std::vector<RankedHistoryEntry> ranked_entries;
  ranked_entries.reserve(stable_joint_history_.size());
  for (const StableHistoryEntry& entry : stable_joint_history_)
  {
    if (entry.joint_positions.size() != link_joint_num_)
    {
      continue;
    }

    RankedHistoryEntry ranked_entry;
    ranked_entry.entry = entry;
    ranked_entry.direction_angle = computeTargetDirectionAngle(current_target_direction, entry.target_direction);
    ranked_entry.desired_distance = (entry.joint_positions - desired_joint_positions).norm();
    ranked_entry.measured_distance = (entry.joint_positions - measured_joint_positions).norm();
    ranked_entries.push_back(ranked_entry);
  }

  std::sort(ranked_entries.begin(), ranked_entries.end(),
            [](const RankedHistoryEntry& lhs, const RankedHistoryEntry& rhs) {
              if (lhs.direction_angle != rhs.direction_angle)
              {
                return lhs.direction_angle < rhs.direction_angle;
              }
              if (lhs.desired_distance != rhs.desired_distance)
              {
                return lhs.desired_distance < rhs.desired_distance;
              }
              if (lhs.measured_distance != rhs.measured_distance)
              {
                return lhs.measured_distance < rhs.measured_distance;
              }
              return lhs.entry.stamp > rhs.entry.stamp;
            });

  const size_t selected_count = std::min<size_t>(static_cast<size_t>(stability_candidate_top_k_), ranked_entries.size());
  std::vector<StableHistoryEntry> nearest_entries;
  nearest_entries.reserve(selected_count);
  for (size_t i = 0; i < selected_count; ++i)
  {
    nearest_entries.push_back(ranked_entries.at(i).entry);
  }

  return nearest_entries;
}

void CopilotPlanner::rememberStableJointPositions(const Eigen::VectorXd& stable_joint_positions,
                                                  StableCandidateSource source)
{
  const Eigen::VectorXd clamped_joint_positions = clampLinkJointPositions(stable_joint_positions);
  latest_stable_joint_positions_ = clamped_joint_positions;
  has_latest_stable_joint_positions_ = true;

  if (stability_candidate_history_size_ <= 0)
  {
    stable_joint_history_.clear();
    return;
  }

  StableHistoryEntry history_entry;
  history_entry.joint_positions = clamped_joint_positions;
  history_entry.target_direction = getCurrentTargetDirection();
  history_entry.stamp = ros::Time::now();
  history_entry.source = source;

  if (!stable_joint_history_.empty() &&
      areSimilarJointPositions(stable_joint_history_.back().joint_positions, clamped_joint_positions))
  {
    stable_joint_history_.back() = history_entry;
  }
  else
  {
    stable_joint_history_.push_back(history_entry);
  }

  while (stable_joint_history_.size() > static_cast<size_t>(stability_candidate_history_size_))
  {
    stable_joint_history_.pop_front();
  }
}

Eigen::Vector3d CopilotPlanner::getCurrentTargetDirection() const
{
  if (!latest_target_pose_)
  {
    return Eigen::Vector3d::UnitX();
  }

  tf::Quaternion root_quat;
  tf::quaternionMsgToTF(latest_target_pose_->pose.orientation, root_quat);
  const tf::Matrix3x3 root_rotation(root_quat);
  const tf::Vector3 link1_direction_tf = root_rotation * tf::Vector3(1.0, 0.0, 0.0);
  Eigen::Vector3d link1_direction(link1_direction_tf.x(), link1_direction_tf.y(), link1_direction_tf.z());

  if (link1_direction.norm() < kDirectionNormEpsilon)
  {
    return Eigen::Vector3d::UnitX();
  }

  return link1_direction.normalized();
}

double CopilotPlanner::computeTargetDirectionAngle(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) const
{
  if (lhs.norm() < kDirectionNormEpsilon || rhs.norm() < kDirectionNormEpsilon)
  {
    return 0.0;
  }

  return std::acos(clampUnit(lhs.normalized().dot(rhs.normalized())));
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
    case StableCandidateSource::kLatestStable:
      return "latest_stable";
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
