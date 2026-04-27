// copilot_stability.cpp
// Stability evaluation, reference selection, and joint-space helpers

#include <multilink_copilot/copilot.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace multilink_copilot
{
namespace
{
std::string formatJointPositions(const KDL::JntArray& joint_positions)
{
  std::ostringstream stream;
  stream << "[";
  for (unsigned int i = 0; i < joint_positions.rows(); ++i)
  {
    if (i > 0)
    {
      stream << ' ';
    }
    stream << joint_positions(i);
  }
  stream << "]";
  return stream.str();
}

std::string formatPose(const geometry_msgs::Pose& pose)
{
  std::ostringstream stream;
  stream << "pos: [" << pose.position.x << ' ' << pose.position.y << ' ' << pose.position.z
         << "], quat(xyzw): [" << pose.orientation.x << ' ' << pose.orientation.y << ' ' << pose.orientation.z
         << ' ' << pose.orientation.w << "]";
  return stream.str();
}

std::string formatTargetPose(const geometry_msgs::PoseStamped::ConstPtr& target_pose)
{
  if (!target_pose)
  {
    return "unavailable";
  }

  return formatPose(target_pose->pose);
}

}  // namespace

bool CopilotPlanner::computeStableJointPositions(const Eigen::VectorXd& nominal_joint_positions,
                                                 Eigen::VectorXd& stable_joint_positions)
{
  const Eigen::VectorXd precheck_joint_positions = clampLinkJointPositions(getCurrentLinkJointPositions());
  const Eigen::VectorXd desired_joint_positions = clampLinkJointPositions(nominal_joint_positions);

  if (checkStability(desired_joint_positions, false))
  {
    stable_joint_positions = desired_joint_positions;
    last_stable_joint_positions_ = stable_joint_positions;
    has_last_stable_joint_positions_ = true;
    restoreRobotModelToLinkJointPositions(stable_joint_positions);
    return true;
  }

  Eigen::VectorXd stable_reference;
  if (!tryGetStableReferenceJointPositions(stable_reference))
  {
    restoreRobotModelToLinkJointPositions(precheck_joint_positions);
    return false;
  }

  if (!solveStableJointQp(desired_joint_positions, stable_reference, stable_joint_positions))
  {
    stable_joint_positions = stable_reference;
  }

  if (!checkStability(stable_joint_positions, false))
  {
    if (!checkStability(stable_reference, false))
    {
      restoreRobotModelToLinkJointPositions(precheck_joint_positions);
      return false;
    }

    stable_joint_positions = stable_reference;
  }

  stable_joint_positions = clampLinkJointPositions(stable_joint_positions);
  restoreRobotModelToLinkJointPositions(stable_joint_positions);
  last_stable_joint_positions_ = stable_joint_positions;
  has_last_stable_joint_positions_ = true;

  if ((desired_joint_positions - stable_joint_positions).norm() > stability_qp_convergence_tol_)
  {
    ROS_WARN_STREAM_THROTTLE(0.5,
                             "[CopilotPlanner] Solved a conservative stable joint target"
                                 << ", desired: [" << desired_joint_positions.transpose() << "]"
                                 << ", stable: [" << stable_joint_positions.transpose() << "]");
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
  const KDL::JntArray kdl_joint_positions = buildUpdatedJointPositions(clamped_joint_positions);

  updateRobotModelForTargetConfiguration(kdl_joint_positions);
  dragon_robot_model_->updateJacobians(kdl_joint_positions, false);

  metrics.raw_model_stable = dragon_robot_model_->stabilityCheck(false);
  metrics.fc_rp_min = dragon_robot_model_->getFeasibleControlRollPitchMin();
  metrics.fc_t_min = dragon_robot_model_->getFeasibleControlTMin();

  const Eigen::VectorXd& static_thrust = dragon_robot_model_->getStaticThrust();
  if (static_thrust.size() > 0)
  {
    metrics.static_thrust_min = static_thrust.minCoeff();
    metrics.static_thrust_max = static_thrust.maxCoeff();
  }
  else
  {
    metrics.static_thrust_min = 0.0;
    metrics.static_thrust_max = 0.0;
  }

  metrics.overlap_clearance =
      dragon_robot_model_->getClosestRotorDist() - 2.0 * dragon_robot_model_->getEdfRadius();
  metrics.safe = satisfiesSafeStability(metrics);
  return true;
}

bool CopilotPlanner::satisfiesSafeStability(const StabilityMetrics& metrics) const
{
  if (metrics.fc_rp_min + stability_qp_feasibility_tol_ < stability_fc_rp_min_thre_)
  {
    return false;
  }

  if (stability_check_fc_t_ && metrics.fc_t_min + stability_qp_feasibility_tol_ < stability_fc_t_min_thre_)
  {
    return false;
  }

  if (metrics.static_thrust_min + stability_qp_feasibility_tol_ < stability_static_thrust_min_)
  {
    return false;
  }

  if (metrics.static_thrust_max - stability_qp_feasibility_tol_ > stability_static_thrust_max_)
  {
    return false;
  }

  if (metrics.overlap_clearance + stability_qp_feasibility_tol_ < stability_overlap_min_clearance_)
  {
    return false;
  }

  return true;
}

bool CopilotPlanner::checkStability(const Eigen::VectorXd& joint_positions, bool report_result)
{
  const Eigen::VectorXd clamped_joint_positions = clampLinkJointPositions(joint_positions);
  StabilityMetrics metrics;
  evaluateStability(clamped_joint_positions, metrics);

  const bool is_stable = metrics.safe;
  if (!is_stable && report_result)
  {
    const KDL::JntArray kdl_joint_positions = buildUpdatedJointPositions(clamped_joint_positions);
    const std::string root_pose_string =
        latest_target_pose_ ? formatPose(convertLink1TailPoseToRootPose(latest_target_pose_->pose)) : "unavailable";
    ROS_WARN_STREAM_THROTTLE(
        0.5, "[CopilotPlanner] Conservative stability check failed"
                 << ", root_pose: " << root_pose_string
                 << ", link1_tail_pose: " << formatTargetPose(latest_target_pose_)
                 << ", raw_model_stable: " << (metrics.raw_model_stable ? "true" : "false")
                 << ", link_joint_positions: [" << clamped_joint_positions.transpose() << "]"
                 << ", full_joint_positions: " << formatJointPositions(kdl_joint_positions)
                 << ", fc_rp_min/thre: " << metrics.fc_rp_min << "/" << stability_fc_rp_min_thre_
                 << ", static_thrust[min,max]/safe_range: [" << metrics.static_thrust_min << ", "
                 << metrics.static_thrust_max << "]/[" << stability_static_thrust_min_ << ", "
                 << stability_static_thrust_max_ << "]"
                 << ", overlap_clearance/thre: " << metrics.overlap_clearance << "/"
                 << stability_overlap_min_clearance_
                 << ", fc_t_min/thre: " << metrics.fc_t_min << "/" << stability_fc_t_min_thre_);

    if (!metrics.raw_model_stable)
    {
      dragon_robot_model_->stabilityCheck(true);
    }
  }

  if (is_stable && report_result)
  {
    ROS_INFO_STREAM("[CopilotPlanner] Target configuration is conservatively stable"
                    << " (fc_rp_min: " << metrics.fc_rp_min << ", thrust range: ["
                    << metrics.static_thrust_min << ", " << metrics.static_thrust_max
                    << "], overlap clearance: " << metrics.overlap_clearance << ")");
  }
  else if (report_result)
  {
    ROS_WARN_STREAM_THROTTLE(0.5,
                             "[CopilotPlanner] Target configuration is conservatively unstable"
                                 << ", joint_positions: [" << clamped_joint_positions.transpose() << "]");
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

Eigen::VectorXd CopilotPlanner::buildFoldedReferenceJointPositions() const
{
  Eigen::VectorXd folded_joint_positions = Eigen::VectorXd::Zero(link_joint_num_);
  for (const int local_index : yaw_joint_local_indices_)
  {
    if (local_index >= 0 && local_index < link_joint_num_)
    {
      folded_joint_positions(local_index) = M_PI / 2.0;
    }
  }

  return clampLinkJointPositions(folded_joint_positions);
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

bool CopilotPlanner::tryGetStableReferenceJointPositions(Eigen::VectorXd& stable_reference)
{
  std::vector<Eigen::VectorXd> reference_candidates;
  reference_candidates.reserve(6);

  reference_candidates.push_back(getCurrentLinkJointPositions());

  if (has_last_stable_joint_positions_ && last_stable_joint_positions_.size() == link_joint_num_)
  {
    reference_candidates.push_back(last_stable_joint_positions_);
  }

  if (has_latest_desired_joint_positions_ && latest_desired_joint_positions_.size() == link_joint_num_)
  {
    reference_candidates.push_back(latest_desired_joint_positions_);
  }

  reference_candidates.push_back(Eigen::VectorXd::Zero(link_joint_num_));
  reference_candidates.push_back(buildFoldedReferenceJointPositions());
  reference_candidates.push_back(buildDefaultReferenceJointPositions());

  for (const Eigen::VectorXd& raw_candidate : reference_candidates)
  {
    const Eigen::VectorXd candidate = clampLinkJointPositions(raw_candidate);
    if (checkStability(candidate, false))
    {
      stable_reference = candidate;
      last_stable_joint_positions_ = stable_reference;
      has_last_stable_joint_positions_ = true;
      restoreRobotModelToLinkJointPositions(stable_reference);
      return true;
    }
  }

  ROS_ERROR_THROTTLE(1.0, "[CopilotPlanner] Failed to find any conservative stable reference configuration");
  return false;
}

void CopilotPlanner::restoreRobotModelToLinkJointPositions(const Eigen::VectorXd& joint_positions)
{
  updateRobotModelForTargetConfiguration(buildUpdatedJointPositions(clampLinkJointPositions(joint_positions)));
}

}  // namespace multilink_copilot
