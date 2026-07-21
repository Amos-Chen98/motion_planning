// copilot_stability_qp.cpp
// Copilot wrapper around the shared stability projection library.

#include <multilink_copilot/copilot.h>

#include <kdl/frames.hpp>

namespace multilink_copilot
{

bool CopilotPlanner::solveStableJointQp(const Eigen::VectorXd& desired_joint_positions,
                                        const Eigen::VectorXd& reference_joint_positions,
                                        Eigen::VectorXd& stable_joint_positions,
                                        bool allow_unstable_seed)
{
  KDL::Rotation root_rotation = KDL::Rotation::Identity();
  if (latest_target_pose_)
  {
    const geometry_msgs::Pose root_pose = convertLink1TailPoseToRootPose(latest_target_pose_->pose);
    root_rotation = KDL::Rotation::Quaternion(root_pose.orientation.x, root_pose.orientation.y,
                                              root_pose.orientation.z, root_pose.orientation.w);
  }
  stability_evaluator_->setRootLinkRotation(root_rotation);
  return stability_evaluator_->projectToSafe(desired_joint_positions, reference_joint_positions,
                                              stable_joint_positions, allow_unstable_seed);
}

}  // namespace multilink_copilot
