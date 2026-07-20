// -*- mode: c++ -*-
#ifndef MULTILINK_COPILOT_FOLLOW_THE_LEADER_H
#define MULTILINK_COPILOT_FOLLOW_THE_LEADER_H

#include <Eigen/Dense>

#include <deque>
#include <vector>

namespace multilink_copilot
{

struct TrajectoryPoint
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
};

namespace follow_the_leader
{

std::vector<Eigen::Vector3d> computeTargetPositions(
    const std::deque<TrajectoryPoint>& trajectory_buffer,
    const Eigen::Vector3d& current_position,
    const Eigen::Vector3d& fallback_link_direction,
    int link_num,
    double link_length,
    bool extend_short_history);

Eigen::VectorXd computeJointAngles(
    const std::vector<Eigen::Vector3d>& target_positions,
    const Eigen::Vector3d& link1_tail_position,
    const Eigen::Matrix3d& root_link_rotation,
    const std::vector<int>& pitch_joint_indices,
    const std::vector<int>& yaw_joint_indices,
    int joint_num,
    double singularity_xz_norm_threshold,
    const Eigen::VectorXd& reference_joint_positions);

Eigen::VectorXd computeWarmupJointPositions(
    const Eigen::VectorXd& current_joint_positions,
    const Eigen::VectorXd& path_joint_positions,
    double total_arc_length,
    double link_length,
    const std::vector<int>& pitch_joint_indices,
    const std::vector<int>& yaw_joint_indices);

Eigen::Matrix3d rotationAroundY(double angle);
Eigen::Matrix3d rotationAroundZ(double angle);

}  // namespace follow_the_leader
}  // namespace multilink_copilot

#endif  // MULTILINK_COPILOT_FOLLOW_THE_LEADER_H
