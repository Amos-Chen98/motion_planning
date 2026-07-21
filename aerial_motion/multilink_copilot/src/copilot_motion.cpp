// copilot_motion.cpp
// Trajectory management, snake target generation, and visualization

#include <multilink_copilot/copilot.h>

#include <tf/transform_datatypes.h>

namespace multilink_copilot
{
namespace
{
constexpr double kDirectionNormEpsilon = 1e-6;
}  // namespace

void CopilotPlanner::updateTrajectoryBuffer(const Eigen::Vector3d& link1_tail_position,
                                           const Eigen::Vector3d& root_position)
{
  if (!trajectory_initialized_)
  {
    trajectory_buffer_.push_back({link1_tail_position});
    last_recorded_position_ = link1_tail_position;
    trajectory_initialized_ = true;
    link1_tail_pos_world_ = link1_tail_position;
    root_pos_world_ = root_position;
    ROS_INFO("[CopilotPlanner] Link1-tail trajectory initialized at position: [%.3f, %.3f, %.3f]",
             link1_tail_position.x(), link1_tail_position.y(), link1_tail_position.z());
    return;
  }

  const double distance = (link1_tail_position - last_recorded_position_).norm();

  if (distance >= trajectory_sample_interval_)
  {
    trajectory_buffer_.push_back({link1_tail_position});
    total_arc_length_ += distance;
    last_recorded_position_ = link1_tail_position;

    while (!trajectory_buffer_.empty() && total_arc_length_ > trajectory_buffer_max_length_)
    {
      if (trajectory_buffer_.size() <= 1)
      {
        break;
      }

      const double segment_length = (trajectory_buffer_[1].position - trajectory_buffer_[0].position).norm();
      total_arc_length_ -= segment_length;
      trajectory_buffer_.pop_front();
    }
  }

  link1_tail_pos_world_ = link1_tail_position;
  root_pos_world_ = root_position;
}

bool CopilotPlanner::prepareTrajectoryData()
{
  const double min_required_arc_length = (link_num_ - 1) * link_length_;
  if (total_arc_length_ < min_required_arc_length)
  {
    ROS_DEBUG_THROTTLE(2.0, "[CopilotPlanner] Waiting for sufficient trajectory: %.3f / %.3f m", total_arc_length_,
                       min_required_arc_length);
    return false;
  }

  return true;
}

std::vector<Eigen::Vector3d> CopilotPlanner::computeWarmupTargetPositions()
{
  if (trajectory_buffer_.empty() || !latest_target_pose_ ||
      !has_latest_measured_link_joint_positions_ ||
      latest_measured_link_joint_positions_.size() != link_joint_num_)
  {
    return {};
  }

  tf::Quaternion root_quat;
  tf::quaternionMsgToTF(latest_target_pose_->pose.orientation, root_quat);
  const tf::Matrix3x3 root_rotation_tf(root_quat);
  Eigen::Matrix3d root_rotation = Eigen::Matrix3d::Identity();
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      root_rotation(row, col) = root_rotation_tf[row][col];
    }
  }

  const std::deque<TrajectoryPoint> nominal_history =
      follow_the_leader::prependCurrentBodyMorphology(
          trajectory_buffer_, link1_tail_pos_world_, root_rotation,
          latest_measured_link_joint_positions_, pitch_joint_local_indices_,
          yaw_joint_local_indices_, link_num_, link_length_);
  return follow_the_leader::computeTargetPositions(nominal_history, link1_tail_pos_world_,
                                                   link_num_, link_length_);
}

std::vector<Eigen::Vector3d> CopilotPlanner::computeSnakeTargetPositions()
{
  if (trajectory_buffer_.size() < 2 || !latest_target_pose_)
  {
    return {};
  }
  return follow_the_leader::computeTargetPositions(trajectory_buffer_, link1_tail_pos_world_,
                                                   link_num_, link_length_);
}

Eigen::VectorXd CopilotPlanner::computeJointAnglesFromSnakeTarget(const std::vector<Eigen::Vector3d>& target_positions)
{
  tf::Quaternion root_quat;
  tf::quaternionMsgToTF(latest_target_pose_->pose.orientation, root_quat);
  const tf::Matrix3x3 root_rotation(root_quat);
  Eigen::Matrix3d root_rotation_eigen = Eigen::Matrix3d::Identity();
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      root_rotation_eigen(row, col) = root_rotation[row][col];
    }
  }
  Eigen::VectorXd reference = Eigen::VectorXd::Zero(link_joint_num_);
  for (int index = 0; index < link_joint_num_; ++index)
  {
    reference(index) = getReferenceJointPosition(index);
  }
  return follow_the_leader::computeJointAngles(target_positions, link1_tail_pos_world_, root_rotation_eigen,
                                               pitch_joint_local_indices_, yaw_joint_local_indices_, link_joint_num_,
                                               snake_ik_singularity_xz_norm_threshold_, reference);
}

Eigen::VectorXd CopilotPlanner::computeWarmupNominalJointPositions(
    const std::vector<Eigen::Vector3d>& target_positions)
{
  const Eigen::VectorXd measured_joint_positions =
      clampLinkJointPositions(latest_measured_link_joint_positions_);
  if (target_positions.empty())
  {
    return measured_joint_positions;
  }

  return clampLinkJointPositions(computeJointAnglesFromSnakeTarget(target_positions));
}

double CopilotPlanner::getReferenceJointPosition(int local_joint_index) const
{
  if (local_joint_index < 0 || local_joint_index >= link_joint_num_)
  {
    return 0.0;
  }

  if (has_latest_published_joint_positions_ && latest_published_joint_positions_.size() == link_joint_num_)
  {
    return latest_published_joint_positions_(local_joint_index);
  }

  if (has_latest_measured_link_joint_positions_ && latest_measured_link_joint_positions_.size() == link_joint_num_)
  {
    return latest_measured_link_joint_positions_(local_joint_index);
  }

  return 0.0;
}

geometry_msgs::Pose CopilotPlanner::convertLink1TailPoseToRootPose(const geometry_msgs::Pose& pose) const
{
  geometry_msgs::Pose root_pose = pose;

  tf::Quaternion root_quat;
  tf::quaternionMsgToTF(pose.orientation, root_quat);
  const tf::Matrix3x3 root_rotation(root_quat);
  const tf::Vector3 link1_direction_tf = root_rotation * tf::Vector3(1.0, 0.0, 0.0);
  Eigen::Vector3d link1_direction(link1_direction_tf.x(), link1_direction_tf.y(), link1_direction_tf.z());

  if (link1_direction.norm() < kDirectionNormEpsilon)
  {
    link1_direction = Eigen::Vector3d::UnitX();
  }
  else
  {
    link1_direction.normalize();
  }

  root_pose.position.x -= link_length_ * link1_direction.x();
  root_pose.position.y -= link_length_ * link1_direction.y();
  root_pose.position.z -= link_length_ * link1_direction.z();
  return root_pose;
}

Eigen::Vector3d CopilotPlanner::getLink1TailPositionFromPose(const geometry_msgs::Pose& pose) const
{
  return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
}

Eigen::Vector3d CopilotPlanner::getRootPositionFromLink1TailPose(const geometry_msgs::Pose& pose) const
{
  const geometry_msgs::Pose root_pose = convertLink1TailPoseToRootPose(pose);
  return Eigen::Vector3d(root_pose.position.x, root_pose.position.y, root_pose.position.z);
}

visualization_msgs::MarkerArray CopilotPlanner::getTrajectoryVisualization()
{
  visualization_msgs::MarkerArray marker_array;
  if (!trajectory_initialized_)
  {
    return marker_array;
  }

  const ros::Time current_time = ros::Time::now();

  visualization_msgs::Marker trajectory_line;
  trajectory_line.header.frame_id = "world";
  trajectory_line.header.stamp = current_time;
  trajectory_line.ns = "link1_tail_trajectory";
  trajectory_line.id = 0;
  trajectory_line.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_line.action = visualization_msgs::Marker::ADD;
  trajectory_line.pose.orientation.w = 1.0;
  trajectory_line.scale.x = 0.03;
  trajectory_line.color.r = 1.0;
  trajectory_line.color.g = 0.5;
  trajectory_line.color.b = 0.0;
  trajectory_line.color.a = 0.9;
  trajectory_line.lifetime = ros::Duration(0.0);

  for (const auto& point : trajectory_buffer_)
  {
    geometry_msgs::Point marker_point;
    marker_point.x = point.position.x();
    marker_point.y = point.position.y();
    marker_point.z = point.position.z();
    trajectory_line.points.push_back(marker_point);
  }

  if (!trajectory_line.points.empty())
  {
    marker_array.markers.push_back(trajectory_line);
  }

  visualization_msgs::Marker root_marker;
  root_marker.header.frame_id = "world";
  root_marker.header.stamp = current_time;
  root_marker.ns = "root_position";
  root_marker.id = 1;
  root_marker.type = visualization_msgs::Marker::SPHERE;
  root_marker.action = visualization_msgs::Marker::ADD;
  root_marker.pose.position.x = root_pos_world_.x();
  root_marker.pose.position.y = root_pos_world_.y();
  root_marker.pose.position.z = root_pos_world_.z();
  root_marker.pose.orientation.w = 1.0;
  root_marker.scale.x = 0.1;
  root_marker.scale.y = 0.1;
  root_marker.scale.z = 0.1;
  root_marker.color.r = 1.0;
  root_marker.color.g = 0.0;
  root_marker.color.b = 0.0;
  root_marker.color.a = 1.0;
  root_marker.lifetime = ros::Duration(0.1);
  marker_array.markers.push_back(root_marker);

  visualization_msgs::Marker link1_tail_marker;
  link1_tail_marker.header.frame_id = "world";
  link1_tail_marker.header.stamp = current_time;
  link1_tail_marker.ns = "link1_tail_position";
  link1_tail_marker.id = 2;
  link1_tail_marker.type = visualization_msgs::Marker::SPHERE;
  link1_tail_marker.action = visualization_msgs::Marker::ADD;
  link1_tail_marker.pose.position.x = link1_tail_pos_world_.x();
  link1_tail_marker.pose.position.y = link1_tail_pos_world_.y();
  link1_tail_marker.pose.position.z = link1_tail_pos_world_.z();
  link1_tail_marker.pose.orientation.w = 1.0;
  link1_tail_marker.scale.x = 0.08;
  link1_tail_marker.scale.y = 0.08;
  link1_tail_marker.scale.z = 0.08;
  link1_tail_marker.color.r = 0.0;
  link1_tail_marker.color.g = 0.4;
  link1_tail_marker.color.b = 1.0;
  link1_tail_marker.color.a = 1.0;
  link1_tail_marker.lifetime = ros::Duration(0.1);
  marker_array.markers.push_back(link1_tail_marker);

  for (size_t i = 0; i < trajectory_buffer_.size(); ++i)
  {
    visualization_msgs::Marker point_marker;
    point_marker.header.frame_id = "world";
    point_marker.header.stamp = current_time;
    point_marker.ns = "link1_tail_trajectory_points";
    point_marker.id = 100 + i;
    point_marker.type = visualization_msgs::Marker::SPHERE;
    point_marker.action = visualization_msgs::Marker::ADD;
    point_marker.pose.position.x = trajectory_buffer_[i].position.x();
    point_marker.pose.position.y = trajectory_buffer_[i].position.y();
    point_marker.pose.position.z = trajectory_buffer_[i].position.z();
    point_marker.pose.orientation.w = 1.0;
    point_marker.scale.x = 0.04;
    point_marker.scale.y = 0.04;
    point_marker.scale.z = 0.04;
    point_marker.color.r = 0.0;
    point_marker.color.g = 1.0;
    point_marker.color.b = 1.0;
    point_marker.color.a = 0.7;
    point_marker.lifetime = ros::Duration(0.0);
    marker_array.markers.push_back(point_marker);
  }

  for (size_t i = 0; i < latest_snake_targets_.size(); ++i)
  {
    visualization_msgs::Marker target_marker;
    target_marker.header.frame_id = "world";
    target_marker.header.stamp = current_time;
    target_marker.ns = "snake_targets";
    target_marker.id = 200 + i;
    target_marker.type = visualization_msgs::Marker::SPHERE;
    target_marker.action = visualization_msgs::Marker::ADD;
    target_marker.pose.position.x = latest_snake_targets_[i].x();
    target_marker.pose.position.y = latest_snake_targets_[i].y();
    target_marker.pose.position.z = latest_snake_targets_[i].z();
    target_marker.pose.orientation.w = 1.0;
    target_marker.scale.x = 0.08;
    target_marker.scale.y = 0.08;
    target_marker.scale.z = 0.08;
    target_marker.color.r = 1.0;
    target_marker.color.g = 0.0;
    target_marker.color.b = 1.0;
    target_marker.color.a = 0.9;
    target_marker.lifetime = ros::Duration(0.1);
    marker_array.markers.push_back(target_marker);
  }

  return marker_array;
}

}  // namespace multilink_copilot
