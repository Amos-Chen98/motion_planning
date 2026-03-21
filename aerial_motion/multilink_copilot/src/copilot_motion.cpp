// copilot_motion.cpp
// Trajectory management, snake target generation, and visualization

#include <multilink_copilot/copilot.h>

#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace multilink_copilot
{
namespace
{
constexpr double kDirectionNormEpsilon = 1e-6;
constexpr double kWarmupDistanceTolerance = 0.02;

double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

double smoothstep01(double value)
{
  value = clamp01(value);
  return value * value * (3.0 - 2.0 * value);
}

Eigen::Vector3d getRootLinkDirection(const geometry_msgs::PoseStamped::ConstPtr& target_pose)
{
  if (!target_pose)
  {
    return Eigen::Vector3d::UnitX();
  }

  tf::Quaternion root_quat;
  tf::quaternionMsgToTF(target_pose->pose.orientation, root_quat);
  const tf::Matrix3x3 root_rotation(root_quat);
  const tf::Vector3 link1_direction_tf = root_rotation * tf::Vector3(1.0, 0.0, 0.0);
  Eigen::Vector3d link1_direction(link1_direction_tf.x(), link1_direction_tf.y(), link1_direction_tf.z());

  if (link1_direction.norm() < kDirectionNormEpsilon)
  {
    return Eigen::Vector3d::UnitX();
  }

  return link1_direction.normalized();
}

Eigen::Vector3d computeOldestBackwardDirection(const std::deque<TrajectoryPoint>& trajectory_buffer,
                                               const Eigen::Vector3d& fallback_direction)
{
  for (size_t i = 0; i + 1 < trajectory_buffer.size(); ++i)
  {
    const Eigen::Vector3d segment = trajectory_buffer[i + 1].position - trajectory_buffer[i].position;
    if (segment.norm() >= kDirectionNormEpsilon)
    {
      return -segment.normalized();
    }
  }

  if (fallback_direction.norm() >= kDirectionNormEpsilon)
  {
    return -fallback_direction.normalized();
  }

  return Eigen::Vector3d(-1.0, 0.0, 0.0);
}

Eigen::Vector3d extrapolatePointAlongRayAtDistance(const Eigen::Vector3d& from_point,
                                                   const Eigen::Vector3d& ray_origin,
                                                   const Eigen::Vector3d& ray_direction,
                                                   double target_distance)
{
  Eigen::Vector3d direction = ray_direction;
  if (direction.norm() < kDirectionNormEpsilon)
  {
    direction = Eigen::Vector3d(-1.0, 0.0, 0.0);
  }
  else
  {
    direction.normalize();
  }

  const Eigen::Vector3d delta = ray_origin - from_point;
  const double b = delta.dot(direction);
  const double c = delta.squaredNorm() - target_distance * target_distance;
  const double discriminant = b * b - c;

  if (discriminant >= 0.0)
  {
    const double sqrt_discriminant = std::sqrt(discriminant);
    const double lambda_candidates[2] = {-b - sqrt_discriminant, -b + sqrt_discriminant};
    double best_lambda = std::numeric_limits<double>::infinity();

    for (double lambda : lambda_candidates)
    {
      if (lambda >= 0.0 && lambda < best_lambda)
      {
        best_lambda = lambda;
      }
    }

    if (std::isfinite(best_lambda))
    {
      return ray_origin + best_lambda * direction;
    }
  }

  const double closest_lambda = std::max(0.0, -b);
  return ray_origin + closest_lambda * direction;
}
}  // namespace

void CopilotPlanner::updateTrajectoryBuffer(const Eigen::Vector3d& current_position)
{
  if (!trajectory_initialized_)
  {
    trajectory_buffer_.push_back({current_position});
    last_recorded_position_ = current_position;
    trajectory_initialized_ = true;
    root_pos_world_ = current_position;
    ROS_INFO("[CopilotPlanner] Trajectory initialized at position: [%.3f, %.3f, %.3f]", current_position.x(),
             current_position.y(), current_position.z());
    return;
  }

  const double distance = (current_position - last_recorded_position_).norm();

  if (distance >= trajectory_sample_interval_)
  {
    trajectory_buffer_.push_back({current_position});
    total_arc_length_ += distance;
    last_recorded_position_ = current_position;

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

  root_pos_world_ = current_position;
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
  std::vector<Eigen::Vector3d> target_positions;
  if (link_num_ > 1)
  {
    target_positions.reserve(static_cast<size_t>(link_num_ - 1));
  }

  if (trajectory_buffer_.size() < 2 || !latest_target_pose_)
  {
    return target_positions;
  }

  const Eigen::Vector3d current_link_direction = getRootLinkDirection(latest_target_pose_);
  const Eigen::Vector3d oldest_backward_direction =
      computeOldestBackwardDirection(trajectory_buffer_, current_link_direction);
  Eigen::Vector3d current_head = root_pos_world_ + current_link_direction * link_length_;

  bool tail_extension_started = false;
  for (int i = 2; i <= link_num_; ++i)
  {
    Eigen::Vector3d target = current_head;
    if (!tail_extension_started)
    {
      const Eigen::Vector3d trajectory_target = findPointOnTrajectoryAtDistance(current_head, link_length_);
      const double distance_error = std::abs((trajectory_target - current_head).norm() - link_length_);

      if (distance_error <= kWarmupDistanceTolerance)
      {
        target = trajectory_target;
      }
      else
      {
        tail_extension_started = true;
        target = extrapolatePointAlongRayAtDistance(current_head, trajectory_buffer_.front().position,
                                                    oldest_backward_direction, link_length_);
      }
    }
    else
    {
      target = current_head + oldest_backward_direction * link_length_;
    }

    target_positions.push_back(target);
    current_head = target;
  }

  return target_positions;
}

std::vector<Eigen::Vector3d> CopilotPlanner::computeSnakeTargetPositions()
{
  std::vector<Eigen::Vector3d> target_positions;
  if (link_num_ > 1)
  {
    target_positions.reserve(static_cast<size_t>(link_num_ - 1));
  }

  if (trajectory_buffer_.size() < 2 || !latest_target_pose_)
  {
    return target_positions;
  }

  tf::Quaternion root_quat;
  tf::quaternionMsgToTF(latest_target_pose_->pose.orientation, root_quat);
  const tf::Matrix3x3 root_rotation(root_quat);
  const tf::Vector3 link1_direction_tf = root_rotation * tf::Vector3(1.0, 0.0, 0.0);
  const Eigen::Vector3d link1_direction(link1_direction_tf.x(), link1_direction_tf.y(), link1_direction_tf.z());
  const Eigen::Vector3d link1_tail = root_pos_world_ + link1_direction * link_length_;

  Eigen::Vector3d current_head = link1_tail;
  for (int i = 2; i <= link_num_; ++i)
  {
    const Eigen::Vector3d target = findPointOnTrajectoryAtDistance(current_head, link_length_);
    target_positions.push_back(target);
    current_head = target;
  }

  return target_positions;
}

Eigen::Vector3d CopilotPlanner::findPointOnTrajectoryAtDistance(const Eigen::Vector3d& from_point,
                                                                double target_distance)
{
  if (trajectory_buffer_.size() < 2)
  {
    return from_point;
  }

  int start_segment = 0;
  double min_dist_to_trajectory = std::numeric_limits<double>::max();

  for (size_t i = 0; i + 1 < trajectory_buffer_.size(); ++i)
  {
    const Eigen::Vector3d& p1 = trajectory_buffer_[i].position;
    const Eigen::Vector3d& p2 = trajectory_buffer_[i + 1].position;
    const Eigen::Vector3d segment = p2 - p1;
    const double segment_length_sq = segment.squaredNorm();

    double projection = 0.0;
    if (segment_length_sq > 1e-10)
    {
      projection = std::max(0.0, std::min(1.0, (from_point - p1).dot(segment) / segment_length_sq));
    }

    const Eigen::Vector3d closest = p1 + projection * segment;
    const double distance = (from_point - closest).norm();
    if (distance < min_dist_to_trajectory)
    {
      min_dist_to_trajectory = distance;
      start_segment = static_cast<int>(i);
    }
  }

  Eigen::Vector3d best_point = trajectory_buffer_.front().position;
  double best_distance_error = std::numeric_limits<double>::max();

  for (int i = start_segment; i >= 0; --i)
  {
    const Eigen::Vector3d& p1 = trajectory_buffer_[i].position;
    const Eigen::Vector3d& p2 = trajectory_buffer_[i + 1].position;
    const Eigen::Vector3d segment = p2 - p1;
    const double segment_length = segment.norm();

    if (segment_length < 1e-6)
    {
      continue;
    }

    for (double alpha = 0.0; alpha <= 1.0; alpha += 0.1)
    {
      const Eigen::Vector3d candidate = p1 + alpha * segment;
      const double error = std::abs((from_point - candidate).norm() - target_distance);
      if (error < best_distance_error)
      {
        best_distance_error = error;
        best_point = candidate;
      }
    }

    if (best_distance_error < 0.01)
    {
      return best_point;
    }
  }

  return best_point;
}

Eigen::VectorXd CopilotPlanner::computeJointAnglesFromSnakeTarget(const std::vector<Eigen::Vector3d>& target_positions)
{
  Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(link_joint_num_);

  tf::Quaternion root_quat;
  tf::quaternionMsgToTF(latest_target_pose_->pose.orientation, root_quat);
  const tf::Matrix3x3 root_rotation(root_quat);
  const tf::Vector3 link1_direction_tf = root_rotation * tf::Vector3(1.0, 0.0, 0.0);
  Eigen::Vector3d current_link_direction(link1_direction_tf.x(), link1_direction_tf.y(), link1_direction_tf.z());
  Eigen::Vector3d current_tail_position = root_pos_world_ + current_link_direction * link_length_;

  for (size_t i = 0; i < target_positions.size(); ++i)
  {
    const Eigen::Vector3d segment = target_positions[i] - current_tail_position;
    if (segment.norm() < kDirectionNormEpsilon)
    {
      continue;
    }

    const Eigen::Vector3d desired_next_direction = segment.normalized();
    Eigen::Vector2d current_dir_xy(current_link_direction.x(), current_link_direction.y());
    Eigen::Vector2d desired_dir_xy(desired_next_direction.x(), desired_next_direction.y());

    const double current_dir_xy_norm = current_dir_xy.norm();
    const double desired_dir_xy_norm = desired_dir_xy.norm();
    if (current_dir_xy_norm < kDirectionNormEpsilon || desired_dir_xy_norm < kDirectionNormEpsilon)
    {
      current_link_direction = desired_next_direction;
      current_tail_position = target_positions[i];
      continue;
    }

    current_dir_xy /= current_dir_xy_norm;
    desired_dir_xy /= desired_dir_xy_norm;

    double joint_yaw = std::atan2(desired_dir_xy.y(), desired_dir_xy.x()) -
                       std::atan2(current_dir_xy.y(), current_dir_xy.x());
    while (joint_yaw > M_PI)
    {
      joint_yaw -= 2.0 * M_PI;
    }
    while (joint_yaw < -M_PI)
    {
      joint_yaw += 2.0 * M_PI;
    }

    const int yaw_joint_index = static_cast<int>(i) * 2 + 1;
    if (yaw_joint_index < link_joint_num_)
    {
      joint_positions(yaw_joint_index) = joint_yaw;
    }

    current_link_direction = desired_next_direction;
    current_tail_position = target_positions[i];
  }

  return joint_positions;
}

Eigen::VectorXd CopilotPlanner::computeWarmupJointPositions(const std::vector<Eigen::Vector3d>& target_positions)
{
  Eigen::VectorXd desired_joint_positions = buildFoldedReferenceJointPositions();
  if (target_positions.empty())
  {
    return desired_joint_positions;
  }

  const Eigen::VectorXd path_joint_positions = computeJointAnglesFromSnakeTarget(target_positions);
  const double activation_distance = std::max(link_length_, kDirectionNormEpsilon);

  for (size_t i = 0; i < target_positions.size(); ++i)
  {
    const int yaw_joint_index = static_cast<int>(i) * 2 + 1;
    if (yaw_joint_index >= link_joint_num_)
    {
      break;
    }

    const double joint_progress = (total_arc_length_ - static_cast<double>(i) * link_length_) / activation_distance;
    const double activation = smoothstep01(joint_progress);
    desired_joint_positions(yaw_joint_index) +=
        activation * (path_joint_positions(yaw_joint_index) - desired_joint_positions(yaw_joint_index));
  }

  return clampLinkJointPositions(desired_joint_positions);
}

Eigen::Vector3d CopilotPlanner::getRootPositionFromPose(const geometry_msgs::Pose& pose)
{
  return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
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
  trajectory_line.ns = "root_trajectory";
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

  for (size_t i = 0; i < trajectory_buffer_.size(); ++i)
  {
    visualization_msgs::Marker point_marker;
    point_marker.header.frame_id = "world";
    point_marker.header.stamp = current_time;
    point_marker.ns = "trajectory_points";
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
