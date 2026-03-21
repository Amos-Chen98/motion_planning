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
constexpr double kSearchParameterEpsilon = 1e-9;

struct TrajectorySearchCursor
{
  int segment_index = -1;  // Segment [segment_index, segment_index + 1] on an oldest-to-newest polyline.
  double alpha = 0.0;      // Interpolation factor on the segment: 0 -> older endpoint, 1 -> newer endpoint.
  Eigen::Vector3d position = Eigen::Vector3d::Zero();

  bool valid() const
  {
    return segment_index >= 0;
  }
};

struct TrajectorySearchResult
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  TrajectorySearchCursor cursor;
  double distance_error = std::numeric_limits<double>::infinity();
  bool exact_match = false;
};

double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

double smoothstep01(double value)
{
  value = clamp01(value);
  return value * value * (3.0 - 2.0 * value);
}

Eigen::Vector3d interpolateSegmentPoint(const std::vector<Eigen::Vector3d>& polyline, int segment_index, double alpha)
{
  return polyline[segment_index] + alpha * (polyline[segment_index + 1] - polyline[segment_index]);
}

std::vector<Eigen::Vector3d> buildTrajectorySearchPolyline(const std::deque<TrajectoryPoint>& trajectory_buffer,
                                                           const Eigen::Vector3d& current_position)
{
  std::vector<Eigen::Vector3d> polyline;
  polyline.reserve(trajectory_buffer.size() + 1);
  for (const auto& point : trajectory_buffer)
  {
    polyline.push_back(point.position);
  }

  if (polyline.empty() || (polyline.back() - current_position).norm() > kDirectionNormEpsilon)
  {
    polyline.push_back(current_position);
  }

  return polyline;
}

TrajectorySearchCursor makeNewestTrajectoryCursor(const std::vector<Eigen::Vector3d>& polyline)
{
  TrajectorySearchCursor cursor;
  if (polyline.size() < 2)
  {
    return cursor;
  }

  cursor.segment_index = static_cast<int>(polyline.size()) - 2;
  cursor.alpha = 1.0;
  cursor.position = polyline.back();
  return cursor;
}

TrajectorySearchCursor normalizeCursorForBackwardTraversal(const std::vector<Eigen::Vector3d>& polyline,
                                                           const TrajectorySearchCursor& raw_cursor)
{
  TrajectorySearchCursor cursor = raw_cursor;
  if (polyline.size() < 2 || cursor.segment_index < 0 || cursor.segment_index + 1 >= static_cast<int>(polyline.size()))
  {
    cursor.segment_index = -1;
    return cursor;
  }

  cursor.alpha = clamp01(cursor.alpha);
  cursor.position = interpolateSegmentPoint(polyline, cursor.segment_index, cursor.alpha);

  if (cursor.alpha <= kSearchParameterEpsilon && cursor.segment_index > 0)
  {
    --cursor.segment_index;
    cursor.alpha = 1.0;
    cursor.position = polyline[cursor.segment_index + 1];
  }

  return cursor;
}

void updateBestSearchResult(const Eigen::Vector3d& from_point,
                            double target_distance,
                            const Eigen::Vector3d& segment_start,
                            const Eigen::Vector3d& segment_delta,
                            int segment_index,
                            double alpha_start,
                            double u,
                            TrajectorySearchResult& best_result)
{
  const double clamped_u = clamp01(u);
  const Eigen::Vector3d candidate = segment_start + clamped_u * segment_delta;
  const double distance_error = std::abs((candidate - from_point).norm() - target_distance);
  if (distance_error >= best_result.distance_error)
  {
    return;
  }

  best_result.position = candidate;
  best_result.cursor.segment_index = segment_index;
  best_result.cursor.alpha = alpha_start * (1.0 - clamped_u);
  best_result.cursor.position = candidate;
  best_result.distance_error = distance_error;
  best_result.exact_match = false;
}

TrajectorySearchResult findPointOnTrajectoryAtDistance(const std::vector<Eigen::Vector3d>& polyline,
                                                       const TrajectorySearchCursor& start_cursor,
                                                       const Eigen::Vector3d& from_point,
                                                       double target_distance)
{
  TrajectorySearchResult best_result;
  best_result.position = polyline.front();
  best_result.cursor.segment_index = 0;
  best_result.cursor.alpha = 0.0;
  best_result.cursor.position = polyline.front();
  best_result.distance_error = std::abs((polyline.front() - from_point).norm() - target_distance);

  const TrajectorySearchCursor cursor = normalizeCursorForBackwardTraversal(polyline, start_cursor);
  if (!cursor.valid())
  {
    return best_result;
  }

  for (int segment_index = cursor.segment_index; segment_index >= 0; --segment_index)
  {
    const double alpha_start = (segment_index == cursor.segment_index) ? cursor.alpha : 1.0;
    const Eigen::Vector3d segment_start =
        (segment_index == cursor.segment_index) ? cursor.position : polyline[segment_index + 1];
    const Eigen::Vector3d segment_end = polyline[segment_index];
    const Eigen::Vector3d segment_delta = segment_end - segment_start;
    const double segment_length_sq = segment_delta.squaredNorm();

    if (segment_length_sq < kDirectionNormEpsilon * kDirectionNormEpsilon)
    {
      updateBestSearchResult(from_point, target_distance, segment_start, segment_delta, segment_index, alpha_start, 0.0,
                             best_result);
      continue;
    }

    const Eigen::Vector3d relative_start = segment_start - from_point;
    const double a = segment_length_sq;
    const double b = 2.0 * segment_delta.dot(relative_start);
    const double c = relative_start.squaredNorm() - target_distance * target_distance;
    const double discriminant = b * b - 4.0 * a * c;

    if (discriminant >= -kSearchParameterEpsilon)
    {
      const double sqrt_discriminant = std::sqrt(std::max(0.0, discriminant));
      const double inv_denominator = 0.5 / a;
      const double u_candidates[2] = {(-b - sqrt_discriminant) * inv_denominator,
                                      (-b + sqrt_discriminant) * inv_denominator};
      double best_u = std::numeric_limits<double>::infinity();

      for (double u : u_candidates)
      {
        if (u >= -kSearchParameterEpsilon && u <= 1.0 + kSearchParameterEpsilon && u < best_u)
        {
          best_u = clamp01(u);
        }
      }

      if (std::isfinite(best_u))
      {
        TrajectorySearchResult result;
        result.position = segment_start + best_u * segment_delta;
        result.cursor.segment_index = segment_index;
        result.cursor.alpha = alpha_start * (1.0 - best_u);
        result.cursor.position = result.position;
        result.cursor = normalizeCursorForBackwardTraversal(polyline, result.cursor);
        result.distance_error = std::abs((result.position - from_point).norm() - target_distance);
        result.exact_match = true;
        return result;
      }
    }

    updateBestSearchResult(from_point, target_distance, segment_start, segment_delta, segment_index, alpha_start, 0.0,
                           best_result);
    updateBestSearchResult(from_point, target_distance, segment_start, segment_delta, segment_index, alpha_start, 1.0,
                           best_result);

    const double projection_u =
        clamp01((from_point - segment_start).dot(segment_delta) / std::max(segment_length_sq, kDirectionNormEpsilon));
    updateBestSearchResult(from_point, target_distance, segment_start, segment_delta, segment_index, alpha_start,
                           projection_u, best_result);
  }

  best_result.cursor = normalizeCursorForBackwardTraversal(polyline, best_result.cursor);
  return best_result;
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
  std::vector<Eigen::Vector3d> target_positions;
  if (link_num_ > 1)
  {
    target_positions.reserve(static_cast<size_t>(link_num_ - 1));
  }

  if (trajectory_buffer_.size() < 2 || !latest_target_pose_)
  {
    return target_positions;
  }

  const std::vector<Eigen::Vector3d> trajectory_polyline =
      buildTrajectorySearchPolyline(trajectory_buffer_, link1_tail_pos_world_);
  if (trajectory_polyline.size() < 2)
  {
    return target_positions;
  }

  const Eigen::Vector3d current_link_direction = getRootLinkDirection(latest_target_pose_);
  const Eigen::Vector3d oldest_backward_direction =
      computeOldestBackwardDirection(trajectory_buffer_, current_link_direction);
  Eigen::Vector3d current_head = link1_tail_pos_world_;
  TrajectorySearchCursor trajectory_cursor = makeNewestTrajectoryCursor(trajectory_polyline);

  bool tail_extension_started = false;
  for (int i = 2; i <= link_num_; ++i)
  {
    Eigen::Vector3d target = current_head;
    if (!tail_extension_started)
    {
      const TrajectorySearchResult search_result =
          findPointOnTrajectoryAtDistance(trajectory_polyline, trajectory_cursor, current_head, link_length_);
      const double distance_error = search_result.distance_error;

      if (distance_error <= kWarmupDistanceTolerance)
      {
        target = search_result.position;
        trajectory_cursor = search_result.cursor;
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

  const std::vector<Eigen::Vector3d> trajectory_polyline =
      buildTrajectorySearchPolyline(trajectory_buffer_, link1_tail_pos_world_);
  if (trajectory_polyline.size() < 2)
  {
    return target_positions;
  }

  Eigen::Vector3d current_head = link1_tail_pos_world_;
  TrajectorySearchCursor trajectory_cursor = makeNewestTrajectoryCursor(trajectory_polyline);
  for (int i = 2; i <= link_num_; ++i)
  {
    const TrajectorySearchResult search_result =
        findPointOnTrajectoryAtDistance(trajectory_polyline, trajectory_cursor, current_head, link_length_);
    target_positions.push_back(search_result.position);
    current_head = search_result.position;
    trajectory_cursor = search_result.cursor;
  }

  return target_positions;
}

Eigen::VectorXd CopilotPlanner::computeJointAnglesFromSnakeTarget(const std::vector<Eigen::Vector3d>& target_positions)
{
  Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(link_joint_num_);

  tf::Quaternion root_quat;
  tf::quaternionMsgToTF(latest_target_pose_->pose.orientation, root_quat);
  const tf::Matrix3x3 root_rotation(root_quat);
  const tf::Vector3 link1_direction_tf = root_rotation * tf::Vector3(1.0, 0.0, 0.0);
  Eigen::Vector3d current_link_direction(link1_direction_tf.x(), link1_direction_tf.y(), link1_direction_tf.z());
  Eigen::Vector3d current_tail_position = link1_tail_pos_world_;

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
