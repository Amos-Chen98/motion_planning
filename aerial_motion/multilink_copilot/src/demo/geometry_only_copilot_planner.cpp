// Geometry-only copilot planner for nominal multilink RViz simulation.

#include <aerial_robot_msgs/FullStateTarget.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <multilink_copilot/follow_the_leader.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <deque>
#include <exception>
#include <string>
#include <vector>

namespace multilink_copilot
{
namespace
{
constexpr double kDirectionNormEpsilon = 1e-6;

double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

double clampUnit(double value)
{
  return std::max(-1.0, std::min(1.0, value));
}

double smoothstep01(double value)
{
  value = clamp01(value);
  return value * value * (3.0 - 2.0 * value);
}

Eigen::Matrix3d rotationAroundY(double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);

  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  rotation(0, 0) = c;
  rotation(0, 2) = s;
  rotation(2, 0) = -s;
  rotation(2, 2) = c;
  return rotation;
}

Eigen::Matrix3d rotationAroundZ(double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);

  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  rotation(0, 0) = c;
  rotation(0, 1) = -s;
  rotation(1, 0) = s;
  rotation(1, 1) = c;
  return rotation;
}

tf::Quaternion normalizedQuaternionFromMsg(const geometry_msgs::Quaternion& msg)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg, quat);
  if (quat.length2() <= kDirectionNormEpsilon * kDirectionNormEpsilon)
  {
    quat.setValue(0.0, 0.0, 0.0, 1.0);
  }
  else
  {
    quat.normalize();
  }
  return quat;
}

Eigen::Vector3d getRootLinkDirectionFromPose(const geometry_msgs::Pose& pose)
{
  const tf::Quaternion root_quat = normalizedQuaternionFromMsg(pose.orientation);
  const tf::Matrix3x3 root_rotation(root_quat);
  const tf::Vector3 link1_direction_tf = root_rotation * tf::Vector3(1.0, 0.0, 0.0);
  Eigen::Vector3d link1_direction(link1_direction_tf.x(), link1_direction_tf.y(), link1_direction_tf.z());

  if (link1_direction.norm() < kDirectionNormEpsilon)
  {
    return Eigen::Vector3d::UnitX();
  }

  return link1_direction.normalized();
}

geometry_msgs::Point toPoint(const Eigen::Vector3d& position)
{
  geometry_msgs::Point point;
  point.x = position.x();
  point.y = position.y();
  point.z = position.z();
  return point;
}

geometry_msgs::Quaternion toQuaternionMsg(const Eigen::Matrix3d& rotation)
{
  Eigen::Quaterniond quat(rotation);
  quat.normalize();

  geometry_msgs::Quaternion msg;
  msg.x = quat.x();
  msg.y = quat.y();
  msg.z = quat.z();
  msg.w = quat.w();
  return msg;
}

}  // namespace

class GeometryOnlyCopilotPlanner
{
public:
  GeometryOnlyCopilotPlanner()
    : nh_("")
    , pnh_("~")
  {
    loadParameters();
    buildJointNames();

    target_pose_sub_ = nh_.subscribe("root/target_pose", 1, &GeometryOnlyCopilotPlanner::targetPoseCallback, this);
    mono_planner_trajectory_marker_sub_ =
        nh_.subscribe("mono_planner/traj_marker", 1, &GeometryOnlyCopilotPlanner::monoPlannerTrajectoryMarkerCallback,
                      this);
    full_state_target_pub_ = nh_.advertise<aerial_robot_msgs::FullStateTarget>("full_state_target", 1);
    root_link_tail_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("geometry/root_link_tail_pose", 1);
    last_link_tail_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("geometry/last_link_tail_pose", 1);
    trajectory_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("geometry/trajectory_markers", 1, true);
    control_timer_ =
        nh_.createTimer(ros::Duration(1.0 / control_loop_rate_), &GeometryOnlyCopilotPlanner::controlTimerCallback,
                        this);

    ROS_INFO("[GeometryOnlyCopilotPlanner] Node initialized");
    ROS_INFO("[GeometryOnlyCopilotPlanner] link_num=%d link_length=%.3f m control_loop_rate=%.1f Hz", link_num_,
             link_length_, control_loop_rate_);
    ROS_INFO("[GeometryOnlyCopilotPlanner] Input root/target_pose is interpreted as root-link tail pose");
  }

private:
  void loadParameters()
  {
    pnh_.param("link_num", link_num_, 4);
    pnh_.param("link_length", link_length_, 0.455);
    pnh_.param("control_loop_rate", control_loop_rate_, 40.0);
    pnh_.param("trajectory_sample_interval", trajectory_sample_interval_, 0.05);
    pnh_.param("trajectory_buffer_max_length", trajectory_buffer_max_length_, 10.0);
    pnh_.param("target_pose_frame_type", target_pose_frame_type_, std::string("FLU"));
    pnh_.param("publish_only_on_significant_root_motion", publish_only_on_significant_root_motion_, true);
    pnh_.param("publish_root_translation_threshold", publish_root_translation_threshold_, 0.05);
    pnh_.param("publish_root_rotation_threshold", publish_root_rotation_threshold_, 0.0872664626);
    pnh_.param("visualization_history_sample_interval", visualization_history_sample_interval_, 0.02);

    if (link_num_ < 1)
    {
      ROS_WARN("[GeometryOnlyCopilotPlanner] Invalid link_num=%d; clamping to 1", link_num_);
      link_num_ = 1;
    }
    if (link_num_ < 3 || link_num_ > 5)
    {
      ROS_WARN("[GeometryOnlyCopilotPlanner] Nominal geometry study is intended for link_num in {3,4,5}; using %d",
               link_num_);
    }
    if (link_length_ <= 0.0)
    {
      ROS_WARN("[GeometryOnlyCopilotPlanner] Invalid link_length=%.3f; using 0.455 m", link_length_);
      link_length_ = 0.455;
    }
    if (control_loop_rate_ <= 0.0)
    {
      ROS_WARN("[GeometryOnlyCopilotPlanner] Invalid control_loop_rate=%.3f; using 40 Hz", control_loop_rate_);
      control_loop_rate_ = 40.0;
    }
    if (trajectory_sample_interval_ <= 0.0)
    {
      ROS_WARN("[GeometryOnlyCopilotPlanner] Invalid trajectory_sample_interval=%.3f; using 0.05 m",
               trajectory_sample_interval_);
      trajectory_sample_interval_ = 0.05;
    }
    if (trajectory_buffer_max_length_ <= 0.0)
    {
      ROS_WARN("[GeometryOnlyCopilotPlanner] Invalid trajectory_buffer_max_length=%.3f; using 10 m",
               trajectory_buffer_max_length_);
      trajectory_buffer_max_length_ = 10.0;
    }
    if (visualization_history_sample_interval_ <= 0.0)
    {
      visualization_history_sample_interval_ = trajectory_sample_interval_;
    }

    std::transform(target_pose_frame_type_.begin(), target_pose_frame_type_.end(), target_pose_frame_type_.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    if (target_pose_frame_type_ != "FLU" && target_pose_frame_type_ != "LINK")
    {
      ROS_WARN("[GeometryOnlyCopilotPlanner] Unknown target_pose_frame_type '%s'; using FLU",
               target_pose_frame_type_.c_str());
      target_pose_frame_type_ = "FLU";
    }
  }

  void buildJointNames()
  {
    link_joint_names_.clear();
    pitch_joint_indices_.clear();
    yaw_joint_indices_.clear();
    link_joint_names_.reserve(static_cast<size_t>(std::max(0, 2 * (link_num_ - 1))));
    pitch_joint_indices_.reserve(static_cast<size_t>(std::max(0, link_num_ - 1)));
    yaw_joint_indices_.reserve(static_cast<size_t>(std::max(0, link_num_ - 1)));
    for (int link_index = 1; link_index < link_num_; ++link_index)
    {
      pitch_joint_indices_.push_back(2 * (link_index - 1));
      yaw_joint_indices_.push_back(2 * (link_index - 1) + 1);
      link_joint_names_.push_back("joint" + std::to_string(link_index) + "_pitch");
      link_joint_names_.push_back("joint" + std::to_string(link_index) + "_yaw");
    }
    link_joint_num_ = static_cast<int>(link_joint_names_.size());
    latest_published_joint_positions_ = Eigen::VectorXd::Zero(link_joint_num_);
  }

  void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    if (target_pose_frame_type_ == "FLU")
    {
      geometry_msgs::PoseStamped::Ptr converted_msg(new geometry_msgs::PoseStamped(*msg));

      const tf::Quaternion original_quat = normalizedQuaternionFromMsg(msg->pose.orientation);
      const tf::Quaternion local_z_rotation(0.0, 0.0, 1.0, 0.0);
      tf::quaternionTFToMsg(original_quat * local_z_rotation, converted_msg->pose.orientation);
      latest_target_pose_ = converted_msg;
      return;
    }

    latest_target_pose_ = msg;
  }

  void monoPlannerTrajectoryMarkerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
  {
    for (const auto& marker : msg->markers)
    {
      if (marker.ns != "trajectory_line" || marker.type != visualization_msgs::Marker::LINE_STRIP ||
          marker.action != visualization_msgs::Marker::ADD)
      {
        continue;
      }

      std::vector<Eigen::Vector3d> trajectory_points;
      trajectory_points.reserve(marker.points.size());
      for (const geometry_msgs::Point& point : marker.points)
      {
        trajectory_points.emplace_back(point.x, point.y, point.z);
      }

      if (trajectory_points.empty())
      {
        return;
      }

      planned_root_link_tail_trajectory_ = trajectory_points;
      if (!marker.header.frame_id.empty())
      {
        marker_frame_id_ = marker.header.frame_id;
      }
      ROS_INFO_ONCE("[GeometryOnlyCopilotPlanner] Captured full mono_planner root-link trajectory for time-colored "
                    "visualization");
      return;
    }
  }

  void controlTimerCallback(const ros::TimerEvent&)
  {
    if (!latest_target_pose_)
    {
      return;
    }

    marker_frame_id_ = latest_target_pose_->header.frame_id.empty() ? std::string("world") :
                                                                 latest_target_pose_->header.frame_id;
    const Eigen::Vector3d link1_tail_position = getLink1TailPositionFromPose(latest_target_pose_->pose);
    const geometry_msgs::Pose root_target_pose = convertLink1TailPoseToRootPose(latest_target_pose_->pose);
    const Eigen::Vector3d root_position(root_target_pose.position.x, root_target_pose.position.y,
                                        root_target_pose.position.z);
    updateTrajectoryBuffer(link1_tail_position);

    std::vector<Eigen::Vector3d> target_positions;
    if (prepareTrajectoryData())
    {
      target_positions = computeSnakeTargetPositions();
    }
    else
    {
      target_positions = computeWarmupTargetPositions();
    }

    const Eigen::VectorXd nominal_joint_positions = computeJointAnglesFromSnakeTarget(target_positions);
    updateLatestBody(root_position, link1_tail_position, target_positions, nominal_joint_positions);

    publishGeometryPoses();
    publishTrajectoryMarkers();

    if (shouldPublishFullStateTarget(root_target_pose))
    {
      publishFullStateTarget(root_target_pose, nominal_joint_positions);
      latest_published_joint_positions_ = nominal_joint_positions;
      recordPublishedRootPose(root_target_pose);
    }
  }

  void updateTrajectoryBuffer(const Eigen::Vector3d& link1_tail_position)
  {
    if (!trajectory_initialized_)
    {
      trajectory_buffer_.push_back({link1_tail_position});
      last_recorded_position_ = link1_tail_position;
      trajectory_initialized_ = true;
      total_arc_length_ = 0.0;
      appendHistoryPoint(root_link_tail_history_, link1_tail_position, true);
      ROS_INFO("[GeometryOnlyCopilotPlanner] Root-link tail trajectory initialized at [%.3f, %.3f, %.3f]",
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

    appendHistoryPoint(root_link_tail_history_, link1_tail_position, false);
  }

  bool prepareTrajectoryData() const
  {
    const double min_required_arc_length = static_cast<double>(link_num_ - 1) * link_length_;
    return total_arc_length_ >= min_required_arc_length;
  }

  std::vector<Eigen::Vector3d> computeWarmupTargetPositions() const
  {
    if (link_num_ <= 1 || trajectory_buffer_.empty() || !latest_target_pose_ ||
        latest_published_joint_positions_.size() != link_joint_num_)
    {
      return {};
    }

    const tf::Quaternion root_quat = normalizedQuaternionFromMsg(latest_target_pose_->pose.orientation);
    const tf::Matrix3x3 root_rotation_tf(root_quat);
    Eigen::Matrix3d root_rotation = Eigen::Matrix3d::Identity();
    for (int row = 0; row < 3; ++row)
    {
      for (int col = 0; col < 3; ++col)
      {
        root_rotation(row, col) = root_rotation_tf[row][col];
      }
    }

    const Eigen::Vector3d current_root_tail =
        getLink1TailPositionFromPose(latest_target_pose_->pose);
    const std::deque<TrajectoryPoint> nominal_history =
        follow_the_leader::prependCurrentBodyMorphology(
            trajectory_buffer_, current_root_tail, root_rotation,
            latest_published_joint_positions_, pitch_joint_indices_, yaw_joint_indices_,
            link_num_, link_length_);
    return follow_the_leader::computeTargetPositions(
        nominal_history, current_root_tail, link_num_, link_length_);
  }

  std::vector<Eigen::Vector3d> computeSnakeTargetPositions() const
  {
    if (link_num_ <= 1 || trajectory_buffer_.size() < 2 || !latest_target_pose_)
    {
      return {};
    }

    const Eigen::Vector3d current_root_tail =
        getLink1TailPositionFromPose(latest_target_pose_->pose);
    return follow_the_leader::computeTargetPositions(
        trajectory_buffer_, current_root_tail, link_num_, link_length_);
  }

  Eigen::VectorXd computeJointAnglesFromSnakeTarget(const std::vector<Eigen::Vector3d>& target_positions) const
  {
    Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(link_joint_num_);
    if (!latest_target_pose_)
    {
      return joint_positions;
    }

    const tf::Quaternion root_quat = normalizedQuaternionFromMsg(latest_target_pose_->pose.orientation);
    const tf::Matrix3x3 root_rotation(root_quat);
    Eigen::Matrix3d current_link_rotation = Eigen::Matrix3d::Identity();
    for (int row = 0; row < 3; ++row)
    {
      for (int col = 0; col < 3; ++col)
      {
        current_link_rotation(row, col) = root_rotation[row][col];
      }
    }
    Eigen::Vector3d current_tail_position = getLink1TailPositionFromPose(latest_target_pose_->pose);

    for (size_t i = 0; i < target_positions.size() && i < static_cast<size_t>(link_num_ - 1); ++i)
    {
      const Eigen::Vector3d segment = target_positions[i] - current_tail_position;
      if (segment.norm() < kDirectionNormEpsilon)
      {
        continue;
      }

      const Eigen::Vector3d desired_next_direction = segment.normalized();
      Eigen::Vector3d desired_next_direction_local = current_link_rotation.transpose() * desired_next_direction;
      const double desired_next_direction_local_norm = desired_next_direction_local.norm();
      if (desired_next_direction_local_norm < kDirectionNormEpsilon)
      {
        current_tail_position = target_positions[i];
        continue;
      }

      desired_next_direction_local /= desired_next_direction_local_norm;

      const double joint_yaw = std::asin(clampUnit(desired_next_direction_local.y()));
      const double xz_norm = std::hypot(desired_next_direction_local.x(), desired_next_direction_local.z());
      const double joint_pitch =
          xz_norm < kDirectionNormEpsilon ? 0.0 : std::atan2(-desired_next_direction_local.z(),
                                                             desired_next_direction_local.x());

      const int pitch_joint_index = static_cast<int>(2 * i);
      const int yaw_joint_index = pitch_joint_index + 1;
      if (pitch_joint_index >= 0 && pitch_joint_index < link_joint_num_)
      {
        joint_positions(pitch_joint_index) = joint_pitch;
      }
      if (yaw_joint_index >= 0 && yaw_joint_index < link_joint_num_)
      {
        joint_positions(yaw_joint_index) = joint_yaw;
      }

      current_link_rotation = current_link_rotation * rotationAroundY(joint_pitch) * rotationAroundZ(joint_yaw);
      current_tail_position = target_positions[i];
    }

    return joint_positions;
  }

  void updateLatestBody(const Eigen::Vector3d& root_position,
                        const Eigen::Vector3d& link1_tail_position,
                        const std::vector<Eigen::Vector3d>& target_positions,
                        const Eigen::VectorXd& nominal_joint_positions)
  {
    latest_body_points_.clear();
    latest_tail_points_.clear();
    latest_body_points_.reserve(static_cast<size_t>(link_num_ + 1));
    latest_tail_points_.reserve(static_cast<size_t>(link_num_));
    latest_body_points_.push_back(root_position);
    latest_body_points_.push_back(link1_tail_position);
    latest_tail_points_.push_back(link1_tail_position);

    for (int i = 0; i < link_num_ - 1 && i < static_cast<int>(target_positions.size()); ++i)
    {
      latest_body_points_.push_back(target_positions[static_cast<size_t>(i)]);
      latest_tail_points_.push_back(target_positions[static_cast<size_t>(i)]);
    }

    latest_link_rotations_ = computeLinkRotations(nominal_joint_positions);
  }

  std::vector<Eigen::Matrix3d> computeLinkRotations(const Eigen::VectorXd& joint_positions) const
  {
    std::vector<Eigen::Matrix3d> rotations;
    rotations.reserve(static_cast<size_t>(link_num_));

    if (!latest_target_pose_)
    {
      rotations.push_back(Eigen::Matrix3d::Identity());
      return rotations;
    }

    const tf::Quaternion root_quat = normalizedQuaternionFromMsg(latest_target_pose_->pose.orientation);
    const tf::Matrix3x3 root_rotation_tf(root_quat);
    Eigen::Matrix3d current_rotation = Eigen::Matrix3d::Identity();
    for (int row = 0; row < 3; ++row)
    {
      for (int col = 0; col < 3; ++col)
      {
        current_rotation(row, col) = root_rotation_tf[row][col];
      }
    }

    rotations.push_back(current_rotation);
    for (int i = 0; i < link_num_ - 1; ++i)
    {
      const int pitch_joint_index = 2 * i;
      const int yaw_joint_index = pitch_joint_index + 1;
      const double pitch = pitch_joint_index < joint_positions.size() ? joint_positions(pitch_joint_index) : 0.0;
      const double yaw = yaw_joint_index < joint_positions.size() ? joint_positions(yaw_joint_index) : 0.0;
      current_rotation = current_rotation * rotationAroundY(pitch) * rotationAroundZ(yaw);
      rotations.push_back(current_rotation);
    }

    return rotations;
  }

  geometry_msgs::Pose convertLink1TailPoseToRootPose(const geometry_msgs::Pose& pose) const
  {
    geometry_msgs::Pose root_pose = pose;
    const Eigen::Vector3d link1_direction = getRootLinkDirectionFromPose(pose);
    root_pose.position.x -= link_length_ * link1_direction.x();
    root_pose.position.y -= link_length_ * link1_direction.y();
    root_pose.position.z -= link_length_ * link1_direction.z();
    return root_pose;
  }

  Eigen::Vector3d getLink1TailPositionFromPose(const geometry_msgs::Pose& pose) const
  {
    return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  }

  bool shouldPublishFullStateTarget(const geometry_msgs::Pose& root_target_pose) const
  {
    if (!publish_only_on_significant_root_motion_ || !has_last_published_root_pose_)
    {
      return true;
    }

    const Eigen::Vector3d current_position(root_target_pose.position.x, root_target_pose.position.y,
                                           root_target_pose.position.z);
    const Eigen::Vector3d previous_position(last_published_root_pose_.position.x, last_published_root_pose_.position.y,
                                            last_published_root_pose_.position.z);
    if ((current_position - previous_position).norm() > publish_root_translation_threshold_)
    {
      return true;
    }

    const tf::Quaternion current_orientation = normalizedQuaternionFromMsg(root_target_pose.orientation);
    const tf::Quaternion previous_orientation = normalizedQuaternionFromMsg(last_published_root_pose_.orientation);
    return previous_orientation.angleShortestPath(current_orientation) > publish_root_rotation_threshold_;
  }

  void recordPublishedRootPose(const geometry_msgs::Pose& root_target_pose)
  {
    last_published_root_pose_ = root_target_pose;
    has_last_published_root_pose_ = true;
  }

  void publishFullStateTarget(const geometry_msgs::Pose& root_target_pose,
                              const Eigen::VectorXd& nominal_joint_positions) const
  {
    aerial_robot_msgs::FullStateTarget full_state_msg;
    full_state_msg.header.stamp = ros::Time::now();
    full_state_msg.root_state.header = latest_target_pose_->header;
    full_state_msg.root_state.pose.pose = root_target_pose;
    full_state_msg.joint_state.header = latest_target_pose_->header;
    full_state_msg.joint_state.name = link_joint_names_;
    full_state_msg.joint_state.position.resize(static_cast<size_t>(link_joint_num_));

    for (int i = 0; i < link_joint_num_; ++i)
    {
      full_state_msg.joint_state.position[static_cast<size_t>(i)] = nominal_joint_positions(i);
    }

    full_state_target_pub_.publish(full_state_msg);
  }

  void publishGeometryPoses() const
  {
    if (!latest_target_pose_ || latest_tail_points_.empty())
    {
      return;
    }

    geometry_msgs::PoseStamped root_tail_pose_msg = *latest_target_pose_;
    root_tail_pose_msg.header.stamp = ros::Time::now();
    root_link_tail_pose_pub_.publish(root_tail_pose_msg);

    geometry_msgs::PoseStamped last_tail_pose_msg;
    last_tail_pose_msg.header = root_tail_pose_msg.header;
    last_tail_pose_msg.pose.position.x = latest_tail_points_.back().x();
    last_tail_pose_msg.pose.position.y = latest_tail_points_.back().y();
    last_tail_pose_msg.pose.position.z = latest_tail_points_.back().z();
    if (!latest_link_rotations_.empty())
    {
      last_tail_pose_msg.pose.orientation = toQuaternionMsg(latest_link_rotations_.back());
    }
    else
    {
      last_tail_pose_msg.pose.orientation.w = 1.0;
    }
    last_link_tail_pose_pub_.publish(last_tail_pose_msg);
  }

  void publishTrajectoryMarkers() const
  {
    visualization_msgs::MarkerArray marker_array;
    const ros::Time stamp = ros::Time::now();

    marker_array.markers.push_back(buildDeleteMarker("last_link_tail_history", 1, stamp));
    marker_array.markers.push_back(buildDeleteMarker("last_link_tail_history_points", 7, stamp));
    marker_array.markers.push_back(buildDeleteMarker("current_body", 2, stamp));
    marker_array.markers.push_back(buildDeleteMarker("root_link_tail_history", 0, stamp));
    appendRootLinkTailTimeColorbarDeleteMarkers(marker_array, stamp);
    marker_array.markers.push_back(buildTimeColoredLineStripMarker("root_link_tail_history_time_colored", 8,
                                                                   getRootLinkTailVisualizationPoints(), 0.055, stamp));
    appendBodyLinkCylinderMarkers(marker_array, stamp);
    marker_array.markers.push_back(buildSphereListMarker("current_tail_points", 3, latest_tail_points_,
                                                        0.20, 0.55, 1.0, 1.0, 0.10, stamp));

    if (!latest_body_points_.empty())
    {
      marker_array.markers.push_back(buildSphereMarker("current_root_pose", 4, latest_body_points_.front(),
                                                      1.0, 0.15, 0.10, 1.0, 0.13, stamp));
    }
    if (latest_body_points_.size() > 1)
    {
      marker_array.markers.push_back(buildSphereMarker("current_root_link_tail", 5, latest_body_points_[1],
                                                      0.10, 0.45, 1.0, 1.0, 0.12, stamp));
    }
    if (!latest_tail_points_.empty())
    {
      marker_array.markers.push_back(buildSphereMarker("current_last_link_tail", 6, latest_tail_points_.back(),
                                                      0.85, 0.15, 0.85, 1.0, 0.13, stamp));
    }

    trajectory_marker_pub_.publish(marker_array);
  }

  const std::vector<Eigen::Vector3d>& getRootLinkTailVisualizationPoints() const
  {
    if (!planned_root_link_tail_trajectory_.empty())
    {
      return planned_root_link_tail_trajectory_;
    }

    return root_link_tail_history_;
  }

  void appendBodyLinkCylinderMarkers(visualization_msgs::MarkerArray& marker_array, const ros::Time& stamp) const
  {
    constexpr int kMaxNominalBodyLinkMarkers = 8;
    const int segment_count = std::max(0, static_cast<int>(latest_body_points_.size()) - 1);
    for (int i = 0; i < segment_count; ++i)
    {
      marker_array.markers.push_back(buildCylinderMarker("current_body_links", i, latest_body_points_[i],
                                                         latest_body_points_[i + 1], 0.0, 0.0, 0.0, 1.0, 0.045,
                                                         stamp));
    }

    for (int i = segment_count; i < kMaxNominalBodyLinkMarkers; ++i)
    {
      marker_array.markers.push_back(buildDeleteMarker("current_body_links", i, stamp));
    }
  }

  std_msgs::ColorRGBA interpolateTimeColor(double normalized_time) const
  {
    const double t = clamp01(normalized_time);
    std_msgs::ColorRGBA color;
    color.a = 1.0;

    struct ColorStop
    {
      double time;
      double r;
      double g;
      double b;
    };

    const ColorStop color_stops[] = {
        {0.00, 0.00, 0.00, 0.45},  // bottom: deep blue
        {0.16, 0.00, 0.00, 1.00},
        {0.32, 0.00, 0.55, 1.00},
        {0.46, 0.00, 1.00, 0.90},
        {0.58, 0.55, 1.00, 0.20},
        {0.70, 1.00, 1.00, 0.00},
        {0.82, 1.00, 0.48, 0.00},
        {0.94, 1.00, 0.00, 0.00},
        {1.00, 0.55, 0.00, 0.00},  // top: deep red
    };

    for (size_t i = 0; i + 1 < sizeof(color_stops) / sizeof(color_stops[0]); ++i)
    {
      const ColorStop& lower = color_stops[i];
      const ColorStop& upper = color_stops[i + 1];
      if (t > upper.time)
      {
        continue;
      }

      const double local_t = smoothstep01((t - lower.time) / (upper.time - lower.time));
      color.r = lower.r + local_t * (upper.r - lower.r);
      color.g = lower.g + local_t * (upper.g - lower.g);
      color.b = lower.b + local_t * (upper.b - lower.b);
      return color;
    }

    color.r = color_stops[sizeof(color_stops) / sizeof(color_stops[0]) - 1].r;
    color.g = color_stops[sizeof(color_stops) / sizeof(color_stops[0]) - 1].g;
    color.b = color_stops[sizeof(color_stops) / sizeof(color_stops[0]) - 1].b;
    return color;
  }

  visualization_msgs::Marker buildTimeColoredLineStripMarker(const std::string& ns,
                                                             int id,
                                                             const std::vector<Eigen::Vector3d>& points,
                                                             double width,
                                                             const ros::Time& stamp) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = marker_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = width;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0.0);
    marker.points.reserve(points.size());
    marker.colors.reserve(points.size());

    const double denominator = points.size() > 1 ? static_cast<double>(points.size() - 1) : 1.0;
    for (size_t i = 0; i < points.size(); ++i)
    {
      marker.points.push_back(toPoint(points[i]));
      marker.colors.push_back(interpolateTimeColor(static_cast<double>(i) / denominator));
    }

    return marker;
  }

  void appendRootLinkTailTimeColorbarDeleteMarkers(visualization_msgs::MarkerArray& marker_array,
                                                   const ros::Time& stamp) const
  {
    for (int id = 0; id < 5; ++id)
    {
      marker_array.markers.push_back(buildDeleteMarker("root_link_tail_time_colorbar", id, stamp));
    }
    marker_array.markers.push_back(buildDeleteMarker("root_link_tail_time_colorbar", 100, stamp));
    marker_array.markers.push_back(buildDeleteMarker("root_link_tail_time_colorbar", 101, stamp));
  }

  visualization_msgs::Marker buildCylinderMarker(const std::string& ns,
                                                 int id,
                                                 const Eigen::Vector3d& start,
                                                 const Eigen::Vector3d& end,
                                                 double r,
                                                 double g,
                                                 double b,
                                                 double a,
                                                 double diameter,
                                                 const ros::Time& stamp) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = marker_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    const Eigen::Vector3d segment = end - start;
    const double length = segment.norm();
    if (length < kDirectionNormEpsilon)
    {
      marker.action = visualization_msgs::Marker::DELETE;
      return marker;
    }

    const Eigen::Vector3d midpoint = 0.5 * (start + end);
    marker.pose.position.x = midpoint.x();
    marker.pose.position.y = midpoint.y();
    marker.pose.position.z = midpoint.z();

    const Eigen::Quaterniond orientation =
        Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), segment.normalized());
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();

    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = length;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = ros::Duration(0.0);
    return marker;
  }

  visualization_msgs::Marker buildDeleteMarker(const std::string& ns, int id, const ros::Time& stamp) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = marker_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.action = visualization_msgs::Marker::DELETE;
    return marker;
  }

  visualization_msgs::Marker buildSphereListMarker(const std::string& ns,
                                                   int id,
                                                   const std::vector<Eigen::Vector3d>& points,
                                                   double r,
                                                   double g,
                                                   double b,
                                                   double a,
                                                   double diameter,
                                                   const ros::Time& stamp) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = marker_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = diameter;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = ros::Duration(0.0);
    marker.points.reserve(points.size());
    for (const Eigen::Vector3d& point : points)
    {
      marker.points.push_back(toPoint(point));
    }
    return marker;
  }

  visualization_msgs::Marker buildSphereMarker(const std::string& ns,
                                               int id,
                                               const Eigen::Vector3d& position,
                                               double r,
                                               double g,
                                               double b,
                                               double a,
                                               double diameter,
                                               const ros::Time& stamp) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = marker_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = diameter;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = ros::Duration(0.0);
    return marker;
  }

  void appendHistoryPoint(std::vector<Eigen::Vector3d>& history,
                          const Eigen::Vector3d& point,
                          bool force) const
  {
    if (force || history.empty() || (history.back() - point).norm() >= visualization_history_sample_interval_)
    {
      history.push_back(point);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber target_pose_sub_;
  ros::Subscriber mono_planner_trajectory_marker_sub_;
  ros::Publisher full_state_target_pub_;
  ros::Publisher root_link_tail_pose_pub_;
  ros::Publisher last_link_tail_pose_pub_;
  ros::Publisher trajectory_marker_pub_;
  ros::Timer control_timer_;

  int link_num_ = 4;
  int link_joint_num_ = 0;
  double link_length_ = 0.455;
  double control_loop_rate_ = 40.0;
  double trajectory_sample_interval_ = 0.05;
  double trajectory_buffer_max_length_ = 10.0;
  double total_arc_length_ = 0.0;
  double publish_root_translation_threshold_ = 0.05;
  double publish_root_rotation_threshold_ = 0.0872664626;
  double visualization_history_sample_interval_ = 0.02;
  bool publish_only_on_significant_root_motion_ = true;
  bool trajectory_initialized_ = false;
  bool has_last_published_root_pose_ = false;
  std::string target_pose_frame_type_ = "FLU";
  std::string marker_frame_id_ = "world";
  std::vector<std::string> link_joint_names_;
  std::vector<int> pitch_joint_indices_;
  std::vector<int> yaw_joint_indices_;

  geometry_msgs::PoseStamped::ConstPtr latest_target_pose_;
  geometry_msgs::Pose last_published_root_pose_;
  Eigen::Vector3d last_recorded_position_ = Eigen::Vector3d::Zero();
  Eigen::VectorXd latest_published_joint_positions_;
  std::deque<TrajectoryPoint> trajectory_buffer_;
  std::vector<Eigen::Vector3d> root_link_tail_history_;
  std::vector<Eigen::Vector3d> planned_root_link_tail_trajectory_;
  std::vector<Eigen::Vector3d> latest_body_points_;
  std::vector<Eigen::Vector3d> latest_tail_points_;
  std::vector<Eigen::Matrix3d> latest_link_rotations_;
};

}  // namespace multilink_copilot

int main(int argc, char** argv)
{
  ros::init(argc, argv, "geometry_only_copilot_planner");

  try
  {
    multilink_copilot::GeometryOnlyCopilotPlanner planner;
    ros::spin();
  }
  catch (const std::exception& exc)
  {
    ROS_ERROR("Exception in GeometryOnlyCopilotPlanner: %s", exc.what());
    return 1;
  }

  return 0;
}
