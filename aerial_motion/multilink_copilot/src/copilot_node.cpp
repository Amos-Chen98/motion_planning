// copilot_node.cpp
// Planner node that generates full state targets with snake-like motion

#include <multilink_copilot/copilot.h>

#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <string>
#include <vector>

namespace multilink_copilot
{
namespace
{
constexpr char kForcedRobotModelPluginName[] = "dragon/hydrus_like_robot_model";
constexpr char kJointPrefix[] = "joint";

bool extractSegmentIndex(const std::string& joint_name, const std::string& suffix, int& segment_index)
{
  const size_t prefix_length = std::char_traits<char>::length(kJointPrefix);
  if (joint_name.compare(0, prefix_length, kJointPrefix) != 0)
  {
    return false;
  }

  const size_t suffix_pos = joint_name.rfind(suffix);
  if (suffix_pos == std::string::npos || suffix_pos + suffix.size() != joint_name.size() || suffix_pos <= prefix_length)
  {
    return false;
  }

  try
  {
    segment_index = std::stoi(joint_name.substr(prefix_length, suffix_pos - prefix_length)) - 1;
  }
  catch (const std::exception&)
  {
    return false;
  }

  return segment_index >= 0;
}

geometry_msgs::PoseStamped::Ptr convertFluPoseToLinkFrame(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  geometry_msgs::PoseStamped::Ptr converted_msg(new geometry_msgs::PoseStamped(*msg));

  tf::Quaternion original_quat;
  tf::quaternionMsgToTF(msg->pose.orientation, original_quat);

  // The LINK frame differs from FLU by a 180-degree rotation around the body's local Z axis,
  // so convert by post-multiplying the fixed frame-offset quaternion.
  const tf::Quaternion local_z_rotation(0.0, 0.0, 1.0, 0.0);
  tf::quaternionTFToMsg(original_quat * local_z_rotation, converted_msg->pose.orientation);

  return converted_msg;
}
}  // namespace

CopilotPlanner::CopilotPlanner()
  : nh_("")
  , pnh_("~")
  , robot_model_loader_("aerial_robot_model", "aerial_robot_model::RobotModel")
  , total_arc_length_(0.0)
  , trajectory_initialized_(false)
  , has_latest_measured_link_joint_positions_(false)
  , has_latest_published_joint_positions_(false)
  , has_latest_stable_joint_positions_(false)
  , stability_debug_timer_started_(false)
  , has_last_published_root_pose_(false)
{
  loadParameters();
  initializeRobotModel();

  target_pose_sub_ = nh_.subscribe("root/target_pose", 1, &CopilotPlanner::targetPoseCallback, this); // root link's tail pose
  joint_state_sub_ = nh_.subscribe("joint_states", 1, &CopilotPlanner::jointStateCallback, this);
  full_state_target_pub_ = nh_.advertise<aerial_robot_msgs::FullStateTarget>("full_state_target", 1);
  trajectory_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_visualization", 1);
  if (publish_stability_metrics_)
  {
    stability_fc_rp_min_pub_ = nh_.advertise<std_msgs::Float64>("stability/fc_rp_min", 1);
    stability_overlap_clearance_pub_ = nh_.advertise<std_msgs::Float64>("stability/overlap_clearance", 1);
  }
  control_timer_ =
      nh_.createTimer(ros::Duration(1.0 / control_loop_rate_), &CopilotPlanner::controlTimerCallback, this);
  stability_debug_timer_ =
      nh_.createTimer(ros::Duration(0.5), &CopilotPlanner::stabilityDebugTimerCallback, this, false, false);

  ROS_INFO("[CopilotPlanner] Node initialized");
  ROS_INFO("[CopilotPlanner] Snake mode enabled: %s", snake_mode_enabled_ ? "true" : "false");
  ROS_INFO("[CopilotPlanner] Link number: %d, Link length: %.3f", link_num_, link_length_);
  ROS_INFO("[CopilotPlanner] Interpreting root/target_pose as the first-link tail pose and deriving root pose internally");
}

void CopilotPlanner::loadParameters()
{
  pnh_.param("control_loop_rate", control_loop_rate_, 50.0);
  pnh_.param("trajectory_sample_interval", trajectory_sample_interval_, 0.05);
  pnh_.param("trajectory_buffer_max_length", trajectory_buffer_max_length_, 10.0);
  pnh_.param("snake_mode_enabled", snake_mode_enabled_, true);
  pnh_.param("target_pose_frame_type", target_pose_frame_type_, std::string("LINK"));
  pnh_.param("publish_only_on_significant_root_motion", publish_only_on_significant_root_motion_, false);
  pnh_.param("publish_root_translation_threshold", publish_root_translation_threshold_, 0.05);
  pnh_.param("publish_root_rotation_threshold", publish_root_rotation_threshold_, 0.0872664626);
  pnh_.param("publish_stability_metrics", publish_stability_metrics_, false);
  pnh_.param("verbose", verbose_, false);
  pnh_.param("stability_qp_max_iterations", stability_qp_max_iterations_, 20);
  pnh_.param("stability_qp_joint_step_limit", stability_qp_joint_step_limit_, 0.1);
  pnh_.param("stability_qp_regularization", stability_qp_regularization_, 1e-3);
  pnh_.param("stability_qp_convergence_tol", stability_qp_convergence_tol_, 1e-3);
  pnh_.param("stability_qp_feasibility_tol", stability_qp_feasibility_tol_, 1e-4);
  pnh_.param("stability_check_fc_t", stability_check_fc_t_, false);
  pnh_.param("stability_fc_rp_min_thre", stability_fc_rp_min_thre_, 4.9);
  pnh_.param("stability_fc_t_min_thre", stability_fc_t_min_thre_, 0.01);
  pnh_.param("stability_static_thrust_min", stability_static_thrust_min_, 15.6);
  pnh_.param("stability_static_thrust_max", stability_static_thrust_max_, 23.6);
  pnh_.param("stability_overlap_min_clearance", stability_overlap_min_clearance_, 0.01);
  pnh_.param("stability_candidate_history_size", stability_candidate_history_size_, 32);
  pnh_.param("stability_candidate_top_k", stability_candidate_top_k_, 6);
  pnh_.param("stability_candidate_max_repairs", stability_candidate_max_repairs_, 2);
  stability_candidate_history_size_ = std::max(0, stability_candidate_history_size_);
  stability_candidate_top_k_ = std::max(0, stability_candidate_top_k_);
  stability_candidate_max_repairs_ = std::max(0, stability_candidate_max_repairs_);

  ROS_INFO("[CopilotPlanner] Loaded parameters:");
  ROS_INFO("  control_loop_rate: %.1f Hz", control_loop_rate_);
  ROS_INFO("  trajectory_sample_interval: %.3f m", trajectory_sample_interval_);
  ROS_INFO("  snake_mode_enabled: %s", snake_mode_enabled_ ? "true" : "false");
  ROS_INFO("  target_pose_frame_type: %s", target_pose_frame_type_.c_str());
  ROS_INFO("  publish_only_on_significant_root_motion: %s",
           publish_only_on_significant_root_motion_ ? "true" : "false");
  ROS_INFO("  publish_root_translation_threshold: %.3f m", publish_root_translation_threshold_);
  ROS_INFO("  publish_root_rotation_threshold: %.3f rad", publish_root_rotation_threshold_);
  ROS_INFO("  publish_stability_metrics: %s", publish_stability_metrics_ ? "true" : "false");
  ROS_INFO("  verbose: %s", verbose_ ? "true" : "false");
  ROS_INFO("  stability_qp_max_iterations: %d", stability_qp_max_iterations_);
  ROS_INFO("  stability_qp_joint_step_limit: %.3f", stability_qp_joint_step_limit_);
  ROS_INFO("  stability_fc_rp_min_thre: %.3f", stability_fc_rp_min_thre_);
  ROS_INFO("  stability_static_thrust_range: [%.3f, %.3f]", stability_static_thrust_min_,
           stability_static_thrust_max_);
  ROS_INFO("  stability_overlap_min_clearance: %.3f", stability_overlap_min_clearance_);
  ROS_INFO("  stability_check_fc_t: %s", stability_check_fc_t_ ? "true" : "false");
  ROS_INFO("  stability_candidate_history_size: %d", stability_candidate_history_size_);
  ROS_INFO("  stability_candidate_top_k: %d", stability_candidate_top_k_);
  ROS_INFO("  stability_candidate_max_repairs: %d", stability_candidate_max_repairs_);
}

void CopilotPlanner::initializeRobotModel()
{
  std::string configured_plugin_name;
  if (nh_.getParam("robot_model_plugin_name", configured_plugin_name) &&
      configured_plugin_name != kForcedRobotModelPluginName)
  {
    ROS_WARN("[CopilotPlanner] Ignoring robot_model_plugin_name '%s' and forcing '%s' for copilot planning",
             configured_plugin_name.c_str(), kForcedRobotModelPluginName);
  }

  ROS_INFO("[CopilotPlanner] Forcing robot model plugin: %s", kForcedRobotModelPluginName);

  try
  {
    auto loaded_model = robot_model_loader_.createInstance(kForcedRobotModelPluginName);
    dragon_robot_model_ = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(loaded_model);
    ROS_INFO("[CopilotPlanner] Loaded robot model plugin: %s", kForcedRobotModelPluginName);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("[CopilotPlanner] Failed to load robot model plugin '%s': %s", kForcedRobotModelPluginName,
              ex.what());
    ros::shutdown();
    return;
  }

  ros::Duration(1.0).sleep();

  if (!dragon_robot_model_)
  {
    ROS_ERROR("[CopilotPlanner] Failed to cast robot model to Dragon::HydrusLikeRobotModel");
    ros::shutdown();
    return;
  }

  link_num_ = dragon_robot_model_->getRotorNum();
  link_length_ = dragon_robot_model_->getLinkLength();
  link_joint_names_ = dragon_robot_model_->getLinkJointNames();
  link_joint_indices_ = dragon_robot_model_->getLinkJointIndices();
  link_joint_num_ = link_joint_indices_.size();
  latest_measured_link_joint_positions_ = Eigen::VectorXd::Zero(link_joint_num_);
  has_latest_measured_link_joint_positions_ = (link_joint_num_ == 0);
  link_joint_name_to_local_index_.clear();
  pitch_joint_local_indices_.assign(std::max(0, link_num_ - 1), -1);
  yaw_joint_local_indices_.clear();
  yaw_joint_local_indices_.assign(std::max(0, link_num_ - 1), -1);
  for (int i = 0; i < link_joint_num_; ++i)
  {
    const std::string& joint_name = i < static_cast<int>(link_joint_names_.size()) ? link_joint_names_.at(i) : "";
    if (!joint_name.empty())
    {
      link_joint_name_to_local_index_[joint_name] = i;
    }

    int segment_index = -1;

    if (extractSegmentIndex(joint_name, "_pitch", segment_index) &&
        segment_index < static_cast<int>(pitch_joint_local_indices_.size()))
    {
      pitch_joint_local_indices_.at(segment_index) = i;
    }
    else if (extractSegmentIndex(joint_name, "_yaw", segment_index) &&
             segment_index < static_cast<int>(yaw_joint_local_indices_.size()))
    {
      yaw_joint_local_indices_.at(segment_index) = i;
    }
  }

  const int pitch_joint_count =
      std::count_if(pitch_joint_local_indices_.begin(), pitch_joint_local_indices_.end(), [](int index) {
        return index >= 0;
      });
  const int yaw_joint_count =
      std::count_if(yaw_joint_local_indices_.begin(), yaw_joint_local_indices_.end(), [](int index) {
        return index >= 0;
      });

  ROS_INFO("[CopilotPlanner] Robot model initialized successfully");
  ROS_INFO("  Link num: %d, Joint num: %d", link_num_, link_joint_num_);
  ROS_INFO("  Pitch joint num: %d", pitch_joint_count);
  ROS_INFO("  Yaw joint num: %d", yaw_joint_count);
  if (pitch_joint_count != std::max(0, link_num_ - 1) || yaw_joint_count != std::max(0, link_num_ - 1))
  {
    ROS_WARN("[CopilotPlanner] Incomplete pitch/yaw joint mapping: expected %d pairs, got pitch=%d yaw=%d",
             std::max(0, link_num_ - 1), pitch_joint_count, yaw_joint_count);
  }
  ROS_INFO("[CopilotPlanner] Dragon model type: hydrus_like (forced for copilot)");
}

void CopilotPlanner::targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (target_pose_frame_type_ == "FLU")
  {
    latest_target_pose_ = convertFluPoseToLinkFrame(msg);
    ROS_DEBUG_THROTTLE(1.0, "[CopilotPlanner] Converted FLU frame to LINK frame");
    return;
  }

  latest_target_pose_ = msg;
}

void CopilotPlanner::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  if (link_joint_num_ <= 0)
  {
    has_latest_measured_link_joint_positions_ = true;
    latest_measured_link_joint_positions_.resize(0);
    return;
  }

  Eigen::VectorXd measured_joint_positions = Eigen::VectorXd::Zero(link_joint_num_);
  std::vector<bool> seen_joint_positions(static_cast<size_t>(link_joint_num_), false);
  const size_t entry_count = std::min(msg->name.size(), msg->position.size());

  for (size_t i = 0; i < entry_count; ++i)
  {
    const auto mapping = link_joint_name_to_local_index_.find(msg->name[i]);
    if (mapping == link_joint_name_to_local_index_.end())
    {
      continue;
    }

    const double position = msg->position[i];
    if (!std::isfinite(position))
    {
      ROS_WARN_THROTTLE(1.0, "[CopilotPlanner] Ignoring joint_states message with non-finite position for %s",
                        msg->name[i].c_str());
      return;
    }

    measured_joint_positions(mapping->second) = position;
    seen_joint_positions[static_cast<size_t>(mapping->second)] = true;
  }

  const auto missing_joint = std::find(seen_joint_positions.begin(), seen_joint_positions.end(), false);
  if (missing_joint != seen_joint_positions.end())
  {
    const size_t missing_index = static_cast<size_t>(std::distance(seen_joint_positions.begin(), missing_joint));
    const std::string missing_name =
        missing_index < link_joint_names_.size() ? link_joint_names_.at(missing_index)
                                                 : std::string("joint") + std::to_string(missing_index + 1);
    ROS_WARN_THROTTLE(1.0,
                      "[CopilotPlanner] Waiting for a complete joint_states message; missing link joint %s",
                      missing_name.c_str());
    return;
  }

  latest_measured_link_joint_positions_ = clampLinkJointPositions(measured_joint_positions);
  if (!has_latest_measured_link_joint_positions_)
  {
    ROS_INFO("[CopilotPlanner] Received first complete link joint state for warmup anchoring");
  }
  has_latest_measured_link_joint_positions_ = true;
}

void CopilotPlanner::controlTimerCallback(const ros::TimerEvent&)
{
  publishCurrentStabilityMetrics();

  if (!latest_target_pose_)
  {
    return;
  }

  const Eigen::Vector3d link1_tail_position = getLink1TailPositionFromPose(latest_target_pose_->pose);
  const geometry_msgs::Pose root_target_pose = convertLink1TailPoseToRootPose(latest_target_pose_->pose);
  const Eigen::Vector3d root_position = getRootPositionFromLink1TailPose(latest_target_pose_->pose);
  updateTrajectoryBuffer(link1_tail_position, root_position);

  if (!has_latest_measured_link_joint_positions_)
  {
    trajectory_viz_pub_.publish(getTrajectoryVisualization());
    ROS_WARN_THROTTLE(1.0,
                      "[CopilotPlanner] Waiting for complete joint_states before publishing full_state_target");
    return;
  }

  const auto planning_start_time = std::chrono::steady_clock::now();

  restoreRobotModelToLinkJointPositions(latest_measured_link_joint_positions_);

  Eigen::VectorXd desired_joint_positions = Eigen::VectorXd::Zero(link_joint_num_);
  latest_snake_targets_.clear();

  if (snake_mode_enabled_ && prepareTrajectoryData())
  {
    latest_snake_targets_ = computeSnakeTargetPositions();
    desired_joint_positions = computeJointAnglesFromSnakeTarget(latest_snake_targets_);
  }
  else if (snake_mode_enabled_)
  {
    latest_snake_targets_ = computeWarmupTargetPositions();
    desired_joint_positions = computeWarmupJointPositions(latest_snake_targets_);
  }
  else
  {
    desired_joint_positions = buildDefaultReferenceJointPositions();
  }

  Eigen::VectorXd stable_joint_positions;
  const bool stable_solved = computeStableJointPositions(desired_joint_positions, stable_joint_positions);

  const auto planning_end_time = std::chrono::steady_clock::now();
  const double planning_duration_ms =
      std::chrono::duration<double, std::milli>(planning_end_time - planning_start_time).count();
  ROS_INFO_THROTTLE(1.0, "[CopilotPlanner] Joint cmd planning time: %.3f ms", planning_duration_ms);

  if (!stable_solved)
  {
    ROS_ERROR_THROTTLE(1.0, "[CopilotPlanner] Failed to find a stable joint target; skipping this control cycle");
    return;
  }

  if (!shouldPublishFullStateTarget(root_target_pose))
  {
    trajectory_viz_pub_.publish(getTrajectoryVisualization());
    ROS_DEBUG_THROTTLE(1.0,
                       "[CopilotPlanner] Skipping full_state_target publish because root motion is below thresholds");
    return;
  }

  latest_published_joint_positions_ = stable_joint_positions;
  has_latest_published_joint_positions_ = true;

  if (verbose_ && !stability_debug_timer_started_)
  {
    stability_debug_timer_.start();
    stability_debug_timer_started_ = true;
    ROS_INFO("[CopilotPlanner] Started periodic stability debug timer");
  }

  aerial_robot_msgs::FullStateTarget full_state_msg;
  full_state_msg.header.stamp = ros::Time::now();
  full_state_msg.root_state.header = latest_target_pose_->header;
  full_state_msg.root_state.pose.pose = root_target_pose;
  full_state_msg.joint_state.header = latest_target_pose_->header;
  if (link_joint_names_.size() == static_cast<size_t>(link_joint_num_))
  {
    full_state_msg.joint_state.name = link_joint_names_;
  }
  else
  {
    full_state_msg.joint_state.name.resize(link_joint_num_);
    for (int i = 0; i < link_joint_num_; ++i)
    {
      full_state_msg.joint_state.name[i] = std::string("joint") + std::to_string(i + 1);
    }
  }
  full_state_msg.joint_state.position.resize(link_joint_num_);

  for (int i = 0; i < link_joint_num_; ++i)
  {
    full_state_msg.joint_state.position[i] = stable_joint_positions(i);
  }

  full_state_target_pub_.publish(full_state_msg);
  recordPublishedRootPose(root_target_pose);
  trajectory_viz_pub_.publish(getTrajectoryVisualization());
}

void CopilotPlanner::stabilityDebugTimerCallback(const ros::TimerEvent&)
{
  if (!has_latest_published_joint_positions_)
  {
    return;
  }

  checkStability(latest_published_joint_positions_);
}

void CopilotPlanner::publishStabilityMetrics(const StabilityMetrics& metrics)
{
  if (!publish_stability_metrics_)
  {
    return;
  }

  std_msgs::Float64 fc_rp_min_msg;
  fc_rp_min_msg.data = metrics.fc_rp_min;
  stability_fc_rp_min_pub_.publish(fc_rp_min_msg);

  std_msgs::Float64 overlap_clearance_msg;
  overlap_clearance_msg.data = metrics.overlap_clearance;
  stability_overlap_clearance_pub_.publish(overlap_clearance_msg);
}

void CopilotPlanner::publishCurrentStabilityMetrics()
{
  if (!publish_stability_metrics_ || !has_latest_measured_link_joint_positions_)
  {
    return;
  }

  StabilityMetrics current_metrics;
  if (evaluateStability(latest_measured_link_joint_positions_, current_metrics))
  {
    publishStabilityMetrics(current_metrics);
  }
}

bool CopilotPlanner::shouldPublishFullStateTarget(const geometry_msgs::Pose& root_target_pose) const
{
  if (!publish_only_on_significant_root_motion_ || !has_last_published_root_pose_)
  {
    return true;
  }

  const Eigen::Vector3d current_position(root_target_pose.position.x, root_target_pose.position.y,
                                         root_target_pose.position.z);
  const Eigen::Vector3d previous_position(last_published_root_pose_.position.x, last_published_root_pose_.position.y,
                                          last_published_root_pose_.position.z);
  const double translation_delta = (current_position - previous_position).norm();
  if (translation_delta > publish_root_translation_threshold_)
  {
    return true;
  }

  tf::Quaternion current_orientation;
  tf::Quaternion previous_orientation;
  tf::quaternionMsgToTF(root_target_pose.orientation, current_orientation);
  tf::quaternionMsgToTF(last_published_root_pose_.orientation, previous_orientation);

  if (current_orientation.length2() <= 0.0 || previous_orientation.length2() <= 0.0)
  {
    return false;
  }

  current_orientation.normalize();
  previous_orientation.normalize();
  const double rotation_delta = previous_orientation.angleShortestPath(current_orientation);
  return rotation_delta > publish_root_rotation_threshold_;
}

void CopilotPlanner::recordPublishedRootPose(const geometry_msgs::Pose& root_target_pose)
{
  last_published_root_pose_ = root_target_pose;
  has_last_published_root_pose_ = true;
}

}  // namespace multilink_copilot

int main(int argc, char** argv)
{
  ros::init(argc, argv, "copilot_planner");

  try
  {
    multilink_copilot::CopilotPlanner planner;
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Exception in CopilotPlanner: %s", e.what());
    return 1;
  }

  return 0;
}
