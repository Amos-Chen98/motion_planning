// copilot_node.cpp
// Planner node that generates full state targets with snake-like motion

#include <multilink_copilot/copilot.h>

#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

namespace multilink_copilot
{
namespace
{
constexpr char kForcedRobotModelPluginName[] = "dragon/hydrus_like_robot_model";

geometry_msgs::PoseStamped::Ptr convertFluPoseToLinkFrame(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  geometry_msgs::PoseStamped::Ptr converted_msg(new geometry_msgs::PoseStamped(*msg));

  tf::Quaternion original_quat;
  tf::quaternionMsgToTF(msg->pose.orientation, original_quat);

  tf::Quaternion z_rotation(0.0, 0.0, 1.0, 0.0);
  tf::quaternionTFToMsg(z_rotation * original_quat, converted_msg->pose.orientation);

  return converted_msg;
}
}  // namespace

CopilotPlanner::CopilotPlanner()
  : nh_("")
  , pnh_("~")
  , robot_model_loader_("aerial_robot_model", "aerial_robot_model::RobotModel")
  , total_arc_length_(0.0)
  , trajectory_initialized_(false)
  , has_latest_desired_joint_positions_(false)
  , has_last_stable_joint_positions_(false)
  , stability_debug_timer_started_(false)
{
  loadParameters();
  initializeRobotModel();

  target_pose_sub_ = nh_.subscribe("root/target_pose", 1, &CopilotPlanner::targetPoseCallback, this); // root link's tail pose
  full_state_target_pub_ = nh_.advertise<aerial_robot_msgs::FullStateTarget>("full_state_target", 1);
  trajectory_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_visualization", 1);
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

  ROS_INFO("[CopilotPlanner] Loaded parameters:");
  ROS_INFO("  control_loop_rate: %.1f Hz", control_loop_rate_);
  ROS_INFO("  trajectory_sample_interval: %.3f m", trajectory_sample_interval_);
  ROS_INFO("  snake_mode_enabled: %s", snake_mode_enabled_ ? "true" : "false");
  ROS_INFO("  target_pose_frame_type: %s", target_pose_frame_type_.c_str());
  ROS_INFO("  stability_qp_max_iterations: %d", stability_qp_max_iterations_);
  ROS_INFO("  stability_qp_joint_step_limit: %.3f", stability_qp_joint_step_limit_);
  ROS_INFO("  stability_fc_rp_min_thre: %.3f", stability_fc_rp_min_thre_);
  ROS_INFO("  stability_static_thrust_range: [%.3f, %.3f]", stability_static_thrust_min_,
           stability_static_thrust_max_);
  ROS_INFO("  stability_overlap_min_clearance: %.3f", stability_overlap_min_clearance_);
  ROS_INFO("  stability_check_fc_t: %s", stability_check_fc_t_ ? "true" : "false");
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
  link_joint_indices_ = dragon_robot_model_->getLinkJointIndices();
  link_joint_num_ = link_joint_indices_.size();
  yaw_joint_local_indices_.clear();

  const auto& link_joint_names = dragon_robot_model_->getLinkJointNames();
  for (int i = 0; i < link_joint_num_; ++i)
  {
    const std::string& joint_name = i < static_cast<int>(link_joint_names.size()) ? link_joint_names.at(i) : "";
    if (joint_name.find("_yaw") != std::string::npos)
    {
      yaw_joint_local_indices_.push_back(i);
    }
  }

  ROS_INFO("[CopilotPlanner] Robot model initialized successfully");
  ROS_INFO("  Link num: %d, Joint num: %d", link_num_, link_joint_num_);
  ROS_INFO("  Yaw joint num: %zu", yaw_joint_local_indices_.size());
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

void CopilotPlanner::controlTimerCallback(const ros::TimerEvent&)
{
  if (!latest_target_pose_)
  {
    return;
  }

  const Eigen::Vector3d link1_tail_position = getLink1TailPositionFromPose(latest_target_pose_->pose);
  const geometry_msgs::Pose root_target_pose = convertLink1TailPoseToRootPose(latest_target_pose_->pose);
  const Eigen::Vector3d root_position = getRootPositionFromLink1TailPose(latest_target_pose_->pose);
  updateTrajectoryBuffer(link1_tail_position, root_position);

  Eigen::VectorXd desired_joint_positions = Eigen::VectorXd::Zero(link_joint_num_);
  latest_snake_targets_.clear();

  if (snake_mode_enabled_ && prepareTrajectoryData())
  {
    latest_snake_targets_ = computeSnakeTargetPositions();
    desired_joint_positions = computeJointAnglesFromSnakeTarget(latest_snake_targets_);
  }
  else if (snake_mode_enabled_ && trajectory_initialized_)
  {
    latest_snake_targets_ = computeWarmupTargetPositions();
    desired_joint_positions = computeWarmupJointPositions(latest_snake_targets_);
  }
  else
  {
    desired_joint_positions = buildDefaultReferenceJointPositions();
  }

  Eigen::VectorXd stable_joint_positions;
  if (!computeStableJointPositions(desired_joint_positions, stable_joint_positions))
  {
    ROS_ERROR_THROTTLE(1.0, "[CopilotPlanner] Failed to find a stable joint target; skipping this control cycle");
    return;
  }

  latest_desired_joint_positions_ = stable_joint_positions;
  has_latest_desired_joint_positions_ = true;

  if (!stability_debug_timer_started_)
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
  full_state_msg.joint_state.name.resize(link_joint_num_);
  full_state_msg.joint_state.position.resize(link_joint_num_);

  for (int i = 0; i < link_joint_num_; ++i)
  {
    full_state_msg.joint_state.name[i] = std::string("joint") + std::to_string(i + 1);
    full_state_msg.joint_state.position[i] = stable_joint_positions(i);
  }

  full_state_target_pub_.publish(full_state_msg);
  trajectory_viz_pub_.publish(getTrajectoryVisualization());
}

void CopilotPlanner::stabilityDebugTimerCallback(const ros::TimerEvent&)
{
  if (!has_latest_desired_joint_positions_)
  {
    return;
  }

  checkStability(latest_desired_joint_positions_);
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
