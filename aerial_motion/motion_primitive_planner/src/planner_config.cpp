#include <motion_primitive_planner/planner_config.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion_primitive_planner
{
namespace
{
template <typename T>
void loadParam(const ros::NodeHandle& nh, const char* name, T& value)
{
  nh.param(name, value, value);
}
}  // namespace

void PrimitiveConfig::validateOrThrow() const
{
  if (candidate_count <= 0 || !std::isfinite(max_offset) || max_offset < 0.0 ||
      !std::isfinite(max_velocity) || max_velocity <= 0.0 ||
      !std::isfinite(cruise_velocity) || cruise_velocity <= 0.0 ||
      !std::isfinite(minimum_piece_duration) || minimum_piece_duration <= 0.0)
  {
    throw std::invalid_argument("Invalid motion primitive configuration");
  }
}

FollowerConfig FollowerConfig::fromRos(const ros::NodeHandle& private_nh)
{
  FollowerConfig config;
  loadParam(private_nh, "CommandHz", config.command_hz);
  loadParam(private_nh, "TrajectorySampleInterval", config.trajectory_sample_interval);
  loadParam(private_nh, "TrajectoryBufferMaxLength", config.trajectory_buffer_max_length);
  loadParam(private_nh, "SnakeIkSingularityThreshold", config.ik_singularity_threshold);
  loadParam(private_nh, "MaxAngularVel", config.max_angular_vel);
  loadParam(private_nh, "PublishYawCommand", config.publish_yaw_command);
  config.validateOrThrow();
  return config;
}

void FollowerConfig::validateOrThrow() const
{
  if (!std::isfinite(command_hz) || command_hz <= 0.0 ||
      !std::isfinite(trajectory_sample_interval) || trajectory_sample_interval <= 0.0 ||
      !std::isfinite(trajectory_buffer_max_length) || trajectory_buffer_max_length <= 0.0 ||
      !std::isfinite(ik_singularity_threshold) || ik_singularity_threshold < 0.0 ||
      !std::isfinite(max_angular_vel) || max_angular_vel <= 0.0)
  {
    throw std::invalid_argument("Invalid follow-the-leader configuration");
  }
}

SharedPlannerConfig::SharedPlannerConfig(const ros::NodeHandle& private_nh) : common(private_nh)
{
  loadParam(private_nh, "ReplanTriggerRatio", replan_trigger_ratio);
  loadParam(private_nh, "GoalTolerance", goal_tolerance);
  loadParam(private_nh, "PlanningHorizon", planning_horizon);
  loadParam(private_nh, "ZeroLocalTargetVel", zero_local_target_vel);
  loadParam(private_nh, "CandidateCount", primitive.candidate_count);
  loadParam(private_nh, "MaxPrimitiveOffset", primitive.max_offset);
  private_nh.param("PrimitiveCruiseVelocity", primitive.cruise_velocity, 0.8 * common.maxVelMag);
  loadParam(private_nh, "MinimumPieceDuration", primitive.minimum_piece_duration);
  primitive.max_velocity = common.maxVelMag;
  validateOrThrow();
}

void SharedPlannerConfig::validateOrThrow() const
{
  common.validateOrThrow();
  if (!std::isfinite(replan_trigger_ratio) ||
      replan_trigger_ratio <= 0.0 || replan_trigger_ratio >= 1.0 ||
      !std::isfinite(goal_tolerance) || goal_tolerance <= 0.0 ||
      !std::isfinite(planning_horizon) || planning_horizon <= 0.0)
  {
    throw std::invalid_argument("Invalid shared motion primitive planner configuration");
  }
  primitive.validateOrThrow();
}

multilink_copilot::StabilityConfig loadStabilityConfig(const ros::NodeHandle& private_nh)
{
  multilink_copilot::StabilityConfig config;
  loadParam(private_nh, "StabilityQpMaxIterations", config.qp_max_iterations);
  loadParam(private_nh, "StabilityQpJointStepLimit", config.qp_joint_step_limit);
  loadParam(private_nh, "StabilityQpRegularization", config.qp_regularization);
  loadParam(private_nh, "StabilityQpConvergenceTolerance", config.qp_convergence_tolerance);
  loadParam(private_nh, "FeasibilityTolerance", config.feasibility_tolerance);
  loadParam(private_nh, "StabilityCheckFcT", config.check_fc_t);
  loadParam(private_nh, "FcRpMinThreshold", config.fc_rp_min_threshold);
  loadParam(private_nh, "FcTMinThreshold", config.fc_t_min_threshold);
  loadParam(private_nh, "StaticThrustMin", config.static_thrust_min);
  loadParam(private_nh, "StaticThrustMax", config.static_thrust_max);
  loadParam(private_nh, "OverlapMinClearance", config.overlap_min_clearance);
  loadParam(private_nh, "MaxBaselinkTilt", config.max_baselink_tilt);
  return config;
}

void JointPlannerConfig::validateOrThrow() const
{
  if (reference_dt <= 0.0 || planning_timeout <= 0.0 || validity_resolution <= 0.0 ||
      max_joint_command_step <= 0.0 || follower.command_hz <= 0.0 ||
      follower.max_angular_vel <= 0.0)
  {
    throw std::invalid_argument("Invalid joint trajectory planner configuration");
  }
}

WholeBodyPlannerConfig::WholeBodyPlannerConfig(const ros::NodeHandle& private_nh)
  : shared(private_nh)
  , stability(loadStabilityConfig(private_nh))
{
  joint.follower = FollowerConfig::fromRos(private_nh);
  loadParam(private_nh, "PlanningThreads", planning_threads);
  loadParam(private_nh, "JointReferenceDt", joint.reference_dt);
  loadParam(private_nh, "JointPlanningTimeout", joint.planning_timeout);
  loadParam(private_nh, "JointValidityResolution", joint.validity_resolution);
  loadParam(private_nh, "MaxJointCommandStep", joint.max_joint_command_step);
  int random_seed = static_cast<int>(joint.random_seed);
  loadParam(private_nh, "JointPlannerSeed", random_seed);
  joint.random_seed = static_cast<unsigned int>(std::max(0, random_seed));
  loadParam(private_nh, "PlanActivationLeadTime", activation_lead_time);
  loadParam(private_nh, "JointMotionCostWeight", joint_motion_cost_weight);
  loadParam(private_nh, "TrackingErrorCostWeight", tracking_error_cost_weight);
  loadParam(private_nh, "RootChildFrameId", root_child_frame_id);
  loadParam(private_nh, "Verbose", verbose);

  if (planning_threads < 0 ||
      !std::isfinite(activation_lead_time) || activation_lead_time <= 0.0 ||
      !std::isfinite(joint_motion_cost_weight) || joint_motion_cost_weight < 0.0 ||
      !std::isfinite(tracking_error_cost_weight) || tracking_error_cost_weight < 0.0 ||
      root_child_frame_id.empty())
  {
    throw std::invalid_argument("Invalid whole-body motion primitive planner configuration");
  }
}

}  // namespace motion_primitive_planner
