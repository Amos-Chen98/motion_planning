#include <motion_primitive_planner/planner_config.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion_primitive_planner
{

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
  private_nh.param("CommandHz", config.command_hz, config.command_hz);
  private_nh.param("TrajectorySampleInterval", config.trajectory_sample_interval,
                   config.trajectory_sample_interval);
  private_nh.param("TrajectoryBufferMaxLength", config.trajectory_buffer_max_length,
                   config.trajectory_buffer_max_length);
  private_nh.param("SnakeIkSingularityThreshold", config.ik_singularity_threshold,
                   config.ik_singularity_threshold);
  private_nh.param("MaxAngularVel", config.max_angular_vel, config.max_angular_vel);
  private_nh.param("PublishYawCommand", config.publish_yaw_command, config.publish_yaw_command);
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
  private_nh.param("ReplanTriggerRatio", replan_trigger_ratio, replan_trigger_ratio);
  private_nh.param("GoalTolerance", goal_tolerance, goal_tolerance);
  private_nh.param("PlanningHorizon", planning_horizon, planning_horizon);
  private_nh.param("ZeroLocalTargetVel", zero_local_target_vel, zero_local_target_vel);
  private_nh.param("CandidateCount", primitive.candidate_count, primitive.candidate_count);
  private_nh.param("MaxPrimitiveOffset", primitive.max_offset, primitive.max_offset);
  private_nh.param("PrimitiveCruiseVelocity", primitive.cruise_velocity, 0.8 * common.maxVelMag);
  private_nh.param("MinimumPieceDuration", primitive.minimum_piece_duration,
                   primitive.minimum_piece_duration);
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
  private_nh.param("StabilityQpMaxIterations", config.qp_max_iterations, config.qp_max_iterations);
  private_nh.param("StabilityQpJointStepLimit", config.qp_joint_step_limit,
                   config.qp_joint_step_limit);
  private_nh.param("StabilityQpRegularization", config.qp_regularization,
                   config.qp_regularization);
  private_nh.param("StabilityQpConvergenceTolerance", config.qp_convergence_tolerance,
                   config.qp_convergence_tolerance);
  private_nh.param("FeasibilityTolerance", config.feasibility_tolerance,
                   config.feasibility_tolerance);
  private_nh.param("StabilityCheckFcT", config.check_fc_t, config.check_fc_t);
  private_nh.param("FcRpMinThreshold", config.fc_rp_min_threshold, config.fc_rp_min_threshold);
  private_nh.param("FcTMinThreshold", config.fc_t_min_threshold, config.fc_t_min_threshold);
  private_nh.param("StaticThrustMin", config.static_thrust_min, config.static_thrust_min);
  private_nh.param("StaticThrustMax", config.static_thrust_max, config.static_thrust_max);
  private_nh.param("OverlapMinClearance", config.overlap_min_clearance,
                   config.overlap_min_clearance);
  private_nh.param("MaxBaselinkTilt", config.max_baselink_tilt, config.max_baselink_tilt);
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
  private_nh.param("JointReferenceDt", joint.reference_dt, joint.reference_dt);
  private_nh.param("JointPlanningTimeout", joint.planning_timeout, joint.planning_timeout);
  private_nh.param("JointValidityResolution", joint.validity_resolution, joint.validity_resolution);
  private_nh.param("MaxJointCommandStep", joint.max_joint_command_step, joint.max_joint_command_step);
  int random_seed = static_cast<int>(joint.random_seed);
  private_nh.param("JointPlannerSeed", random_seed, random_seed);
  joint.random_seed = static_cast<unsigned int>(std::max(0, random_seed));
  private_nh.param("PlanActivationLeadTime", activation_lead_time, activation_lead_time);
  private_nh.param("JointMotionCostWeight", joint_motion_cost_weight, joint_motion_cost_weight);
  private_nh.param("TrackingErrorCostWeight", tracking_error_cost_weight,
                   tracking_error_cost_weight);
  private_nh.param("RootChildFrameId", root_child_frame_id, root_child_frame_id);
  private_nh.param("Verbose", verbose, verbose);

  if (!std::isfinite(activation_lead_time) || activation_lead_time <= 0.0 ||
      !std::isfinite(joint_motion_cost_weight) || joint_motion_cost_weight < 0.0 ||
      !std::isfinite(tracking_error_cost_weight) || tracking_error_cost_weight < 0.0 ||
      root_child_frame_id.empty())
  {
    throw std::invalid_argument("Invalid whole-body motion primitive planner configuration");
  }
}

}  // namespace motion_primitive_planner
