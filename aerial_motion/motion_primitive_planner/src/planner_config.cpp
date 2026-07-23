#include <motion_primitive_planner/planner_config.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion_primitive_planner
{

RootPlannerConfig::RootPlannerConfig(const ros::NodeHandle& private_nh)
  : shared(private_nh)
  , follower(FollowerConfig::fromRos(private_nh))
  , stability(loadStabilityConfig(private_nh))
{
  private_nh.param("PredictionDt", prediction_dt, prediction_dt);
  private_nh.param("AllowCopilotStabilityProjectionFallback",
                   allow_copilot_stability_projection_fallback,
                   allow_copilot_stability_projection_fallback);
  if (!std::isfinite(prediction_dt) || prediction_dt <= 0.0)
  {
    throw std::invalid_argument("Invalid root motion primitive planner configuration");
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
  private_nh.param("MaxJointVelocity", joint.max_joint_velocity, joint.max_joint_velocity);
  private_nh.param("MaxJointCommandStep", joint.max_joint_command_step, joint.max_joint_command_step);
  int random_seed = static_cast<int>(joint.random_seed);
  private_nh.param("JointPlannerSeed", random_seed, random_seed);
  joint.random_seed = static_cast<unsigned int>(std::max(0, random_seed));
  private_nh.param("WholeBodyRadius", whole_body_radius, whole_body_radius);
  private_nh.param("PlanActivationLeadTime", activation_lead_time, activation_lead_time);
  private_nh.param("RootChildFrameId", root_child_frame_id, root_child_frame_id);

  if (!std::isfinite(whole_body_radius) || whole_body_radius < 0.0 ||
      !std::isfinite(activation_lead_time) || activation_lead_time <= 0.0 ||
      root_child_frame_id.empty())
  {
    throw std::invalid_argument("Invalid whole-body motion primitive planner configuration");
  }
}

}  // namespace motion_primitive_planner
