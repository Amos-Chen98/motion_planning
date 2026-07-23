// -*- mode: c++ -*-
#ifndef MOTION_PRIMITIVE_PLANNER_PLANNER_CONFIG_H
#define MOTION_PRIMITIVE_PLANNER_PLANNER_CONFIG_H

#include <motion_primitive_planner/joint_trajectory_planner.h>
#include <motion_primitive_planner/planner_common.h>

namespace motion_primitive_planner
{

struct RootPlannerConfig
{
  SharedPlannerConfig shared;
  FollowerConfig follower;
  multilink_copilot::StabilityConfig stability;
  double prediction_dt = 0.10;
  bool allow_copilot_stability_projection_fallback = false;
  bool verbose = true;

  explicit RootPlannerConfig(const ros::NodeHandle& private_nh);
};

struct WholeBodyPlannerConfig
{
  SharedPlannerConfig shared;
  JointPlannerConfig joint;
  multilink_copilot::StabilityConfig stability;
  // Each link capsule encloses the approximately 0.40 m-wide gimbal projection.
  double whole_body_radius = 0.20;
  double activation_lead_time = 0.75;
  std::string root_child_frame_id = "root";
  bool verbose = true;

  explicit WholeBodyPlannerConfig(const ros::NodeHandle& private_nh);
};

}  // namespace motion_primitive_planner

#endif  // MOTION_PRIMITIVE_PLANNER_PLANNER_CONFIG_H
