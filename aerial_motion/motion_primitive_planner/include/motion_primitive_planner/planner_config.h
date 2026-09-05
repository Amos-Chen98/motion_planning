// -*- mode: c++ -*-
#ifndef MOTION_PRIMITIVE_PLANNER_PLANNER_CONFIG_H
#define MOTION_PRIMITIVE_PLANNER_PLANNER_CONFIG_H

#include <gcopter/planner_common.hpp>
#include <multilink_copilot/stability_evaluator.h>
#include <ros/node_handle.h>

#include <string>

namespace motion_primitive_planner
{

struct PrimitiveConfig
{
  int candidate_count = 9;
  double max_offset = 0.8;
  double max_velocity = 0.2;
  double cruise_velocity = 0.16;
  double minimum_piece_duration = 0.2;

  void validateOrThrow() const;
};

struct FollowerConfig
{
  double command_hz = 40.0;
  double trajectory_sample_interval = 0.05;
  double trajectory_buffer_max_length = 10.0;
  double ik_singularity_threshold = 0.10;
  double max_angular_vel = 0.2;
  bool publish_yaw_command = true;

  static FollowerConfig fromRos(const ros::NodeHandle& private_nh);
  void validateOrThrow() const;
};

struct SharedPlannerConfig
{
  gcopter_planner::CommonPlannerConfig common;
  PrimitiveConfig primitive;
  double replan_trigger_ratio = 0.5;
  double goal_tolerance = 0.2;
  double planning_horizon = 3.0;
  bool zero_local_target_vel = true;

  SharedPlannerConfig() = default;
  explicit SharedPlannerConfig(const ros::NodeHandle& private_nh);
  void validateOrThrow() const;
};

multilink_copilot::StabilityConfig loadStabilityConfig(const ros::NodeHandle& private_nh);

struct JointPlannerConfig
{
  FollowerConfig follower;
  double reference_dt = 0.10;
  //! Total per-candidate joint-planning budget.
  double planning_timeout = 0.15;
  double validity_resolution = 0.025;
  double max_joint_command_step = 0.10;
  unsigned int random_seed = 1;

  void validateOrThrow() const;
};

struct WholeBodyPlannerConfig
{
  SharedPlannerConfig shared;
  JointPlannerConfig joint;
  multilink_copilot::StabilityConfig stability;
  double activation_lead_time = 0.75;
  //! Equivalent seconds charged per radian of whole-body joint path length.
  double joint_motion_cost_weight = 0.25;
  //! Equivalent seconds charged per metre of downstream-link tracking RMS.
  double tracking_error_cost_weight = 6.0;
  std::string root_child_frame_id = "root";
  bool verbose = true;

  explicit WholeBodyPlannerConfig(const ros::NodeHandle& private_nh);
};

}  // namespace motion_primitive_planner

#endif  // MOTION_PRIMITIVE_PLANNER_PLANNER_CONFIG_H
