// -*- mode: c++ -*-
#ifndef MOTION_PRIMITIVE_PLANNER_WHOLE_BODY_PLANNER_H
#define MOTION_PRIMITIVE_PLANNER_WHOLE_BODY_PLANNER_H

#include <motion_primitive_planner/joint_trajectory_planner.h>
#include <motion_primitive_planner/root_primitive_generator.h>

#include <ros/time.h>

namespace motion_primitive_planner
{

struct WholeBodyCandidateScore
{
  bool feasible = false;
  double duration = std::numeric_limits<double>::infinity();
  double joint_motion = std::numeric_limits<double>::infinity();
  double root_jerk = std::numeric_limits<double>::infinity();
  double tracking_error_rms = 0.0;
};

//! The two weights convert joint-space path length [rad] and downstream-link
//! tracking RMS [m] into equivalent execution-time penalties [s].
int selectBestWholeBodyCandidate(const std::vector<WholeBodyCandidateScore>& candidates,
                                 double joint_motion_cost_weight = 0.25,
                                 double tracking_error_cost_weight = 6.0);

struct WholeBodyCandidate
{
  Candidate root;
  Trajectory<5> scaled_root;
  JointPlanResult joints;
  CandidateStatus status = CandidateStatus::kGenerationFailed;
  std::string detail;
};

struct WholeBodyPlanResult
{
  std::vector<WholeBodyCandidate> candidates;
  int selected = -1;
};

//! Evaluates one batch on the planning worker; it owns no ROS IO or execution state.
class WholeBodyPlanner
{
public:
  WholeBodyPlanner(
      const WholeBodyPlannerConfig& config, const DragonCollisionGeometry& geometry,
      const std::vector<std::shared_ptr<multilink_copilot::StabilityEvaluator>>& evaluators);

  //! The root batch and occupancy must come from the same map snapshot.
  //! The shared deadline uses ROS time, as does the node's activation schedule.
  WholeBodyPlanResult plan(
      const PrimitiveBatch& batch,
      const std::shared_ptr<const gcopter_planner::PlannerBackend>& occupancy,
      const Eigen::VectorXd& start_joints, const RootAttitude& start_attitude,
      const NominalJointContext& nominal_context, const ros::Time& deadline);

private:
  bool wholeBodyTrajectoryCollides(
      const Trajectory<5>& root, const JointPlanResult& joints,
      const std::shared_ptr<const gcopter_planner::PlannerBackend>& occupancy) const;
  int selectBest(const std::vector<WholeBodyCandidate>& candidates) const;

  WholeBodyPlannerConfig config_;
  DragonCollisionGeometry collision_geometry_;
  std::vector<std::unique_ptr<JointTrajectoryPlanner>> joint_planners_;
};

}  // namespace motion_primitive_planner

#endif  // MOTION_PRIMITIVE_PLANNER_WHOLE_BODY_PLANNER_H
