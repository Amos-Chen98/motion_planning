#ifndef MOTION_PRIMITIVE_PLANNER_TRAJECTORY_COLLISION_H
#define MOTION_PRIMITIVE_PLANNER_TRAJECTORY_COLLISION_H

#include <motion_primitive_planner/dragon_collision_checker.h>
#include <motion_primitive_planner/joint_trajectory_planner.h>

namespace motion_primitive_planner
{
//! The final synchronized motion, sampled at CommandHz including both endpoints.
bool trajectoryCollides(const Trajectory<5>& root, const JointPlanResult& joints,
                        double command_hz, const CollisionEnvironment& environment,
                        DragonCollisionChecker& checker);
}  // namespace motion_primitive_planner

#endif  // MOTION_PRIMITIVE_PLANNER_TRAJECTORY_COLLISION_H
