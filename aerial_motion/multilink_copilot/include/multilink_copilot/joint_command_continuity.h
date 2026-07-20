#ifndef MULTILINK_COPILOT_JOINT_COMMAND_CONTINUITY_H
#define MULTILINK_COPILOT_JOINT_COMMAND_CONTINUITY_H

#include <Eigen/Dense>

#include <cmath>

namespace multilink_copilot
{

inline bool limitJointPositionStep(const Eigen::VectorXd& reference_joint_positions,
                                   const Eigen::VectorXd& requested_joint_positions,
                                   double max_joint_step,
                                   Eigen::VectorXd& limited_joint_positions,
                                   bool& was_limited)
{
  was_limited = false;
  if (reference_joint_positions.size() != requested_joint_positions.size() ||
      !reference_joint_positions.allFinite() || !requested_joint_positions.allFinite() ||
      !std::isfinite(max_joint_step))
  {
    return false;
  }

  limited_joint_positions = requested_joint_positions;
  if (max_joint_step <= 0.0 || requested_joint_positions.size() == 0)
  {
    return true;
  }

  const Eigen::VectorXd joint_delta = requested_joint_positions - reference_joint_positions;
  const double largest_joint_step = joint_delta.cwiseAbs().maxCoeff();
  if (largest_joint_step <= max_joint_step)
  {
    return true;
  }

  limited_joint_positions = reference_joint_positions + (max_joint_step / largest_joint_step) * joint_delta;
  was_limited = true;
  return true;
}

}  // namespace multilink_copilot

#endif  // MULTILINK_COPILOT_JOINT_COMMAND_CONTINUITY_H
