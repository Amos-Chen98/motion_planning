// -*- mode: c++ -*-
#ifndef MOTION_PRIMITIVE_PLANNER_WHOLE_BODY_PLANNER_NODE_H
#define MOTION_PRIMITIVE_PLANNER_WHOLE_BODY_PLANNER_NODE_H

#include <motion_primitive_planner/root_primitive_generator.h>

#include <std_msgs/ColorRGBA.h>

#include <cmath>
#include <stdexcept>

namespace motion_primitive_planner
{

class TrajectoryReplanTrigger
{
public:
  explicit TrajectoryReplanTrigger(double trigger_ratio = 0.5);

  void arm(double start_time, double duration, bool terminal);
  void reset();
  bool shouldTrigger(double current_time);

  double triggerRatio() const { return trigger_ratio_; }
  bool armed() const { return armed_; }
  bool triggered() const { return triggered_; }
  bool terminal() const { return terminal_; }

private:
  double trigger_ratio_ = 0.5;
  double start_time_ = 0.0;
  double duration_ = 0.0;
  bool armed_ = false;
  bool triggered_ = false;
  bool terminal_ = false;
};

inline TrajectoryReplanTrigger::TrajectoryReplanTrigger(double trigger_ratio)
  : trigger_ratio_(trigger_ratio)
{
  if (!std::isfinite(trigger_ratio_) || trigger_ratio_ <= 0.0 || trigger_ratio_ >= 1.0)
  {
    throw std::invalid_argument("Replan trigger ratio must be finite and inside (0, 1)");
  }
}

inline void TrajectoryReplanTrigger::arm(double start_time, double duration, bool terminal)
{
  if (!std::isfinite(start_time) || !std::isfinite(duration) || duration <= 0.0)
  {
    throw std::invalid_argument("Invalid trajectory replan trigger interval");
  }
  start_time_ = start_time;
  duration_ = duration;
  terminal_ = terminal;
  triggered_ = false;
  armed_ = !terminal;
}

inline void TrajectoryReplanTrigger::reset()
{
  start_time_ = 0.0;
  duration_ = 0.0;
  armed_ = false;
  triggered_ = false;
  terminal_ = false;
}

inline bool TrajectoryReplanTrigger::shouldTrigger(double current_time)
{
  if (!armed_ || triggered_ || !std::isfinite(current_time))
  {
    return false;
  }
  const double trigger_time = start_time_ + trigger_ratio_ * duration_;
  if (current_time + 1e-6 < trigger_time)
  {
    return false;
  }
  triggered_ = true;
  return true;
}

inline std_msgs::ColorRGBA candidateColor(CandidateStatus status, bool selected)
{
  std_msgs::ColorRGBA color;
  color.a = 0.9;
  if (selected || status == CandidateStatus::kSelected)
  {
    color.g = 1.0;
  }
  else if (status == CandidateStatus::kFeasible)
  {
    color.g = 1.0;
    color.b = 1.0;
  }
  else if (status == CandidateStatus::kJointPlanningFailed)
  {
    color.r = 1.0;
    color.g = 0.5;
  }
  else
  {
    color.r = 1.0;
  }
  return color;
}

}  // namespace motion_primitive_planner

#endif  // MOTION_PRIMITIVE_PLANNER_WHOLE_BODY_PLANNER_NODE_H
