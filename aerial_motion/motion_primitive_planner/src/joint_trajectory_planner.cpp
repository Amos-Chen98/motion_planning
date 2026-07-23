#include <motion_primitive_planner/joint_trajectory_planner.h>

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/RandomNumbers.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motion_primitive_planner
{
namespace
{
constexpr double kEpsilon = 1e-9;

template <typename Waypoint>
size_t upperWaypointIndex(const std::vector<Waypoint>& waypoints, double time)
{
  const auto upper = std::upper_bound(waypoints.begin(), waypoints.end(), time,
                                      [](double value, const Waypoint& waypoint) {
                                        return value < waypoint.time;
                                      });
  return static_cast<size_t>(std::distance(waypoints.begin(), upper));
}

double interpolateYaw(double start, double goal, double ratio)
{
  return start + ratio * (goal - start);
}
}  // namespace

Eigen::VectorXd JointPlanResult::jointPositions(double time) const
{
  if (joint_waypoints.empty())
  {
    return Eigen::VectorXd();
  }
  if (time <= joint_waypoints.front().time)
  {
    return joint_waypoints.front().positions;
  }
  const size_t upper = upperWaypointIndex(joint_waypoints, time);
  if (upper >= joint_waypoints.size())
  {
    return joint_waypoints.back().positions;
  }
  const TimedJointWaypoint& before = joint_waypoints[upper - 1];
  const TimedJointWaypoint& after = joint_waypoints[upper];
  const double duration = after.time - before.time;
  const double ratio = duration > kEpsilon ? (time - before.time) / duration : 1.0;
  return before.positions + ratio * (after.positions - before.positions);
}

Eigen::VectorXd JointPlanResult::jointVelocities(double time) const
{
  if (joint_waypoints.empty())
  {
    return Eigen::VectorXd();
  }
  const size_t upper = upperWaypointIndex(joint_waypoints, time);
  if (upper == 0 || upper >= joint_waypoints.size())
  {
    return Eigen::VectorXd::Zero(joint_waypoints.front().positions.size());
  }
  const TimedJointWaypoint& before = joint_waypoints[upper - 1];
  const TimedJointWaypoint& after = joint_waypoints[upper];
  const double duration = after.time - before.time;
  if (duration <= kEpsilon)
  {
    return Eigen::VectorXd::Zero(before.positions.size());
  }
  return (after.positions - before.positions) / duration;
}

double JointPlanResult::yaw(double time) const
{
  if (yaw_waypoints.empty())
  {
    return 0.0;
  }
  if (time <= yaw_waypoints.front().time)
  {
    return yaw_waypoints.front().yaw;
  }
  const size_t upper = upperWaypointIndex(yaw_waypoints, time);
  if (upper >= yaw_waypoints.size())
  {
    return yaw_waypoints.back().yaw;
  }
  const TimedYawWaypoint& before = yaw_waypoints[upper - 1];
  const TimedYawWaypoint& after = yaw_waypoints[upper];
  const double duration = after.time - before.time;
  const double ratio = duration > kEpsilon ? (time - before.time) / duration : 1.0;
  return interpolateYaw(before.yaw, after.yaw, ratio);
}

double JointPlanResult::yawRate(double time) const
{
  if (yaw_waypoints.empty())
  {
    return 0.0;
  }
  const size_t upper = upperWaypointIndex(yaw_waypoints, time);
  if (upper == 0 || upper >= yaw_waypoints.size())
  {
    return 0.0;
  }
  const TimedYawWaypoint& before = yaw_waypoints[upper - 1];
  const TimedYawWaypoint& after = yaw_waypoints[upper];
  const double duration = after.time - before.time;
  return duration > kEpsilon ? (after.yaw - before.yaw) / duration : 0.0;
}

JointTrajectoryPlanner::JointTrajectoryPlanner(
    const JointPlannerConfig& config,
    const std::shared_ptr<multilink_copilot::StabilityEvaluator>& stability_evaluator)
  : config_(config), stability_evaluator_(stability_evaluator), nominal_predictor_(config.follower)
{
  if (!stability_evaluator_ || config_.reference_dt <= 0.0 ||
      config_.planning_timeout <= 0.0 || config_.validity_resolution <= 0.0 ||
      config_.max_joint_velocity <= 0.0 || config_.max_joint_command_step <= 0.0 ||
      config_.follower.command_hz <= 0.0)
  {
    throw std::invalid_argument("Invalid joint trajectory planner configuration");
  }
  ompl::RNG::setSeed(config_.random_seed);
}

JointPlanResult JointTrajectoryPlanner::plan(const Trajectory<5>& root_trajectory,
                                             const NominalJointContext& context,
                                             const Eigen::VectorXd& start_joints,
                                             double start_yaw)
{
  JointPlanResult result;
  if (root_trajectory.getPieceNum() <= 0 || start_joints.size() != stability_evaluator_->jointCount() ||
      context.link_num <= 0 || context.link_length <= 0.0)
  {
    result.detail = "invalid root trajectory or DRAGON joint context";
    return result;
  }

  multilink_copilot::StabilityMetrics start_metrics;
  if (!configurationIsSafe(start_joints, start_yaw + M_PI, &start_metrics))
  {
    result.detail = "start joint configuration is not flight-feasible";
    return result;
  }

  const std::vector<NominalSample> nominal = buildNominalSamples(root_trajectory, context, start_joints, start_yaw);
  if (nominal.empty())
  {
    result.detail = "failed to build nominal follow-the-leader joint samples";
    return result;
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(config_.planning_timeout));
  result.joint_waypoints.push_back({0.0, start_joints});
  result.minimum_fc_rp = start_metrics.fc_rp_min;
  double last_anchor_yaw = start_yaw;

  for (size_t index = 1; index < nominal.size(); ++index)
  {
    const bool final_sample = index + 1 == nominal.size();
    if (!nominal[index].safe && !final_sample)
    {
      continue;
    }

    Eigen::VectorXd target = nominal[index].joints;
    if (!nominal[index].safe)
    {
      bool projected = false;
      target = projectedTerminal(target, result.joint_waypoints.back().positions, start_joints, context,
                                 nominal[index].yaw + M_PI, projected);
      if (!projected)
      {
        result.detail = "no stable terminal projection for nominal joint target";
        return result;
      }
    }

    const TimedJointWaypoint goal{nominal[index].time, target};
    if (!appendConnection(result.joint_waypoints.back(), goal, last_anchor_yaw, nominal[index].yaw,
                          deadline, result.joint_waypoints, result.minimum_fc_rp))
    {
      result.detail = "RRT-Connect failed to bridge an unstable nominal joint interval";
      return result;
    }
    last_anchor_yaw = nominal[index].yaw;
  }

  if (result.joint_waypoints.back().time + kEpsilon < root_trajectory.getTotalDuration())
  {
    result.joint_waypoints.push_back(
        {root_trajectory.getTotalDuration(), result.joint_waypoints.back().positions});
  }

  result.yaw_waypoints.reserve(nominal.size());
  for (const NominalSample& sample : nominal)
  {
    result.yaw_waypoints.push_back({sample.time, sample.yaw});
  }

  double time_scale = 1.0;
  result.joint_motion = 0.0;
  for (size_t index = 1; index < result.joint_waypoints.size(); ++index)
  {
    const Eigen::VectorXd delta = result.joint_waypoints[index].positions -
                                  result.joint_waypoints[index - 1].positions;
    result.joint_motion += delta.norm();
    const double maximum_delta = delta.size() > 0 ? delta.cwiseAbs().maxCoeff() : 0.0;
    const double available = result.joint_waypoints[index].time - result.joint_waypoints[index - 1].time;
    // Leave a small scheduling margin because ROS timer callbacks are not
    // perfectly periodic; an exactly saturated analytical step can otherwise
    // exceed the command-step limit when two publications are slightly late.
    constexpr double kCommandStepSchedulingMargin = 0.99;
    const double required = std::max(
        maximum_delta / config_.max_joint_velocity,
        maximum_delta /
            (kCommandStepSchedulingMargin * config_.max_joint_command_step *
             config_.follower.command_hz));
    if (required > kEpsilon && available <= kEpsilon)
    {
      result.detail = "joint detour contains a zero-duration motion";
      return result;
    }
    if (available > kEpsilon)
    {
      time_scale = std::max(time_scale, required / available);
    }
  }

  result.time_scale = std::max(1.0, time_scale);
  for (TimedJointWaypoint& waypoint : result.joint_waypoints)
  {
    waypoint.time *= result.time_scale;
  }
  for (TimedYawWaypoint& waypoint : result.yaw_waypoints)
  {
    waypoint.time *= result.time_scale;
  }
  result.duration = root_trajectory.getTotalDuration() * result.time_scale;
  result.success = true;
  return result;
}

bool JointTrajectoryPlanner::planStableConnection(const Eigen::VectorXd& start,
                                                   const Eigen::VectorXd& goal,
                                                   double root_link_yaw,
                                                   std::vector<Eigen::VectorXd>& path,
                                                   double& minimum_fc_rp,
                                                   std::string* failure_reason)
{
  path.clear();
  minimum_fc_rp = std::numeric_limits<double>::infinity();
  multilink_copilot::StabilityMetrics start_metrics;
  multilink_copilot::StabilityMetrics goal_metrics;
  if (!configurationIsSafe(start, root_link_yaw, &start_metrics) ||
      !configurationIsSafe(goal, root_link_yaw, &goal_metrics))
  {
    if (failure_reason) *failure_reason = "connection endpoint is not flight-feasible";
    return false;
  }
  minimum_fc_rp = std::min(start_metrics.fc_rp_min, goal_metrics.fc_rp_min);
  double direct_minimum = minimum_fc_rp;
  if (directEdgeIsSafe(start, goal, root_link_yaw, root_link_yaw, direct_minimum))
  {
    path = {start, goal};
    minimum_fc_rp = direct_minimum;
    return true;
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(config_.planning_timeout));
  std::vector<Eigen::VectorXd> raw_path;
  if (!searchJointDetour(start, goal, root_link_yaw, deadline, raw_path) || raw_path.size() < 2)
  {
    if (failure_reason) *failure_reason = "RRT-Connect did not find a stable joint connection";
    return false;
  }

  path.push_back(raw_path.front());
  size_t current = 0;
  while (current + 1 < raw_path.size())
  {
    size_t next = raw_path.size() - 1;
    for (; next > current + 1; --next)
    {
      double ignored = std::numeric_limits<double>::infinity();
      if (directEdgeIsSafe(raw_path[current], raw_path[next], root_link_yaw, root_link_yaw, ignored))
      {
        break;
      }
    }
    path.push_back(raw_path[next]);
    current = next;
  }
  for (size_t index = 1; index < path.size(); ++index)
  {
    double edge_minimum = minimum_fc_rp;
    if (!directEdgeIsSafe(path[index - 1], path[index], root_link_yaw, root_link_yaw, edge_minimum))
    {
      path.clear();
      if (failure_reason) *failure_reason = "shortcut failed full-resolution stability validation";
      return false;
    }
    minimum_fc_rp = std::min(minimum_fc_rp, edge_minimum);
  }
  return true;
}

std::vector<JointTrajectoryPlanner::NominalSample> JointTrajectoryPlanner::buildNominalSamples(
    const Trajectory<5>& root_trajectory,
    const NominalJointContext& context,
    const Eigen::VectorXd& start_joints,
    double start_yaw)
{
  std::vector<NominalSample> samples;
  const std::vector<NominalJointSample> nominal = nominal_predictor_.predict(
      root_trajectory, context, start_joints, start_yaw, config_.reference_dt);
  samples.reserve(nominal.size());
  for (const NominalJointSample& sample : nominal)
  {
    multilink_copilot::StabilityMetrics metrics;
    const bool safe = configurationIsSafe(sample.joints, sample.yaw + M_PI, &metrics);
    samples.push_back({sample.time, sample.yaw, sample.joints, safe});
  }
  return samples;
}

bool JointTrajectoryPlanner::appendConnection(
    const TimedJointWaypoint& start,
    const TimedJointWaypoint& goal,
    double start_yaw,
    double goal_yaw,
    const std::chrono::steady_clock::time_point& deadline,
    std::vector<TimedJointWaypoint>& path,
    double& minimum_fc_rp)
{
  double direct_minimum = minimum_fc_rp;
  if (directEdgeIsSafe(start.positions, goal.positions, start_yaw + M_PI, goal_yaw + M_PI,
                       direct_minimum))
  {
    path.push_back(goal);
    minimum_fc_rp = std::min(minimum_fc_rp, direct_minimum);
    return true;
  }

  std::vector<Eigen::VectorXd> detour;
  if (!searchJointDetour(start.positions, goal.positions, goal_yaw + M_PI, deadline, detour) ||
      detour.size() < 2)
  {
    return false;
  }

  std::vector<Eigen::VectorXd> shortcut;
  shortcut.push_back(detour.front());
  size_t current = 0;
  while (current + 1 < detour.size())
  {
    size_t next = detour.size() - 1;
    for (; next > current + 1; --next)
    {
      double ignored_minimum = std::numeric_limits<double>::infinity();
      if (directEdgeIsSafe(detour[current], detour[next], goal_yaw + M_PI, goal_yaw + M_PI,
                           ignored_minimum))
      {
        break;
      }
    }
    shortcut.push_back(detour[next]);
    current = next;
  }

  std::vector<double> cumulative(shortcut.size(), 0.0);
  for (size_t index = 1; index < shortcut.size(); ++index)
  {
    cumulative[index] = cumulative[index - 1] + (shortcut[index] - shortcut[index - 1]).norm();
  }
  const double total = cumulative.back();
  for (size_t index = 1; index < shortcut.size(); ++index)
  {
    const double ratio = total > kEpsilon ? cumulative[index] / total :
                                           static_cast<double>(index) / (shortcut.size() - 1);
    const double waypoint_yaw = interpolateYaw(start_yaw, goal_yaw, ratio);
    double edge_minimum = minimum_fc_rp;
    if (!directEdgeIsSafe(shortcut[index - 1], shortcut[index],
                          interpolateYaw(start_yaw, goal_yaw,
                                         total > kEpsilon ? cumulative[index - 1] / total : 0.0) + M_PI,
                          waypoint_yaw + M_PI, edge_minimum))
    {
      return false;
    }
    minimum_fc_rp = std::min(minimum_fc_rp, edge_minimum);
    path.push_back({start.time + ratio * (goal.time - start.time), shortcut[index]});
  }
  path.back().time = goal.time;
  return true;
}

bool JointTrajectoryPlanner::directEdgeIsSafe(const Eigen::VectorXd& start,
                                              const Eigen::VectorXd& goal,
                                              double start_yaw,
                                              double goal_yaw,
                                              double& minimum_fc_rp)
{
  if (start.size() != goal.size())
  {
    return false;
  }
  const double maximum_delta = start.size() > 0 ? (goal - start).cwiseAbs().maxCoeff() : 0.0;
  const int subdivisions = std::max(1, static_cast<int>(std::ceil(maximum_delta / config_.validity_resolution)));
  for (int index = 0; index <= subdivisions; ++index)
  {
    const double ratio = static_cast<double>(index) / subdivisions;
    const Eigen::VectorXd joints = start + ratio * (goal - start);
    multilink_copilot::StabilityMetrics metrics;
    if (!configurationIsSafe(joints, interpolateYaw(start_yaw, goal_yaw, ratio), &metrics))
    {
      return false;
    }
    minimum_fc_rp = std::min(minimum_fc_rp, metrics.fc_rp_min);
  }
  return true;
}

bool JointTrajectoryPlanner::searchJointDetour(
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    double yaw,
    const std::chrono::steady_clock::time_point& deadline,
    std::vector<Eigen::VectorXd>& path)
{
  path.clear();
  const double remaining = std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
  if (remaining <= 0.0)
  {
    return false;
  }

  auto space = std::make_shared<ompl::base::RealVectorStateSpace>(start.size());
  ompl::base::RealVectorBounds bounds(start.size());
  const std::vector<double>& lower = stability_evaluator_->robotModel()->getLinkJointLowerLimits();
  const std::vector<double>& upper = stability_evaluator_->robotModel()->getLinkJointUpperLimits();
  for (int index = 0; index < start.size(); ++index)
  {
    bounds.setLow(index, lower[static_cast<size_t>(index)]);
    bounds.setHigh(index, upper[static_cast<size_t>(index)]);
  }
  space->setBounds(bounds);

  ompl::geometric::SimpleSetup setup(space);
  setup.setStateValidityChecker([this, yaw](const ompl::base::State* state) {
    const auto* vector_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
    Eigen::VectorXd joints(stability_evaluator_->jointCount());
    for (int index = 0; index < joints.size(); ++index)
    {
      joints(index) = vector_state->values[index];
    }
    return configurationIsSafe(joints, yaw);
  });
  const double extent = std::max(space->getMaximumExtent(), config_.validity_resolution);
  setup.getSpaceInformation()->setStateValidityCheckingResolution(config_.validity_resolution / extent);

  ompl::base::ScopedState<> start_state(space);
  ompl::base::ScopedState<> goal_state(space);
  for (int index = 0; index < start.size(); ++index)
  {
    start_state[index] = start(index);
    goal_state[index] = goal(index);
  }
  setup.setStartAndGoalStates(start_state, goal_state);
  auto planner = std::make_shared<ompl::geometric::RRTConnect>(setup.getSpaceInformation());
  planner->setRange(std::max(0.1, 4.0 * config_.validity_resolution));
  setup.setPlanner(planner);
  const ompl::base::PlannerStatus status = setup.solve(remaining);
  if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
  {
    return false;
  }

  const ompl::geometric::PathGeometric& solution = setup.getSolutionPath();
  path.reserve(solution.getStateCount());
  for (size_t state_index = 0; state_index < solution.getStateCount(); ++state_index)
  {
    const auto* state = solution.getState(state_index)->as<ompl::base::RealVectorStateSpace::StateType>();
    Eigen::VectorXd joints(start.size());
    for (int joint_index = 0; joint_index < joints.size(); ++joint_index)
    {
      joints(joint_index) = state->values[joint_index];
    }
    path.push_back(joints);
  }
  return true;
}

bool JointTrajectoryPlanner::configurationIsSafe(const Eigen::VectorXd& joints, double yaw,
                                                 multilink_copilot::StabilityMetrics* metrics)
{
  stability_evaluator_->setRootLinkRotation(KDL::Rotation::RotZ(yaw));
  multilink_copilot::StabilityMetrics evaluated;
  const bool valid = stability_evaluator_->evaluate(joints, evaluated) && evaluated.safe;
  if (metrics)
  {
    *metrics = evaluated;
  }
  return valid;
}

Eigen::VectorXd JointTrajectoryPlanner::projectedTerminal(const Eigen::VectorXd& desired,
                                                          const Eigen::VectorXd& current,
                                                          const Eigen::VectorXd& start,
                                                          const NominalJointContext& context,
                                                          double yaw,
                                                          bool& success)
{
  Eigen::VectorXd positive_fold = Eigen::VectorXd::Zero(desired.size());
  Eigen::VectorXd negative_fold = Eigen::VectorXd::Zero(desired.size());
  for (const int yaw_index : context.yaw_joint_indices)
  {
    if (yaw_index >= 0 && yaw_index < desired.size())
    {
      positive_fold(yaw_index) = M_PI_2;
      negative_fold(yaw_index) = -M_PI_2;
    }
  }
  stability_evaluator_->setRootLinkRotation(KDL::Rotation::RotZ(yaw));
  Eigen::VectorXd projected;
  success = stability_evaluator_->projectNearestSafe(
      desired, {current, start, positive_fold, negative_fold}, projected);
  return success ? projected : Eigen::VectorXd();
}

}  // namespace motion_primitive_planner
