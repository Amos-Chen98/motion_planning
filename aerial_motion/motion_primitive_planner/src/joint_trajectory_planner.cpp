#include <motion_primitive_planner/joint_trajectory_planner.h>
#include <motion_primitive_planner/planner_core.h>

#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/RandomNumbers.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <random>
#include <stdexcept>
#include <utility>

namespace motion_primitive_planner
{
namespace
{
constexpr double kEpsilon = 1e-9;
constexpr double kInfinity = std::numeric_limits<double>::infinity();
//! Resolution relaxation used by the sampling-based search and its shortcut,
//! whose output is always re-validated at the configured resolution.
constexpr double kSearchResolutionFactor = 4.0;

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
  return start + ratio * shortestYawDelta(start, goal);
}

std::chrono::steady_clock::duration toDuration(double seconds)
{
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(seconds));
}

//! Normalized distances of the chain vertices along the chain, used both for
//! root-yaw interpolation and for time allocation.
std::vector<double> chainRatios(const std::vector<Eigen::VectorXd>& chain)
{
  std::vector<double> ratios(chain.size(), 0.0);
  double total = 0.0;
  for (size_t index = 1; index < chain.size(); ++index)
  {
    total += (chain[index] - chain[index - 1]).norm();
    ratios[index] = total;
  }
  for (size_t index = 1; index < chain.size(); ++index)
  {
    ratios[index] = total > kEpsilon ? ratios[index] / total :
                                       static_cast<double>(index) / (chain.size() - 1);
  }
  return ratios;
}

//! Per-planner seeded implementation of OMPL's standard box-uniform sampler.
//! Keeping the generator local makes candidate-specific seeds reproducible
//! without introducing any fold-, endpoint-, or zero-configuration bias.
class SeededUniformSampler : public ompl::base::StateSampler
{
public:
  SeededUniformSampler(const ompl::base::StateSpace* space, unsigned int seed)
    : ompl::base::StateSampler(space)
    , bounds_(space->as<ompl::base::RealVectorStateSpace>()->getBounds())
    , generator_(seed)
  {
  }

  void sampleUniform(ompl::base::State* state) override
  {
    double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    for (size_t index = 0; index < bounds_.low.size(); ++index)
    {
      std::uniform_real_distribution<double> uniform(bounds_.low[index], bounds_.high[index]);
      values[index] = uniform(generator_);
    }
  }

  void sampleUniformNear(ompl::base::State* state, const ompl::base::State* near,
                         double distance) override
  {
    const double* center = near->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    for (size_t index = 0; index < bounds_.low.size(); ++index)
    {
      const double low = std::max(bounds_.low[index], center[index] - distance);
      const double high = std::min(bounds_.high[index], center[index] + distance);
      std::uniform_real_distribution<double> uniform(low, high);
      values[index] = uniform(generator_);
    }
  }

  void sampleGaussian(ompl::base::State* state, const ompl::base::State* mean,
                      double std_deviation) override
  {
    const double* center = mean->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    std::normal_distribution<double> normal(0.0, std_deviation);
    for (size_t index = 0; index < bounds_.low.size(); ++index)
    {
      values[index] = clamp(index, center[index] + normal(generator_));
    }
  }

private:
  double clamp(size_t index, double value) const
  {
    return std::max(bounds_.low[index], std::min(bounds_.high[index], value));
  }

  ompl::base::RealVectorBounds bounds_;
  std::mt19937 generator_;
};

void seedOmplOnce(unsigned int seed)
{
  // OMPL refuses to reseed once its global generator has been used, and every
  // additional attempt logs an error.  Seed it exactly once per process.
  static std::once_flag flag;
  std::call_once(flag, [seed]() { ompl::RNG::setSeed(seed); });
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
  return duration > kEpsilon ? shortestYawDelta(before.yaw, after.yaw) / duration : 0.0;
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
  seedOmplOnce(config_.random_seed);
}

JointPlanResult JointTrajectoryPlanner::plan(const Trajectory<5>& root_trajectory,
                                             const NominalJointContext& context,
                                             const Eigen::VectorXd& start_joints,
                                             double start_yaw,
                                             double time_budget)
{
  JointPlanResult result;
  if (root_trajectory.getPieceNum() <= 0 || start_joints.size() != stability_evaluator_->jointCount() ||
      context.link_num <= 0 || context.link_length <= 0.0)
  {
    result.detail = "invalid root trajectory or DRAGON joint context";
    return result;
  }

  const double budget = time_budget > 0.0 ? std::min(time_budget, config_.planning_timeout) :
                                            config_.planning_timeout;
  deadline_ = Clock::now() + toDuration(budget);
  const Clock::time_point deadline = deadline_;

  Eigen::VectorXd origin = start_joints;
  multilink_copilot::StabilityMetrics origin_metrics;
  bool repaired_start = false;
  if (!configurationIsSafe(origin, start_yaw + M_PI, &origin_metrics))
  {
    if (!repairEndpoint(start_joints, start_joints, start_yaw + M_PI, true, origin) ||
        !configurationIsSafe(origin, start_yaw + M_PI, &origin_metrics))
    {
      result.detail = "start joint configuration could not be repaired";
      return result;
    }
    repaired_start = true;
  }

  // Re-seed follow-the-leader with the safe RRT origin. This makes the nominal
  // terminal morphology consistent with the state from which this candidate is
  // actually planned.
  const std::vector<NominalSample> nominal =
      buildNominalSamples(root_trajectory, context, origin, start_yaw);
  if (nominal.size() < 2)
  {
    result.detail = "failed to build nominal follow-the-leader joint samples";
    return result;
  }

  const NominalSample& terminal = nominal.back();
  Eigen::VectorXd goal = terminal.joints;
  multilink_copilot::StabilityMetrics goal_metrics;
  bool repaired_goal = false;
  if (!configurationIsSafe(goal, terminal.yaw + M_PI, &goal_metrics))
  {
    if (!repairEndpoint(terminal.joints, origin, terminal.yaw + M_PI, false, goal) ||
        !configurationIsSafe(goal, terminal.yaw + M_PI, &goal_metrics))
    {
      result.detail = "terminal joint configuration could not be repaired";
      return result;
    }
    repaired_goal = true;
  }

  // One global RRT-Connect replaces safe-run analysis and all local bridge
  // heuristics. The fixed-yaw search is deliberately followed by the
  // authoritative validation against the full time-varying yaw schedule below.
  std::vector<Eigen::VectorXd> sampled_path;
  if (!searchJointPath(origin, goal, terminal.yaw + M_PI, deadline, sampled_path) ||
      sampled_path.size() < 2)
  {
    result.detail = "global joint-space RRT failed";
    return result;
  }
  std::vector<Eigen::VectorXd> joint_path =
      shortcutChain(sampled_path, terminal.yaw + M_PI);
  if (joint_path.size() < 2 || budgetExpired())
  {
    result.detail = "global joint-space RRT shortcut failed";
    return result;
  }
  joint_path.front() = origin;
  joint_path.back() = goal;

  std::vector<Eigen::VectorXd> output_path;
  output_path.reserve(joint_path.size() + (repaired_start ? 1u : 0u));
  if (repaired_start)
  {
    // Keep command continuity at the measured state. This recovery prefix is
    // intentionally outside the strict flight-feasibility guarantee, but remains
    // part of the final whole-body environmental collision sweep.
    output_path.push_back(start_joints);
  }
  output_path.insert(output_path.end(), joint_path.begin(), joint_path.end());
  const std::vector<double> path_ratios = chainRatios(output_path);
  const double root_duration = root_trajectory.getTotalDuration();
  result.joint_waypoints.reserve(output_path.size());
  for (size_t index = 0; index < output_path.size(); ++index)
  {
    result.joint_waypoints.push_back({path_ratios[index] * root_duration, output_path[index]});
  }
  result.joint_waypoints.front().time = 0.0;
  result.joint_waypoints.back().time = root_duration;

  result.yaw_waypoints.reserve(nominal.size());
  for (const NominalSample& sample : nominal)
  {
    result.yaw_waypoints.push_back({sample.time, sample.yaw});
  }

  result.minimum_fc_rp = std::min(origin_metrics.fc_rp_min, goal_metrics.fc_rp_min);
  const size_t safe_origin_index = repaired_start ? 1u : 0u;
  multilink_copilot::StabilityMetrics timed_origin_metrics;
  if (safe_origin_index >= result.joint_waypoints.size() ||
      !configurationIsSafe(
          result.joint_waypoints[safe_origin_index].positions,
          nominalYawAt(nominal, result.joint_waypoints[safe_origin_index].time) + M_PI,
          &timed_origin_metrics))
  {
    result.detail = "repaired joint origin is infeasible under the root-yaw schedule";
    return result;
  }
  result.minimum_fc_rp =
      std::min(result.minimum_fc_rp, timed_origin_metrics.fc_rp_min);
  for (size_t index = safe_origin_index + 1;
       index < result.joint_waypoints.size(); ++index)
  {
    if (!timedConfigurationPathIsSafe(result.joint_waypoints[index - 1],
                                      result.joint_waypoints[index], nominal,
                                      result.minimum_fc_rp))
    {
      result.detail =
          "global joint path is infeasible under the root-yaw schedule";
      return result;
    }
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
      result.detail = "joint path contains a zero-duration motion";
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
  if (!computeTrackingError(root_trajectory, context, nominal, result) ||
      budgetExpired())
  {
    result.detail = budgetExpired() ?
                        "joint planning deadline expired during tracking evaluation" :
                        "failed to evaluate joint-path tracking error";
    return result;
  }
  if (repaired_start && repaired_goal)
  {
    result.detail = "repaired start and terminal joint configurations";
  }
  else if (repaired_start)
  {
    result.detail = "repaired start joint configuration";
  }
  else if (repaired_goal)
  {
    result.detail = "repaired terminal joint configuration";
  }
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
  minimum_fc_rp = kInfinity;
  deadline_ = Clock::now() + toDuration(config_.planning_timeout);
  multilink_copilot::StabilityMetrics start_metrics;
  multilink_copilot::StabilityMetrics goal_metrics;
  if (!configurationIsSafe(start, root_link_yaw, &start_metrics) ||
      !configurationIsSafe(goal, root_link_yaw, &goal_metrics))
  {
    if (failure_reason) *failure_reason = "connection endpoint is not flight-feasible";
    return false;
  }
  minimum_fc_rp = std::min(start_metrics.fc_rp_min, goal_metrics.fc_rp_min);

  std::vector<Eigen::VectorXd> sampled_path;
  if (!searchJointPath(start, goal, root_link_yaw, deadline_, sampled_path) ||
      sampled_path.size() < 2)
  {
    if (failure_reason) *failure_reason = "global joint-space RRT failed";
    return false;
  }

  path = shortcutChain(sampled_path, root_link_yaw);
  if (path.size() < 2 || budgetExpired())
  {
    path.clear();
    if (failure_reason) *failure_reason = "global joint-space RRT shortcut failed";
    return false;
  }
  path.front() = start;
  path.back() = goal;

  double path_minimum = minimum_fc_rp;
  if (!chainIsSafe(path, root_link_yaw, root_link_yaw, path_minimum))
  {
    path.clear();
    if (failure_reason) *failure_reason = "global joint-space RRT path failed full validation";
    return false;
  }
  minimum_fc_rp = path_minimum;
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
    samples.push_back({sample.time, sample.yaw, sample.joints});
  }
  return samples;
}

bool JointTrajectoryPlanner::chainIsSafe(const std::vector<Eigen::VectorXd>& chain,
                                         double start_yaw, double goal_yaw,
                                         double& minimum_fc_rp)
{
  if (chain.size() < 2)
  {
    return false;
  }
  const std::vector<double> ratios = chainRatios(chain);
  // Vertices first: one evaluation each rejects most hopeless chains before the
  // far more expensive dense edge validation runs.
  for (size_t index = 0; index < chain.size(); ++index)
  {
    multilink_copilot::StabilityMetrics metrics;
    if (budgetExpired() ||
        !configurationIsSafe(chain[index], interpolateYaw(start_yaw, goal_yaw, ratios[index]),
                             &metrics))
    {
      return false;
    }
    minimum_fc_rp = std::min(minimum_fc_rp, metrics.fc_rp_min);
  }
  for (size_t index = 1; index < chain.size(); ++index)
  {
    double edge_minimum = kInfinity;
    if (!edgeInteriorIsSafe(chain[index - 1], chain[index],
                            interpolateYaw(start_yaw, goal_yaw, ratios[index - 1]),
                            interpolateYaw(start_yaw, goal_yaw, ratios[index]),
                            config_.validity_resolution, edge_minimum))
    {
      return false;
    }
    minimum_fc_rp = std::min(minimum_fc_rp, edge_minimum);
  }
  return true;
}

std::vector<Eigen::VectorXd> JointTrajectoryPlanner::shortcutChain(
    const std::vector<Eigen::VectorXd>& chain, double root_yaw)
{
  std::vector<Eigen::VectorXd> shortcut;
  if (chain.empty())
  {
    return shortcut;
  }
  const double coarse_resolution = kSearchResolutionFactor * config_.validity_resolution;
  shortcut.push_back(chain.front());
  size_t current = 0;
  while (current + 1 < chain.size())
  {
    size_t next = chain.size() - 1;
    for (; next > current + 1 && !budgetExpired(); --next)
    {
      double ignored = kInfinity;
      if (edgeInteriorIsSafe(chain[current], chain[next], root_yaw, root_yaw, coarse_resolution,
                             ignored))
      {
        break;
      }
    }
    shortcut.push_back(chain[next]);
    current = next;
  }
  return shortcut;
}

bool JointTrajectoryPlanner::edgeInteriorIsSafe(const Eigen::VectorXd& start,
                                                const Eigen::VectorXd& goal,
                                                double start_yaw,
                                                double goal_yaw,
                                                double resolution,
                                                double& minimum_fc_rp)
{
  if (start.size() != goal.size() || resolution <= 0.0)
  {
    return false;
  }
  const double maximum_delta = start.size() > 0 ? (goal - start).cwiseAbs().maxCoeff() : 0.0;
  const int subdivisions = std::max(1, static_cast<int>(std::ceil(maximum_delta / resolution)));
  for (int index = 1; index < subdivisions; ++index)
  {
    if (budgetExpired())
    {
      return false;
    }
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

bool JointTrajectoryPlanner::budgetExpired() const
{
  return Clock::now() >= deadline_;
}

bool JointTrajectoryPlanner::searchJointPath(
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    double yaw,
    const Clock::time_point& deadline,
    std::vector<Eigen::VectorXd>& path)
{
  path.clear();
  const double remaining = std::chrono::duration<double>(deadline - Clock::now()).count();
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

  const unsigned int seed = config_.random_seed;
  space->setStateSamplerAllocator(
      [seed](const ompl::base::StateSpace* state_space) -> ompl::base::StateSamplerPtr {
        return std::make_shared<SeededUniformSampler>(state_space, seed);
      });

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
  // The returned path is re-validated at the full resolution, so the search itself
  // can afford the coarser motion check that keeps it inside the online budget.
  const double extent = std::max(space->getMaximumExtent(), config_.validity_resolution);
  setup.getSpaceInformation()->setStateValidityCheckingResolution(
      std::min(0.5, kSearchResolutionFactor * config_.validity_resolution / extent));

  ompl::base::ScopedState<> start_state(space);
  ompl::base::ScopedState<> goal_state(space);
  for (int index = 0; index < start.size(); ++index)
  {
    start_state[index] = start(index);
    goal_state[index] = goal(index);
  }
  setup.setStartAndGoalStates(start_state, goal_state);
  auto planner = std::make_shared<ompl::geometric::RRTConnect>(setup.getSpaceInformation());
  setup.setPlanner(planner);
  const double solve_time =
      std::chrono::duration<double>(deadline - Clock::now()).count();
  if (solve_time <= 0.0)
  {
    return false;
  }
  const ompl::base::PlannerStatus status =
      setup.solve(solve_time);
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

double JointTrajectoryPlanner::nominalYawAt(const std::vector<NominalSample>& samples,
                                            double time)
{
  if (samples.empty())
  {
    return 0.0;
  }
  if (time <= samples.front().time)
  {
    return samples.front().yaw;
  }
  const size_t upper = upperWaypointIndex(samples, time);
  if (upper >= samples.size())
  {
    return samples.back().yaw;
  }
  const NominalSample& before = samples[upper - 1];
  const NominalSample& after = samples[upper];
  const double duration = after.time - before.time;
  const double ratio = duration > kEpsilon ? (time - before.time) / duration : 1.0;
  return interpolateYaw(before.yaw, after.yaw, ratio);
}

bool JointTrajectoryPlanner::timedConfigurationPathIsSafe(
    const TimedJointWaypoint& start, const TimedJointWaypoint& goal,
    const std::vector<NominalSample>& nominal, double& minimum_fc_rp)
{
  if (start.positions.size() != goal.positions.size() || nominal.empty() ||
      !start.positions.allFinite() || !goal.positions.allFinite() ||
      !std::isfinite(start.time) || !std::isfinite(goal.time) ||
      goal.time + kEpsilon < start.time)
  {
    return false;
  }

  std::vector<double> times{start.time, goal.time};
  const double duration = goal.time - start.time;
  const double maximum_delta =
      start.positions.size() > 0 ?
          (goal.positions - start.positions).cwiseAbs().maxCoeff() : 0.0;
  const int joint_subdivisions =
      std::max(1, static_cast<int>(std::ceil(maximum_delta / config_.validity_resolution)));
  for (int subdivision = 1; subdivision < joint_subdivisions; ++subdivision)
  {
    times.push_back(start.time + duration * static_cast<double>(subdivision) /
                                     joint_subdivisions);
  }

  // The nominal yaw is piecewise linear. Add enough samples on every overlapping
  // yaw segment that neither joint motion nor root-yaw motion can step over a
  // thin infeasible shell.
  for (size_t index = 1; index < nominal.size(); ++index)
  {
    const double interval_start = std::max(start.time, nominal[index - 1].time);
    const double interval_end = std::min(goal.time, nominal[index].time);
    if (interval_end <= interval_start + kEpsilon)
    {
      continue;
    }
    const double yaw_start = nominalYawAt(nominal, interval_start);
    const double yaw_end = nominalYawAt(nominal, interval_end);
    const int yaw_subdivisions = std::max(
        1, static_cast<int>(std::ceil(
               std::abs(shortestYawDelta(yaw_start, yaw_end)) /
               config_.validity_resolution)));
    for (int subdivision = 0; subdivision <= yaw_subdivisions; ++subdivision)
    {
      times.push_back(interval_start +
                      (interval_end - interval_start) *
                          static_cast<double>(subdivision) / yaw_subdivisions);
    }
  }

  std::sort(times.begin(), times.end());
  times.erase(std::unique(times.begin(), times.end(),
                          [](double lhs, double rhs) {
                            return std::abs(lhs - rhs) <= kEpsilon;
                          }),
              times.end());
  double path_minimum = kInfinity;
  for (const double time : times)
  {
    if (budgetExpired())
    {
      return false;
    }
    const double ratio = duration > kEpsilon ? (time - start.time) / duration : 1.0;
    const Eigen::VectorXd joints =
        start.positions + ratio * (goal.positions - start.positions);
    multilink_copilot::StabilityMetrics metrics;
    if (!configurationIsSafe(joints, nominalYawAt(nominal, time) + M_PI, &metrics))
    {
      return false;
    }
    path_minimum = std::min(path_minimum, metrics.fc_rp_min);
  }
  minimum_fc_rp = std::min(minimum_fc_rp, path_minimum);
  return true;
}

bool JointTrajectoryPlanner::computeTrackingError(
    const Trajectory<5>& root_trajectory, const NominalJointContext& context,
    const std::vector<NominalSample>& nominal, JointPlanResult& result) const
{
  result.tracking_error_rms = 0.0;
  result.tracking_error_max = 0.0;
  if (budgetExpired())
  {
    return false;
  }
  const int downstream_link_count = context.link_num - 1;
  if (!std::isfinite(result.duration) || !std::isfinite(result.time_scale) ||
      result.duration <= kEpsilon || result.time_scale <= 0.0 ||
      downstream_link_count <= 0 || nominal.empty())
  {
    return true;
  }

  const double required_history =
      static_cast<double>(downstream_link_count) * context.link_length;
  const Eigen::Vector3d initial_root_tail = root_trajectory.getPos(0.0);
  const Eigen::Matrix3d initial_root_rotation =
      Eigen::AngleAxisd(nominal.front().yaw + M_PI,
                        Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const std::deque<multilink_copilot::TrajectoryPoint> initial_trace =
      context.executed_history.arcLength() < required_history
          ? multilink_copilot::follow_the_leader::prependCurrentBodyMorphology(
                context.executed_history.points(), initial_root_tail,
                initial_root_rotation, nominal.front().joints,
                context.pitch_joint_indices, context.yaw_joint_indices,
                context.link_num, context.link_length)
          : context.executed_history.points();
  std::vector<Eigen::Vector3d> trace;
  trace.reserve(initial_trace.size() +
                static_cast<size_t>(std::ceil(result.duration *
                                              config_.follower.command_hz)) + 1);
  for (const multilink_copilot::TrajectoryPoint& point : initial_trace)
  {
    trace.push_back(point.position);
  }
  if (trace.empty() || (trace.back() - initial_root_tail).norm() > kEpsilon)
  {
    trace.push_back(initial_root_tail);
  }
  const auto distanceToTrace = [](const Eigen::Vector3d& point,
                                  const std::vector<Eigen::Vector3d>& polyline) {
    if (polyline.empty())
    {
      return kInfinity;
    }
    double minimum = (point - polyline.front()).norm();
    for (size_t index = 1; index < polyline.size(); ++index)
    {
      const Eigen::Vector3d start = polyline[index - 1];
      const Eigen::Vector3d delta = polyline[index] - start;
      const double length_squared = delta.squaredNorm();
      const double ratio =
          length_squared > kEpsilon * kEpsilon
              ? std::max(0.0, std::min(1.0,
                  (point - start).dot(delta) / length_squared))
              : 0.0;
      minimum = std::min(minimum, (point - (start + ratio * delta)).norm());
    }
    return minimum;
  };

  const double command_dt = 1.0 / config_.follower.command_hz;
  const int sample_count =
      std::max(1, static_cast<int>(std::ceil(result.duration / command_dt)));
  const double sample_dt = result.duration / sample_count;
  double squared_error_integral = 0.0;
  double previous_mean_squared_error = 0.0;
  for (int sample = 0; sample <= sample_count; ++sample)
  {
    if (budgetExpired())
    {
      return false;
    }
    const double scaled_time =
        result.duration * static_cast<double>(sample) / sample_count;
    const double nominal_time = scaled_time / result.time_scale;
    const Eigen::Vector3d root_position = root_trajectory.getPos(nominal_time);
    if ((trace.back() - root_position).norm() > kEpsilon)
    {
      trace.push_back(root_position);
    }
    const Eigen::Matrix3d root_rotation =
        Eigen::AngleAxisd(result.yaw(scaled_time) + M_PI,
                          Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const std::vector<Eigen::Vector3d> actual_endpoints = linkEndpoints(
        root_position, root_rotation, result.jointPositions(scaled_time),
        context.pitch_joint_indices, context.yaw_joint_indices,
        context.link_num, context.link_length);
    if (actual_endpoints.size() < static_cast<size_t>(context.link_num + 1))
    {
      result.tracking_error_rms = kInfinity;
      result.tracking_error_max = kInfinity;
      return false;
    }

    double mean_squared_error = 0.0;
    for (int link = 2; link <= context.link_num; ++link)
    {
      const double error = distanceToTrace(
          actual_endpoints[static_cast<size_t>(link)], trace);
      mean_squared_error += error * error;
      result.tracking_error_max = std::max(result.tracking_error_max, error);
    }
    mean_squared_error /= downstream_link_count;
    if (sample > 0)
    {
      squared_error_integral +=
          0.5 * (previous_mean_squared_error + mean_squared_error) * sample_dt;
    }
    previous_mean_squared_error = mean_squared_error;
  }
  result.tracking_error_rms =
      std::sqrt(std::max(0.0, squared_error_integral / result.duration));
  return true;
}

bool JointTrajectoryPlanner::repairEndpoint(const Eigen::VectorXd& desired,
                                            const Eigen::VectorXd& reference,
                                            double yaw,
                                            bool allow_unstable_seed,
                                            Eigen::VectorXd& repaired)
{
  repaired.resize(0);
  if (desired.size() != stability_evaluator_->jointCount() ||
      reference.size() != desired.size() || !desired.allFinite() ||
      !reference.allFinite())
  {
    return false;
  }
  if (configurationIsSafe(desired, yaw))
  {
    repaired = desired;
    return true;
  }

  stability_evaluator_->setRootLinkRotation(KDL::Rotation::RotZ(yaw));
  return stability_evaluator_->projectToSafe(
             desired, reference, repaired, allow_unstable_seed) &&
         configurationIsSafe(repaired, yaw);
}

}  // namespace motion_primitive_planner
