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
#include <numeric>
#include <random>
#include <stdexcept>
#include <utility>

namespace motion_primitive_planner
{
namespace
{
constexpr double kEpsilon = 1e-9;
constexpr double kInfinity = std::numeric_limits<double>::infinity();
//! Marks a commit point that is not one of the nominal samples.
constexpr size_t kOffNominal = std::numeric_limits<size_t>::max();
//! Resolution relaxation used by the sampling-based search and its shortcut,
//! whose output is always re-validated at the configured resolution.
constexpr double kSearchResolutionFactor = 4.0;
//! Fraction of RRT samples snapped onto single-joint fold corners.
constexpr double kStructuredSampleRatio = 0.65;
//! Upper bound on the anchor pairs tried for one infeasible nominal interval.
constexpr size_t kMaxAnchorCandidates = 3;

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

bool expired(const std::chrono::steady_clock::time_point& deadline)
{
  return std::chrono::steady_clock::now() >= deadline;
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

// DRAGON's infeasible joint-space regions are thin shells around the shapes whose
// rotors become coplanar, and the shortest escape from them is almost always a
// single-joint fold.  Biasing a large fraction of the samples onto the corners
// spanned by the bridge endpoints and the joint-limit folds turns RRT-Connect
// into an efficient search over exactly those staircase detours.
class FoldBiasedSampler : public ompl::base::StateSampler
{
public:
  FoldBiasedSampler(const ompl::base::StateSpace* space,
                    const std::vector<std::vector<double>>& anchors,
                    unsigned int seed)
    : ompl::base::StateSampler(space)
    , bounds_(space->as<ompl::base::RealVectorStateSpace>()->getBounds())
    , anchors_(anchors)
    , generator_(seed)
  {
  }

  void sampleUniform(ompl::base::State* state) override
  {
    double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const bool structured = unit(generator_) < kStructuredSampleRatio;
    for (size_t index = 0; index < anchors_.size(); ++index)
    {
      double value = bounds_.low[index] +
                     unit(generator_) * (bounds_.high[index] - bounds_.low[index]);
      if (structured && !anchors_[index].empty())
      {
        std::uniform_int_distribution<size_t> pick(0, anchors_[index].size() - 1);
        value = anchors_[index][pick(generator_)];
      }
      values[index] = clamp(index, value);
    }
  }

  void sampleUniformNear(ompl::base::State* state, const ompl::base::State* near,
                         double distance) override
  {
    const double* center = near->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    std::uniform_real_distribution<double> offset(-distance, distance);
    for (size_t index = 0; index < anchors_.size(); ++index)
    {
      values[index] = clamp(index, center[index] + offset(generator_));
    }
  }

  void sampleGaussian(ompl::base::State* state, const ompl::base::State* mean,
                      double std_deviation) override
  {
    const double* center = mean->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    std::normal_distribution<double> normal(0.0, std_deviation);
    for (size_t index = 0; index < anchors_.size(); ++index)
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
  std::vector<std::vector<double>> anchors_;
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
      config_.planning_timeout <= 0.0 || config_.bridge_timeout <= 0.0 ||
      config_.validity_resolution <= 0.0 || config_.anchor_backoff_time < 0.0 ||
      config_.max_joint_velocity <= 0.0 || config_.max_joint_command_step <= 0.0 ||
      config_.follower.command_hz <= 0.0)
  {
    throw std::invalid_argument("Invalid joint trajectory planner configuration");
  }
  seedOmplOnce(config_.random_seed);

  const int joint_count = stability_evaluator_->jointCount();
  const std::vector<double>& lower = stability_evaluator_->robotModel()->getLinkJointLowerLimits();
  const std::vector<double>& upper = stability_evaluator_->robotModel()->getLinkJointUpperLimits();
  fold_magnitude_ = Eigen::VectorXd::Zero(joint_count);
  for (int index = 0; index < joint_count; ++index)
  {
    const size_t limit_index = static_cast<size_t>(index);
    const double reach = limit_index < lower.size() && limit_index < upper.size() ?
                             std::min(std::abs(lower[limit_index]), std::abs(upper[limit_index])) :
                             M_PI_2;
    fold_magnitude_(index) = std::min(reach, M_PI_2);
  }
  try
  {
    const DragonModelInfo model_info(stability_evaluator_->robotModel());
    pitch_joint_indices_ = model_info.pitchJointIndices();
    yaw_joint_indices_ = model_info.yawJointIndices();
  }
  catch (const std::exception&)
  {
    // Without the DRAGON pitch/yaw split the planner falls back to plain joint
    // ordering; every detour is still validated against the same constraints.
  }
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

  multilink_copilot::StabilityMetrics start_metrics;
  Eigen::VectorXd origin = start_joints;
  bool repaired_start = false;
  if (!configurationIsSafe(start_joints, start_yaw + M_PI, &start_metrics))
  {
    // Never strand the robot: recover onto the closest stable fold instead of
    // refusing to produce any command for an already-infeasible measurement.
    bool projected = false;
    origin = projectedTerminal(start_joints, start_joints, start_joints, start_yaw + M_PI, projected);
    if (!projected || !configurationIsSafe(origin, start_yaw + M_PI, &start_metrics))
    {
      result.detail = "start joint configuration is not flight-feasible";
      return result;
    }
    repaired_start = true;
  }

  const std::vector<NominalSample> nominal =
      buildNominalSamples(root_trajectory, context, start_joints, start_yaw);
  if (nominal.size() < 2)
  {
    result.detail = "failed to build nominal follow-the-leader joint samples";
    return result;
  }

  result.joint_waypoints.push_back({0.0, start_joints});
  result.minimum_fc_rp = start_metrics.fc_rp_min;

  struct CommitPoint
  {
    size_t nominal_index = kOffNominal;
    size_t waypoint_count = 0;
    double minimum_fc_rp = kInfinity;
    double yaw = 0.0;
  };
  std::vector<CommitPoint> tracked;
  if (repaired_start)
  {
    result.joint_waypoints.push_back({0.5 * nominal[1].time, origin});
    result.detail = "recovered from an infeasible start configuration";
  }
  tracked.push_back({repaired_start ? kOffNominal : 0u, result.joint_waypoints.size(),
                     result.minimum_fc_rp, start_yaw});

  // A rewind must be able to both shrink and re-grow the committed tail, so it
  // always replays a snapshot instead of resizing the live vector in place.
  std::vector<TimedJointWaypoint> committed = result.joint_waypoints;
  const auto rewind = [&result, &tracked, &committed](size_t position) {
    const size_t count = std::min(tracked[position].waypoint_count, committed.size());
    result.joint_waypoints.assign(committed.begin(), committed.begin() + count);
    result.minimum_fc_rp = tracked[position].minimum_fc_rp;
  };
  // Ordered exit anchors: the sample bordering the infeasible interval first, then
  // progressively earlier and better-conditioned samples of the same safe run.
  const auto exitCandidates = [&nominal, &tracked, this]() {
    std::vector<size_t> positions{tracked.size() - 1};
    const size_t window =
        std::max<size_t>(1, static_cast<size_t>(config_.anchor_backoff_time / config_.reference_dt));
    const size_t lowest = tracked.size() - 1 >= window ? tracked.size() - 1 - window : 0;
    size_t best = tracked.size() - 1;
    double best_margin = -kInfinity;
    for (size_t position = lowest; position < tracked.size(); ++position)
    {
      const size_t index = tracked[position].nominal_index;
      const double margin = index == kOffNominal ? -kInfinity : nominal[index].margin;
      if (margin > best_margin)
      {
        best_margin = margin;
        best = position;
      }
    }
    if (best != positions.front())
    {
      positions.push_back(best);
    }
    if (lowest != positions.front() && lowest != best && positions.size() < kMaxAnchorCandidates)
    {
      positions.push_back(lowest);
    }
    return positions;
  };

  const std::vector<SafeRun> runs = safeRuns(nominal);
  for (const SafeRun& run : runs)
  {
    const size_t committed_index = tracked.back().nominal_index;
    if (committed_index != kOffNominal && run.last <= committed_index)
    {
      continue;
    }

    size_t entry = run.first;
    if (committed_index != run.first)
    {
      const std::vector<size_t> entries = anchorCandidates(nominal, run.first, run.last);
      bool entered = false;
      for (const size_t exit_position : exitCandidates())
      {
        const CommitPoint exit = tracked[exit_position];
        for (const size_t candidate : entries)
        {
          if (expired(deadline))
          {
            break;
          }
          if (exit.nominal_index != kOffNominal && candidate <= exit.nominal_index)
          {
            continue;
          }
          rewind(exit_position);
          if (appendBridge(result.joint_waypoints.back(), exit.yaw + M_PI,
                           nominal[candidate].joints, nominal[candidate].time,
                           nominal[candidate].yaw + M_PI, deadline, result.joint_waypoints,
                           result.minimum_fc_rp, result.bridge_count))
          {
            entry = candidate;
            entered = true;
            break;
          }
        }
        if (entered)
        {
          break;
        }
      }
      if (!entered)
      {
        // Hold the last validated shape through this run; the root trajectory is
        // still executed and the next replan starts from a feasible state.
        rewind(tracked.size() - 1);
        ++result.hold_count;
        continue;
      }
      committed = result.joint_waypoints;
      tracked.clear();
      tracked.push_back({entry, result.joint_waypoints.size(), result.minimum_fc_rp,
                         nominal[entry].yaw});
    }

    for (size_t index = entry + 1; index <= run.last; ++index)
    {
      // Both endpoints are already validated samples, so only the edge interior
      // still has to be swept.
      double edge_minimum = nominal[index].fc_rp_min;
      if (!edgeInteriorIsSafe(result.joint_waypoints.back().positions, nominal[index].joints,
                              tracked.back().yaw + M_PI, nominal[index].yaw + M_PI,
                              config_.validity_resolution, edge_minimum))
      {
        continue;
      }
      result.joint_waypoints.push_back({nominal[index].time, nominal[index].joints});
      result.minimum_fc_rp = std::min(result.minimum_fc_rp, edge_minimum);
      tracked.push_back({index, result.joint_waypoints.size(), result.minimum_fc_rp,
                         nominal[index].yaw});
    }
    committed = result.joint_waypoints;
  }

  // An infeasible terminal sample never belongs to a safe run, so it is still
  // unreached at this point.
  const NominalSample& terminal = nominal.back();
  if (!terminal.safe)
  {
    // The nominal terminal shape is infeasible; steer to the closest stable fold
    // so that the next replanning cycle starts from a well-conditioned state.
    bool projected = false;
    const Eigen::VectorXd target =
        projectedTerminal(terminal.joints, result.joint_waypoints.back().positions, start_joints,
                          terminal.yaw + M_PI, projected);
    bool reached = false;
    if (projected)
    {
      for (const size_t exit_position : exitCandidates())
      {
        if (expired(deadline))
        {
          break;
        }
        const CommitPoint exit = tracked[exit_position];
        rewind(exit_position);
        if (terminal.time <= result.joint_waypoints.back().time + kEpsilon)
        {
          continue;
        }
        if (appendEarlyDirectRecovery(result.joint_waypoints.back(), target, terminal.time,
                                      nominal, result.joint_waypoints,
                                      result.minimum_fc_rp) ||
            appendBridge(result.joint_waypoints.back(), exit.yaw + M_PI, target, terminal.time,
                         terminal.yaw + M_PI, deadline, result.joint_waypoints,
                         result.minimum_fc_rp, result.bridge_count))
        {
          reached = true;
          break;
        }
      }
    }
    if (!reached)
    {
      rewind(tracked.size() - 1);
      ++result.hold_count;
    }
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
  computeTrackingError(root_trajectory, context, nominal, result);
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
  double direct_minimum = minimum_fc_rp;
  if (directEdgeIsSafe(start, goal, root_link_yaw, root_link_yaw, direct_minimum))
  {
    path = {start, goal};
    minimum_fc_rp = direct_minimum;
    return true;
  }

  std::vector<Eigen::VectorXd> bridge;
  double bridge_minimum = kInfinity;
  if (!planBridge(start, goal, root_link_yaw, root_link_yaw, deadline_, bridge, bridge_minimum))
  {
    if (failure_reason) *failure_reason = "no stable joint connection was found";
    return false;
  }
  path.push_back(start);
  path.insert(path.end(), bridge.begin(), bridge.end());
  minimum_fc_rp = std::min(minimum_fc_rp, bridge_minimum);
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
    bool safe = false;
    double fc_rp_min = 0.0;
    const double margin = stabilityMargin(sample.joints, sample.yaw + M_PI, safe, fc_rp_min);
    samples.push_back({sample.time, sample.yaw, sample.joints, safe, margin, fc_rp_min});
  }
  return samples;
}

std::vector<JointTrajectoryPlanner::SafeRun> JointTrajectoryPlanner::safeRuns(
    const std::vector<NominalSample>& samples)
{
  std::vector<SafeRun> runs;
  size_t index = 0;
  while (index < samples.size())
  {
    if (!samples[index].safe)
    {
      ++index;
      continue;
    }
    const size_t first = index;
    while (index + 1 < samples.size() && samples[index + 1].safe)
    {
      ++index;
    }
    runs.push_back({first, index});
    ++index;
  }
  return runs;
}

std::vector<size_t> JointTrajectoryPlanner::anchorCandidates(
    const std::vector<NominalSample>& samples, size_t nominal_index, size_t limit_index) const
{
  const size_t window =
      std::max<size_t>(1, static_cast<size_t>(config_.anchor_backoff_time / config_.reference_dt));
  const size_t highest = std::min(limit_index, nominal_index + window);
  std::vector<size_t> candidates{nominal_index};
  size_t best = nominal_index;
  double best_margin = -kInfinity;
  for (size_t index = nominal_index; index <= highest; ++index)
  {
    if (samples[index].margin > best_margin)
    {
      best_margin = samples[index].margin;
      best = index;
    }
  }
  if (best != nominal_index)
  {
    candidates.push_back(best);
  }
  if (highest != nominal_index && highest != best && candidates.size() < kMaxAnchorCandidates)
  {
    candidates.push_back(highest);
  }
  return candidates;
}

bool JointTrajectoryPlanner::appendBridge(TimedJointWaypoint start,
                                          double start_root_yaw,
                                          const Eigen::VectorXd& goal,
                                          double goal_time,
                                          double goal_root_yaw,
                                          const Clock::time_point& deadline,
                                          std::vector<TimedJointWaypoint>& path,
                                          double& minimum_fc_rp,
                                          int& bridge_count)
{
  double direct_minimum = kInfinity;
  if (directEdgeIsSafe(start.positions, goal, start_root_yaw, goal_root_yaw, direct_minimum))
  {
    path.push_back({goal_time, goal});
    minimum_fc_rp = std::min(minimum_fc_rp, direct_minimum);
    return true;
  }

  std::vector<Eigen::VectorXd> bridge;
  double bridge_minimum = kInfinity;
  if (!planBridge(start.positions, goal, start_root_yaw, goal_root_yaw, deadline, bridge,
                  bridge_minimum))
  {
    return false;
  }

  std::vector<Eigen::VectorXd> chain;
  chain.reserve(bridge.size() + 1);
  chain.push_back(start.positions);
  chain.insert(chain.end(), bridge.begin(), bridge.end());
  const std::vector<double> ratios = chainRatios(chain);
  for (size_t index = 1; index < chain.size(); ++index)
  {
    path.push_back({start.time + ratios[index] * (goal_time - start.time), chain[index]});
  }
  path.back().time = goal_time;
  minimum_fc_rp = std::min(minimum_fc_rp, bridge_minimum);
  ++bridge_count;
  return true;
}

bool JointTrajectoryPlanner::planBridge(const Eigen::VectorXd& start,
                                        const Eigen::VectorXd& goal,
                                        double start_yaw,
                                        double goal_yaw,
                                        const Clock::time_point& deadline,
                                        std::vector<Eigen::VectorXd>& path,
                                        double& minimum_fc_rp)
{
  path.clear();
  // The bridge budget is bounded both by its own share and by the plan deadline
  // that every validation loop below also honours.
  const Clock::time_point bridge_deadline =
      std::min(deadline, Clock::now() + toDuration(config_.bridge_timeout));
  const Clock::time_point outer_deadline = deadline_;
  deadline_ = std::min(outer_deadline, bridge_deadline);
  struct DeadlineGuard
  {
    ~DeadlineGuard() { *slot = value; }
    Clock::time_point* slot;
    Clock::time_point value;
  } guard{&deadline_, outer_deadline};

  // Structured detours first: a chain of single-joint folds crosses the thin
  // infeasible shells that a straight joint-space interpolation cannot.
  for (const std::vector<Eigen::VectorXd>& chain : bridgeChains(start, goal))
  {
    if (budgetExpired())
    {
      break;
    }
    double chain_minimum = kInfinity;
    if (!chainIsSafe(chain, start_yaw, goal_yaw, chain_minimum))
    {
      continue;
    }
    path.assign(chain.begin() + 1, chain.end());
    minimum_fc_rp = std::min(minimum_fc_rp, chain_minimum);
    return true;
  }

  std::vector<Eigen::VectorXd> detour;
  if (!searchJointDetour(start, goal, goal_yaw, deadline_, detour) || detour.size() < 2)
  {
    return false;
  }
  const std::vector<Eigen::VectorXd> shortcut = shortcutChain(detour, goal_yaw);
  double shortcut_minimum = kInfinity;
  if (shortcut.size() < 2 || !chainIsSafe(shortcut, start_yaw, goal_yaw, shortcut_minimum))
  {
    return false;
  }
  path.assign(shortcut.begin() + 1, shortcut.end());
  minimum_fc_rp = std::min(minimum_fc_rp, shortcut_minimum);
  return true;
}

std::vector<std::vector<Eigen::VectorXd>> JointTrajectoryPlanner::bridgeChains(
    const Eigen::VectorXd& start, const Eigen::VectorXd& goal) const
{
  // Folding both endpoints out to the joint limits before re-folding moves the
  // search away from the marginal shapes that border the infeasible interval.
  const Eigen::VectorXd deep_start = deepFold(start, start);
  const Eigen::VectorXd deep_goal = deepFold(goal, goal);
  const std::vector<std::vector<int>> orders = jointOrders(start, goal);
  // Each entry is (key configurations, how many joint orders to expand it with).
  // Direct staircases come first because they deviate least from the nominal
  // shape; the deep-fold detours are the expensive last resort.
  const std::vector<std::pair<std::vector<Eigen::VectorXd>, size_t>> key_chains = {
      {{start, goal}, orders.size()},
      {{start, deep_start, goal}, 2},
      {{start, deep_goal, goal}, 2},
      {{start, deep_start, deep_goal, goal}, 2}};

  std::vector<std::vector<Eigen::VectorXd>> chains;
  for (const auto& entry : key_chains)
  {
    for (size_t index = 0; index < std::min(entry.second, orders.size()); ++index)
    {
      std::vector<Eigen::VectorXd> chain = expandChain(entry.first, orders[index]);
      if (chain.size() > 2)
      {
        chains.push_back(std::move(chain));
      }
    }
  }
  return chains;
}

std::vector<std::vector<int>> JointTrajectoryPlanner::jointOrders(const Eigen::VectorXd& start,
                                                                  const Eigen::VectorXd& goal) const
{
  const int joint_count = static_cast<int>(start.size());
  std::vector<int> forward(static_cast<size_t>(joint_count));
  std::iota(forward.begin(), forward.end(), 0);
  const Eigen::VectorXd travel = (goal - start).cwiseAbs();

  std::vector<int> descending = forward;
  std::stable_sort(descending.begin(), descending.end(),
                   [&travel](int lhs, int rhs) { return travel(lhs) > travel(rhs); });
  std::vector<int> ascending(descending.rbegin(), descending.rend());
  std::vector<int> reverse(forward.rbegin(), forward.rend());

  // The DRAGON yaw joints dominate the in-plane shape, so re-folding them before
  // the pitch joints is the detour that keeps the body furthest from collapse.
  std::vector<int> yaw_first;
  std::vector<int> pitch_first;
  if (!yaw_joint_indices_.empty() && !pitch_joint_indices_.empty())
  {
    for (const int index : descending)
    {
      if (std::find(yaw_joint_indices_.begin(), yaw_joint_indices_.end(), index) !=
          yaw_joint_indices_.end())
      {
        yaw_first.push_back(index);
      }
    }
    for (const int index : descending)
    {
      if (std::find(yaw_joint_indices_.begin(), yaw_joint_indices_.end(), index) ==
          yaw_joint_indices_.end())
      {
        yaw_first.push_back(index);
      }
    }
    pitch_first.assign(yaw_first.rbegin(), yaw_first.rend());
  }

  std::vector<std::vector<int>> orders{descending};
  if (!yaw_first.empty()) orders.push_back(yaw_first);
  orders.push_back(ascending);
  orders.push_back(forward);
  orders.push_back(reverse);
  if (!pitch_first.empty()) orders.push_back(pitch_first);
  return orders;
}

std::vector<Eigen::VectorXd> JointTrajectoryPlanner::expandChain(
    const std::vector<Eigen::VectorXd>& keys, const std::vector<int>& order)
{
  std::vector<Eigen::VectorXd> chain;
  if (keys.empty())
  {
    return chain;
  }
  chain.push_back(keys.front());
  for (size_t key = 1; key < keys.size(); ++key)
  {
    Eigen::VectorXd current = chain.back();
    for (const int joint : order)
    {
      if (joint < 0 || joint >= current.size() ||
          std::abs(keys[key](joint) - current(joint)) <= kEpsilon)
      {
        continue;
      }
      current(joint) = keys[key](joint);
      chain.push_back(current);
    }
    if ((chain.back() - keys[key]).cwiseAbs().maxCoeff() > kEpsilon)
    {
      chain.push_back(keys[key]);
    }
  }
  return chain;
}

Eigen::VectorXd JointTrajectoryPlanner::deepFold(const Eigen::VectorXd& reference,
                                                 const Eigen::VectorXd& sign_source) const
{
  Eigen::VectorXd folded = reference;
  for (const int joint : yaw_joint_indices_)
  {
    if (joint < 0 || joint >= folded.size() || joint >= fold_magnitude_.size())
    {
      continue;
    }
    folded(joint) = sign_source(joint) >= 0.0 ? fold_magnitude_(joint) : -fold_magnitude_(joint);
  }
  return folded;
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

bool JointTrajectoryPlanner::directEdgeIsSafe(const Eigen::VectorXd& start,
                                              const Eigen::VectorXd& goal,
                                              double start_yaw,
                                              double goal_yaw,
                                              double& minimum_fc_rp)
{
  multilink_copilot::StabilityMetrics start_metrics;
  multilink_copilot::StabilityMetrics goal_metrics;
  if (!configurationIsSafe(start, start_yaw, &start_metrics) ||
      !configurationIsSafe(goal, goal_yaw, &goal_metrics))
  {
    return false;
  }
  minimum_fc_rp = std::min(minimum_fc_rp, std::min(start_metrics.fc_rp_min, goal_metrics.fc_rp_min));
  return edgeInteriorIsSafe(start, goal, start_yaw, goal_yaw, config_.validity_resolution,
                            minimum_fc_rp);
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

bool JointTrajectoryPlanner::searchJointDetour(
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

  std::vector<std::vector<double>> anchors(static_cast<size_t>(start.size()));
  for (int index = 0; index < start.size(); ++index)
  {
    anchors[static_cast<size_t>(index)] = {start(index), goal(index), fold_magnitude_(index),
                                           -fold_magnitude_(index), 0.0};
  }
  const unsigned int seed = config_.random_seed;
  space->setStateSamplerAllocator(
      [anchors, seed](const ompl::base::StateSpace* state_space) -> ompl::base::StateSamplerPtr {
        return std::make_shared<FoldBiasedSampler>(state_space, anchors, seed);
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
  // Long extensions let one CONNECT step traverse a whole single-joint fold.
  planner->setRange(std::max(M_PI_2, 0.25 * space->getMaximumExtent()));
  setup.setPlanner(planner);
  const ompl::base::PlannerStatus status =
      setup.solve(std::chrono::duration<double>(deadline - Clock::now()).count());
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

double JointTrajectoryPlanner::stabilityMargin(const Eigen::VectorXd& joints, double yaw,
                                               bool& safe, double& fc_rp_min)
{
  stability_evaluator_->setRootLinkRotation(KDL::Rotation::RotZ(yaw));
  multilink_copilot::StabilityMetrics metrics;
  if (!stability_evaluator_->evaluate(joints, metrics))
  {
    safe = false;
    fc_rp_min = 0.0;
    return -kInfinity;
  }
  safe = metrics.safe;
  fc_rp_min = metrics.fc_rp_min;
  const multilink_copilot::StabilityConfig& limits = stability_evaluator_->config();
  const auto normalized = [](double slack, double scale) {
    return slack / std::max(std::abs(scale), 1e-6);
  };
  double margin = normalized(metrics.fc_rp_min - limits.fc_rp_min_threshold,
                             limits.fc_rp_min_threshold);
  if (limits.check_fc_t)
  {
    margin = std::min(margin, normalized(metrics.fc_t_min - limits.fc_t_min_threshold,
                                         limits.fc_t_min_threshold));
  }
  margin = std::min(margin, normalized(metrics.static_thrust_min - limits.static_thrust_min,
                                       limits.static_thrust_min));
  margin = std::min(margin, normalized(limits.static_thrust_max - metrics.static_thrust_max,
                                       limits.static_thrust_max));
  margin = std::min(margin, normalized(metrics.overlap_clearance - limits.overlap_min_clearance,
                                       limits.overlap_min_clearance));
  if (limits.max_baselink_tilt > 0.0)
  {
    margin = std::min(margin, normalized(limits.max_baselink_tilt - metrics.baselink_tilt,
                                         limits.max_baselink_tilt));
  }
  return std::isfinite(margin) ? margin : -kInfinity;
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

bool JointTrajectoryPlanner::appendEarlyDirectRecovery(
    TimedJointWaypoint start, const Eigen::VectorXd& target,
    double terminal_time, const std::vector<NominalSample>& nominal,
    std::vector<TimedJointWaypoint>& path, double& minimum_fc_rp)
{
  if (target.size() != start.positions.size() || !target.allFinite() ||
      terminal_time <= start.time + kEpsilon)
  {
    return false;
  }

  constexpr double kCommandStepSchedulingMargin = 0.99;
  const double effective_joint_rate = std::min(
      config_.max_joint_velocity,
      kCommandStepSchedulingMargin * config_.max_joint_command_step *
          config_.follower.command_hz);
  const double maximum_delta =
      target.size() > 0 ? (target - start.positions).cwiseAbs().maxCoeff() : 0.0;
  const double arrival_time = start.time + maximum_delta / effective_joint_rate;
  if (arrival_time >= terminal_time - kEpsilon)
  {
    return false;
  }

  double recovery_minimum = kInfinity;
  const TimedJointWaypoint arrival{arrival_time, target};
  const TimedJointWaypoint terminal{terminal_time, target};
  if (!timedConfigurationPathIsSafe(start, arrival, nominal, recovery_minimum) ||
      !timedConfigurationPathIsSafe(arrival, terminal, nominal, recovery_minimum))
  {
    return false;
  }

  if (arrival.time > path.back().time + kEpsilon)
  {
    path.push_back(arrival);
  }
  else
  {
    path.back().positions = target;
  }
  path.push_back(terminal);
  minimum_fc_rp = std::min(minimum_fc_rp, recovery_minimum);
  return true;
}

void JointTrajectoryPlanner::computeTrackingError(
    const Trajectory<5>& root_trajectory, const NominalJointContext& context,
    const std::vector<NominalSample>& nominal, JointPlanResult& result) const
{
  result.tracking_error_rms = 0.0;
  result.tracking_error_max = 0.0;
  const int downstream_link_count = context.link_num - 1;
  if (!std::isfinite(result.duration) || !std::isfinite(result.time_scale) ||
      result.duration <= kEpsilon || result.time_scale <= 0.0 ||
      downstream_link_count <= 0 || nominal.empty())
  {
    return;
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
      return;
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
}

Eigen::VectorXd JointTrajectoryPlanner::projectedTerminal(const Eigen::VectorXd& desired,
                                                          const Eigen::VectorXd& current,
                                                          const Eigen::VectorXd& start,
                                                          double yaw,
                                                          bool& success)
{
  Eigen::VectorXd positive_fold = Eigen::VectorXd::Zero(desired.size());
  Eigen::VectorXd negative_fold = Eigen::VectorXd::Zero(desired.size());
  for (const int yaw_index : yaw_joint_indices_)
  {
    if (yaw_index >= 0 && yaw_index < desired.size() && yaw_index < fold_magnitude_.size())
    {
      positive_fold(yaw_index) = fold_magnitude_(yaw_index);
      negative_fold(yaw_index) = -fold_magnitude_(yaw_index);
    }
  }
  stability_evaluator_->setRootLinkRotation(KDL::Rotation::RotZ(yaw));
  Eigen::VectorXd projected;
  success = stability_evaluator_->projectNearestSafe(
      desired, {current, start, positive_fold, negative_fold}, projected);
  return success ? projected : Eigen::VectorXd();
}

}  // namespace motion_primitive_planner
