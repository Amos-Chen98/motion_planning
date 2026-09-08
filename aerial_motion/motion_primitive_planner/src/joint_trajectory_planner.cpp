#include <motion_primitive_planner/joint_trajectory_planner.h>

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
#include <set>
#include <stdexcept>
#include <utility>

namespace motion_primitive_planner
{
namespace
{
constexpr double kEpsilon = 1e-9;
constexpr double kPredictionTimeTolerance = 1e-6;
constexpr double kInfinity = std::numeric_limits<double>::infinity();
//! Resolution relaxation used by the sampling-based search and its shortcut,
//! whose output is always re-validated at the configured resolution.
constexpr double kSearchResolutionFactor = 4.0;
constexpr double kTraceParameterTolerance = 1e-9;
constexpr double kTraceDistanceTolerance = 1e-6;
using PlanningClock = std::chrono::steady_clock;

//! Coefficients use descending powers of normalized local time, including for
//! linear initial-body segments. Bounds can clip a MINCO piece at handover.
struct TerminalTraceSegment
{
  Piece<5>::CoefficientMat coefficients = Piece<5>::CoefficientMat::Zero();
  double lower = 0.0;
  double upper = 1.0;

  Eigen::Vector3d position(double parameter) const
  {
    Eigen::Vector3d value = coefficients.col(0);
    for (int column = 1; column < coefficients.cols(); ++column)
    {
      value = value * parameter + coefficients.col(column);
    }
    return value;
  }
};

struct TerminalTraceCursor
{
  size_t segment = 0;
  double parameter = 1.0;
};

//! RootFinder expects non-root interval boundaries. Widen its interval and
//! explicitly handle the actual segment boundaries in the caller.
bool stationaryParameters(const Eigen::VectorXd& squared_distance,
                           double lower, double upper,
                           const PlanningClock::time_point& deadline,
                           std::vector<double>& parameters)
{
  if (PlanningClock::now() >= deadline || !squared_distance.allFinite())
  {
    return false;
  }
  Eigen::VectorXd derivative(squared_distance.size() - 1);
  for (int index = 0; index < derivative.size(); ++index)
  {
    derivative(index) = (derivative.size() - index) * squared_distance(index);
  }
  const double scale = derivative.cwiseAbs().maxCoeff();
  if (!std::isfinite(scale)) return false;
  if (scale == 0.0 || upper <= lower) return true;
  derivative /= scale;
  double padding = 0.0625;
  double left = lower - padding;
  double right = upper + padding;
  for (int attempt = 0; attempt < 32; ++attempt)
  {
    const double left_value = RootFinder::polyVal(derivative, left);
    const double right_value = RootFinder::polyVal(derivative, right);
    if (!std::isfinite(left_value) || !std::isfinite(right_value)) return false;
    if (std::abs(left_value) > std::numeric_limits<double>::epsilon() &&
        std::abs(right_value) > std::numeric_limits<double>::epsilon())
    {
      const std::set<double> roots = RootFinder::solvePolynomial(
          derivative, left, right, kTraceParameterTolerance);
      for (const double root : roots)
      {
        if (std::isfinite(root) && root > lower && root < upper)
        {
          parameters.push_back(root);
        }
      }
      return PlanningClock::now() < deadline;
    }
    padding *= 1.5;
    left = lower - padding;
    right = upper + padding;
  }
  return false;
}

//! Find the first sphere intersection backwards along the trace. Stationary
//! distances partition the degree-ten distance equation into monotone
//! intervals: bracketed roots cover crossings, and the partition boundaries
//! cover tangencies. They also give the global nearest-distance fallback.
bool findTerminalTail(const std::vector<TerminalTraceSegment>& trace,
                      const Eigen::Vector3d& head, double length,
                      const PlanningClock::time_point& deadline,
                      TerminalTraceCursor& cursor, Eigen::Vector3d& tail)
{
  double best_error = kInfinity;
  TerminalTraceCursor best_cursor = cursor;
  Eigen::Vector3d best_position = head;
  for (size_t reverse = cursor.segment + 1; reverse > 0; --reverse)
  {
    if (PlanningClock::now() >= deadline) return false;
    const size_t index = reverse - 1;
    const TerminalTraceSegment& segment = trace[index];
    const double upper = index == cursor.segment ? cursor.parameter : segment.upper;
    const double lower = segment.lower;
    auto relative = segment.coefficients;
    relative.col(5) -= head;
    const Eigen::VectorXd squared_distance =
        RootFinder::polySqr(relative.row(0)) + RootFinder::polySqr(relative.row(1)) +
        RootFinder::polySqr(relative.row(2));
    std::vector<double> parameters{upper, lower};
    if (!stationaryParameters(squared_distance, lower, upper, deadline, parameters)) return false;
    std::sort(parameters.begin(), parameters.end(), std::greater<double>());
    parameters.erase(std::unique(parameters.begin(), parameters.end(),
                                  [](double lhs, double rhs) {
                                    return std::abs(lhs - rhs) <= kTraceParameterTolerance;
                                  }), parameters.end());

    const auto distance_error = [&](double parameter) {
      return (segment.position(parameter) - head).norm() - length;
    };
    for (size_t sample = 0; sample < parameters.size(); ++sample)
    {
      if (PlanningClock::now() >= deadline) return false;
      const double parameter = parameters[sample];
      const double error = distance_error(parameter);
      if (!std::isfinite(error)) return false;
      if (std::abs(error) < best_error - kTraceDistanceTolerance)
      {
        best_error = std::abs(error);
        best_cursor = {index, parameter};
        best_position = segment.position(parameter);
      }
      if (std::abs(error) <= kTraceDistanceTolerance)
      {
        cursor = {index, parameter};
        tail = segment.position(parameter);
        return true;
      }
      if (sample + 1 >= parameters.size()) continue;
      double left = parameters[sample + 1];
      double right = parameter;
      const double left_error = distance_error(left);
      if (!std::isfinite(left_error)) return false;
      if ((error < 0.0) == (left_error < 0.0)) continue;
      // No squared polynomial evaluation here: Cartesian residuals avoid
      // cancellation around a root. Keep bisecting until the distance is valid.
      for (int iteration = 0; iteration < 80; ++iteration)
      {
        if (PlanningClock::now() >= deadline) return false;
        const double middle = 0.5 * (left + right);
        const double middle_error = distance_error(middle);
        if (!std::isfinite(middle_error)) return false;
        if (std::abs(middle_error) <= kTraceDistanceTolerance)
        {
          cursor = {index, middle};
          tail = segment.position(middle);
          return true;
        }
        if (middle == left || middle == right) return false;
        if ((middle_error < 0.0) == (error < 0.0)) right = middle;
        else left = middle;
      }
      return false;
    }
  }
  if (!std::isfinite(best_error)) return false;
  cursor = best_cursor;
  tail = best_position;
  return true;
}

template <typename Waypoint>
size_t upperWaypointIndex(const std::vector<Waypoint>& waypoints, double time)
{
  const auto upper = std::upper_bound(waypoints.begin(), waypoints.end(), time,
                                      [](double value, const Waypoint& waypoint) {
                                        return value < waypoint.time;
                                      });
  return static_cast<size_t>(std::distance(waypoints.begin(), upper));
}

template <typename Waypoint>
const Waypoint* activeWaypointEnd(const std::vector<Waypoint>& waypoints, double time)
{
  const size_t upper = upperWaypointIndex(waypoints, time);
  return upper > 0 && upper < waypoints.size() ? &waypoints[upper] : nullptr;
}

std::chrono::steady_clock::duration toDuration(double seconds)
{
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(seconds));
}

double interpolateYaw(double start, double goal, double ratio)
{
  return start + ratio * shortestYawDelta(start, goal);
}

//! Normalized distances of the chain vertices along the chain, used both for
//! root-attitude interpolation and for time allocation.
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

TrajectoryHistory::TrajectoryHistory(const FollowerConfig& config)
  : sample_interval_(config.trajectory_sample_interval)
  , maximum_length_(config.trajectory_buffer_max_length)
{
  config.validateOrThrow();
}

bool TrajectoryHistory::append(const Eigen::Vector3d& position)
{
  if (!position.allFinite() || sample_interval_ <= 0.0 || maximum_length_ <= 0.0)
  {
    return false;
  }
  if (points_.empty())
  {
    points_.push_back({position});
    return true;
  }
  const double distance = (position - points_.back().position).norm();
  if (distance < sample_interval_)
  {
    return false;
  }
  points_.push_back({position});
  arc_length_ += distance;
  while (points_.size() > 1 && arc_length_ > maximum_length_)
  {
    arc_length_ -= (points_[1].position - points_[0].position).norm();
    points_.pop_front();
  }
  return true;
}

NominalJointContext makeNominalJointContext(const TrajectoryHistory& history,
                                            const DragonModelInfo& model)
{
  NominalJointContext context;
  context.executed_history = history;
  context.link_num = model.linkNum();
  context.link_length = model.linkLength();
  context.pitch_joint_indices = model.pitchJointIndices();
  context.yaw_joint_indices = model.yawJointIndices();
  return context;
}

std::vector<TimedRootAttitudeWaypoint> predictRootAttitudes(
    const Trajectory<5>& root_trajectory, const RootAttitude& start_attitude,
    const FollowerConfig& config, double sample_dt, bool command_pitch,
    double trajectory_start_time, double output_time_offset,
    PlanningClock::time_point deadline)
{
  config.validateOrThrow();
  std::vector<TimedRootAttitudeWaypoint> samples;
  if (root_trajectory.getPieceNum() <= 0 || !std::isfinite(sample_dt) || sample_dt <= 0.0 ||
      !std::isfinite(trajectory_start_time) || trajectory_start_time < 0.0 ||
      !std::isfinite(output_time_offset) || output_time_offset < 0.0 ||
      !std::isfinite(start_attitude.yaw) || !std::isfinite(start_attitude.pitch))
  {
    return samples;
  }
  for (int piece = 0; piece < root_trajectory.getPieceNum(); ++piece)
  {
    if (!std::isfinite(root_trajectory[piece].getDuration()) ||
        root_trajectory[piece].getDuration() < 0.0 ||
        !root_trajectory[piece].getCoeffMat().allFinite()) return samples;
  }
  RootAttitude attitude = start_attitude;
  const double duration = root_trajectory.getTotalDuration();
  if (!std::isfinite(duration) || duration < 0.0 ||
      trajectory_start_time > duration + kPredictionTimeTolerance) return samples;
  const double remaining_duration = std::max(0.0, duration - trajectory_start_time);
  const double count = std::ceil(remaining_duration / sample_dt);
  if (!std::isfinite(count) || count >= std::numeric_limits<int>::max()) return samples;
  const int sample_count = remaining_duration > kPredictionTimeTolerance ?
                               std::max(1, static_cast<int>(count)) : 0;
  double previous_time = trajectory_start_time;
  for (int index = 0; index <= sample_count; ++index)
  {
    if (PlanningClock::now() >= deadline) return {};
    const double time = sample_count > 0 ?
        trajectory_start_time + remaining_duration * static_cast<double>(index) / sample_count :
        trajectory_start_time;
    const Eigen::Vector3d position = root_trajectory.getPos(time);
    const Eigen::Vector3d velocity = root_trajectory.getVel(time);
    if (!position.allFinite() || !velocity.allFinite()) return {};
    if (index > 0)
    {
      attitude = advanceRootAttitude(attitude, velocity, time - previous_time, config, command_pitch);
    }
    samples.push_back({output_time_offset + time - trajectory_start_time, attitude});
    previous_time = time;
  }
  return samples;
}

TerminalJointTargetResult computeTerminalJointTarget(
    const Trajectory<5>& root_trajectory, double trajectory_start_time,
    const RootAttitude& terminal_attitude, const WholeBodyConfiguration& aligned_body,
    const DragonCollisionGeometry& geometry, double ik_singularity_threshold,
    PlanningClock::time_point deadline)
{
  TerminalJointTargetResult result;
  const auto fail = [&](const std::string& reason) {
    result.success = false;
    result.joints.resize(0);
    result.target_positions.clear();
    result.detail = PlanningClock::now() >= deadline ?
                        "terminal target generation deadline expired" : reason;
    return result;
  };
  if (PlanningClock::now() >= deadline) return fail("");
  if (root_trajectory.getPieceNum() <= 0 || !std::isfinite(trajectory_start_time) ||
      trajectory_start_time < 0.0 || !std::isfinite(terminal_attitude.yaw) ||
      !std::isfinite(terminal_attitude.pitch) || !aligned_body.link1_tail.allFinite() ||
      !aligned_body.root_link_rotation.allFinite() || !aligned_body.joint_positions.allFinite() ||
      !aligned_body.root_link_rotation.isUnitary(1e-6) ||
      std::abs(aligned_body.root_link_rotation.determinant() - 1.0) > 1e-6 ||
      geometry.link_num <= 0 || !std::isfinite(geometry.link_length) || geometry.link_length <= 0.0 ||
      !std::isfinite(ik_singularity_threshold) || ik_singularity_threshold < 0.0 ||
      geometry.pitch_joint_indices.size() != static_cast<size_t>(geometry.link_num - 1) ||
      geometry.yaw_joint_indices.size() != static_cast<size_t>(geometry.link_num - 1))
  {
    return fail("invalid terminal target geometry or aligned state");
  }
  std::vector<bool> used(static_cast<size_t>(aligned_body.joint_positions.size()), false);
  for (int link = 0; link < geometry.link_num - 1; ++link)
  {
    for (const int joint : {geometry.pitch_joint_indices[link], geometry.yaw_joint_indices[link]})
    {
      if (joint < 0 || joint >= aligned_body.joint_positions.size() || used[joint])
        return fail("invalid terminal target joint mapping");
      used[joint] = true;
    }
  }
  for (int piece = 0; piece < root_trajectory.getPieceNum(); ++piece)
  {
    if (PlanningClock::now() >= deadline) return fail("");
    if (!std::isfinite(root_trajectory[piece].getDuration()) ||
        root_trajectory[piece].getDuration() < 0.0 ||
        !root_trajectory[piece].getCoeffMat().allFinite())
      return fail("invalid terminal target root trajectory");
  }
  const double duration = root_trajectory.getTotalDuration();
  if (!std::isfinite(duration) || trajectory_start_time > duration ||
      !root_trajectory.getPos(duration).allFinite() ||
      !root_trajectory.getPos(trajectory_start_time).allFinite() ||
      (root_trajectory.getPos(trajectory_start_time) - aligned_body.link1_tail).norm() >
          kTraceDistanceTolerance)
    return fail("terminal target root trajectory does not match aligned state");

  const std::vector<Eigen::Vector3d> endpoints = linkEndpoints(
      aligned_body.link1_tail, aligned_body.root_link_rotation, aligned_body.joint_positions,
      geometry.pitch_joint_indices, geometry.yaw_joint_indices, geometry.link_num, geometry.link_length);
  std::vector<TerminalTraceSegment> trace;
  trace.reserve(static_cast<size_t>(geometry.link_num - 1 + root_trajectory.getPieceNum()));
  // Oldest to newest: linkN tail -> ... -> link2 tail -> aligned root tail.
  for (int link = geometry.link_num; link > 1; --link)
  {
    TerminalTraceSegment segment;
    segment.coefficients.col(5) = endpoints[link];
    segment.coefficients.col(4) = endpoints[link - 1] - endpoints[link];
    if (!segment.coefficients.allFinite()) return fail("non-finite initial body geometry");
    trace.push_back(segment);
  }
  double piece_start = 0.0;
  for (int piece = 0; piece < root_trajectory.getPieceNum(); ++piece)
  {
    if (PlanningClock::now() >= deadline) return fail("");
    const double piece_duration = root_trajectory[piece].getDuration();
    const double piece_end = piece_start + piece_duration;
    if (piece_duration > 0.0 && piece_end > trajectory_start_time)
    {
      TerminalTraceSegment segment;
      segment.coefficients = root_trajectory[piece].normalizePosCoeffMat();
      segment.lower = std::max(0.0, (trajectory_start_time - piece_start) / piece_duration);
      if (!segment.coefficients.allFinite()) return fail("non-finite normalized MINCO coefficients");
      const Eigen::Vector3d previous_tail = trace.empty() ? aligned_body.link1_tail :
          trace.back().position(trace.back().upper);
      if ((segment.position(segment.lower) - previous_tail).norm() > kTraceDistanceTolerance)
        return fail("discontinuous terminal target root trajectory");
      trace.push_back(segment);
    }
    piece_start = piece_end;
  }
  Eigen::Vector3d head = root_trajectory.getPos(duration);
  if (geometry.link_num > 1)
  {
    if (trace.empty()) return fail("empty terminal target trace");
    TerminalTraceCursor cursor{trace.size() - 1, trace.back().upper};
    for (int link = 2; link <= geometry.link_num; ++link)
    {
      Eigen::Vector3d tail;
      if (!findTerminalTail(trace, head, geometry.link_length, deadline, cursor, tail))
        return fail("failed to solve terminal target trace intersections");
      if (!tail.allFinite() || (tail - head).norm() < kTraceDistanceTolerance)
        return fail("degenerate terminal target link direction");
      result.target_positions.push_back(tail);
      head = tail;
    }
  }
  if (PlanningClock::now() >= deadline) return fail("");
  result.joints = multilink_copilot::follow_the_leader::computeJointAngles(
      result.target_positions, root_trajectory.getPos(duration), linkRotation(terminal_attitude),
      geometry.pitch_joint_indices, geometry.yaw_joint_indices, aligned_body.joint_positions.size(),
      ik_singularity_threshold, aligned_body.joint_positions);
  if (!result.joints.allFinite()) return fail("non-finite terminal joint target");
  if (PlanningClock::now() >= deadline) return fail("");
  result.success = true;
  return result;
}

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
  const TimedJointWaypoint* after = activeWaypointEnd(joint_waypoints, time);
  if (!after)
  {
    return Eigen::VectorXd::Zero(joint_waypoints.front().positions.size());
  }
  const TimedJointWaypoint& before = after[-1];
  const double duration = after->time - before.time;
  if (duration <= kEpsilon)
  {
    return Eigen::VectorXd::Zero(before.positions.size());
  }
  return (after->positions - before.positions) / duration;
}

RootAttitude JointPlanResult::attitude(double time) const
{
  if (attitude_waypoints.empty())
  {
    return RootAttitude();
  }
  if (time <= attitude_waypoints.front().time)
  {
    return attitude_waypoints.front().attitude;
  }
  const size_t upper = upperWaypointIndex(attitude_waypoints, time);
  if (upper >= attitude_waypoints.size())
  {
    return attitude_waypoints.back().attitude;
  }
  const TimedRootAttitudeWaypoint& before = attitude_waypoints[upper - 1];
  const TimedRootAttitudeWaypoint& after = attitude_waypoints[upper];
  const double duration = after.time - before.time;
  const double ratio = duration > kEpsilon ? (time - before.time) / duration : 1.0;
  return interpolateRootAttitude(before.attitude, after.attitude, ratio);
}

Eigen::Matrix3d JointPlanResult::rootLinkRotation(double time) const
{
  return linkRotation(attitude(time));
}

Eigen::Vector3d JointPlanResult::angularVelocity(double time) const
{
  const TimedRootAttitudeWaypoint* after = activeWaypointEnd(attitude_waypoints, time);
  if (!after)
  {
    return Eigen::Vector3d::Zero();
  }
  const TimedRootAttitudeWaypoint& before = after[-1];
  const double duration = after->time - before.time;
  if (duration <= kEpsilon)
  {
    return Eigen::Vector3d::Zero();
  }
  const double yaw_rate = shortestYawDelta(before.attitude.yaw, after->attitude.yaw) / duration;
  const double pitch_rate = (after->attitude.pitch - before.attitude.pitch) / duration;
  return worldAngularVelocity(attitude(time), yaw_rate, pitch_rate);
}

double JointPlanResult::yaw(double time) const
{
  return attitude(time).yaw;
}

double JointPlanResult::yawRate(double time) const
{
  return angularVelocity(time).z();
}

double JointPlanResult::pitch(double time) const
{
  return attitude(time).pitch;
}

double JointPlanResult::pitchRate(double time) const
{
  const TimedRootAttitudeWaypoint* after = activeWaypointEnd(attitude_waypoints, time);
  if (!after)
  {
    return 0.0;
  }
  const TimedRootAttitudeWaypoint& before = after[-1];
  const double duration = after->time - before.time;
  return duration > kEpsilon ? (after->attitude.pitch - before.attitude.pitch) / duration : 0.0;
}

JointTrajectoryPlanner::JointTrajectoryPlanner(
    const JointPlannerConfig& config,
    const std::shared_ptr<multilink_copilot::StabilityEvaluator>& stability_evaluator)
  : config_(config), stability_evaluator_(stability_evaluator)
{
  if (!stability_evaluator_)
  {
    throw std::invalid_argument("Invalid joint trajectory planner configuration");
  }
  config_.validateOrThrow();
  seedOmplOnce(config_.random_seed);
}

Eigen::VectorXd JointTrajectoryPlanner::joint1PriorityConfiguration(
    const Eigen::VectorXd& start_joints,
    int joint1_pitch,
    int joint1_yaw,
    const std::vector<double>& lower,
    const std::vector<double>& upper,
    const RootAttitude& start_attitude,
    const RootAttitude& goal_attitude,
    double progress)
{
  if (joint1_pitch < 0 || joint1_pitch >= start_joints.size() ||
      joint1_yaw < 0 || joint1_yaw >= start_joints.size() ||
      lower.size() != static_cast<size_t>(start_joints.size()) ||
      upper.size() != static_cast<size_t>(start_joints.size()))
  {
    return Eigen::VectorXd();
  }
  const double clamped_progress = std::max(0.0, std::min(1.0, progress));
  const RootAttitude attitude = interpolateRootAttitude(
      start_attitude, goal_attitude, clamped_progress);
  Eigen::VectorXd joints = start_joints;
  const double pitch_delta = attitude.pitch - start_attitude.pitch;
  const double yaw_delta = shortestYawDelta(start_attitude.yaw, attitude.yaw);
  // The root link points opposite the FLU x axis. Consequently a root pitch is
  // cancelled by the same signed joint1 pitch, whereas yaw remains opposite.
  joints(joint1_pitch) = std::max(
      lower[static_cast<size_t>(joint1_pitch)],
      std::min(upper[static_cast<size_t>(joint1_pitch)],
               start_joints(joint1_pitch) + pitch_delta));
  joints(joint1_yaw) = std::max(
      lower[static_cast<size_t>(joint1_yaw)],
      std::min(upper[static_cast<size_t>(joint1_yaw)],
               start_joints(joint1_yaw) - yaw_delta));
  return joints;
}

JointPlanResult JointTrajectoryPlanner::plan(const Trajectory<5>& root_trajectory,
                                             const NominalJointContext& context,
                                             const Eigen::VectorXd& start_joints,
                                             const RootAttitude& start_attitude,
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
  const double root_duration = root_trajectory.getTotalDuration();
  if (!std::isfinite(root_duration) || root_duration <= kEpsilon)
  {
    result.detail = "invalid root trajectory duration";
    return result;
  }

  Eigen::VectorXd origin = start_joints;
  multilink_copilot::StabilityMetrics origin_metrics;
  bool repaired_start = false;
  if (!configurationIsSafe(origin, start_attitude, &origin_metrics))
  {
    if (!repairEndpoint(start_joints, start_joints, start_attitude, true, origin) ||
        !configurationIsSafe(origin, start_attitude, &origin_metrics))
    {
      result.detail = "start joint configuration could not be repaired";
      return result;
    }
    repaired_start = true;
  }

  RootAttitude alignment_goal = start_attitude;
  bool found_tangent = false;
  const int tangent_sample_count = std::max(
      1, static_cast<int>(std::ceil(root_duration / config_.reference_dt)));
  for (int sample = 1; sample <= tangent_sample_count; ++sample)
  {
    const double time = root_duration * static_cast<double>(sample) / tangent_sample_count;
    const Eigen::Vector3d velocity = root_trajectory.getVel(time);
    if (velocity.norm() > 1e-3)
    {
      alignment_goal = tangentAttitude(velocity, start_attitude);
      found_tangent = true;
      break;
    }
  }
  if (!found_tangent)
  {
    const Eigen::Vector3d chord = root_trajectory.getPos(root_duration) -
                                  root_trajectory.getPos(0.0);
    alignment_goal = tangentAttitude(chord, start_attitude);
  }
  if (!config_.follower.publish_yaw_command)
  {
    alignment_goal.yaw = start_attitude.yaw;
  }

  const double yaw_delta = shortestYawDelta(start_attitude.yaw, alignment_goal.yaw);
  const double pitch_delta = alignment_goal.pitch - start_attitude.pitch;
  const bool attitude_change = std::abs(yaw_delta) > kEpsilon ||
                               std::abs(pitch_delta) > kEpsilon;

  if (context.pitch_joint_indices.empty() || context.yaw_joint_indices.empty())
  {
    result.detail = "missing joint1 pitch/yaw mapping";
    return result;
  }
  const int joint1_pitch = context.pitch_joint_indices.front();
  const int joint1_yaw = context.yaw_joint_indices.front();
  if (joint1_pitch < 0 || joint1_pitch >= origin.size() ||
      joint1_yaw < 0 || joint1_yaw >= origin.size())
  {
    result.detail = "invalid joint1 pitch/yaw mapping";
    return result;
  }
  const std::vector<double>& lower =
      stability_evaluator_->robotModel()->getLinkJointLowerLimits();
  const std::vector<double>& upper =
      stability_evaluator_->robotModel()->getLinkJointUpperLimits();
  if (lower.size() != static_cast<size_t>(origin.size()) ||
      upper.size() != static_cast<size_t>(origin.size()))
  {
    result.detail = "invalid DRAGON joint limits";
    return result;
  }

  const Eigen::VectorXd alignment_joints = joint1PriorityConfiguration(
      origin, joint1_pitch, joint1_yaw, lower, upper,
      start_attitude, alignment_goal, 1.0);
  if (alignment_joints.size() != origin.size())
  {
    result.detail = "failed to allocate root attitude to joint1";
    return result;
  }

  constexpr double kCommandStepSchedulingMargin = 0.99;
  const double effective_joint_rate = std::min(
      config_.follower.max_angular_vel,
      kCommandStepSchedulingMargin * config_.max_joint_command_step *
          config_.follower.command_hz);
  double required_alignment_duration = 0.0;
  const auto include_alignment_duration = [&](double delta, int joint) {
    if (std::abs(delta) > kEpsilon)
    {
      required_alignment_duration = std::max(
          required_alignment_duration,
          std::abs(delta) / config_.follower.max_angular_vel);
      const double joint_delta = std::abs(alignment_joints(joint) - origin(joint));
      if (joint_delta > kEpsilon)
      {
        required_alignment_duration = std::max(
            required_alignment_duration, joint_delta / effective_joint_rate);
      }
    }
  };
  include_alignment_duration(yaw_delta, joint1_yaw);
  include_alignment_duration(pitch_delta, joint1_pitch);

  const bool stationary_start = root_trajectory.getVel(0.0).norm() < 1e-3;
  const double recovery_duration = repaired_start ? 1.0 / config_.follower.command_hz : 0.0;
  double alignment_start_time = recovery_duration;
  double alignment_end_time = recovery_duration;
  double trajectory_start_time = 0.0;
  double output_time_offset = 0.0;
  double root_delay = 0.0;
  if (attitude_change)
  {
    alignment_end_time += std::max(required_alignment_duration,
                                   1.0 / config_.follower.command_hz);
    if (stationary_start)
    {
      root_delay = alignment_end_time;
      output_time_offset = root_delay;
    }
    else
    {
      if (alignment_end_time >= root_duration - kEpsilon)
      {
        result.detail = "moving root trajectory is too short for joint1-priority allocation";
        return result;
      }
      trajectory_start_time = alignment_end_time;
      output_time_offset = trajectory_start_time;
    }
  }
  else if (repaired_start && stationary_start)
  {
    root_delay = recovery_duration;
    output_time_offset = root_delay;
  }

  result.joint_waypoints.reserve(8);
  if (repaired_start)
  {
    result.joint_waypoints.push_back({0.0, start_joints});
    result.joint_waypoints.push_back({recovery_duration, origin});
  }
  else
  {
    result.joint_waypoints.push_back({0.0, origin});
  }
  result.attitude_waypoints.push_back({0.0, start_attitude});
  if (alignment_start_time > kEpsilon)
  {
    result.attitude_waypoints.push_back({alignment_start_time, start_attitude});
  }

  if (attitude_change)
  {
    std::vector<double> progress_values{0.0, 1.0};
    const double absorbed_pitch = alignment_joints(joint1_pitch) - origin(joint1_pitch);
    const double absorbed_yaw = origin(joint1_yaw) - alignment_joints(joint1_yaw);
    const auto include_saturation_progress = [&](double absorbed, double delta) {
      if (std::abs(delta) > kEpsilon)
      {
        const double fraction = std::abs(absorbed / delta);
        if (fraction > kEpsilon && fraction < 1.0 - kEpsilon)
        {
          progress_values.push_back(fraction);
        }
      }
    };
    include_saturation_progress(absorbed_pitch, pitch_delta);
    include_saturation_progress(absorbed_yaw, yaw_delta);
    std::sort(progress_values.begin(), progress_values.end());
    progress_values.erase(std::unique(progress_values.begin(), progress_values.end(),
                                      [](double lhs, double rhs) {
                                        return std::abs(lhs - rhs) <= kEpsilon;
                                      }),
                          progress_values.end());
    for (const double progress : progress_values)
    {
      if (progress <= kEpsilon)
      {
        continue;
      }
      const Eigen::VectorXd joints = joint1PriorityConfiguration(
          origin, joint1_pitch, joint1_yaw, lower, upper,
          start_attitude, alignment_goal, progress);
      result.joint_waypoints.push_back(
          {alignment_start_time + progress * (alignment_end_time - alignment_start_time),
           joints});
    }
    result.attitude_waypoints.push_back({alignment_end_time, alignment_goal});
  }

  // Reject an infeasible deterministic prefix before invoking the global RRT.
  // In particular, the RRT is never allowed to repair joint1 allocation by
  // moving any other link joint during this interval.
  double prefix_minimum_fc_rp = origin_metrics.fc_rp_min;
  if (attitude_change)
  {
    const auto& prefix_schedule = result.attitude_waypoints;
    TimedJointWaypoint previous{alignment_start_time, origin};
    for (const TimedJointWaypoint& waypoint : result.joint_waypoints)
    {
      if (waypoint.time <= alignment_start_time + kEpsilon)
      {
        continue;
      }
      if (!timedConfigurationPathIsSafe(previous, waypoint, prefix_schedule,
                                        prefix_minimum_fc_rp))
      {
        result.detail = "joint1-priority interval is infeasible";
        return result;
      }
      previous = waypoint;
    }
  }

  const std::vector<TimedRootAttitudeWaypoint> attitude_schedule = predictRootAttitudes(
      root_trajectory, alignment_goal, config_.follower, config_.reference_dt, true,
      trajectory_start_time, output_time_offset, deadline);
  if (attitude_schedule.empty())
  {
    result.detail = "failed to predict root attitude schedule";
    return result;
  }
  const RootAttitude terminal_attitude = attitude_schedule.back().attitude;
  const WholeBodyConfiguration aligned_body{
      root_trajectory.getPos(trajectory_start_time), linkRotation(alignment_goal), alignment_joints};
  const DragonCollisionGeometry geometry{
      context.link_num, context.link_length, context.pitch_joint_indices, context.yaw_joint_indices};
  const TerminalJointTargetResult terminal = computeTerminalJointTarget(
      root_trajectory, trajectory_start_time, terminal_attitude, aligned_body, geometry,
      config_.follower.ik_singularity_threshold, deadline);
  if (!terminal.success)
  {
    result.detail = terminal.detail;
    return result;
  }
  Eigen::VectorXd goal = terminal.joints;
  multilink_copilot::StabilityMetrics goal_metrics;
  bool repaired_goal = false;
  if (!configurationIsSafe(goal, terminal_attitude, &goal_metrics))
  {
    if (!repairEndpoint(terminal.joints, alignment_joints, terminal_attitude,
                        false, goal) ||
        !configurationIsSafe(goal, terminal_attitude, &goal_metrics))
    {
      result.detail = "terminal joint configuration could not be repaired";
      return result;
    }
    repaired_goal = true;
  }

  // One global RRT-Connect retains the existing whole-body morphology search,
  // starting only after the deterministic joint1-priority interval.
  std::vector<Eigen::VectorXd> sampled_path;
  if ((alignment_joints - goal).norm() <= kEpsilon)
  {
    sampled_path = {alignment_joints, goal};
  }
  else if (!searchJointPath(alignment_joints, goal,
                            linkRotation(terminal_attitude), deadline,
                            sampled_path) || sampled_path.size() < 2)
  {
    result.detail = "global joint-space RRT failed";
    return result;
  }
  std::vector<Eigen::VectorXd> joint_path =
      shortcutChain(sampled_path, linkRotation(terminal_attitude));
  if (joint_path.size() < 2 || budgetExpired())
  {
    result.detail = "global joint-space RRT shortcut failed";
    return result;
  }
  joint_path.front() = alignment_joints;
  joint_path.back() = goal;

  const double total_base_duration = root_delay + root_duration;
  const double rrt_start_time = stationary_start ? root_delay : alignment_end_time;
  if (total_base_duration <= rrt_start_time + kEpsilon &&
      (alignment_joints - goal).norm() > kEpsilon)
  {
    result.detail = "joint1-priority interval leaves no time for the global joint path";
    return result;
  }
  const std::vector<double> path_ratios = chainRatios(joint_path);
  for (size_t index = 1; index < joint_path.size(); ++index)
  {
    result.joint_waypoints.push_back(
        {rrt_start_time + path_ratios[index] * (total_base_duration - rrt_start_time),
         joint_path[index]});
  }
  result.joint_waypoints.front().time = 0.0;
  result.joint_waypoints.back().time = total_base_duration;

  for (const TimedRootAttitudeWaypoint& sample : attitude_schedule)
  {
    if (!result.attitude_waypoints.empty() &&
        std::abs(result.attitude_waypoints.back().time - sample.time) <= kEpsilon)
    {
      result.attitude_waypoints.back().attitude = sample.attitude;
    }
    else
    {
      result.attitude_waypoints.push_back({sample.time, sample.attitude});
    }
  }

  result.minimum_fc_rp = std::min(prefix_minimum_fc_rp, goal_metrics.fc_rp_min);
  const auto& complete_schedule = result.attitude_waypoints;
  const size_t safe_origin_index = repaired_start ? 1u : 0u;
  multilink_copilot::StabilityMetrics timed_origin_metrics;
  if (safe_origin_index >= result.joint_waypoints.size() ||
      !configurationIsSafe(
          result.joint_waypoints[safe_origin_index].positions,
          scheduledAttitudeAt(complete_schedule,
                            result.joint_waypoints[safe_origin_index].time),
          &timed_origin_metrics))
  {
    result.detail = "repaired joint origin is infeasible under the root-attitude schedule";
    return result;
  }
  result.minimum_fc_rp =
      std::min(result.minimum_fc_rp, timed_origin_metrics.fc_rp_min);
  for (size_t index = safe_origin_index + 1;
       index < result.joint_waypoints.size(); ++index)
  {
    if (!timedConfigurationPathIsSafe(result.joint_waypoints[index - 1],
                                      result.joint_waypoints[index], complete_schedule,
                                      result.minimum_fc_rp))
    {
      result.detail =
          result.joint_waypoints[index].time <=
                  alignment_end_time + kEpsilon ?
              "joint1-priority interval is infeasible" :
              "global joint path is infeasible under the root-attitude schedule";
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
    const double required = std::max(
        maximum_delta / config_.follower.max_angular_vel,
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

  for (size_t index = 1; index < result.attitude_waypoints.size(); ++index)
  {
    const TimedRootAttitudeWaypoint& before = result.attitude_waypoints[index - 1];
    const TimedRootAttitudeWaypoint& after = result.attitude_waypoints[index];
    const double available = after.time - before.time;
    if (available <= kEpsilon)
    {
      continue;
    }
    time_scale = std::max(
        time_scale,
        std::abs(shortestYawDelta(before.attitude.yaw, after.attitude.yaw)) /
            (config_.follower.max_angular_vel * available));
    time_scale = std::max(
        time_scale,
        std::abs(after.attitude.pitch - before.attitude.pitch) /
            (config_.follower.max_angular_vel * available));
  }

  result.time_scale = std::max(1.0, time_scale);
  if (!stationary_start && result.time_scale > 1.0 + kEpsilon)
  {
    result.detail = "moving retarget would violate root state continuity after time scaling";
    return result;
  }
  for (TimedJointWaypoint& waypoint : result.joint_waypoints)
  {
    waypoint.time *= result.time_scale;
  }
  for (TimedRootAttitudeWaypoint& waypoint : result.attitude_waypoints)
  {
    waypoint.time *= result.time_scale;
  }
  result.root_translation_delay = root_delay * result.time_scale;
  result.duration = total_base_duration * result.time_scale;
  if (!computeTrackingError(root_trajectory, context, result) ||
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
  const Eigen::Matrix3d root_link_rotation =
      Eigen::AngleAxisd(root_link_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  std::vector<Eigen::VectorXd> sampled_path;
  if (!searchJointPath(start, goal, root_link_rotation, deadline_, sampled_path) ||
      sampled_path.size() < 2)
  {
    if (failure_reason) *failure_reason = "global joint-space RRT failed";
    return false;
  }

  path = shortcutChain(sampled_path, root_link_rotation);
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
    const std::vector<Eigen::VectorXd>& chain,
    const Eigen::Matrix3d& root_link_rotation)
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
      const Eigen::VectorXd delta = chain[next] - chain[current];
      const double maximum_delta = delta.size() > 0 ? delta.cwiseAbs().maxCoeff() : 0.0;
      const int subdivisions = std::max(
          1, static_cast<int>(std::ceil(maximum_delta / coarse_resolution)));
      bool safe = true;
      for (int subdivision = 1; subdivision < subdivisions; ++subdivision)
      {
        const double ratio = static_cast<double>(subdivision) / subdivisions;
        if (budgetExpired() ||
            !configurationIsSafe(chain[current] + ratio * delta,
                                 root_link_rotation))
        {
          safe = false;
          break;
        }
      }
      if (safe)
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
    const Eigen::Matrix3d& root_link_rotation,
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
  setup.setStateValidityChecker([this, root_link_rotation](const ompl::base::State* state) {
    const auto* vector_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
    Eigen::VectorXd joints(stability_evaluator_->jointCount());
    for (int index = 0; index < joints.size(); ++index)
    {
      joints(index) = vector_state->values[index];
    }
    return configurationIsSafe(joints, root_link_rotation);
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
  return configurationIsSafe(
      joints, Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix(), metrics);
}

bool JointTrajectoryPlanner::configurationIsSafe(
    const Eigen::VectorXd& joints, const RootAttitude& attitude,
    multilink_copilot::StabilityMetrics* metrics)
{
  return configurationIsSafe(joints, linkRotation(attitude), metrics);
}

bool JointTrajectoryPlanner::configurationIsSafe(
    const Eigen::VectorXd& joints, const Eigen::Matrix3d& root_link_rotation,
    multilink_copilot::StabilityMetrics* metrics)
{
  const Eigen::Quaterniond quaternion(root_link_rotation);
  stability_evaluator_->setRootLinkRotation(KDL::Rotation::Quaternion(
      quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()));
  multilink_copilot::StabilityMetrics evaluated;
  const bool valid = stability_evaluator_->evaluate(joints, evaluated) && evaluated.safe;
  if (metrics)
  {
    *metrics = evaluated;
  }
  return valid;
}

RootAttitude JointTrajectoryPlanner::scheduledAttitudeAt(
    const std::vector<TimedRootAttitudeWaypoint>& samples, double time)
{
  if (samples.empty())
  {
    return RootAttitude();
  }
  if (time <= samples.front().time)
  {
    return samples.front().attitude;
  }
  const size_t upper = upperWaypointIndex(samples, time);
  if (upper >= samples.size())
  {
    return samples.back().attitude;
  }
  const TimedRootAttitudeWaypoint& before = samples[upper - 1];
  const TimedRootAttitudeWaypoint& after = samples[upper];
  const double duration = after.time - before.time;
  const double ratio = duration > kEpsilon ? (time - before.time) / duration : 1.0;
  return interpolateRootAttitude(before.attitude, after.attitude, ratio);
}

bool JointTrajectoryPlanner::timedConfigurationPathIsSafe(
    const TimedJointWaypoint& start, const TimedJointWaypoint& goal,
    const std::vector<TimedRootAttitudeWaypoint>& attitude_schedule, double& minimum_fc_rp)
{
  if (start.positions.size() != goal.positions.size() || attitude_schedule.empty() ||
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

  // The root attitude is piecewise linear. Add enough samples on every
  // overlapping segment that neither joint nor root-attitude motion can step
  // over a thin infeasible shell.
  for (size_t index = 1; index < attitude_schedule.size(); ++index)
  {
    const double interval_start = std::max(start.time, attitude_schedule[index - 1].time);
    const double interval_end = std::min(goal.time, attitude_schedule[index].time);
    if (interval_end <= interval_start + kEpsilon)
    {
      continue;
    }
    const RootAttitude attitude_start = scheduledAttitudeAt(attitude_schedule, interval_start);
    const RootAttitude attitude_end = scheduledAttitudeAt(attitude_schedule, interval_end);
    const double attitude_delta = std::max(
        std::abs(shortestYawDelta(attitude_start.yaw, attitude_end.yaw)),
        std::abs(attitude_end.pitch - attitude_start.pitch));
    const int attitude_subdivisions = std::max(
        1, static_cast<int>(std::ceil(
               attitude_delta / config_.validity_resolution)));
    for (int subdivision = 0; subdivision <= attitude_subdivisions; ++subdivision)
    {
      times.push_back(interval_start +
                      (interval_end - interval_start) *
                          static_cast<double>(subdivision) / attitude_subdivisions);
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
    if (!configurationIsSafe(joints, scheduledAttitudeAt(attitude_schedule, time), &metrics))
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
    JointPlanResult& result) const
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
      downstream_link_count <= 0 || result.attitude_waypoints.empty())
  {
    return true;
  }

  const double required_history =
      static_cast<double>(downstream_link_count) * context.link_length;
  const Eigen::Vector3d initial_root_tail = root_trajectory.getPos(0.0);
  const Eigen::Matrix3d initial_root_rotation = result.rootLinkRotation(0.0);
  const std::deque<multilink_copilot::TrajectoryPoint> initial_trace =
      context.executed_history.arcLength() < required_history
          ? multilink_copilot::follow_the_leader::prependCurrentBodyMorphology(
                context.executed_history.points(), initial_root_tail,
                initial_root_rotation, result.jointPositions(0.0),
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
    const double root_time = std::max(
        0.0, std::min(root_trajectory.getTotalDuration(),
                      (scaled_time - result.root_translation_delay) /
                          result.time_scale));
    const Eigen::Vector3d root_position = root_trajectory.getPos(root_time);
    if ((trace.back() - root_position).norm() > kEpsilon)
    {
      trace.push_back(root_position);
    }
    const Eigen::Matrix3d root_rotation = result.rootLinkRotation(scaled_time);
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
                                            const RootAttitude& attitude,
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
  if (configurationIsSafe(desired, attitude))
  {
    repaired = desired;
    return true;
  }

  const Eigen::Quaterniond quaternion(linkRotation(attitude));
  stability_evaluator_->setRootLinkRotation(KDL::Rotation::Quaternion(
      quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()));
  return stability_evaluator_->projectToSafe(
             desired, reference, repaired, allow_unstable_seed) &&
         configurationIsSafe(repaired, attitude);
}

}  // namespace motion_primitive_planner
