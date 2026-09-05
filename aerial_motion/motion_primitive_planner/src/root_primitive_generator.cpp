#include <motion_primitive_planner/root_primitive_generator.h>

#include <gcopter/minco.hpp>

#include <algorithm>
#include <cmath>

namespace motion_primitive_planner
{
namespace
{
constexpr double kEpsilon = 1e-6;
constexpr double kMinimumChordLength = 1e-3;
constexpr double kShortRouteOffsetRatio = 0.4;

Eigen::Vector3d safeNormal(const Eigen::Vector3d& direction)
{
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ().cross(direction);
  if (normal.norm() < kEpsilon)
  {
    normal = Eigen::Vector3d::UnitX().cross(direction);
  }
  return normal.normalized();
}
}  // namespace

PrimitiveGenerator::PrimitiveGenerator(const PrimitiveConfig& config) : config_(config)
{
  config_.validateOrThrow();
}

std::vector<Eigen::Vector3d> PrimitiveGenerator::candidateRoute(
    const Eigen::Vector3d& start, const Eigen::Vector3d& target,
    int candidate_index) const
{
  if (candidate_index == 0 || config_.candidate_count == 1)
  {
    return {start, target};
  }

  const Eigen::Vector3d chord = target - start;
  const double chord_length = chord.norm();
  const Eigen::Vector3d forward = chord.normalized();
  const Eigen::Vector3d normal = safeNormal(forward);
  const Eigen::Vector3d binormal = forward.cross(normal).normalized();
  const int alternative_count = config_.candidate_count - 1;
  const int radial_level_count = alternative_count >= 8 ? 2 : 1;
  const int direction_count = (alternative_count + radial_level_count - 1) / radial_level_count;
  const int radial_level = (candidate_index - 1) / direction_count + 1;
  const int direction_index = (candidate_index - 1) % direction_count;
  const double angle = 2.0 * M_PI * static_cast<double>(direction_index) /
                       static_cast<double>(direction_count);
  const double maximum_amplitude =
      std::min(config_.max_offset, kShortRouteOffsetRatio * chord_length);
  const double amplitude = maximum_amplitude * static_cast<double>(radial_level) /
                           static_cast<double>(radial_level_count);
  const Eigen::Vector3d offset_direction = std::cos(angle) * normal + std::sin(angle) * binormal;
  return {start, 0.5 * (start + target) + amplitude * offset_direction, target};
}

bool PrimitiveGenerator::buildTrajectory(const std::vector<Eigen::Vector3d>& route,
                                         const Eigen::Matrix3d& initial_state,
                                         const Eigen::Matrix3d& final_state,
                                         Candidate& candidate) const
{
  const int piece_count = static_cast<int>(route.size()) - 1;
  if (piece_count <= 0)
  {
    return false;
  }

  Eigen::Matrix3Xd inner_points(3, std::max(0, piece_count - 1));
  for (int index = 0; index < piece_count - 1; ++index)
  {
    inner_points.col(index) = route[static_cast<size_t>(index + 1)];
  }
  Eigen::VectorXd durations(piece_count);
  for (int index = 0; index < piece_count; ++index)
  {
    const double distance = (route[static_cast<size_t>(index + 1)] - route[static_cast<size_t>(index)]).norm();
    durations(index) = std::max(config_.minimum_piece_duration, distance / config_.cruise_velocity);
  }

  minco::MINCO_S3NU minco;
  minco.setConditions(initial_state, final_state, piece_count);
  for (int iteration = 0; iteration < 5; ++iteration)
  {
    minco.setParameters(inner_points, durations);
    minco.getTrajectory(candidate.trajectory);
    const double maximum_velocity = candidate.trajectory.getMaxVelRate();
    if (!std::isfinite(maximum_velocity))
    {
      return false;
    }
    if (maximum_velocity <= config_.max_velocity * (1.0 + 1e-6))
    {
      minco.getEnergy(candidate.jerk_energy);
      candidate.path_length = sampledLength(candidate.trajectory);
      return std::isfinite(candidate.jerk_energy) && std::isfinite(candidate.path_length);
    }
    durations *= maximum_velocity / (0.95 * config_.max_velocity);
  }
  return false;
}

std::vector<Candidate> PrimitiveGenerator::generate(const Eigen::Matrix3d& initial_state,
                                                    const Eigen::Matrix3d& final_state) const
{
  std::vector<Candidate> candidates(static_cast<size_t>(config_.candidate_count));
  const Eigen::Vector3d start = initial_state.col(0);
  const Eigen::Vector3d target = final_state.col(0);
  if (!initial_state.allFinite() || !final_state.allFinite() ||
      (target - start).norm() <= kMinimumChordLength)
  {
    for (Candidate& candidate : candidates)
    {
      candidate.status = CandidateStatus::kGenerationFailed;
      candidate.detail = "invalid endpoint state or near-zero chord";
    }
    return candidates;
  }

  for (int index = 0; index < config_.candidate_count; ++index)
  {
    Candidate& candidate = candidates[static_cast<size_t>(index)];
    if (!buildTrajectory(candidateRoute(start, target, index),
                         initial_state, final_state, candidate))
    {
      candidate.status = CandidateStatus::kGenerationFailed;
      candidate.detail = "MINCO generation or velocity enforcement failed";
    }
  }
  return candidates;
}

double PrimitiveGenerator::sampledLength(const Trajectory<5>& trajectory)
{
  if (trajectory.getPieceNum() <= 0)
  {
    return 0.0;
  }
  const double duration = trajectory.getTotalDuration();
  const int sample_count = std::max(20, trajectory.getPieceNum() * 20);
  double length = 0.0;
  Eigen::Vector3d previous = trajectory.getPos(0.0);
  for (int index = 1; index <= sample_count; ++index)
  {
    const Eigen::Vector3d current = trajectory.getPos(duration * static_cast<double>(index) / sample_count);
    length += (current - previous).norm();
    previous = current;
  }
  return length;
}

PlanningEnvironment::PlanningEnvironment(const SharedPlannerConfig& config)
  : config_(config), generator_(config.primitive)
{
  replaceMap({});
}

void PlanningEnvironment::replaceMap(
    const std::vector<Eigen::Vector3d>& occupied_voxel_centers)
{
  std::shared_ptr<gcopter_planner::PlannerBackend> backend(
      new gcopter_planner::PlannerBackend(config_.common));
  backend->setMapPoints(occupied_voxel_centers);
  std::atomic_store(&backend_, backend);
}

std::shared_ptr<const gcopter_planner::PlannerBackend>
PlanningEnvironment::occupancySnapshot() const
{
  return std::atomic_load(&backend_);
}

bool PlanningEnvironment::occupied(const Eigen::Vector3d& point) const
{
  return occupancySnapshot()->query(point);
}

double PlanningEnvironment::voxelScale() const
{
  return occupancySnapshot()->voxelScale();
}

Eigen::Vector3d PlanningEnvironment::mapOrigin() const
{
  return occupancySnapshot()->mapOrigin();
}

Eigen::Vector3d PlanningEnvironment::mapCorner() const
{
  return occupancySnapshot()->mapCorner();
}

Eigen::Vector3d PlanningEnvironment::clampTarget(const Eigen::Vector3d& requested,
                                                 double clearance) const
{
  return occupancySnapshot()->clampInsideMap(requested, clearance);
}

Eigen::Vector3d PlanningEnvironment::truncateRoute(const std::vector<Eigen::Vector3d>& full_route,
                                                   double horizon,
                                                   std::vector<Eigen::Vector3d>& local_route)
{
  local_route.clear();
  if (full_route.empty())
  {
    return Eigen::Vector3d::Zero();
  }
  local_route.push_back(full_route.front());
  double length = 0.0;
  for (size_t index = 1; index < full_route.size(); ++index)
  {
    const double segment = (full_route[index] - full_route[index - 1]).norm();
    if (length + segment >= horizon)
    {
      const double ratio = segment > kEpsilon ? (horizon - length) / segment : 0.0;
      local_route.push_back(full_route[index - 1] + ratio * (full_route[index] - full_route[index - 1]));
      return local_route.back();
    }
    length += segment;
    local_route.push_back(full_route[index]);
  }
  return local_route.back();
}

PrimitiveBatch PlanningEnvironment::generate(const RootState& start, const Eigen::Vector3d& target)
{
  PrimitiveBatch result;
  const std::shared_ptr<gcopter_planner::PlannerBackend> backend = std::atomic_load(&backend_);
  if (backend->query(start.position))
  {
    result.failure = PrimitiveBatchFailure::kStartCollision;
    result.detail = "root start is in collision";
    return result;
  }
  std::vector<Eigen::Vector3d> full_route;
  if (!backend->searchPath(start.position, target, full_route))
  {
    result.failure = PrimitiveBatchFailure::kRouteSearchFailed;
    result.detail = "root route search failed";
    return result;
  }
  result.local_target = truncateRoute(full_route, config_.planning_horizon, result.local_route);
  if (result.local_route.size() < 2)
  {
    result.failure = PrimitiveBatchFailure::kLocalRouteEmpty;
    result.detail = "local route is empty";
    return result;
  }
  result.terminal = (result.local_target - full_route.back()).norm() <= kEpsilon;
  Eigen::Vector3d final_velocity = Eigen::Vector3d::Zero();
  const Eigen::Vector3d tangent =
      result.local_route.back() - result.local_route[result.local_route.size() - 2];
  if (!config_.zero_local_target_vel && !result.terminal && tangent.norm() > kEpsilon)
  {
    final_velocity = config_.primitive.cruise_velocity * tangent.normalized();
  }
  Eigen::Matrix3d initial_state;
  initial_state.col(0) = start.position;
  initial_state.col(1) = start.velocity;
  initial_state.col(2) = start.acceleration;
  Eigen::Matrix3d final_state;
  final_state.col(0) = result.local_target;
  final_state.col(1) = final_velocity;
  final_state.col(2).setZero();
  result.candidates = generator_.generate(initial_state, final_state);
  return result;
}

}  // namespace motion_primitive_planner
