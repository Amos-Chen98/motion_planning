#include <motion_primitive_planner/root_primitive_generator.h>
#include <motion_primitive_planner/dragon_collision_checker.h>

#include <gcopter/minco.hpp>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <tuple>

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
  const gcopter_planner::RoutePlannerBackend backend(config_.common);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = backend.mapOrigin();
  replaceMap(std::make_shared<const octomap::OcTree>(backend.voxelScale()), transform,
             backend.mapOrigin(), backend.mapCorner());
}

void PlanningEnvironment::replaceMap(
    std::shared_ptr<const octomap::OcTree> tree, const Eigen::Isometry3d& world_from_grid,
    const Eigen::Vector3d& origin, const Eigen::Vector3d& corner, const ros::Time& stamp)
{
  auto backend = std::make_shared<gcopter_planner::RoutePlannerBackend>(config_.common);
  if (!tree || std::abs(tree->getResolution() - backend->voxelScale()) > 1e-9 ||
      !origin.isApprox(backend->mapOrigin(), 1e-9) || !corner.isApprox(backend->mapCorner(), 1e-9))
    throw std::invalid_argument("OctoMap grid does not match the planning grid");
  auto collision = std::make_shared<CollisionEnvironment>(tree, world_from_grid, origin, corner);
  std::vector<Eigen::Vector3d> cells;
  const double width = tree->getResolution();
  for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it)
  {
    if (!tree->isNodeOccupied(*it)) continue;
    const Eigen::Vector3d center(it.getX(), it.getY(), it.getZ());
    // A pruned leaf covers 2^k base cells per axis. Integer bounds avoid floating
    // gaps and do not expand or mutate the OctoMap used by collision queries.
    const Eigen::Vector3d lower = (center.array() - it.getSize() / 2).matrix() / width;
    const Eigen::Vector3d upper = (center.array() + it.getSize() / 2).matrix() / width;
    const Eigen::Vector3i begin = lower.array().round().cast<int>().matrix().cwiseMax(Eigen::Vector3i::Zero());
    const Eigen::Vector3i end = upper.array().round().cast<int>().matrix().cwiseMin(backend->mapSize());
    for (int x = begin.x(); x < end.x(); ++x)
      for (int y = begin.y(); y < end.y(); ++y)
        for (int z = begin.z(); z < end.z(); ++z)
          cells.push_back(origin + width * Eigen::Vector3d(x + .5, y + .5, z + .5));
  }
  backend->setMapPoints(cells);
  auto scene = std::make_shared<PlanningSceneSnapshot>();
  scene->route = backend;
  scene->collision = std::move(collision);
  scene->map_stamp = stamp;
  std::atomic_store(&scene_, std::shared_ptr<const PlanningSceneSnapshot>(scene));
}

std::shared_ptr<const PlanningSceneSnapshot>
PlanningEnvironment::snapshot() const
{
  return std::atomic_load(&scene_);
}

bool PlanningEnvironment::occupied(const Eigen::Vector3d& point) const
{
  return snapshot()->route->query(point);
}

double PlanningEnvironment::voxelScale() const
{
  return snapshot()->route->voxelScale();
}

Eigen::Vector3d PlanningEnvironment::mapOrigin() const
{
  return snapshot()->route->mapOrigin();
}

Eigen::Vector3d PlanningEnvironment::mapCorner() const
{
  return snapshot()->route->mapCorner();
}

Eigen::Vector3d PlanningEnvironment::clampTarget(const Eigen::Vector3d& requested,
                                                 double clearance) const
{
  return snapshot()->route->clampInsideMap(requested, clearance);
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

void PlanningEnvironment::replaceScene(std::shared_ptr<const PlanningSceneSnapshot> scene)
{
  if (!scene || !scene->route || !scene->collision) throw std::invalid_argument("Incomplete planning scene");
  std::atomic_store(&scene_, std::move(scene));
}

PrimitiveBatch PlanningEnvironment::generate(const RootState& start, const Eigen::Vector3d& target)
{
  return generate(start, target, snapshot());
}

PrimitiveBatch PlanningEnvironment::generate(const RootState& start, const Eigen::Vector3d& target,
                                           std::shared_ptr<const PlanningSceneSnapshot> scene)
{
  PrimitiveBatch result;
  const auto backend = scene->route;
  if (!start.position.allFinite() || !target.allFinite() || backend->query(start.position))
  {
    result.failure = PrimitiveBatchFailure::kStartCollision;
    result.detail = "root start is in collision or outside local coverage";
    return result;
  }
  using Clock = std::chrono::steady_clock;
  const auto search_start = Clock::now();
  const auto deadline = search_start + std::chrono::duration<double>(config_.common.timeoutRRT);
  const double clearance = config_.common.dilateRadius + backend->voxelScale();
  const Eigen::Vector3d low = backend->mapOrigin() + Eigen::Vector3d::Constant(clearance);
  const Eigen::Vector3d high = backend->mapCorner() - Eigen::Vector3d::Constant(clearance);
  const bool target_inside = (target.array() >= low.array()).all() && (target.array() < high.array()).all();
  Eigen::Vector3d projected = target;
  if (!target_inside && scene->epoch != 0) {
    // Clip the actual start-to-goal segment against the inset window. In
    // particular, a start near a window edge must not change this direction.
    const Eigen::Vector3d direction = target - start.position;
    double enter = 0, leave = 1;
    for (int a = 0; a < 3; ++a) {
      const double upper = std::nextafter(high[a], low[a]);
      if (std::abs(direction[a]) <= kEpsilon) {
        if (start.position[a] < low[a] || start.position[a] > upper) leave = -1;
        continue;
      }
      double first = (low[a]-start.position[a])/direction[a];
      double last = (upper-start.position[a])/direction[a];
      if (first > last) std::swap(first, last);
      enter = std::max(enter, first);
      leave = std::min(leave, last);
    }
    if (leave < enter) {
      result.failure = PrimitiveBatchFailure::kRouteSearchFailed;
      result.detail = "goal direction does not enter the inset local window";
      return result;
    }
    projected = start.position + leave * direction;
  }
  std::vector<Eigen::Vector3d> endpoints;
  if (!backend->query(projected)) endpoints.push_back(projected);
  // Keep fixed-map fixtures and legacy users' endpoint behavior. Received local
  // snapshots carry an epoch and enable rolling goal selection.
  if (scene->epoch != 0) {
    struct Endpoint { Eigen::Vector3d point; double projection_distance, goal_distance; long key; };
    std::vector<Endpoint> nearby;
    const double width = backend->voxelScale();
    const Eigen::Vector3i begin = ((projected-Eigen::Vector3d::Ones()-backend->mapOrigin())/width)
        .array().floor().cast<int>().matrix().cwiseMax(Eigen::Vector3i::Zero());
    const Eigen::Vector3i end = ((projected+Eigen::Vector3d::Ones()-backend->mapOrigin())/width)
        .array().floor().cast<int>().matrix().cwiseMin(backend->mapSize()-Eigen::Vector3i::Ones());
    for (int z = begin.z(); z <= end.z(); ++z) for (int y = begin.y(); y <= end.y(); ++y)
      for (int x = begin.x(); x <= end.x(); ++x) {
        const Eigen::Vector3d p = backend->mapOrigin()+width*Eigen::Vector3d(x+.5,y+.5,z+.5);
        const double pd = (p-projected).squaredNorm(), gd = (p-target).squaredNorm();
        if (pd > 1 || pd < 1e-12 || gd >= (start.position-target).squaredNorm() ||
            (p.array() < low.array()).any() || (p.array() >= high.array()).any() || backend->query(p)) continue;
        nearby.push_back({p, pd, gd, backend->voxelKey(p)});
      }
    auto less = [](const Endpoint& a, const Endpoint& b) {
      return std::tie(a.projection_distance,a.goal_distance,a.key) < std::tie(b.projection_distance,b.goal_distance,b.key);
    };
    const size_t count = std::min(9-endpoints.size(), nearby.size());
    std::partial_sort(nearby.begin(), nearby.begin()+count, nearby.end(), less);
    for (size_t i = 0; i < count; ++i) endpoints.push_back(nearby[i].point);
  }
  std::vector<Eigen::Vector3d> full_route;
  bool found = false;
  for (size_t i = 0; i < endpoints.size(); ++i) {
    const double remaining = std::chrono::duration<double>(deadline-Clock::now()).count();
    if (remaining <= 0) break;
    const double budget = i == 0 && endpoints.size() > 1 ? remaining*.5 : remaining/(endpoints.size()-i);
    gcopter_planner::RouteSearchTiming timing;
    if (backend->searchPath(start.position, endpoints[i], full_route, &timing, budget)) {
      found = true;
      result.route_search_timing.first_exact_solution_ms = timing.first_exact_solution_ms;
      break;
    }
  }
  result.route_search_timing.total_ms = std::chrono::duration<double,std::milli>(Clock::now()-search_start).count();
  if (!found) {
    result.failure = PrimitiveBatchFailure::kRouteSearchFailed;
    result.detail = "root route search failed within local window";
    return result;
  }
  result.local_target = truncateRoute(full_route, config_.planning_horizon, result.local_route);
  if (result.local_route.size() < 2)
  {
    result.failure = PrimitiveBatchFailure::kLocalRouteEmpty;
    result.detail = "local route is empty";
    return result;
  }
  result.terminal = (result.local_target - target).norm() <= kEpsilon;
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
