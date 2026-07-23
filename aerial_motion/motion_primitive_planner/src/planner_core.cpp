#include <motion_primitive_planner/planner_core.h>

#include <gcopter/minco.hpp>
#include <multilink_copilot/follow_the_leader.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

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
  if (config_.candidate_count <= 0 || !std::isfinite(config_.max_offset) || config_.max_offset < 0.0 ||
      !std::isfinite(config_.max_velocity) || config_.max_velocity <= 0.0 ||
      !std::isfinite(config_.cruise_velocity) || config_.cruise_velocity <= 0.0 ||
      !std::isfinite(config_.minimum_piece_duration) || config_.minimum_piece_duration <= 0.0)
  {
    throw std::invalid_argument("Invalid motion primitive configuration");
  }
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

int selectBestCandidate(const std::vector<Candidate>& candidates,
                        bool allow_stability_projection_fallback)
{
  int best = -1;
  for (size_t index = 0; index < candidates.size(); ++index)
  {
    if (!candidates[index].feasible())
    {
      continue;
    }
    if (best < 0 || candidates[index].path_length < candidates[static_cast<size_t>(best)].path_length - 1e-6 ||
        (std::abs(candidates[index].path_length - candidates[static_cast<size_t>(best)].path_length) <= 1e-6 &&
         candidates[index].jerk_energy < candidates[static_cast<size_t>(best)].jerk_energy))
    {
      best = static_cast<int>(index);
    }
  }
  if (best >= 0 || !allow_stability_projection_fallback)
  {
    return best;
  }

  // A root primitive whose nominal follow-the-leader shape has a low
  // roll/pitch margin is still executable when the downstream Copilot can
  // project that shape to a stable joint target.  Keep the root motion small
  // so that the projected command is less likely to hit Copilot's joint-step
  // or baselink-tilt publication gates.  For equivalent root paths, preserve
  // the current fold by minimizing nominal joint motion.  Use nominal margin
  // and smoothness as the remaining tie-breakers.
  for (size_t index = 0; index < candidates.size(); ++index)
  {
    const Candidate& candidate = candidates[index];
    if (candidate.status != CandidateStatus::kStabilityProjection ||
        !candidate.requires_stability_projection || !std::isfinite(candidate.min_fc_rp))
    {
      continue;
    }
    if (best < 0 || candidate.path_length < candidates[static_cast<size_t>(best)].path_length - 1e-6 ||
        (std::abs(candidate.path_length - candidates[static_cast<size_t>(best)].path_length) <= 1e-6 &&
         (candidate.joint_motion < candidates[static_cast<size_t>(best)].joint_motion - 1e-6 ||
          (std::abs(candidate.joint_motion - candidates[static_cast<size_t>(best)].joint_motion) <= 1e-6 &&
           (candidate.min_fc_rp > candidates[static_cast<size_t>(best)].min_fc_rp + 1e-6 ||
            (std::abs(candidate.min_fc_rp - candidates[static_cast<size_t>(best)].min_fc_rp) <= 1e-6 &&
             candidate.jerk_energy < candidates[static_cast<size_t>(best)].jerk_energy))))))
    {
      best = static_cast<int>(index);
    }
  }
  return best;
}

int selectBestWholeBodyCandidate(const std::vector<WholeBodyCandidateScore>& candidates)
{
  int best = -1;
  for (size_t index = 0; index < candidates.size(); ++index)
  {
    const WholeBodyCandidateScore& candidate = candidates[index];
    if (!candidate.feasible)
    {
      continue;
    }
    if (best < 0)
    {
      best = static_cast<int>(index);
      continue;
    }
    const WholeBodyCandidateScore& current = candidates[static_cast<size_t>(best)];
    if (candidate.duration < current.duration - kEpsilon ||
        (std::abs(candidate.duration - current.duration) <= kEpsilon &&
         (candidate.joint_motion < current.joint_motion - kEpsilon ||
          (std::abs(candidate.joint_motion - current.joint_motion) <= kEpsilon &&
           candidate.root_jerk < current.root_jerk))))
    {
      best = static_cast<int>(index);
    }
  }
  return best;
}

RootCommandKinematics tailFluToRootLinkCommand(const Eigen::Vector3d& tail_position,
                                                const Eigen::Vector3d& tail_velocity,
                                                double tail_yaw,
                                                double tail_yaw_rate,
                                                double link_length)
{
  RootCommandKinematics command;
  command.yaw = tail_yaw + M_PI;
  command.yaw_rate = tail_yaw_rate;
  const Eigen::Vector3d link_direction(std::cos(command.yaw), std::sin(command.yaw), 0.0);
  const Eigen::Vector3d angular_velocity(0.0, 0.0, tail_yaw_rate);
  command.position = tail_position - link_length * link_direction;
  command.linear_velocity = tail_velocity - link_length * angular_velocity.cross(link_direction);
  return command;
}

std::vector<Eigen::Vector3d> linkEndpoints(const Eigen::Vector3d& link1_tail,
                                           const Eigen::Matrix3d& root_link_rotation,
                                           const Eigen::VectorXd& joint_positions,
                                           const std::vector<int>& pitch_joint_indices,
                                           const std::vector<int>& yaw_joint_indices,
                                           int link_num,
                                           double link_length)
{
  std::vector<Eigen::Vector3d> endpoints;
  if (link_num <= 0 || link_length <= 0.0)
  {
    return endpoints;
  }
  endpoints.reserve(static_cast<size_t>(link_num + 1));
  Eigen::Matrix3d rotation = root_link_rotation;
  Eigen::Vector3d position = link1_tail - link_length * rotation.col(0);
  endpoints.push_back(position);
  endpoints.push_back(link1_tail);
  position = link1_tail;
  for (int link = 1; link < link_num; ++link)
  {
    const size_t joint = static_cast<size_t>(link - 1);
    const int pitch_index = joint < pitch_joint_indices.size() ? pitch_joint_indices[joint] : -1;
    const int yaw_index = joint < yaw_joint_indices.size() ? yaw_joint_indices[joint] : -1;
    const double pitch = pitch_index >= 0 && pitch_index < joint_positions.size() ? joint_positions(pitch_index) : 0.0;
    const double yaw = yaw_index >= 0 && yaw_index < joint_positions.size() ? joint_positions(yaw_index) : 0.0;
    rotation = rotation * multilink_copilot::follow_the_leader::rotationAroundY(pitch) *
               multilink_copilot::follow_the_leader::rotationAroundZ(yaw);
    position += link_length * rotation.col(0);
    endpoints.push_back(position);
  }
  return endpoints;
}

bool bodyCollides(const std::vector<Eigen::Vector3d>& endpoints,
                  double sample_spacing,
                  const std::function<bool(const Eigen::Vector3d&)>& occupied)
{
  if (endpoints.size() < 2 || sample_spacing <= 0.0 || !occupied)
  {
    return true;
  }
  for (size_t segment = 1; segment < endpoints.size(); ++segment)
  {
    const Eigen::Vector3d delta = endpoints[segment] - endpoints[segment - 1];
    const int sample_count = std::max(1, static_cast<int>(std::ceil(delta.norm() / sample_spacing)));
    for (int sample = 0; sample <= sample_count; ++sample)
    {
      if (occupied(endpoints[segment - 1] + static_cast<double>(sample) / sample_count * delta))
      {
        return true;
      }
    }
  }
  return false;
}

bool wholeBodyCollides(const WholeBodyConfiguration& configuration,
                       const DragonCollisionGeometry& geometry,
                       double sample_spacing,
                       const std::function<bool(const Eigen::Vector3d&)>& occupied)
{
  const std::vector<Eigen::Vector3d> endpoints =
      linkEndpoints(configuration.link1_tail, configuration.root_link_rotation,
                    configuration.joint_positions, geometry.pitch_joint_indices,
                    geometry.yaw_joint_indices, geometry.link_num, geometry.link_length);
  return bodyCollides(endpoints, sample_spacing, occupied);
}

double shortestYawDelta(double start_yaw, double end_yaw)
{
  if (!std::isfinite(start_yaw) || !std::isfinite(end_yaw))
  {
    throw std::invalid_argument("Yaw endpoints must be finite");
  }
  return std::remainder(end_yaw - start_yaw, 2.0 * M_PI);
}

int wholeBodyMotionSubdivisionCount(double interval_duration,
                                    double maximum_root_velocity,
                                    double body_length,
                                    double yaw_delta,
                                    const Eigen::VectorXd& joint_delta,
                                    double spatial_resolution)
{
  if (!std::isfinite(interval_duration) || interval_duration < 0.0 ||
      !std::isfinite(maximum_root_velocity) || maximum_root_velocity < 0.0 ||
      !std::isfinite(body_length) || body_length < 0.0 ||
      !std::isfinite(yaw_delta) || !joint_delta.allFinite() ||
      !std::isfinite(spatial_resolution) || spatial_resolution <= 0.0)
  {
    throw std::invalid_argument("Invalid whole-body motion subdivision input");
  }
  const double angular_delta =
      std::abs(yaw_delta) + (joint_delta.size() > 0 ? joint_delta.cwiseAbs().sum() : 0.0);
  const double displacement_bound =
      maximum_root_velocity * interval_duration + body_length * angular_delta;
  if (!std::isfinite(displacement_bound))
  {
    throw std::overflow_error("Whole-body motion displacement bound overflowed");
  }
  const double subdivisions = std::ceil(displacement_bound / spatial_resolution);
  if (subdivisions > static_cast<double>(std::numeric_limits<int>::max()))
  {
    throw std::overflow_error("Whole-body motion subdivision count overflowed");
  }
  return std::max(1, static_cast<int>(subdivisions));
}

const char* candidateStatusName(CandidateStatus status)
{
  switch (status)
  {
    case CandidateStatus::kUnevaluated: return "unevaluated";
    case CandidateStatus::kGenerationFailed: return "generation_failed";
    case CandidateStatus::kCollision: return "collision";
    case CandidateStatus::kJointLimit: return "joint_limit";
    case CandidateStatus::kStability: return "stability";
    case CandidateStatus::kStabilityProjection: return "stability_projection";
    case CandidateStatus::kJointPlanningFailed: return "joint_planning_failed";
    case CandidateStatus::kFeasible: return "feasible";
    case CandidateStatus::kSelected: return "selected";
  }
  return "unknown";
}

}  // namespace motion_primitive_planner
