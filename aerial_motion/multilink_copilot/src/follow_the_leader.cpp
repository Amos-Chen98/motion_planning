#include <multilink_copilot/follow_the_leader.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace multilink_copilot
{
namespace follow_the_leader
{
namespace
{
constexpr double kDirectionNormEpsilon = 1e-6;
constexpr double kWarmupDistanceTolerance = 0.02;
constexpr double kSearchParameterEpsilon = 1e-9;

struct TrajectorySearchCursor
{
  int segment_index = -1;
  double alpha = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();

  bool valid() const { return segment_index >= 0; }
};

struct TrajectorySearchResult
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  TrajectorySearchCursor cursor;
  double distance_error = std::numeric_limits<double>::infinity();
};

double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

double clampUnit(double value)
{
  return std::max(-1.0, std::min(1.0, value));
}

double smoothstep01(double value)
{
  value = clamp01(value);
  return value * value * (3.0 - 2.0 * value);
}

std::vector<Eigen::Vector3d> buildPolyline(const std::deque<TrajectoryPoint>& trajectory_buffer,
                                           const Eigen::Vector3d& current_position)
{
  std::vector<Eigen::Vector3d> polyline;
  polyline.reserve(trajectory_buffer.size() + 1);
  for (const TrajectoryPoint& point : trajectory_buffer)
  {
    polyline.push_back(point.position);
  }
  if (polyline.empty() || (polyline.back() - current_position).norm() > kDirectionNormEpsilon)
  {
    polyline.push_back(current_position);
  }
  return polyline;
}

Eigen::Vector3d interpolate(const std::vector<Eigen::Vector3d>& polyline, int segment_index, double alpha)
{
  return polyline[segment_index] + alpha * (polyline[segment_index + 1] - polyline[segment_index]);
}

TrajectorySearchCursor normalizeCursor(const std::vector<Eigen::Vector3d>& polyline,
                                       const TrajectorySearchCursor& raw_cursor)
{
  TrajectorySearchCursor cursor = raw_cursor;
  if (polyline.size() < 2 || cursor.segment_index < 0 || cursor.segment_index + 1 >= static_cast<int>(polyline.size()))
  {
    cursor.segment_index = -1;
    return cursor;
  }
  cursor.alpha = clamp01(cursor.alpha);
  cursor.position = interpolate(polyline, cursor.segment_index, cursor.alpha);
  if (cursor.alpha <= kSearchParameterEpsilon && cursor.segment_index > 0)
  {
    --cursor.segment_index;
    cursor.alpha = 1.0;
    cursor.position = polyline[cursor.segment_index + 1];
  }
  return cursor;
}

TrajectorySearchCursor newestCursor(const std::vector<Eigen::Vector3d>& polyline)
{
  TrajectorySearchCursor cursor;
  if (polyline.size() >= 2)
  {
    cursor.segment_index = static_cast<int>(polyline.size()) - 2;
    cursor.alpha = 1.0;
    cursor.position = polyline.back();
  }
  return cursor;
}

void updateBest(const Eigen::Vector3d& from_point,
                double target_distance,
                const Eigen::Vector3d& segment_start,
                const Eigen::Vector3d& segment_delta,
                int segment_index,
                double alpha_start,
                double u,
                TrajectorySearchResult& best)
{
  const double clamped_u = clamp01(u);
  const Eigen::Vector3d candidate = segment_start + clamped_u * segment_delta;
  const double error = std::abs((candidate - from_point).norm() - target_distance);
  if (error >= best.distance_error)
  {
    return;
  }
  best.position = candidate;
  best.cursor.segment_index = segment_index;
  best.cursor.alpha = alpha_start * (1.0 - clamped_u);
  best.cursor.position = candidate;
  best.distance_error = error;
}

TrajectorySearchResult findAtDistance(const std::vector<Eigen::Vector3d>& polyline,
                                      const TrajectorySearchCursor& start_cursor,
                                      const Eigen::Vector3d& from_point,
                                      double target_distance)
{
  TrajectorySearchResult best;
  best.position = polyline.front();
  best.cursor.segment_index = 0;
  best.cursor.position = polyline.front();
  best.distance_error = std::abs((polyline.front() - from_point).norm() - target_distance);

  const TrajectorySearchCursor cursor = normalizeCursor(polyline, start_cursor);
  if (!cursor.valid())
  {
    return best;
  }

  for (int segment_index = cursor.segment_index; segment_index >= 0; --segment_index)
  {
    const double alpha_start = segment_index == cursor.segment_index ? cursor.alpha : 1.0;
    const Eigen::Vector3d segment_start =
        segment_index == cursor.segment_index ? cursor.position : polyline[segment_index + 1];
    const Eigen::Vector3d segment_delta = polyline[segment_index] - segment_start;
    const double length_squared = segment_delta.squaredNorm();
    if (length_squared < kDirectionNormEpsilon * kDirectionNormEpsilon)
    {
      updateBest(from_point, target_distance, segment_start, segment_delta, segment_index, alpha_start, 0.0, best);
      continue;
    }

    const Eigen::Vector3d relative_start = segment_start - from_point;
    const double b = 2.0 * segment_delta.dot(relative_start);
    const double c = relative_start.squaredNorm() - target_distance * target_distance;
    const double discriminant = b * b - 4.0 * length_squared * c;
    if (discriminant >= -kSearchParameterEpsilon)
    {
      const double root = std::sqrt(std::max(0.0, discriminant));
      const double denominator = 0.5 / length_squared;
      const double candidates[2] = {(-b - root) * denominator, (-b + root) * denominator};
      double best_u = std::numeric_limits<double>::infinity();
      for (double candidate : candidates)
      {
        if (candidate >= -kSearchParameterEpsilon && candidate <= 1.0 + kSearchParameterEpsilon &&
            candidate < best_u)
        {
          best_u = clamp01(candidate);
        }
      }
      if (std::isfinite(best_u))
      {
        TrajectorySearchResult result;
        result.position = segment_start + best_u * segment_delta;
        result.cursor.segment_index = segment_index;
        result.cursor.alpha = alpha_start * (1.0 - best_u);
        result.cursor.position = result.position;
        result.cursor = normalizeCursor(polyline, result.cursor);
        result.distance_error = std::abs((result.position - from_point).norm() - target_distance);
        return result;
      }
    }

    updateBest(from_point, target_distance, segment_start, segment_delta, segment_index, alpha_start, 0.0, best);
    updateBest(from_point, target_distance, segment_start, segment_delta, segment_index, alpha_start, 1.0, best);
    const double projection = clamp01((from_point - segment_start).dot(segment_delta) /
                                      std::max(length_squared, kDirectionNormEpsilon));
    updateBest(from_point, target_distance, segment_start, segment_delta, segment_index, alpha_start, projection, best);
  }
  best.cursor = normalizeCursor(polyline, best.cursor);
  return best;
}

Eigen::Vector3d oldestBackwardDirection(const std::deque<TrajectoryPoint>& trajectory_buffer,
                                        const Eigen::Vector3d& fallback_direction)
{
  for (size_t i = 0; i + 1 < trajectory_buffer.size(); ++i)
  {
    const Eigen::Vector3d segment = trajectory_buffer[i + 1].position - trajectory_buffer[i].position;
    if (segment.norm() >= kDirectionNormEpsilon)
    {
      return -segment.normalized();
    }
  }
  return fallback_direction.norm() >= kDirectionNormEpsilon ? -fallback_direction.normalized()
                                                            : Eigen::Vector3d(-1.0, 0.0, 0.0);
}

Eigen::Vector3d extrapolateAtDistance(const Eigen::Vector3d& from_point,
                                      const Eigen::Vector3d& ray_origin,
                                      const Eigen::Vector3d& ray_direction,
                                      double target_distance)
{
  const Eigen::Vector3d direction = ray_direction.norm() >= kDirectionNormEpsilon
                                        ? ray_direction.normalized()
                                        : Eigen::Vector3d(-1.0, 0.0, 0.0);
  const Eigen::Vector3d delta = ray_origin - from_point;
  const double b = delta.dot(direction);
  const double discriminant = b * b - (delta.squaredNorm() - target_distance * target_distance);
  if (discriminant >= 0.0)
  {
    const double root = std::sqrt(discriminant);
    const double candidates[2] = {-b - root, -b + root};
    double best = std::numeric_limits<double>::infinity();
    for (double candidate : candidates)
    {
      if (candidate >= 0.0 && candidate < best)
      {
        best = candidate;
      }
    }
    if (std::isfinite(best))
    {
      return ray_origin + best * direction;
    }
  }
  return ray_origin + std::max(0.0, -b) * direction;
}
}  // namespace

Eigen::Matrix3d rotationAroundY(double angle)
{
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  rotation(0, 0) = cosine;
  rotation(0, 2) = sine;
  rotation(2, 0) = -sine;
  rotation(2, 2) = cosine;
  return rotation;
}

Eigen::Matrix3d rotationAroundZ(double angle)
{
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  rotation(0, 0) = cosine;
  rotation(0, 1) = -sine;
  rotation(1, 0) = sine;
  rotation(1, 1) = cosine;
  return rotation;
}

std::vector<Eigen::Vector3d> computeTargetPositions(const std::deque<TrajectoryPoint>& trajectory_buffer,
                                                    const Eigen::Vector3d& current_position,
                                                    const Eigen::Vector3d& fallback_link_direction,
                                                    int link_num,
                                                    double link_length,
                                                    bool extend_short_history)
{
  std::vector<Eigen::Vector3d> targets;
  targets.reserve(static_cast<size_t>(std::max(0, link_num - 1)));
  const std::vector<Eigen::Vector3d> polyline = buildPolyline(trajectory_buffer, current_position);
  if (polyline.size() < 2 || link_num <= 1 || link_length <= 0.0)
  {
    return targets;
  }

  Eigen::Vector3d head = current_position;
  TrajectorySearchCursor cursor = newestCursor(polyline);
  const Eigen::Vector3d backward_direction = oldestBackwardDirection(trajectory_buffer, fallback_link_direction);
  bool extending = false;
  for (int link = 2; link <= link_num; ++link)
  {
    Eigen::Vector3d target;
    if (!extending)
    {
      const TrajectorySearchResult result = findAtDistance(polyline, cursor, head, link_length);
      if (!extend_short_history || result.distance_error <= kWarmupDistanceTolerance)
      {
        target = result.position;
        cursor = result.cursor;
      }
      else
      {
        extending = true;
        target = extrapolateAtDistance(head, trajectory_buffer.front().position, backward_direction, link_length);
      }
    }
    else
    {
      target = head + backward_direction * link_length;
    }
    targets.push_back(target);
    head = target;
  }
  return targets;
}

Eigen::VectorXd computeJointAngles(const std::vector<Eigen::Vector3d>& target_positions,
                                   const Eigen::Vector3d& link1_tail_position,
                                   const Eigen::Matrix3d& root_link_rotation,
                                   const std::vector<int>& pitch_joint_indices,
                                   const std::vector<int>& yaw_joint_indices,
                                   int joint_num,
                                   double singularity_xz_norm_threshold,
                                   const Eigen::VectorXd& reference_joint_positions)
{
  Eigen::VectorXd joints = Eigen::VectorXd::Zero(joint_num);
  Eigen::Matrix3d link_rotation = root_link_rotation;
  Eigen::Vector3d tail = link1_tail_position;
  for (size_t index = 0; index < target_positions.size(); ++index)
  {
    const Eigen::Vector3d segment = target_positions[index] - tail;
    if (segment.norm() < kDirectionNormEpsilon)
    {
      continue;
    }
    Eigen::Vector3d local_direction = link_rotation.transpose() * segment.normalized();
    if (local_direction.norm() < kDirectionNormEpsilon)
    {
      tail = target_positions[index];
      continue;
    }
    local_direction.normalize();

    const int pitch_index = index < pitch_joint_indices.size() ? pitch_joint_indices[index] : -1;
    const int yaw_index = index < yaw_joint_indices.size() ? yaw_joint_indices[index] : -1;
    const double xz_norm = std::hypot(local_direction.x(), local_direction.z());
    double pitch = 0.0;
    if (singularity_xz_norm_threshold > 0.0 && xz_norm < singularity_xz_norm_threshold)
    {
      pitch = pitch_index >= 0 && pitch_index < reference_joint_positions.size()
                  ? reference_joint_positions(pitch_index)
                  : 0.0;
    }
    else if (xz_norm >= kDirectionNormEpsilon)
    {
      pitch = std::atan2(-local_direction.z(), local_direction.x());
    }
    const double yaw = std::asin(clampUnit(local_direction.y()));
    if (pitch_index >= 0 && pitch_index < joint_num)
    {
      joints(pitch_index) = pitch;
    }
    if (yaw_index >= 0 && yaw_index < joint_num)
    {
      joints(yaw_index) = yaw;
    }
    link_rotation = link_rotation * rotationAroundY(pitch) * rotationAroundZ(yaw);
    tail = target_positions[index];
  }
  return joints;
}

Eigen::VectorXd computeWarmupJointPositions(const Eigen::VectorXd& current_joint_positions,
                                            const Eigen::VectorXd& path_joint_positions,
                                            double total_arc_length,
                                            double link_length,
                                            const std::vector<int>& pitch_joint_indices,
                                            const std::vector<int>& yaw_joint_indices)
{
  Eigen::VectorXd desired = current_joint_positions;
  const double activation_distance = std::max(link_length, kDirectionNormEpsilon);
  const size_t segment_count = std::max(pitch_joint_indices.size(), yaw_joint_indices.size());
  for (size_t index = 0; index < segment_count; ++index)
  {
    const double progress = (total_arc_length - static_cast<double>(index) * link_length) / activation_distance;
    const double activation = smoothstep01(progress);
    const int indices[2] = {
        index < pitch_joint_indices.size() ? pitch_joint_indices[index] : -1,
        index < yaw_joint_indices.size() ? yaw_joint_indices[index] : -1};
    for (int joint_index : indices)
    {
      if (joint_index >= 0 && joint_index < desired.size() && joint_index < path_joint_positions.size())
      {
        desired(joint_index) += activation * (path_joint_positions(joint_index) - desired(joint_index));
      }
    }
  }
  return desired;
}

}  // namespace follow_the_leader
}  // namespace multilink_copilot
