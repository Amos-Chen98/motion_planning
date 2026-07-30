#include <motion_primitive_planner/planner_common.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion_primitive_planner
{
namespace
{
constexpr double kEpsilon = 1e-6;

bool extractSegmentIndex(const std::string& name, const std::string& suffix, int& segment)
{
  constexpr char prefix[] = "joint";
  const size_t suffix_position = name.rfind(suffix);
  if (name.compare(0, sizeof(prefix) - 1, prefix) != 0 || suffix_position == std::string::npos ||
      suffix_position + suffix.size() != name.size())
  {
    return false;
  }
  try
  {
    segment = std::stoi(name.substr(sizeof(prefix) - 1, suffix_position - (sizeof(prefix) - 1))) - 1;
  }
  catch (const std::exception&)
  {
    return false;
  }
  return segment >= 0;
}
}  // namespace

FollowerConfig FollowerConfig::fromRos(const ros::NodeHandle& private_nh)
{
  FollowerConfig config;
  private_nh.param("CommandHz", config.command_hz, config.command_hz);
  private_nh.param("TrajectorySampleInterval", config.trajectory_sample_interval,
                   config.trajectory_sample_interval);
  private_nh.param("TrajectoryBufferMaxLength", config.trajectory_buffer_max_length,
                   config.trajectory_buffer_max_length);
  private_nh.param("SnakeIkSingularityThreshold", config.ik_singularity_threshold,
                   config.ik_singularity_threshold);
  private_nh.param("MaxYawRate", config.max_yaw_rate, config.max_yaw_rate);
  private_nh.param("PublishYawCommand", config.publish_yaw_command, config.publish_yaw_command);
  config.validateOrThrow();
  return config;
}

void FollowerConfig::validateOrThrow() const
{
  if (!std::isfinite(command_hz) || command_hz <= 0.0 ||
      !std::isfinite(trajectory_sample_interval) || trajectory_sample_interval <= 0.0 ||
      !std::isfinite(trajectory_buffer_max_length) || trajectory_buffer_max_length <= 0.0 ||
      !std::isfinite(ik_singularity_threshold) || ik_singularity_threshold < 0.0 ||
      !std::isfinite(max_yaw_rate) || max_yaw_rate < 0.0)
  {
    throw std::invalid_argument("Invalid follow-the-leader configuration");
  }
}

SharedPlannerConfig::SharedPlannerConfig(const ros::NodeHandle& private_nh) : common(private_nh)
{
  private_nh.param("ReplanTriggerRatio", replan_trigger_ratio, replan_trigger_ratio);
  private_nh.param("UseAccumulatedMap", use_accumulated_map, use_accumulated_map);
  private_nh.param("GoalTolerance", goal_tolerance, goal_tolerance);
  private_nh.param("PlanningHorizon", planning_horizon, planning_horizon);
  private_nh.param("ZeroLocalTargetVel", zero_local_target_vel, zero_local_target_vel);
  private_nh.param("CandidateCount", primitive.candidate_count, primitive.candidate_count);
  private_nh.param("MaxPrimitiveOffset", primitive.max_offset, primitive.max_offset);
  private_nh.param("PrimitiveCruiseVelocity", primitive.cruise_velocity, 0.8 * common.maxVelMag);
  private_nh.param("MinimumPieceDuration", primitive.minimum_piece_duration,
                   primitive.minimum_piece_duration);
  primitive.max_velocity = common.maxVelMag;
  validateOrThrow();
}

void SharedPlannerConfig::validateOrThrow() const
{
  common.validateOrThrow();
  if (!std::isfinite(replan_trigger_ratio) ||
      replan_trigger_ratio <= 0.0 || replan_trigger_ratio >= 1.0 ||
      !std::isfinite(goal_tolerance) || goal_tolerance <= 0.0 ||
      !std::isfinite(planning_horizon) || planning_horizon <= 0.0)
  {
    throw std::invalid_argument("Invalid shared motion primitive planner configuration");
  }
  // PrimitiveGenerator performs the complete primitive-specific validation.
  PrimitiveGenerator validation(primitive);
  (void)validation;
}

multilink_copilot::StabilityConfig loadStabilityConfig(const ros::NodeHandle& private_nh)
{
  multilink_copilot::StabilityConfig config;
  private_nh.param("StabilityQpMaxIterations", config.qp_max_iterations, config.qp_max_iterations);
  private_nh.param("StabilityQpJointStepLimit", config.qp_joint_step_limit,
                   config.qp_joint_step_limit);
  private_nh.param("StabilityQpRegularization", config.qp_regularization,
                   config.qp_regularization);
  private_nh.param("StabilityQpConvergenceTolerance", config.qp_convergence_tolerance,
                   config.qp_convergence_tolerance);
  private_nh.param("FeasibilityTolerance", config.feasibility_tolerance,
                   config.feasibility_tolerance);
  private_nh.param("StabilityCheckFcT", config.check_fc_t, config.check_fc_t);
  private_nh.param("FcRpMinThreshold", config.fc_rp_min_threshold, config.fc_rp_min_threshold);
  private_nh.param("FcTMinThreshold", config.fc_t_min_threshold, config.fc_t_min_threshold);
  private_nh.param("StaticThrustMin", config.static_thrust_min, config.static_thrust_min);
  private_nh.param("StaticThrustMax", config.static_thrust_max, config.static_thrust_max);
  private_nh.param("OverlapMinClearance", config.overlap_min_clearance,
                   config.overlap_min_clearance);
  private_nh.param("MaxBaselinkTilt", config.max_baselink_tilt, config.max_baselink_tilt);
  return config;
}

double yawFromQuaternion(const Eigen::Quaterniond& quaternion)
{
  if (quaternion.norm() < kEpsilon)
  {
    return 0.0;
  }
  const Eigen::Quaterniond q = quaternion.normalized();
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion)
{
  return yawFromQuaternion(Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z));
}

double advanceYaw(double current_yaw, const Eigen::Vector3d& velocity, double dt,
                  const FollowerConfig& config)
{
  if (!config.publish_yaw_command || dt <= 0.0 || velocity.head<2>().squaredNorm() <= 1e-6)
  {
    return current_yaw;
  }
  double difference = std::remainder(std::atan2(velocity.y(), velocity.x()) - current_yaw, 2.0 * M_PI);
  if (config.max_yaw_rate > 0.0)
  {
    const double limit = config.max_yaw_rate * dt;
    difference = std::max(-limit, std::min(limit, difference));
  }
  return std::remainder(current_yaw + difference, 2.0 * M_PI);
}

DragonModelInfo::DragonModelInfo(const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& model)
{
  if (!model)
  {
    throw std::invalid_argument("DragonModelInfo requires a DRAGON robot model");
  }
  link_num_ = model->getRotorNum();
  link_length_ = model->getLinkLength();
  link_joint_names_ = model->getLinkJointNames();
  link_joint_indices_ = model->getLinkJointIndices();
  pitch_joint_indices_.assign(static_cast<size_t>(std::max(0, link_num_ - 1)), -1);
  yaw_joint_indices_.assign(static_cast<size_t>(std::max(0, link_num_ - 1)), -1);
  for (size_t index = 0; index < link_joint_names_.size(); ++index)
  {
    joint_name_to_index_[link_joint_names_[index]] = static_cast<int>(index);
    int segment = -1;
    if (extractSegmentIndex(link_joint_names_[index], "_pitch", segment) &&
        segment < static_cast<int>(pitch_joint_indices_.size()))
    {
      pitch_joint_indices_[static_cast<size_t>(segment)] = static_cast<int>(index);
    }
    else if (extractSegmentIndex(link_joint_names_[index], "_yaw", segment) &&
             segment < static_cast<int>(yaw_joint_indices_.size()))
    {
      yaw_joint_indices_[static_cast<size_t>(segment)] = static_cast<int>(index);
    }
  }
  if (link_num_ <= 0 || link_length_ <= 0.0 || link_joint_names_.empty() ||
      link_joint_names_.size() != link_joint_indices_.size() ||
      std::find(pitch_joint_indices_.begin(), pitch_joint_indices_.end(), -1) != pitch_joint_indices_.end() ||
      std::find(yaw_joint_indices_.begin(), yaw_joint_indices_.end(), -1) != yaw_joint_indices_.end())
  {
    throw std::runtime_error("Incomplete DRAGON link geometry or joint mapping");
  }
}

bool DragonModelInfo::readCompleteJointState(const sensor_msgs::JointState& message,
                                             Eigen::VectorXd& joint_positions) const
{
  Eigen::VectorXd measured = joint_positions.size() == jointCount() ?
                                 joint_positions : Eigen::VectorXd::Zero(jointCount());
  std::vector<bool> seen(link_joint_names_.size(), false);
  const size_t count = std::min(message.name.size(), message.position.size());
  for (size_t index = 0; index < count; ++index)
  {
    const auto found = joint_name_to_index_.find(message.name[index]);
    if (found == joint_name_to_index_.end() || !std::isfinite(message.position[index]))
    {
      continue;
    }
    measured(found->second) = message.position[index];
    seen[static_cast<size_t>(found->second)] = true;
  }
  if (std::find(seen.begin(), seen.end(), false) != seen.end())
  {
    return false;
  }
  joint_positions = measured;
  return true;
}

DragonCollisionGeometry DragonModelInfo::collisionGeometry() const
{
  DragonCollisionGeometry geometry;
  geometry.link_num = link_num_;
  geometry.link_length = link_length_;
  geometry.pitch_joint_indices = pitch_joint_indices_;
  geometry.yaw_joint_indices = yaw_joint_indices_;
  return geometry;
}

TrajectoryHistory::TrajectoryHistory(const FollowerConfig& config)
  : sample_interval_(config.trajectory_sample_interval)
  , maximum_length_(config.trajectory_buffer_max_length)
{
  config.validateOrThrow();
}

bool TrajectoryHistory::append(const Eigen::Vector3d& position)
{
  return append(position, sample_interval_, maximum_length_);
}

bool TrajectoryHistory::append(const Eigen::Vector3d& position, double sample_interval,
                               double maximum_length)
{
  if (!position.allFinite() || sample_interval <= 0.0 || maximum_length <= 0.0)
  {
    return false;
  }
  if (points_.empty())
  {
    points_.push_back({position});
    return true;
  }
  const double distance = (position - points_.back().position).norm();
  if (distance < sample_interval)
  {
    return false;
  }
  points_.push_back({position});
  arc_length_ += distance;
  while (points_.size() > 1 && arc_length_ > maximum_length)
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

std::vector<NominalJointSample> NominalJointPredictor::predict(
    const Trajectory<5>& root_trajectory, const NominalJointContext& context,
    const Eigen::VectorXd& start_joints, double start_yaw, double sample_dt) const
{
  std::vector<NominalJointSample> samples;
  if (root_trajectory.getPieceNum() <= 0 || sample_dt <= 0.0 || context.link_num <= 0 ||
      context.link_length <= 0.0 || start_joints.size() <= 0)
  {
    return samples;
  }

  TrajectoryHistory history = context.executed_history;
  Eigen::VectorXd predicted_joints = start_joints;
  double yaw = start_yaw;
  const double duration = root_trajectory.getTotalDuration();
  const int sample_count = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
  const double required_history = static_cast<double>(context.link_num - 1) * context.link_length;
  const Eigen::Vector3d initial_root_tail = root_trajectory.getPos(0.0);
  const Eigen::Matrix3d initial_root_rotation =
      Eigen::AngleAxisd(start_yaw + M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  samples.reserve(static_cast<size_t>(sample_count + 1));

  double previous_time = 0.0;
  for (int sample_index = 0; sample_index <= sample_count; ++sample_index)
  {
    const double time = duration * static_cast<double>(sample_index) / sample_count;
    const Eigen::Vector3d position = root_trajectory.getPos(time);
    const Eigen::Vector3d velocity = root_trajectory.getVel(time);
    if (sample_index > 0)
    {
      yaw = advanceYaw(yaw, velocity, time - previous_time, config_);
    }
    const bool history_changed = history.append(position);

    // Preserve the measured/commanded state exactly at the handover sample.  All
    // later samples use the same FTL reconstruction in both planner modes.
    if (sample_index > 0)
    {
      const Eigen::Matrix3d root_rotation =
          Eigen::AngleAxisd(yaw + M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      const std::deque<multilink_copilot::TrajectoryPoint> nominal_history =
          history.arcLength() < required_history
              ? multilink_copilot::follow_the_leader::prependCurrentBodyMorphology(
                    history.points(), initial_root_tail, initial_root_rotation, start_joints,
                    context.pitch_joint_indices, context.yaw_joint_indices,
                    context.link_num, context.link_length)
              : history.points();
      const std::vector<Eigen::Vector3d> targets =
          multilink_copilot::follow_the_leader::computeTargetPositions(
              nominal_history, position, context.link_num, context.link_length);
      if (!targets.empty())
      {
        predicted_joints = multilink_copilot::follow_the_leader::computeJointAngles(
            targets, position, root_rotation, context.pitch_joint_indices,
            context.yaw_joint_indices, predicted_joints.size(),
            config_.ik_singularity_threshold, predicted_joints);
      }
    }
    samples.push_back({time, yaw, position, predicted_joints, history_changed});
    previous_time = time;
  }
  return samples;
}

PlanningEnvironment::PlanningEnvironment(const SharedPlannerConfig& config)
  : config_(config), generator_(config.primitive)
{
  rebuildMap();
}

void PlanningEnvironment::rebuildMap()
{
  std::shared_ptr<gcopter_planner::PlannerBackend> backend(
      new gcopter_planner::PlannerBackend(config_.common));
  backend->setMapVoxels(occupiedVoxels(*backend));
  std::atomic_store(&backend_, backend);
}

void PlanningEnvironment::updateMap(const std::vector<Eigen::Vector3d>& points)
{
  if (!config_.use_accumulated_map)
  {
    occupied_voxel_keys_.clear();
  }
  const std::shared_ptr<const gcopter_planner::PlannerBackend> backend = occupancySnapshot();
  for (const Eigen::Vector3d& point : points)
  {
    const long key = backend->voxelKey(point);
    if (key >= 0)
    {
      occupied_voxel_keys_.insert(key);
    }
  }
  rebuildMap();
}

std::vector<Eigen::Vector3i> PlanningEnvironment::occupiedVoxels(
    const gcopter_planner::PlannerBackend& backend) const
{
  std::vector<Eigen::Vector3i> occupied;
  occupied.reserve(occupied_voxel_keys_.size());
  for (const long key : occupied_voxel_keys_)
  {
    occupied.push_back(backend.voxelIdFromKey(key));
  }
  return occupied;
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

std_msgs::ColorRGBA candidateColor(CandidateStatus status, bool selected)
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
  else if (status == CandidateStatus::kStability ||
           status == CandidateStatus::kStabilityProjection ||
           status == CandidateStatus::kJointPlanningFailed)
  {
    color.r = 1.0;
    color.g = 0.5;
  }
  else if (status == CandidateStatus::kJointLimit)
  {
    color.r = 1.0;
    color.b = 1.0;
  }
  else
  {
    color.r = 1.0;
  }
  return color;
}

CandidateDiagnosticsPublisher::CandidateDiagnosticsPublisher(
    ros::NodeHandle& nh, const std::string& world_frame_id,
    const std::string& marker_namespace, double marker_width)
  : world_frame_id_(world_frame_id)
  , marker_namespace_(marker_namespace)
  , marker_width_(marker_width)
{
  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("candidate_markers", 1, true);
  selected_candidate_pub_ = nh.advertise<std_msgs::Int32>("selected_candidate", 1, true);
  selected_min_fc_pub_ = nh.advertise<std_msgs::Float64>("selected_min_fc_rp", 1, true);
  selected_joint_motion_pub_ = nh.advertise<std_msgs::Float64>("selected_joint_motion", 1, true);
}

void CandidateDiagnosticsPublisher::publish(
    const std::vector<CandidateVisualization>& candidates, int selected,
    const SelectedCandidateMetrics& metrics) const
{
  visualization_msgs::MarkerArray array;
  visualization_msgs::Marker clear;
  clear.action = visualization_msgs::Marker::DELETEALL;
  array.markers.push_back(clear);
  const ros::Time stamp = ros::Time::now();
  for (size_t index = 0; index < candidates.size(); ++index)
  {
    const CandidateVisualization& candidate = candidates[index];
    if (!candidate.trajectory || candidate.trajectory->getPieceNum() <= 0)
    {
      continue;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = marker_namespace_;
    marker.id = static_cast<int>(index);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker_width_;
    marker.color = candidateColor(candidate.status, static_cast<int>(index) == selected);
    const double duration = candidate.trajectory->getTotalDuration();
    for (int sample = 0; sample <= 80; ++sample)
    {
      const Eigen::Vector3d point = candidate.trajectory->getPos(duration * sample / 80.0);
      geometry_msgs::Point message;
      message.x = point.x();
      message.y = point.y();
      message.z = point.z();
      marker.points.push_back(message);
    }
    array.markers.push_back(marker);
  }
  marker_pub_.publish(array);

  std_msgs::Int32 selected_message;
  selected_message.data = selected;
  selected_candidate_pub_.publish(selected_message);
  std_msgs::Float64 value;
  value.data = metrics.minimum_fc_rp;
  selected_min_fc_pub_.publish(value);
  value.data = metrics.joint_motion;
  selected_joint_motion_pub_.publish(value);
}

}  // namespace motion_primitive_planner
