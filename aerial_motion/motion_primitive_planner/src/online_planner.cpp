#include <motion_primitive_planner/planner_core.h>

#include <dragon/model/hydrus_like_robot_model.h>
#include <gcopter/planner_common.hpp>
#include <multilink_copilot/follow_the_leader.h>

#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_loader.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Geometry>
#include <kdl/frames.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace motion_primitive_planner
{
namespace
{
constexpr char kRobotModelPlugin[] = "dragon/hydrus_like_robot_model";
constexpr double kEpsilon = 1e-6;

enum class StabilityResult
{
  kSafe,
  kLowFcRp,
  kOtherViolation
};

double yawFromQuaternion(const Eigen::Quaterniond& quaternion)
{
  const Eigen::Quaterniond q = quaternion.normalized();
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm < kEpsilon)
  {
    return 0.0;
  }
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y) / (norm * norm),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z) / (norm * norm));
}

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

struct PlannerConfig
{
  gcopter_planner::CommonPlannerConfig common;
  double replan_hz = 2.0;
  bool use_accumulated_map = true;
  double goal_tolerance = 0.2;
  double planning_horizon = 3.0;
  PrimitiveConfig primitive;
  double prediction_dt = 0.10;
  double trajectory_sample_interval = 0.05;
  double trajectory_buffer_max_length = 10.0;
  double command_hz = 40.0;
  bool publish_yaw_command = true;
  double max_yaw_rate = 1.5;
  double ik_singularity_threshold = 0.10;
  double fc_rp_min_threshold = 3.2;
  bool check_fc_t = false;
  double fc_t_min_threshold = 0.01;
  double static_thrust_min = 2.0;
  double static_thrust_max = 30.0;
  double overlap_min_clearance = 0.01;
  double feasibility_tolerance = 1e-4;
  bool allow_copilot_stability_projection_fallback = false;

  explicit PlannerConfig(const ros::NodeHandle& private_nh) : common(private_nh)
  {
    private_nh.param("ReplanHz", replan_hz, replan_hz);
    private_nh.param("UseAccumulatedMap", use_accumulated_map, use_accumulated_map);
    private_nh.param("GoalTolerance", goal_tolerance, goal_tolerance);
    private_nh.param("PlanningHorizon", planning_horizon, planning_horizon);
    private_nh.param("CandidateCount", primitive.candidate_count, primitive.candidate_count);
    private_nh.param("MaxPrimitiveOffset", primitive.max_offset, primitive.max_offset);
    private_nh.param("PrimitiveCruiseVelocity", primitive.cruise_velocity, 0.8 * common.maxVelMag);
    private_nh.param("MinimumPieceDuration", primitive.minimum_piece_duration, primitive.minimum_piece_duration);
    private_nh.param("PredictionDt", prediction_dt, prediction_dt);
    private_nh.param("TrajectorySampleInterval", trajectory_sample_interval, trajectory_sample_interval);
    private_nh.param("TrajectoryBufferMaxLength", trajectory_buffer_max_length, trajectory_buffer_max_length);
    private_nh.param("CommandHz", command_hz, command_hz);
    private_nh.param("PublishYawCommand", publish_yaw_command, publish_yaw_command);
    private_nh.param("MaxYawRate", max_yaw_rate, max_yaw_rate);
    private_nh.param("SnakeIkSingularityThreshold", ik_singularity_threshold, ik_singularity_threshold);
    private_nh.param("FcRpMinThreshold", fc_rp_min_threshold, fc_rp_min_threshold);
    private_nh.param("StabilityCheckFcT", check_fc_t, check_fc_t);
    private_nh.param("FcTMinThreshold", fc_t_min_threshold, fc_t_min_threshold);
    private_nh.param("StaticThrustMin", static_thrust_min, static_thrust_min);
    private_nh.param("StaticThrustMax", static_thrust_max, static_thrust_max);
    private_nh.param("OverlapMinClearance", overlap_min_clearance, overlap_min_clearance);
    private_nh.param("FeasibilityTolerance", feasibility_tolerance, feasibility_tolerance);
    private_nh.param("AllowCopilotStabilityProjectionFallback", allow_copilot_stability_projection_fallback,
                     allow_copilot_stability_projection_fallback);
    primitive.max_velocity = common.maxVelMag;

    common.validateOrThrow();
    const bool valid = std::isfinite(replan_hz) && replan_hz > 0.0 && std::isfinite(goal_tolerance) &&
                       goal_tolerance > 0.0 && std::isfinite(planning_horizon) && planning_horizon > 0.0 &&
                       std::isfinite(prediction_dt) && prediction_dt > 0.0 &&
                       std::isfinite(trajectory_sample_interval) && trajectory_sample_interval > 0.0 &&
                       std::isfinite(trajectory_buffer_max_length) && trajectory_buffer_max_length > 0.0 &&
                       std::isfinite(command_hz) && command_hz > 0.0 &&
                       std::isfinite(fc_rp_min_threshold) && std::isfinite(static_thrust_min) &&
                       std::isfinite(static_thrust_max) && static_thrust_min <= static_thrust_max &&
                       std::isfinite(feasibility_tolerance) && feasibility_tolerance >= 0.0;
    if (!valid)
    {
      throw std::invalid_argument("Invalid motion primitive online-planner configuration");
    }
  }
};
}  // namespace

class OnlinePlanner
{
public:
  OnlinePlanner(const PlannerConfig& config, ros::NodeHandle& nh)
    : config_(config)
    , nh_(nh)
    , backend_(config.common)
    , ros_interface_(config.common, nh_)
    , generator_(config.primitive)
    , robot_model_loader_("aerial_robot_model", "aerial_robot_model::RobotModel")
  {
    initializeRobotModel();
    // Allow planning before the first point cloud arrives.  The backend map is
    // explicitly initialized with no occupied voxels, so the configured map
    // bounds are treated as free space until mapCallback() supplies obstacles.
    rebuildMap();
    ROS_INFO("Motion primitive map starts as free space and will be updated when point clouds arrive.");
    map_sub_ = nh_.subscribe("pcl_topic", 1, &OnlinePlanner::mapCallback, this,
                             ros::TransportHints().tcpNoDelay());
    target_sub_ = nh_.subscribe("target", 1, &OnlinePlanner::targetCallback, this,
                                ros::TransportHints().tcpNoDelay());
    joint_state_sub_ = nh_.subscribe("joint_states", 1, &OnlinePlanner::jointStateCallback, this,
                                     ros::TransportHints().tcpNoDelay());
    executed_command_sub_ = nh_.subscribe("executed_command", 10, &OnlinePlanner::executedCommandCallback, this,
                                          ros::TransportHints().tcpNoDelay());
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("candidate_markers", 1, true);
    selected_candidate_pub_ = nh_.advertise<std_msgs::Int32>("selected_candidate", 1, true);
    selected_min_fc_pub_ = nh_.advertise<std_msgs::Float64>("selected_min_fc_rp", 1, true);
    ros_interface_.visualizer().clearTrajectory();
    timer_ = nh_.createTimer(ros::Duration(1.0 / config_.replan_hz), &OnlinePlanner::timerCallback, this);
    ROS_INFO("Motion primitive planner ready: N=%d, horizon=%.2f m, fc_rp_min>=%.2f, Copilot projection fallback=%s.",
             config_.primitive.candidate_count, config_.planning_horizon, config_.fc_rp_min_threshold,
             config_.allow_copilot_stability_projection_fallback ? "enabled" : "disabled");
  }

private:
  void initializeRobotModel()
  {
    const auto base_model = robot_model_loader_.createInstance(kRobotModelPlugin);
    robot_model_ = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(base_model);
    if (!robot_model_)
    {
      throw std::runtime_error("Could not load dragon/hydrus_like_robot_model");
    }
    link_num_ = robot_model_->getRotorNum();
    link_length_ = robot_model_->getLinkLength();
    link_joint_names_ = robot_model_->getLinkJointNames();
    link_joint_indices_ = robot_model_->getLinkJointIndices();
    current_joints_ = Eigen::VectorXd::Zero(static_cast<int>(link_joint_indices_.size()));
    pitch_indices_.assign(static_cast<size_t>(std::max(0, link_num_ - 1)), -1);
    yaw_indices_.assign(static_cast<size_t>(std::max(0, link_num_ - 1)), -1);
    for (size_t index = 0; index < link_joint_names_.size(); ++index)
    {
      joint_name_to_index_[link_joint_names_[index]] = static_cast<int>(index);
      int segment = -1;
      if (extractSegmentIndex(link_joint_names_[index], "_pitch", segment) &&
          segment < static_cast<int>(pitch_indices_.size()))
      {
        pitch_indices_[static_cast<size_t>(segment)] = static_cast<int>(index);
      }
      else if (extractSegmentIndex(link_joint_names_[index], "_yaw", segment) &&
               segment < static_cast<int>(yaw_indices_.size()))
      {
        yaw_indices_[static_cast<size_t>(segment)] = static_cast<int>(index);
      }
    }
    if (link_num_ <= 0 || link_length_ <= 0.0 ||
        std::find(pitch_indices_.begin(), pitch_indices_.end(), -1) != pitch_indices_.end() ||
        std::find(yaw_indices_.begin(), yaw_indices_.end(), -1) != yaw_indices_.end())
    {
      throw std::runtime_error("Incomplete DRAGON link geometry or joint mapping");
    }
  }

  void rebuildMap()
  {
    std::vector<Eigen::Vector3i> occupied;
    occupied.reserve(occupied_voxel_keys_.size());
    for (const long key : occupied_voxel_keys_)
    {
      occupied.push_back(backend_.voxelIdFromKey(key));
    }
    backend_.setMapVoxels(occupied);
  }

  void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& message)
  {
    std::vector<Eigen::Vector3d> points;
    std::string error;
    if (!ros_interface_.pointCloudToWorld(*message, points, &error))
    {
      if (error != "point-cloud transform is unavailable")
      {
        ROS_WARN_THROTTLE(1.0, "Invalid local point cloud: %s", error.c_str());
      }
      return;
    }
    if (!config_.use_accumulated_map)
    {
      occupied_voxel_keys_.clear();
    }
    for (const Eigen::Vector3d& point : points)
    {
      const long key = backend_.voxelKey(point);
      if (key >= 0)
      {
        occupied_voxel_keys_.insert(key);
      }
    }
    rebuildMap();
  }

  void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& message)
  {
    requested_target_ = Eigen::Vector3d(message->pose.position.x, message->pose.position.y,
                                        config_.common.resolveTargetHeight(*message));
    target_ = backend_.clampInsideMap(requested_target_,
                                     config_.common.dilateRadius + backend_.voxelScale());
    target_clamped_ = (target_ - requested_target_).norm() > 1e-3;
    target_received_ = true;
    goal_latched_ = false;
    ros_interface_.visualizer().visualizeStartGoal(target_, 0.05, 1);
    ROS_INFO("Received motion-primitive goal [%.2f, %.2f, %.2f]%s.", target_.x(), target_.y(), target_.z(),
             target_clamped_ ? " (clamped to map)" : "");
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& message)
  {
    Eigen::VectorXd measured = current_joints_;
    std::vector<bool> seen(link_joint_names_.size(), false);
    const size_t count = std::min(message->name.size(), message->position.size());
    for (size_t index = 0; index < count; ++index)
    {
      const auto found = joint_name_to_index_.find(message->name[index]);
      if (found == joint_name_to_index_.end() || !std::isfinite(message->position[index]))
      {
        continue;
      }
      measured(found->second) = message->position[index];
      seen[static_cast<size_t>(found->second)] = true;
    }
    if (std::find(seen.begin(), seen.end(), false) == seen.end())
    {
      current_joints_ = measured;
      joints_received_ = true;
    }
  }

  static bool appendHistory(const Eigen::Vector3d& position, double sample_interval, double maximum_length,
                            std::deque<multilink_copilot::TrajectoryPoint>& history, double& arc_length)
  {
    if (history.empty())
    {
      history.push_back({position});
      return true;
    }
    const double distance = (position - history.back().position).norm();
    if (distance < sample_interval)
    {
      return false;
    }
    history.push_back({position});
    arc_length += distance;
    while (history.size() > 1 && arc_length > maximum_length)
    {
      arc_length -= (history[1].position - history[0].position).norm();
      history.pop_front();
    }
    return true;
  }

  void executedCommandCallback(const geometry_msgs::PoseStamped::ConstPtr& message)
  {
    appendHistory(Eigen::Vector3d(message->pose.position.x, message->pose.position.y, message->pose.position.z),
                  config_.trajectory_sample_interval, config_.trajectory_buffer_max_length,
                  executed_history_, executed_arc_length_);
    executed_yaw_ = yawFromQuaternion(message->pose.orientation);
    executed_command_received_ = true;
  }

  Eigen::Vector3d truncateRoute(const std::vector<Eigen::Vector3d>& full_route,
                                std::vector<Eigen::Vector3d>& local_route) const
  {
    local_route.clear();
    local_route.push_back(full_route.front());
    double length = 0.0;
    for (size_t index = 1; index < full_route.size(); ++index)
    {
      const double segment = (full_route[index] - full_route[index - 1]).norm();
      if (length + segment >= config_.planning_horizon)
      {
        const double ratio = segment > kEpsilon ? (config_.planning_horizon - length) / segment : 0.0;
        local_route.push_back(full_route[index - 1] + ratio * (full_route[index] - full_route[index - 1]));
        return local_route.back();
      }
      length += segment;
      local_route.push_back(full_route[index]);
    }
    return local_route.back();
  }

  bool activeTrajectory(double now) const
  {
    return trajectory_.getPieceNum() > 0 && now - trajectory_stamp_ < trajectory_.getTotalDuration();
  }

  KDL::JntArray fullJointPositions(const Eigen::VectorXd& link_joints) const
  {
    KDL::JntArray result = robot_model_->getJointPositions();
    if (result.rows() != robot_model_->getTree().getNrOfJoints())
    {
      result.resize(robot_model_->getTree().getNrOfJoints());
    }
    for (int index = 0; index < std::min<int>(link_joints.size(), link_joint_indices_.size()); ++index)
    {
      result(link_joint_indices_[static_cast<size_t>(index)]) = link_joints(index);
    }
    return result;
  }

  bool jointsWithinLimits(const Eigen::VectorXd& joints, std::string& detail) const
  {
    const std::vector<double>& lower = robot_model_->getLinkJointLowerLimits();
    const std::vector<double>& upper = robot_model_->getLinkJointUpperLimits();
    for (int index = 0; index < joints.size(); ++index)
    {
      if (!std::isfinite(joints(index)) || index >= static_cast<int>(lower.size()) ||
          index >= static_cast<int>(upper.size()) || joints(index) < lower[static_cast<size_t>(index)] ||
          joints(index) > upper[static_cast<size_t>(index)])
      {
        detail = "nominal joint limit";
        return false;
      }
    }
    return true;
  }

  StabilityResult evaluateStability(const Eigen::VectorXd& joints, double yaw, double& fc_rp,
                                    std::string& detail)
  {
    KDL::JntArray full_joints = fullJointPositions(joints);
    const KDL::Rotation root_rotation = KDL::Rotation::RotZ(yaw + M_PI);
    const KDL::Frame root_to_base = robot_model_->forwardKinematics<KDL::Frame>(robot_model_->getBaselinkName(),
                                                                               full_joints);
    robot_model_->setCogDesireOrientation(root_rotation * root_to_base.M);
    robot_model_->updateRobotModel(full_joints);
    robot_model_->updateJacobians(full_joints, false);
    fc_rp = robot_model_->getFeasibleControlRollPitchMin();
    const double fc_t = robot_model_->getFeasibleControlTMin();
    const Eigen::VectorXd& thrust = robot_model_->getStaticThrust();
    const double thrust_min = thrust.size() > 0 ? thrust.minCoeff() : 0.0;
    const double thrust_max = thrust.size() > 0 ? thrust.maxCoeff() : 0.0;
    const double overlap = robot_model_->getClosestRotorDist() - 2.0 * robot_model_->getEdfRadius();
    const double tolerance = config_.feasibility_tolerance;
    if (!std::isfinite(fc_rp) || !std::isfinite(fc_t) || !std::isfinite(thrust_min) ||
        !std::isfinite(thrust_max) || !std::isfinite(overlap))
    {
      detail = "non-finite stability metric";
      return StabilityResult::kOtherViolation;
    }
    if ((config_.check_fc_t && fc_t + tolerance < config_.fc_t_min_threshold) ||
        thrust_min + tolerance < config_.static_thrust_min ||
        thrust_max - tolerance > config_.static_thrust_max ||
        overlap + tolerance < config_.overlap_min_clearance)
    {
      detail = "thrust, fc_t, or rotor clearance";
      return StabilityResult::kOtherViolation;
    }
    if (fc_rp + tolerance < config_.fc_rp_min_threshold)
    {
      detail = "nominal fc_rp_min=" + std::to_string(fc_rp);
      return StabilityResult::kLowFcRp;
    }
    return StabilityResult::kSafe;
  }

  void evaluateCandidate(Candidate& candidate)
  {
    if (candidate.status == CandidateStatus::kGenerationFailed)
    {
      return;
    }
    std::deque<multilink_copilot::TrajectoryPoint> history = executed_history_;
    double arc_length = executed_arc_length_;
    Eigen::VectorXd predicted_joints = current_joints_;
    double yaw = config_.publish_yaw_command && executed_command_received_
                     ? executed_yaw_
                     : yawFromQuaternion(ros_interface_.latestOrientation());
    candidate.min_fc_rp = std::numeric_limits<double>::infinity();
    candidate.joint_motion = 0.0;
    candidate.requires_stability_projection = false;
    Eigen::VectorXd previous_predicted_joints = predicted_joints;
    const double duration = candidate.trajectory.getTotalDuration();
    const double command_dt = 1.0 / config_.command_hz;
    const int sample_count = std::max(1, static_cast<int>(std::ceil(duration / command_dt)));
    const double required_history = static_cast<double>(link_num_ - 1) * link_length_;
    double next_evaluation_time = 0.0;
    for (int sample = 0; sample <= sample_count; ++sample)
    {
      const double time = std::min(duration, sample * command_dt);
      const Eigen::Vector3d position = candidate.trajectory.getPos(time);
      const Eigen::Vector3d velocity = candidate.trajectory.getVel(time);
      if (config_.publish_yaw_command && velocity.head<2>().squaredNorm() > 1e-6)
      {
        double difference = std::remainder(std::atan2(velocity.y(), velocity.x()) - yaw, 2.0 * M_PI);
        if (config_.max_yaw_rate > 0.0)
        {
          difference = std::max(-config_.max_yaw_rate * command_dt,
                                std::min(config_.max_yaw_rate * command_dt, difference));
        }
        yaw = std::remainder(yaw + difference, 2.0 * M_PI);
      }
      const bool history_changed = appendHistory(position, config_.trajectory_sample_interval,
                                                 config_.trajectory_buffer_max_length, history, arc_length);
      const Eigen::Matrix3d root_rotation =
          Eigen::AngleAxisd(yaw + M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      const std::vector<Eigen::Vector3d> targets = multilink_copilot::follow_the_leader::computeTargetPositions(
          history, position, root_rotation.col(0), link_num_, link_length_, arc_length < required_history);
      if (!targets.empty())
      {
        const Eigen::VectorXd path_joints = multilink_copilot::follow_the_leader::computeJointAngles(
            targets, position, root_rotation, pitch_indices_, yaw_indices_, predicted_joints.size(),
            config_.ik_singularity_threshold, predicted_joints);
        predicted_joints = arc_length < required_history
                               ? multilink_copilot::follow_the_leader::computeWarmupJointPositions(
                                     predicted_joints, path_joints, arc_length, link_length_, pitch_indices_,
                                     yaw_indices_)
                               : path_joints;
      }
      candidate.joint_motion += (predicted_joints - previous_predicted_joints).norm();
      previous_predicted_joints = predicted_joints;
      const bool final_sample = sample == sample_count;
      if (!history_changed && time + kEpsilon < next_evaluation_time && !final_sample)
      {
        continue;
      }
      while (next_evaluation_time <= time + kEpsilon)
      {
        next_evaluation_time += config_.prediction_dt;
      }
      if (!jointsWithinLimits(predicted_joints, candidate.detail))
      {
        candidate.status = CandidateStatus::kJointLimit;
        return;
      }
      const std::vector<Eigen::Vector3d> endpoints = linkEndpoints(
          position, root_rotation, predicted_joints, pitch_indices_, yaw_indices_, link_num_, link_length_);
      if (bodyCollides(endpoints, 0.5 * backend_.voxelScale(),
                       [this](const Eigen::Vector3d& point) { return backend_.query(point); }))
      {
        candidate.status = CandidateStatus::kCollision;
        candidate.detail = "whole-body swept collision";
        return;
      }
      double fc_rp = 0.0;
      const StabilityResult stability = evaluateStability(predicted_joints, yaw, fc_rp, candidate.detail);
      candidate.min_fc_rp = std::min(candidate.min_fc_rp, fc_rp);
      if (stability == StabilityResult::kOtherViolation)
      {
        candidate.status = CandidateStatus::kStability;
        return;
      }
      if (stability == StabilityResult::kLowFcRp)
      {
        if (!config_.allow_copilot_stability_projection_fallback)
        {
          candidate.status = CandidateStatus::kStability;
          return;
        }
        candidate.requires_stability_projection = true;
      }
    }
    if (candidate.requires_stability_projection)
    {
      candidate.status = CandidateStatus::kStabilityProjection;
      candidate.detail = "nominal fc_rp_min=" + std::to_string(candidate.min_fc_rp) +
                         "; downstream Copilot projection required";
    }
    else
    {
      candidate.status = CandidateStatus::kFeasible;
      candidate.detail.clear();
    }
  }

  void publishDiagnostics(const std::vector<Candidate>& candidates, int selected)
  {
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    array.markers.push_back(clear);
    const ros::Time stamp = ros::Time::now();
    for (size_t index = 0; index < candidates.size(); ++index)
    {
      const Candidate& candidate = candidates[index];
      if (candidate.trajectory.getPieceNum() <= 0)
      {
        continue;
      }
      visualization_msgs::Marker marker;
      marker.header.frame_id = config_.common.worldFrameId;
      marker.header.stamp = stamp;
      marker.ns = "motion_primitive_candidates";
      marker.id = static_cast<int>(index);
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.0125;
      marker.color.a = 0.9;
      if (index == static_cast<size_t>(selected))
      {
        marker.color.g = 1.0;
      }
      else if (candidate.feasible())
      {
        marker.color.g = 1.0;
        marker.color.b = 1.0;
      }
      else if (candidate.status == CandidateStatus::kStability ||
               candidate.status == CandidateStatus::kStabilityProjection)
      {
        marker.color.r = 1.0;
        marker.color.g = 0.5;
      }
      else if (candidate.status == CandidateStatus::kJointLimit)
      {
        marker.color.r = 1.0;
        marker.color.b = 1.0;
      }
      else
      {
        marker.color.r = 1.0;
      }
      const double duration = candidate.trajectory.getTotalDuration();
      for (int sample = 0; sample <= 80; ++sample)
      {
        const Eigen::Vector3d point = candidate.trajectory.getPos(duration * sample / 80.0);
        geometry_msgs::Point message;
        message.x = point.x();
        message.y = point.y();
        message.z = point.z();
        marker.points.push_back(message);
      }
      array.markers.push_back(marker);
    }
    marker_pub_.publish(array);
    ros_interface_.visualizer().clearTrajectory();
    std_msgs::Int32 selected_message;
    selected_message.data = selected;
    selected_candidate_pub_.publish(selected_message);
    std_msgs::Float64 fc_message;
    fc_message.data = selected >= 0 ? candidates[static_cast<size_t>(selected)].min_fc_rp
                                    : std::numeric_limits<double>::quiet_NaN();
    selected_min_fc_pub_.publish(fc_message);
  }

  void timerCallback(const ros::TimerEvent&)
  {
    if (!target_received_ || !ros_interface_.odomReceived() || !joints_received_ || goal_latched_)
    {
      return;
    }
    const double handover = ros::Time::now().toSec();
    Eigen::Vector3d start = ros_interface_.latestPosition();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    if (activeTrajectory(handover))
    {
      const double time = std::max(0.0, std::min(handover - trajectory_stamp_, trajectory_.getTotalDuration()));
      start = trajectory_.getPos(time);
      velocity = trajectory_.getVel(time);
      acceleration = trajectory_.getAcc(time);
    }
    if ((start - target_).norm() <= config_.goal_tolerance)
    {
      goal_latched_ = true;
      return;
    }
    if (backend_.query(start))
    {
      ROS_WARN_THROTTLE(1.0, "Motion primitive start is in collision.");
      return;
    }

    const auto planning_start = std::chrono::steady_clock::now();
    std::vector<Eigen::Vector3d> full_route;
    if (!backend_.searchPath(start, target_, full_route))
    {
      return;
    }
    std::vector<Eigen::Vector3d> local_route;
    const Eigen::Vector3d local_target = truncateRoute(full_route, local_route);
    if (local_route.size() < 2)
    {
      return;
    }
    const bool terminal = (local_target - full_route.back()).norm() <= kEpsilon;
    Eigen::Vector3d final_velocity = Eigen::Vector3d::Zero();
    const Eigen::Vector3d tangent = local_route.back() - local_route[local_route.size() - 2];
    if (!terminal && tangent.norm() > kEpsilon)
    {
      final_velocity = config_.primitive.cruise_velocity * tangent.normalized();
    }
    Eigen::Matrix3d initial_state;
    initial_state.col(0) = start;
    initial_state.col(1) = velocity;
    initial_state.col(2) = acceleration;
    Eigen::Matrix3d final_state;
    final_state.col(0) = local_target;
    final_state.col(1) = final_velocity;
    final_state.col(2).setZero();
    std::vector<Candidate> candidates = generator_.generate(local_route, initial_state, final_state);
    for (Candidate& candidate : candidates)
    {
      evaluateCandidate(candidate);
    }
    for (size_t index = 0; index < candidates.size(); ++index)
    {
      ROS_DEBUG("Primitive %zu: %s, min_fc=%.3f, joint_motion=%.3f, %s", index,
                candidateStatusName(candidates[index].status), candidates[index].min_fc_rp,
                candidates[index].joint_motion, candidates[index].detail.c_str());
    }
    const int selected = selectBestCandidate(candidates, config_.allow_copilot_stability_projection_fallback);
    const bool selected_requires_projection =
        selected >= 0 && candidates[static_cast<size_t>(selected)].requires_stability_projection;
    if (selected >= 0)
    {
      candidates[static_cast<size_t>(selected)].status = CandidateStatus::kSelected;
    }
    publishDiagnostics(candidates, selected);
    const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - planning_start)
                               .count();
    if (selected < 0)
    {
      ROS_WARN("All %zu primitives rejected in %.1f ms; keeping the previous trajectory.", candidates.size(),
               elapsed);
      return;
    }
    if (selected_requires_projection)
    {
      ROS_WARN("No nominally stable primitive was available; selecting primitive %d "
               "(nominal min fc_rp %.3f, joint motion %.3f rad) and relying on multilink_copilot to project "
               "the joint target to a stable configuration.",
               selected, candidates[static_cast<size_t>(selected)].min_fc_rp,
               candidates[static_cast<size_t>(selected)].joint_motion);
    }
    trajectory_ = candidates[static_cast<size_t>(selected)].trajectory;
    trajectory_stamp_ = handover;
    ros_interface_.publishTrajectory(trajectory_, handover);
    goal_latched_ = terminal;
    ROS_INFO("Selected primitive %d/%zu (length %.2f m, min fc_rp %.3f) in %.1f ms.", selected,
             candidates.size(), candidates[static_cast<size_t>(selected)].path_length,
             candidates[static_cast<size_t>(selected)].min_fc_rp, elapsed);
    ros_interface_.visualizer().visualizeStartGoal(start, 0.05, 0);
    ros_interface_.visualizer().visualizeStartGoal(target_, 0.05, 1);
  }

  PlannerConfig config_;
  ros::NodeHandle nh_;
  gcopter_planner::PlannerBackend backend_;
  gcopter_planner::PlannerRosInterface ros_interface_;
  PrimitiveGenerator generator_;
  pluginlib::ClassLoader<aerial_robot_model::RobotModel> robot_model_loader_;
  boost::shared_ptr<Dragon::HydrusLikeRobotModel> robot_model_;

  ros::Subscriber map_sub_;
  ros::Subscriber target_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber executed_command_sub_;
  ros::Publisher marker_pub_;
  ros::Publisher selected_candidate_pub_;
  ros::Publisher selected_min_fc_pub_;
  ros::Timer timer_;

  std::unordered_set<long> occupied_voxel_keys_;
  std::deque<multilink_copilot::TrajectoryPoint> executed_history_;
  double executed_arc_length_ = 0.0;
  double executed_yaw_ = 0.0;
  bool executed_command_received_ = false;
  bool target_received_ = false;
  bool joints_received_ = false;
  bool goal_latched_ = false;
  bool target_clamped_ = false;
  Eigen::Vector3d requested_target_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_ = Eigen::Vector3d::Zero();

  int link_num_ = 0;
  double link_length_ = 0.0;
  std::vector<std::string> link_joint_names_;
  std::vector<int> link_joint_indices_;
  std::vector<int> pitch_indices_;
  std::vector<int> yaw_indices_;
  std::unordered_map<std::string, int> joint_name_to_index_;
  Eigen::VectorXd current_joints_;

  Trajectory<5> trajectory_;
  double trajectory_stamp_ = 0.0;
};

}  // namespace motion_primitive_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_primitive_planner");
  ros::NodeHandle nh;
  try
  {
    motion_primitive_planner::PlannerConfig config(ros::NodeHandle("~"));
    motion_primitive_planner::OnlinePlanner planner(config, nh);
    ros::spin();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL("Motion primitive planner initialization failed: %s", exception.what());
    return 1;
  }
  return 0;
}
