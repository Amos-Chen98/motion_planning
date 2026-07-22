#include <motion_primitive_planner/joint_trajectory_planner.h>
#include <motion_primitive_planner/planner_core.h>

#include <aerial_robot_msgs/FullStateTarget.h>
#include <dragon/model/hydrus_like_robot_model.h>
#include <gcopter/planner_common.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_loader.h>
#include <ros/master.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
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

double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion)
{
  const double norm = std::sqrt(quaternion.x * quaternion.x + quaternion.y * quaternion.y +
                                quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  if (norm < kEpsilon)
  {
    return 0.0;
  }
  return std::atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y) / (norm * norm),
                    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z) / (norm * norm));
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

struct WholeBodyPlannerConfig
{
  gcopter_planner::CommonPlannerConfig common;
  PrimitiveConfig primitive;
  JointPlannerConfig joint;
  multilink_copilot::StabilityConfig stability;
  double replan_hz = 2.0;
  bool use_accumulated_map = true;
  double goal_tolerance = 0.2;
  double planning_horizon = 3.0;
  double whole_body_radius = 0.30;
  double activation_lead_time = 0.75;
  std::string root_child_frame_id = "root";

  explicit WholeBodyPlannerConfig(const ros::NodeHandle& private_nh) : common(private_nh)
  {
    private_nh.param("ReplanHz", replan_hz, replan_hz);
    private_nh.param("UseAccumulatedMap", use_accumulated_map, use_accumulated_map);
    private_nh.param("GoalTolerance", goal_tolerance, goal_tolerance);
    private_nh.param("PlanningHorizon", planning_horizon, planning_horizon);
    private_nh.param("CandidateCount", primitive.candidate_count, primitive.candidate_count);
    private_nh.param("MaxPrimitiveOffset", primitive.max_offset, primitive.max_offset);
    private_nh.param("PrimitiveCruiseVelocity", primitive.cruise_velocity, 0.8 * common.maxVelMag);
    private_nh.param("MinimumPieceDuration", primitive.minimum_piece_duration, primitive.minimum_piece_duration);
    primitive.max_velocity = common.maxVelMag;

    private_nh.param("JointReferenceDt", joint.reference_dt, joint.reference_dt);
    private_nh.param("CommandHz", joint.command_hz, joint.command_hz);
    private_nh.param("JointPlanningTimeout", joint.planning_timeout, joint.planning_timeout);
    private_nh.param("JointValidityResolution", joint.validity_resolution, joint.validity_resolution);
    private_nh.param("MaxJointVelocity", joint.max_joint_velocity, joint.max_joint_velocity);
    private_nh.param("MaxJointCommandStep", joint.max_joint_command_step, joint.max_joint_command_step);
    private_nh.param("TrajectorySampleInterval", joint.trajectory_sample_interval,
                     joint.trajectory_sample_interval);
    private_nh.param("TrajectoryBufferMaxLength", joint.trajectory_buffer_max_length,
                     joint.trajectory_buffer_max_length);
    private_nh.param("SnakeIkSingularityThreshold", joint.ik_singularity_threshold,
                     joint.ik_singularity_threshold);
    private_nh.param("MaxYawRate", joint.max_yaw_rate, joint.max_yaw_rate);
    private_nh.param("PublishYawCommand", joint.publish_yaw_command, joint.publish_yaw_command);
    int random_seed = static_cast<int>(joint.random_seed);
    private_nh.param("JointPlannerSeed", random_seed, random_seed);
    joint.random_seed = static_cast<unsigned int>(std::max(0, random_seed));

    private_nh.param("StabilityQpMaxIterations", stability.qp_max_iterations, stability.qp_max_iterations);
    private_nh.param("StabilityQpJointStepLimit", stability.qp_joint_step_limit,
                     stability.qp_joint_step_limit);
    private_nh.param("StabilityQpRegularization", stability.qp_regularization,
                     stability.qp_regularization);
    private_nh.param("StabilityQpConvergenceTolerance", stability.qp_convergence_tolerance,
                     stability.qp_convergence_tolerance);
    private_nh.param("FeasibilityTolerance", stability.feasibility_tolerance,
                     stability.feasibility_tolerance);
    private_nh.param("StabilityCheckFcT", stability.check_fc_t, stability.check_fc_t);
    private_nh.param("FcRpMinThreshold", stability.fc_rp_min_threshold, stability.fc_rp_min_threshold);
    private_nh.param("FcTMinThreshold", stability.fc_t_min_threshold, stability.fc_t_min_threshold);
    private_nh.param("StaticThrustMin", stability.static_thrust_min, stability.static_thrust_min);
    private_nh.param("StaticThrustMax", stability.static_thrust_max, stability.static_thrust_max);
    private_nh.param("OverlapMinClearance", stability.overlap_min_clearance,
                     stability.overlap_min_clearance);
    private_nh.param("MaxBaselinkTilt", stability.max_baselink_tilt, stability.max_baselink_tilt);
    private_nh.param("WholeBodyRadius", whole_body_radius, whole_body_radius);
    private_nh.param("PlanActivationLeadTime", activation_lead_time, activation_lead_time);
    private_nh.param("RootChildFrameId", root_child_frame_id, root_child_frame_id);

    common.validateOrThrow();
    if (replan_hz <= 0.0 || goal_tolerance <= 0.0 || planning_horizon <= 0.0 ||
        whole_body_radius < 0.0 || activation_lead_time <= 0.0 || root_child_frame_id.empty())
    {
      throw std::invalid_argument("Invalid whole-body motion primitive planner configuration");
    }
  }
};

class ClearanceMap
{
public:
  ClearanceMap(const Eigen::Vector3d& origin, const Eigen::Vector3d& corner, double voxel_width,
               const std::vector<Eigen::Vector3i>& occupied)
    : origin_(origin), corner_(corner), voxel_width_(voxel_width), cloud_(new pcl::PointCloud<pcl::PointXYZ>)
  {
    cloud_->reserve(occupied.size());
    for (const Eigen::Vector3i& voxel : occupied)
    {
      const Eigen::Vector3d center = origin_ + (voxel.cast<double>().array() + 0.5).matrix() * voxel_width_;
      cloud_->push_back(pcl::PointXYZ(center.x(), center.y(), center.z()));
    }
    if (!cloud_->empty())
    {
      kdtree_.setInputCloud(cloud_);
    }
  }

  double voxelWidth() const
  {
    return voxel_width_;
  }

  double clearance(const Eigen::Vector3d& point, double body_radius) const
  {
    const Eigen::Vector3d lower_margin = point - origin_;
    const Eigen::Vector3d upper_margin = corner_ - point;
    if (lower_margin.minCoeff() < body_radius || upper_margin.minCoeff() < body_radius)
    {
      return -std::max(body_radius, voxel_width_);
    }
    if (cloud_->empty())
    {
      return std::numeric_limits<double>::infinity();
    }
    pcl::PointXYZ query(point.x(), point.y(), point.z());
    std::vector<int> nearest_indices(1);
    std::vector<float> nearest_squared_distances(1);
    if (kdtree_.nearestKSearch(query, 1, nearest_indices, nearest_squared_distances) <= 0)
    {
      return -std::numeric_limits<double>::infinity();
    }
    const double half_width = 0.5 * voxel_width_;
    const auto distance_to_voxel = [&](int index) {
      const pcl::PointXYZ& center = cloud_->points[static_cast<size_t>(index)];
      const Eigen::Vector3d delta =
          (point - Eigen::Vector3d(center.x, center.y, center.z)).cwiseAbs() -
          Eigen::Vector3d::Constant(half_width);
      return delta.cwiseMax(0.0).norm();
    };
    double minimum_distance = distance_to_voxel(nearest_indices.front());
    const double voxel_radius = std::sqrt(3.0) * half_width;
    std::vector<int> candidate_indices;
    std::vector<float> candidate_squared_distances;
    kdtree_.radiusSearch(query, minimum_distance + voxel_radius + kEpsilon,
                         candidate_indices, candidate_squared_distances);
    for (const int index : candidate_indices)
    {
      minimum_distance = std::min(minimum_distance, distance_to_voxel(index));
    }
    return minimum_distance - body_radius;
  }

private:
  Eigen::Vector3d origin_;
  Eigen::Vector3d corner_;
  double voxel_width_ = 0.0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  mutable pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
};

struct CommandState
{
  bool valid = false;
  Eigen::Vector3d tail_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d tail_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d tail_acceleration = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double yaw_rate = 0.0;
  Eigen::VectorXd joint_positions;
  Eigen::VectorXd joint_velocities;
};

struct ActivePlan
{
  ros::Time start_time;
  uint64_t target_sequence = 0;
  Trajectory<5> root_trajectory;
  JointPlanResult joint_plan;
  CommandState hold_state;
  bool hold = false;
  bool terminal = false;

  double duration() const
  {
    return hold ? std::numeric_limits<double>::infinity() : root_trajectory.getTotalDuration();
  }

  CommandState sample(const ros::Time& stamp) const
  {
    if (hold)
    {
      return hold_state;
    }
    CommandState state;
    if (root_trajectory.getPieceNum() <= 0 || !joint_plan.success)
    {
      return state;
    }
    const double time = std::max(0.0, std::min((stamp - start_time).toSec(), root_trajectory.getTotalDuration()));
    state.valid = true;
    state.tail_position = root_trajectory.getPos(time);
    state.tail_velocity = root_trajectory.getVel(time);
    state.tail_acceleration = root_trajectory.getAcc(time);
    state.yaw = joint_plan.yaw(time);
    state.yaw_rate = joint_plan.yawRate(time);
    state.joint_positions = joint_plan.jointPositions(time);
    state.joint_velocities = joint_plan.jointVelocities(time);
    return state;
  }
};

enum class WholeBodyCandidateStatus
{
  kGenerationFailed,
  kJointPlanningFailed,
  kCollision,
  kFeasible,
  kSelected
};

struct WholeBodyCandidate
{
  Candidate root;
  Trajectory<5> scaled_root;
  JointPlanResult joints;
  WholeBodyCandidateStatus status = WholeBodyCandidateStatus::kGenerationFailed;
  double minimum_clearance = -std::numeric_limits<double>::infinity();
  std::string detail;
};
}  // namespace

class WholeBodyOnlinePlanner
{
public:
  WholeBodyOnlinePlanner(const WholeBodyPlannerConfig& config, ros::NodeHandle& nh)
    : config_(config)
    , nh_(nh)
    , backend_(config.common)
    , ros_interface_(config.common, nh_)
    , generator_(config.primitive)
    , robot_model_loader_("aerial_robot_model", "aerial_robot_model::RobotModel")
  {
    initializeRobotModels();
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      rebuildMapLocked();
    }
    map_sub_ = nh_.subscribe("pcl_topic", 1, &WholeBodyOnlinePlanner::mapCallback, this,
                             ros::TransportHints().tcpNoDelay());
    target_sub_ = nh_.subscribe("target", 1, &WholeBodyOnlinePlanner::targetCallback, this,
                                ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh_.subscribe("odom", 1, &WholeBodyOnlinePlanner::odomCallback, this,
                              ros::TransportHints().tcpNoDelay());
    joint_state_sub_ = nh_.subscribe("joint_states", 1, &WholeBodyOnlinePlanner::jointStateCallback, this,
                                     ros::TransportHints().tcpNoDelay());
    full_state_topic_ = nh_.resolveName("full_state_target");
    if (hasConflictingFullStatePublisher())
    {
      throw std::runtime_error("Another node already publishes " + full_state_topic_);
    }
    full_state_pub_ = nh_.advertise<aerial_robot_msgs::FullStateTarget>("full_state_target", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("candidate_markers", 1, true);
    selected_candidate_pub_ = nh_.advertise<std_msgs::Int32>("selected_candidate", 1, true);
    selected_min_fc_pub_ = nh_.advertise<std_msgs::Float64>("selected_min_fc_rp", 1, true);
    selected_min_clearance_pub_ = nh_.advertise<std_msgs::Float64>("selected_min_clearance", 1, true);
    selected_joint_motion_pub_ = nh_.advertise<std_msgs::Float64>("selected_joint_motion", 1, true);
    command_timer_ = nh_.createTimer(ros::Duration(1.0 / config_.joint.command_hz),
                                     &WholeBodyOnlinePlanner::commandTimerCallback, this);
    planning_timer_ = nh_.createTimer(ros::Duration(1.0 / config_.replan_hz),
                                      &WholeBodyOnlinePlanner::planningTimerCallback, this);
    publisher_guard_timer_ = nh_.createTimer(ros::Duration(1.0),
                                             &WholeBodyOnlinePlanner::publisherGuardTimerCallback, this);
    ROS_INFO("Whole-body motion primitive planner ready: candidates=%d, fc_rp_min>=%.2f, radius=%.2f m.",
             config_.primitive.candidate_count, config_.stability.fc_rp_min_threshold,
             config_.whole_body_radius);
  }

private:
  bool hasConflictingFullStatePublisher() const
  {
    XmlRpc::XmlRpcValue request;
    XmlRpc::XmlRpcValue response;
    XmlRpc::XmlRpcValue payload;
    request[0] = ros::this_node::getName();
    if (!ros::master::execute("getSystemState", request, response, payload, false) ||
        payload.getType() != XmlRpc::XmlRpcValue::TypeArray || payload.size() < 1)
    {
      return false;
    }
    const XmlRpc::XmlRpcValue& publishers = payload[0];
    if (publishers.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      return false;
    }
    for (int index = 0; index < publishers.size(); ++index)
    {
      const XmlRpc::XmlRpcValue& entry = publishers[index];
      if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray || entry.size() != 2 ||
          entry[0].getType() != XmlRpc::XmlRpcValue::TypeString ||
          static_cast<std::string>(entry[0]) != full_state_topic_ ||
          entry[1].getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        continue;
      }
      for (int node_index = 0; node_index < entry[1].size(); ++node_index)
      {
        if (entry[1][node_index].getType() == XmlRpc::XmlRpcValue::TypeString &&
            static_cast<std::string>(entry[1][node_index]) != ros::this_node::getName())
        {
          return true;
        }
      }
    }
    return false;
  }

  void publisherGuardTimerCallback(const ros::TimerEvent&)
  {
    if (!hasConflictingFullStatePublisher())
    {
      return;
    }
    ROS_FATAL("A second publisher appeared on %s; stopping whole-body command output.",
              full_state_topic_.c_str());
    full_state_pub_.shutdown();
    ros::shutdown();
  }

  void initializeRobotModels()
  {
    for (int candidate_index = 0; candidate_index < config_.primitive.candidate_count; ++candidate_index)
    {
      const auto base_model = robot_model_loader_.createInstance(kRobotModelPlugin);
      const auto model = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(base_model);
      if (!model)
      {
        throw std::runtime_error("Could not load an independent dragon/hydrus_like_robot_model");
      }
      robot_models_.push_back(model);
      auto evaluator = std::make_shared<multilink_copilot::StabilityEvaluator>(model, config_.stability);
      stability_evaluators_.push_back(evaluator);
      JointPlannerConfig joint_config = config_.joint;
      joint_config.random_seed += static_cast<unsigned int>(candidate_index);
      joint_planners_.emplace_back(new JointTrajectoryPlanner(joint_config, evaluator));
    }

    const auto& model = robot_models_.front();
    link_num_ = model->getRotorNum();
    link_length_ = model->getLinkLength();
    link_joint_names_ = model->getLinkJointNames();
    link_joint_indices_ = model->getLinkJointIndices();
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

  void rebuildMapLocked()
  {
    std::vector<Eigen::Vector3i> occupied;
    occupied.reserve(occupied_voxel_keys_.size());
    for (const long key : occupied_voxel_keys_)
    {
      occupied.push_back(backend_.voxelIdFromKey(key));
    }
    backend_.setMapVoxels(occupied);
    clearance_map_ = std::make_shared<ClearanceMap>(backend_.mapOrigin(), backend_.mapCorner(),
                                                    backend_.voxelScale(), occupied);
  }

  void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& message)
  {
    std::vector<Eigen::Vector3d> points;
    std::string error;
    if (!ros_interface_.pointCloudToWorld(*message, points, &error))
    {
      if (error != "point-cloud transform is unavailable")
      {
        ROS_WARN_THROTTLE(1.0, "Invalid point cloud: %s", error.c_str());
      }
      return;
    }
    std::lock_guard<std::mutex> lock(map_mutex_);
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
    rebuildMapLocked();
  }

  void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& message)
  {
    const Eigen::Vector3d requested(message->pose.position.x, message->pose.position.y,
                                    config_.common.resolveTargetHeight(*message));
    Eigen::Vector3d target;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      target = backend_.clampInsideMap(requested, config_.whole_body_radius + backend_.voxelScale());
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      target_ = target;
      target_received_ = true;
      ++target_sequence_;
      goal_latched_.store(false);
    }
    {
      std::lock_guard<std::mutex> lock(plan_mutex_);
      pending_plan_.reset();
    }
    ROS_INFO("Received whole-body goal [%.2f, %.2f, %.2f].", target.x(), target.y(), target.z());
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_odom_state_.valid = true;
    latest_odom_state_.tail_position = Eigen::Vector3d(message->pose.pose.position.x,
                                                       message->pose.pose.position.y,
                                                       message->pose.pose.position.z);
    latest_odom_state_.tail_velocity = Eigen::Vector3d(message->twist.twist.linear.x,
                                                       message->twist.twist.linear.y,
                                                       message->twist.twist.linear.z);
    latest_odom_state_.yaw = yawFromQuaternion(message->pose.pose.orientation);
    latest_odom_state_.yaw_rate = message->twist.twist.angular.z;
    odom_received_ = true;
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
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

  double minimumWholeBodyClearance(const Trajectory<5>& root,
                                   const JointPlanResult& joints,
                                   const std::shared_ptr<const ClearanceMap>& map) const
  {
    const double duration = root.getTotalDuration();
    const double command_dt = 1.0 / config_.joint.command_hz;
    const double spatial_resolution = 0.5 * map->voxelWidth();
    const double body_length = static_cast<double>(link_num_) * link_length_;
    double minimum = std::numeric_limits<double>::infinity();

    const auto evaluate_time = [&](double time, double& current_minimum) {
      const Eigen::Vector3d tail = root.getPos(time);
      const double yaw = joints.yaw(time);
      const Eigen::VectorXd q = joints.jointPositions(time);
      const Eigen::Matrix3d rotation =
          Eigen::AngleAxisd(yaw + M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      const std::vector<Eigen::Vector3d> endpoints =
          linkEndpoints(tail, rotation, q, pitch_indices_, yaw_indices_, link_num_, link_length_);
      for (size_t segment = 1; segment < endpoints.size(); ++segment)
      {
        const Eigen::Vector3d delta = endpoints[segment] - endpoints[segment - 1];
        const int samples = std::max(1, static_cast<int>(std::ceil(delta.norm() / spatial_resolution)));
        for (int sample = 0; sample <= samples; ++sample)
        {
          const Eigen::Vector3d point = endpoints[segment - 1] +
                                        static_cast<double>(sample) / samples * delta;
          current_minimum = std::min(current_minimum, map->clearance(point, config_.whole_body_radius));
          if (current_minimum < 0.0)
          {
            return false;
          }
        }
      }
      return true;
    };

    if (!evaluate_time(0.0, minimum))
    {
      return minimum;
    }
    std::vector<double> breakpoints;
    breakpoints.reserve(static_cast<size_t>(std::ceil(duration / command_dt)) +
                        joints.joint_waypoints.size() + joints.yaw_waypoints.size() + 2);
    breakpoints.push_back(0.0);
    for (double time = command_dt; time < duration; time += command_dt)
    {
      breakpoints.push_back(time);
    }
    for (const TimedJointWaypoint& waypoint : joints.joint_waypoints)
    {
      if (waypoint.time > 0.0 && waypoint.time < duration)
      {
        breakpoints.push_back(waypoint.time);
      }
    }
    for (const TimedYawWaypoint& waypoint : joints.yaw_waypoints)
    {
      if (waypoint.time > 0.0 && waypoint.time < duration)
      {
        breakpoints.push_back(waypoint.time);
      }
    }
    breakpoints.push_back(duration);
    std::sort(breakpoints.begin(), breakpoints.end());
    breakpoints.erase(std::unique(breakpoints.begin(), breakpoints.end(),
                                  [](double lhs, double rhs) {
                                    return std::abs(lhs - rhs) <= kEpsilon;
                                  }),
                      breakpoints.end());

    for (size_t interval = 1; interval < breakpoints.size(); ++interval)
    {
      const double start_time = breakpoints[interval - 1];
      const double end_time = breakpoints[interval];
      // The root path can curve between command samples, so the endpoint chord is not a
      // conservative swept-distance bound. Primitive generation enforces max_velocity.
      const double root_displacement_bound =
          config_.primitive.max_velocity * (end_time - start_time);
      const double yaw_delta = std::abs(joints.yaw(end_time) - joints.yaw(start_time));
      const Eigen::VectorXd q_delta = joints.jointPositions(end_time) - joints.jointPositions(start_time);
      const double angular_delta = q_delta.size() > 0 ? q_delta.cwiseAbs().sum() : 0.0;
      const double displacement_bound =
          root_displacement_bound + body_length * (yaw_delta + angular_delta);
      const int subdivisions = std::max(1, static_cast<int>(std::ceil(displacement_bound / spatial_resolution)));
      for (int subdivision = 1; subdivision <= subdivisions; ++subdivision)
      {
        const double time = start_time + static_cast<double>(subdivision) / subdivisions *
                                             (end_time - start_time);
        if (!evaluate_time(time, minimum))
        {
          return minimum;
        }
      }
    }
    return minimum;
  }

  int selectBest(const std::vector<WholeBodyCandidate>& candidates) const
  {
    std::vector<WholeBodyCandidateScore> scores;
    scores.reserve(candidates.size());
    for (const WholeBodyCandidate& candidate : candidates)
    {
      scores.push_back({candidate.status == WholeBodyCandidateStatus::kFeasible,
                        candidate.minimum_clearance,
                        candidate.joints.duration,
                        candidate.joints.joint_motion,
                        candidate.root.jerk_energy});
    }
    return selectBestWholeBodyCandidate(scores);
  }

  void publishDiagnostics(const std::vector<WholeBodyCandidate>& candidates, int selected)
  {
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    array.markers.push_back(clear);
    const ros::Time stamp = ros::Time::now();
    for (size_t index = 0; index < candidates.size(); ++index)
    {
      const WholeBodyCandidate& candidate = candidates[index];
      const Trajectory<5>& root = candidate.scaled_root.getPieceNum() > 0 ? candidate.scaled_root :
                                                                              candidate.root.trajectory;
      if (root.getPieceNum() <= 0)
      {
        continue;
      }
      visualization_msgs::Marker marker;
      marker.header.frame_id = config_.common.worldFrameId;
      marker.header.stamp = stamp;
      marker.ns = "whole_body_root_candidates";
      marker.id = static_cast<int>(index);
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.015;
      marker.color.a = 0.9;
      if (static_cast<int>(index) == selected)
      {
        marker.color.g = 1.0;
      }
      else if (candidate.status == WholeBodyCandidateStatus::kFeasible)
      {
        marker.color.g = 1.0;
        marker.color.b = 1.0;
      }
      else if (candidate.status == WholeBodyCandidateStatus::kJointPlanningFailed)
      {
        marker.color.r = 1.0;
        marker.color.g = 0.5;
      }
      else
      {
        marker.color.r = 1.0;
      }
      const double duration = root.getTotalDuration();
      for (int sample = 0; sample <= 80; ++sample)
      {
        const Eigen::Vector3d point = root.getPos(duration * sample / 80.0);
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
    value.data = selected >= 0 ? candidates[static_cast<size_t>(selected)].joints.minimum_fc_rp :
                                 std::numeric_limits<double>::quiet_NaN();
    selected_min_fc_pub_.publish(value);
    value.data = selected >= 0 ? candidates[static_cast<size_t>(selected)].minimum_clearance :
                                 std::numeric_limits<double>::quiet_NaN();
    selected_min_clearance_pub_.publish(value);
    value.data = selected >= 0 ? candidates[static_cast<size_t>(selected)].joints.joint_motion :
                                 std::numeric_limits<double>::quiet_NaN();
    selected_joint_motion_pub_.publish(value);
  }

  void planningTimerCallback(const ros::TimerEvent&)
  {
    if (planning_in_progress_.exchange(true))
    {
      return;
    }
    struct PlanningGuard
    {
      explicit PlanningGuard(std::atomic<bool>& flag) : flag_(flag) {}
      ~PlanningGuard() { flag_.store(false); }
      std::atomic<bool>& flag_;
    } guard(planning_in_progress_);

    if (goal_latched_.load())
    {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(plan_mutex_);
      if (pending_plan_)
      {
        return;
      }
    }

    Eigen::Vector3d target;
    uint64_t target_sequence = 0;
    CommandState measured;
    std::deque<multilink_copilot::TrajectoryPoint> history;
    double history_length = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!target_received_ || !odom_received_ || !joints_received_)
      {
        return;
      }
      target = target_;
      target_sequence = target_sequence_;
      measured = latest_odom_state_;
      measured.joint_positions = current_joints_;
      measured.joint_velocities = Eigen::VectorXd::Zero(current_joints_.size());
      history = executed_history_;
      history_length = executed_arc_length_;
    }

    const ros::Time planning_time = ros::Time::now();
    const ros::Time activation_time = planning_time + ros::Duration(config_.activation_lead_time);
    CommandState start = measured;
    std::shared_ptr<const ActivePlan> active_snapshot;
    {
      std::lock_guard<std::mutex> lock(plan_mutex_);
      active_snapshot = active_plan_;
    }
    if (active_snapshot)
    {
      const int future_samples = std::max(
          1, static_cast<int>(std::ceil(config_.activation_lead_time * config_.joint.command_hz)));
      for (int sample = 1; sample <= future_samples; ++sample)
      {
        const ros::Time sample_time = planning_time +
            ros::Duration(config_.activation_lead_time * static_cast<double>(sample) / future_samples);
        const CommandState predicted = active_snapshot->sample(sample_time);
        if (predicted.valid)
        {
          start = predicted;
          appendHistory(predicted.tail_position, config_.joint.trajectory_sample_interval,
                        config_.joint.trajectory_buffer_max_length, history, history_length);
        }
      }
    }
    appendHistory(start.tail_position, config_.joint.trajectory_sample_interval,
                  config_.joint.trajectory_buffer_max_length, history, history_length);

    if ((start.tail_position - target).norm() <= config_.goal_tolerance)
    {
      if (active_snapshot && !active_snapshot->hold)
      {
        return;
      }
      bool completed_current_target = false;
      {
        std::lock_guard<std::mutex> plan_lock(plan_mutex_);
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        if (target_sequence == target_sequence_)
        {
          if (!active_snapshot || active_plan_ == active_snapshot)
          {
            active_plan_.reset();
          }
          pending_plan_.reset();
          goal_latched_.store(true);
          completed_current_target = true;
        }
      }
      if (completed_current_target)
      {
        ROS_INFO("Whole-body target already reached; full-state command output stopped.");
      }
      return;
    }

    std::vector<Eigen::Vector3d> full_route;
    std::shared_ptr<const ClearanceMap> clearance_map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      clearance_map = clearance_map_;
      if (backend_.query(start.tail_position) || !backend_.searchPath(start.tail_position, target, full_route))
      {
        enterHold(currentCommandOr(start), target_sequence, "root route search failed");
        return;
      }
    }

    std::vector<Eigen::Vector3d> local_route;
    const Eigen::Vector3d local_target = truncateRoute(full_route, local_route);
    if (local_route.size() < 2)
    {
      enterHold(currentCommandOr(start), target_sequence, "local route is empty");
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
    initial_state.col(0) = start.tail_position;
    initial_state.col(1) = start.tail_velocity;
    initial_state.col(2) = start.tail_acceleration;
    Eigen::Matrix3d final_state;
    final_state.col(0) = local_target;
    final_state.col(1) = final_velocity;
    final_state.col(2).setZero();

    std::vector<Candidate> root_candidates = generator_.generate(local_route, initial_state, final_state);
    std::vector<WholeBodyCandidate> candidates(root_candidates.size());
    NominalJointContext nominal_context;
    nominal_context.executed_history = history;
    nominal_context.executed_arc_length = history_length;
    nominal_context.link_num = link_num_;
    nominal_context.link_length = link_length_;
    nominal_context.pitch_joint_indices = pitch_indices_;
    nominal_context.yaw_joint_indices = yaw_indices_;

    for (size_t index = 0; index < root_candidates.size(); ++index)
    {
      WholeBodyCandidate& candidate = candidates[index];
      candidate.root = root_candidates[index];
      if (candidate.root.status == CandidateStatus::kGenerationFailed)
      {
        candidate.detail = candidate.root.detail;
        continue;
      }
      candidate.joints = joint_planners_[index]->plan(candidate.root.trajectory, nominal_context,
                                                      start.joint_positions, start.yaw);
      if (!candidate.joints.success)
      {
        candidate.status = WholeBodyCandidateStatus::kJointPlanningFailed;
        candidate.detail = candidate.joints.detail;
        continue;
      }
      candidate.scaled_root = gcopter_planner::PlannerBackend::timeScaledTrajectory(
          candidate.root.trajectory, candidate.joints.time_scale);
      candidate.minimum_clearance = minimumWholeBodyClearance(candidate.scaled_root, candidate.joints,
                                                              clearance_map);
      if (candidate.minimum_clearance < 0.0)
      {
        candidate.status = WholeBodyCandidateStatus::kCollision;
        candidate.detail = "whole-body swept clearance is negative";
        continue;
      }
      candidate.status = WholeBodyCandidateStatus::kFeasible;
    }

    const int selected = selectBest(candidates);
    if (selected >= 0)
    {
      candidates[static_cast<size_t>(selected)].status = WholeBodyCandidateStatus::kSelected;
    }
    publishDiagnostics(candidates, selected);
    if (selected < 0)
    {
      enterHold(currentCommandOr(start), target_sequence, "all whole-body candidates failed");
      return;
    }
    if (ros::Time::now() >= activation_time)
    {
      ROS_WARN_THROTTLE(1.0, "Dropping whole-body plan that missed its activation deadline.");
      return;
    }
    const WholeBodyCandidate& selected_candidate = candidates[static_cast<size_t>(selected)];
    std::shared_ptr<ActivePlan> plan(new ActivePlan);
    plan->start_time = activation_time;
    plan->target_sequence = target_sequence;
    plan->root_trajectory = selected_candidate.scaled_root;
    plan->joint_plan = selected_candidate.joints;
    plan->terminal = terminal;
    {
      std::lock_guard<std::mutex> plan_lock(plan_mutex_);
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      if (target_sequence != target_sequence_)
      {
        return;
      }
      pending_plan_ = plan;
    }
    ROS_INFO("Selected whole-body primitive %d/%zu: clearance=%.3f m, min_fc_rp=%.3f, "
             "joint_motion=%.3f rad, scale=%.2f.",
             selected, candidates.size(), selected_candidate.minimum_clearance,
             selected_candidate.joints.minimum_fc_rp, selected_candidate.joints.joint_motion,
             selected_candidate.joints.time_scale);
  }

  CommandState currentCommandOr(const CommandState& fallback)
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    if (active_plan_)
    {
      const CommandState state = active_plan_->sample(ros::Time::now());
      if (state.valid)
      {
        return state;
      }
    }
    if (last_command_.valid)
    {
      return last_command_;
    }
    return fallback;
  }

  void enterHold(CommandState state, uint64_t target_sequence, const std::string& reason)
  {
    if (!state.valid || state.joint_positions.size() != static_cast<int>(link_joint_names_.size()))
    {
      ROS_ERROR_THROTTLE(1.0, "Cannot enter whole-body hold: no complete state is available.");
      return;
    }
    bool has_safe_command_history = false;
    {
      std::lock_guard<std::mutex> lock(plan_mutex_);
      has_safe_command_history = last_command_.valid;
    }
    if (!has_safe_command_history)
    {
      stability_evaluators_.front()->setRootLinkRotation(KDL::Rotation::RotZ(state.yaw + M_PI));
      multilink_copilot::StabilityMetrics metrics;
      if (!stability_evaluators_.front()->evaluate(state.joint_positions, metrics) || !metrics.safe)
      {
        ROS_ERROR_THROTTLE(1.0, "Refusing to publish an unstable startup hold command.");
        return;
      }
    }
    state.tail_velocity.setZero();
    state.tail_acceleration.setZero();
    state.yaw_rate = 0.0;
    state.joint_velocities = Eigen::VectorXd::Zero(state.joint_positions.size());
    std::shared_ptr<ActivePlan> hold(new ActivePlan);
    hold->start_time = ros::Time::now();
    hold->target_sequence = target_sequence;
    hold->hold = true;
    hold->hold_state = state;
    {
      std::lock_guard<std::mutex> plan_lock(plan_mutex_);
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      if (target_sequence != target_sequence_)
      {
        return;
      }
      active_plan_ = hold;
      pending_plan_.reset();
    }
    ROS_WARN("Whole-body planner entered hover hold: %s", reason.c_str());
  }

  void commandTimerCallback(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    CommandState state;
    bool terminal_complete = false;
    uint64_t completed_target_sequence = 0;
    {
      std::lock_guard<std::mutex> lock(plan_mutex_);
      if (pending_plan_ && now >= pending_plan_->start_time)
      {
        active_plan_ = pending_plan_;
        pending_plan_.reset();
      }
      if (!active_plan_)
      {
        return;
      }
      state = active_plan_->sample(now);
      terminal_complete = active_plan_->terminal && !active_plan_->hold &&
                          (now - active_plan_->start_time).toSec() >= active_plan_->duration();
      if (terminal_complete)
      {
        completed_target_sequence = active_plan_->target_sequence;
      }
      if (state.valid)
      {
        last_command_ = state;
      }
      if (terminal_complete)
      {
        active_plan_.reset();
      }
    }
    if (!state.valid)
    {
      return;
    }
    publishFullStateTarget(state, now);
    bool completed_current_target = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      appendHistory(state.tail_position, config_.joint.trajectory_sample_interval,
                    config_.joint.trajectory_buffer_max_length, executed_history_, executed_arc_length_);
      if (terminal_complete && target_received_ && completed_target_sequence == target_sequence_)
      {
        goal_latched_.store(true);
        completed_current_target = true;
      }
    }
    if (completed_current_target)
    {
      ROS_INFO("Whole-body target reached; full-state command output stopped.");
    }
  }

  void publishFullStateTarget(const CommandState& state, const ros::Time& stamp)
  {
    const RootCommandKinematics root = tailFluToRootLinkCommand(
        state.tail_position, state.tail_velocity, state.yaw, state.yaw_rate, link_length_);

    aerial_robot_msgs::FullStateTarget message;
    message.header.stamp = stamp;
    message.header.frame_id = config_.common.worldFrameId;
    message.root_state.header = message.header;
    message.root_state.child_frame_id = config_.root_child_frame_id;
    message.root_state.pose.pose.position.x = root.position.x();
    message.root_state.pose.pose.position.y = root.position.y();
    message.root_state.pose.pose.position.z = root.position.z();
    message.root_state.pose.pose.orientation.z = std::sin(0.5 * root.yaw);
    message.root_state.pose.pose.orientation.w = std::cos(0.5 * root.yaw);
    message.root_state.twist.twist.linear.x = root.linear_velocity.x();
    message.root_state.twist.twist.linear.y = root.linear_velocity.y();
    message.root_state.twist.twist.linear.z = root.linear_velocity.z();
    message.root_state.twist.twist.angular.z = root.yaw_rate;
    message.joint_state.header = message.header;
    message.joint_state.name = link_joint_names_;
    message.joint_state.position.resize(static_cast<size_t>(state.joint_positions.size()));
    message.joint_state.velocity.resize(static_cast<size_t>(state.joint_velocities.size()));
    for (int index = 0; index < state.joint_positions.size(); ++index)
    {
      message.joint_state.position[static_cast<size_t>(index)] = state.joint_positions(index);
      message.joint_state.velocity[static_cast<size_t>(index)] = state.joint_velocities(index);
    }
    full_state_pub_.publish(message);
  }

  WholeBodyPlannerConfig config_;
  ros::NodeHandle nh_;
  gcopter_planner::PlannerBackend backend_;
  gcopter_planner::PlannerRosInterface ros_interface_;
  PrimitiveGenerator generator_;
  pluginlib::ClassLoader<aerial_robot_model::RobotModel> robot_model_loader_;
  std::vector<boost::shared_ptr<Dragon::HydrusLikeRobotModel>> robot_models_;
  std::vector<std::shared_ptr<multilink_copilot::StabilityEvaluator>> stability_evaluators_;
  std::vector<std::unique_ptr<JointTrajectoryPlanner>> joint_planners_;

  ros::Subscriber map_sub_;
  ros::Subscriber target_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher full_state_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher selected_candidate_pub_;
  ros::Publisher selected_min_fc_pub_;
  ros::Publisher selected_min_clearance_pub_;
  ros::Publisher selected_joint_motion_pub_;
  ros::Timer command_timer_;
  ros::Timer planning_timer_;
  ros::Timer publisher_guard_timer_;
  std::string full_state_topic_;

  std::mutex map_mutex_;
  std::unordered_set<long> occupied_voxel_keys_;
  std::shared_ptr<const ClearanceMap> clearance_map_;

  std::mutex state_mutex_;
  CommandState latest_odom_state_;
  bool odom_received_ = false;
  bool joints_received_ = false;
  bool target_received_ = false;
  uint64_t target_sequence_ = 0;
  Eigen::Vector3d target_ = Eigen::Vector3d::Zero();
  Eigen::VectorXd current_joints_;
  std::deque<multilink_copilot::TrajectoryPoint> executed_history_;
  double executed_arc_length_ = 0.0;

  std::mutex plan_mutex_;
  std::shared_ptr<const ActivePlan> active_plan_;
  std::shared_ptr<const ActivePlan> pending_plan_;
  CommandState last_command_;
  std::atomic<bool> planning_in_progress_{false};
  std::atomic<bool> goal_latched_{false};

  int link_num_ = 0;
  double link_length_ = 0.0;
  std::vector<std::string> link_joint_names_;
  std::vector<int> link_joint_indices_;
  std::vector<int> pitch_indices_;
  std::vector<int> yaw_indices_;
  std::unordered_map<std::string, int> joint_name_to_index_;
};

}  // namespace motion_primitive_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "whole_body_motion_primitive_planner");
  ros::NodeHandle nh;
  try
  {
    motion_primitive_planner::WholeBodyPlannerConfig config(ros::NodeHandle("~"));
    motion_primitive_planner::WholeBodyOnlinePlanner planner(config, nh);
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL("Whole-body motion primitive planner initialization failed: %s", exception.what());
    return 1;
  }
  return 0;
}
