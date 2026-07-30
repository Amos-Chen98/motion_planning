#include <motion_primitive_planner/joint_trajectory_planner.h>
#include <motion_primitive_planner/planner_config.h>
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

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace motion_primitive_planner
{
namespace
{
constexpr char kRobotModelPlugin[] = "dragon/hydrus_like_robot_model";
constexpr double kEpsilon = 1e-6;
//! Wall-clock reserve kept between the end of candidate evaluation and activation.
//! It absorbs the last candidate's swept-collision check and diagnostics.
constexpr double kActivationSafetyMargin = 0.20;

enum class PlanAttemptResult
{
  kIdle,
  kSucceeded,
  kRetry
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

struct WholeBodyCandidate
{
  Candidate root;
  Trajectory<5> scaled_root;
  JointPlanResult joints;
  CandidateStatus status = CandidateStatus::kGenerationFailed;
  std::string detail;
};
}  // namespace

class WholeBodyOnlinePlanner
{
public:
  WholeBodyOnlinePlanner(const WholeBodyPlannerConfig& config, ros::NodeHandle& nh)
    : config_(config)
    , nh_(nh)
    , environment_(config.shared)
    , ros_interface_(config.shared.common, nh_)
    , robot_model_loader_("aerial_robot_model", "aerial_robot_model::RobotModel")
    , executed_history_(config.joint.follower)
    , diagnostics_(nh_, config.shared.common.worldFrameId,
                   "whole_body_root_candidates", 0.015)
    , replan_trigger_(config.shared.replan_trigger_ratio)
  {
    initializeRobotModels();
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
    root_target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("root/target_pose", 10);
    command_timer_ = nh_.createTimer(ros::Duration(1.0 / config_.joint.follower.command_hz),
                                     &WholeBodyOnlinePlanner::commandTimerCallback, this);
    publisher_guard_timer_ = nh_.createTimer(ros::Duration(1.0),
                                             &WholeBodyOnlinePlanner::publisherGuardTimerCallback, this);
    planning_worker_ = std::thread(&WholeBodyOnlinePlanner::planningWorker, this);
    ROS_INFO("Whole-body motion primitive planner ready: candidates=%d, fc_rp_min>=%.2f, "
             "collision radius=%.2f m, replan_ratio=%.2f.",
             config_.shared.primitive.candidate_count, config_.stability.fc_rp_min_threshold,
             config_.shared.common.dilateRadius, config_.shared.replan_trigger_ratio);
  }

  ~WholeBodyOnlinePlanner()
  {
    planning_shutdown_.store(true);
    planning_request_cv_.notify_one();
    if (planning_worker_.joinable())
    {
      planning_worker_.join();
    }
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
    for (int candidate_index = 0;
         candidate_index < config_.shared.primitive.candidate_count; ++candidate_index)
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

    model_info_.reset(new DragonModelInfo(robot_models_.front()));
    collision_geometry_ = model_info_->collisionGeometry();
    current_joints_ = Eigen::VectorXd::Zero(model_info_->jointCount());
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
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      environment_.updateMap(points);
    }
    retryPlanningIfPending();
  }

  void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& message)
  {
    bool odom_received = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      odom_received = odom_received_;
    }
    if (!odom_received)
    {
      ROS_ERROR("No valid root-link odometry has been received from %s; "
                "discarding whole-body motion-primitive target.",
                nh_.resolveName("odom").c_str());
      return;
    }
    const Eigen::Vector3d requested(message->pose.position.x, message->pose.position.y,
                                    config_.shared.common.resolveTargetHeight(*message));
    Eigen::Vector3d target;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      target = environment_.clampTarget(
          requested, config_.shared.common.dilateRadius + environment_.voxelScale());
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
      replan_trigger_.reset();
    }
    retry_pending_.store(false);
    ROS_INFO("Received whole-body goal [%.2f, %.2f, %.2f].", target.x(), target.y(), target.z());
    requestPlanning();
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& message)
  {
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
    retryPlanningIfPending();
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& message)
  {
    bool state_updated = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (model_info_->readCompleteJointState(*message, current_joints_))
      {
        joints_received_ = true;
        state_updated = true;
      }
    }
    if (state_updated)
    {
      retryPlanningIfPending();
    }
  }

  bool wholeBodyTrajectoryCollides(
      const Trajectory<5>& root, const JointPlanResult& joints,
      const std::shared_ptr<const gcopter_planner::PlannerBackend>& occupancy) const
  {
    const double duration = root.getTotalDuration();
    const double command_dt = 1.0 / config_.joint.follower.command_hz;
    const double spatial_resolution = 0.5 * occupancy->voxelScale();
    const double body_length = static_cast<double>(model_info_->linkNum()) * model_info_->linkLength();
    const auto occupied = [&occupancy](const Eigen::Vector3d& point) {
      return occupancy->query(point);
    };

    const auto collides_at_time = [&](double time) {
      WholeBodyConfiguration configuration;
      configuration.link1_tail = root.getPos(time);
      const double yaw = joints.yaw(time);
      configuration.root_link_rotation =
          Eigen::AngleAxisd(yaw + M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      configuration.joint_positions = joints.jointPositions(time);
      return wholeBodyCollides(configuration, collision_geometry_, spatial_resolution, occupied);
    };

    if (collides_at_time(0.0))
    {
      return true;
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
      const double yaw_delta =
          shortestYawDelta(joints.yaw(start_time), joints.yaw(end_time));
      const Eigen::VectorXd q_delta =
          joints.jointPositions(end_time) - joints.jointPositions(start_time);
      const int subdivisions = wholeBodyMotionSubdivisionCount(
          end_time - start_time, config_.shared.primitive.max_velocity,
          body_length, yaw_delta, q_delta, spatial_resolution);
      for (int subdivision = 1; subdivision <= subdivisions; ++subdivision)
      {
        const double time = start_time + static_cast<double>(subdivision) / subdivisions *
                                             (end_time - start_time);
        if (collides_at_time(time))
        {
          return true;
        }
      }
    }
    return false;
  }

  int selectBest(const std::vector<WholeBodyCandidate>& candidates) const
  {
    std::vector<WholeBodyCandidateScore> scores;
    scores.reserve(candidates.size());
    for (const WholeBodyCandidate& candidate : candidates)
    {
      scores.push_back({candidate.status == CandidateStatus::kFeasible,
                        candidate.joints.duration,
                        candidate.joints.joint_motion,
                        candidate.root.jerk_energy});
    }
    return selectBestWholeBodyCandidate(scores, config_.joint_motion_cost_weight);
  }

  void publishDiagnostics(const std::vector<WholeBodyCandidate>& candidates, int selected)
  {
    std::vector<CandidateVisualization> visualizations;
    visualizations.reserve(candidates.size());
    for (const WholeBodyCandidate& candidate : candidates)
    {
      const Trajectory<5>& root = candidate.scaled_root.getPieceNum() > 0 ? candidate.scaled_root :
                                                                              candidate.root.trajectory;
      visualizations.push_back({&root, candidate.status});
    }
    SelectedCandidateMetrics metrics;
    if (selected >= 0)
    {
      const WholeBodyCandidate& candidate = candidates[static_cast<size_t>(selected)];
      metrics.minimum_fc_rp = candidate.joints.minimum_fc_rp;
      metrics.joint_motion = candidate.joints.joint_motion;
    }
    diagnostics_.publish(visualizations, selected, metrics);
  }

  void requestPlanning()
  {
    planning_requested_.store(true);
    if (!planning_in_progress_.load())
    {
      planning_request_cv_.notify_one();
    }
  }

  void planningWorker()
  {
    while (!planning_shutdown_.load())
    {
      std::unique_lock<std::mutex> request_lock(planning_request_mutex_);
      planning_request_cv_.wait(request_lock, [this] {
        return planning_shutdown_.load() || planning_requested_.load();
      });
      if (planning_shutdown_.load())
      {
        return;
      }
      planning_requested_.store(false);
      planning_in_progress_.store(true);
      request_lock.unlock();

      while (!planning_shutdown_.load())
      {
        retry_pending_.store(false);
        const PlanAttemptResult result = planOnce();
        if (result == PlanAttemptResult::kRetry)
        {
          retry_pending_.store(true);
        }
        if (!planning_requested_.exchange(false))
        {
          break;
        }
      }
      planning_in_progress_.store(false);
    }
  }

  void retryPlanningIfPending()
  {
    bool expected = true;
    if (retry_pending_.compare_exchange_strong(expected, false))
    {
      requestPlanning();
    }
  }

  PlanAttemptResult planOnce()
  {
    if (goal_latched_.load())
    {
      return PlanAttemptResult::kIdle;
    }
    {
      std::lock_guard<std::mutex> lock(plan_mutex_);
      if (pending_plan_)
      {
        return PlanAttemptResult::kIdle;
      }
    }

    Eigen::Vector3d target;
    uint64_t target_sequence = 0;
    CommandState measured;
    TrajectoryHistory history;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!target_received_)
      {
        return PlanAttemptResult::kIdle;
      }
      if (!odom_received_ || !joints_received_)
      {
        return PlanAttemptResult::kRetry;
      }
      target = target_;
      target_sequence = target_sequence_;
      measured = latest_odom_state_;
      measured.joint_positions = current_joints_;
      measured.joint_velocities = Eigen::VectorXd::Zero(current_joints_.size());
      history = executed_history_;
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
          1, static_cast<int>(std::ceil(
                 config_.activation_lead_time * config_.joint.follower.command_hz)));
      for (int sample = 1; sample <= future_samples; ++sample)
      {
        const ros::Time sample_time = planning_time +
            ros::Duration(config_.activation_lead_time * static_cast<double>(sample) / future_samples);
        const CommandState predicted = active_snapshot->sample(sample_time);
        if (predicted.valid)
        {
          start = predicted;
          history.append(predicted.tail_position);
        }
      }
    }
    history.append(start.tail_position);

    if ((start.tail_position - target).norm() <= config_.shared.goal_tolerance)
    {
      if (active_snapshot && !active_snapshot->hold)
      {
        return PlanAttemptResult::kIdle;
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
          replan_trigger_.reset();
          goal_latched_.store(true);
          completed_current_target = true;
        }
      }
      if (completed_current_target)
      {
        ROS_INFO("Whole-body target already reached; full-state command output stopped.");
      }
      return PlanAttemptResult::kIdle;
    }

    struct PlanningTimingLogger
    {
      explicit PlanningTimingLogger(bool enabled)
        : enabled_(enabled), start_(std::chrono::steady_clock::now())
      {
      }

      ~PlanningTimingLogger()
      {
        if (enabled_)
        {
          const double elapsed_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - start_).count();
          ROS_INFO_THROTTLE(1.0, "Whole-body local planning completed in %.1f ms.", elapsed_ms);
        }
      }

      bool enabled_;
      std::chrono::steady_clock::time_point start_;
    } timing_logger(config_.verbose);

    std::shared_ptr<const gcopter_planner::PlannerBackend> occupancy;
    PrimitiveBatch batch;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      RootState root_start;
      root_start.position = start.tail_position;
      root_start.velocity = start.tail_velocity;
      root_start.acceleration = start.tail_acceleration;
      batch = environment_.generate(root_start, target);
      occupancy = environment_.occupancySnapshot();
    }
    if (!batch.success())
    {
      enterHold(currentCommandOr(start), target_sequence, batch.detail);
      return PlanAttemptResult::kRetry;
    }
    std::vector<WholeBodyCandidate> candidates(batch.candidates.size());
    const NominalJointContext nominal_context = makeNominalJointContext(history, *model_info_);

    // Joint-space bridging is the expensive part of whole-body planning, so all
    // candidates share one wall-clock budget.  Running out only costs the quality
    // of the remaining candidates; the ones already evaluated stay selectable and
    // the plan still reaches its activation instant.
    const ros::Time joint_planning_deadline =
        activation_time - ros::Duration(kActivationSafetyMargin);
    for (size_t index = 0; index < batch.candidates.size(); ++index)
    {
      WholeBodyCandidate& candidate = candidates[index];
      candidate.root = batch.candidates[index];
      if (candidate.root.status == CandidateStatus::kGenerationFailed)
      {
        candidate.detail = candidate.root.detail;
        continue;
      }
      const double joint_planning_budget = (joint_planning_deadline - ros::Time::now()).toSec();
      if (joint_planning_budget <= 0.0)
      {
        candidate.status = CandidateStatus::kJointPlanningFailed;
        candidate.detail = "whole-body planning budget exhausted";
        continue;
      }
      candidate.joints = joint_planners_[index]->plan(candidate.root.trajectory, nominal_context,
                                                      start.joint_positions, start.yaw,
                                                      joint_planning_budget);
      if (!candidate.joints.success)
      {
        candidate.status = CandidateStatus::kJointPlanningFailed;
        candidate.detail = candidate.joints.detail;
        continue;
      }
      candidate.scaled_root = gcopter_planner::PlannerBackend::timeScaledTrajectory(
          candidate.root.trajectory, candidate.joints.time_scale);
      if (wholeBodyTrajectoryCollides(candidate.scaled_root, candidate.joints, occupancy))
      {
        candidate.status = CandidateStatus::kCollision;
        candidate.detail = "whole-body swept collision";
        continue;
      }
      candidate.status = CandidateStatus::kFeasible;
    }

    const int selected = selectBest(candidates);
    if (selected >= 0)
    {
      candidates[static_cast<size_t>(selected)].status = CandidateStatus::kSelected;
    }
    publishDiagnostics(candidates, selected);
    if (selected < 0)
    {
      enterHold(currentCommandOr(start), target_sequence, "all whole-body candidates failed");
      return PlanAttemptResult::kRetry;
    }
    if (ros::Time::now() >= activation_time)
    {
      ROS_WARN_THROTTLE(1.0, "Dropping whole-body plan that missed its activation deadline.");
      return PlanAttemptResult::kRetry;
    }
    const WholeBodyCandidate& selected_candidate = candidates[static_cast<size_t>(selected)];
    std::shared_ptr<ActivePlan> plan(new ActivePlan);
    plan->start_time = activation_time;
    plan->target_sequence = target_sequence;
    plan->root_trajectory = selected_candidate.scaled_root;
    plan->joint_plan = selected_candidate.joints;
    const double plan_duration = plan->root_trajectory.getTotalDuration();
    plan->terminal = batch.terminal ||
        (plan->root_trajectory.getPos(plan_duration) - target).norm() <=
            config_.shared.goal_tolerance;
    {
      std::lock_guard<std::mutex> plan_lock(plan_mutex_);
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      if (target_sequence != target_sequence_)
      {
        return PlanAttemptResult::kIdle;
      }
      pending_plan_ = plan;
    }
    ROS_INFO("Selected whole-body primitive %d/%zu: duration=%.3f s, min_fc_rp=%.3f, "
             "joint_motion=%.3f rad, scale=%.2f, bridges=%d, holds=%d.",
             selected, candidates.size(), selected_candidate.joints.duration,
             selected_candidate.joints.minimum_fc_rp, selected_candidate.joints.joint_motion,
             selected_candidate.joints.time_scale, selected_candidate.joints.bridge_count,
             selected_candidate.joints.hold_count);
    return PlanAttemptResult::kSucceeded;
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
    if (!state.valid || state.joint_positions.size() != model_info_->jointCount())
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
      replan_trigger_.reset();
    }
    ROS_WARN_THROTTLE(1.0, "Whole-body planner entered hover hold: %s", reason.c_str());
  }

  void commandTimerCallback(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    CommandState state;
    bool terminal_complete = false;
    bool progress_triggered = false;
    uint64_t completed_target_sequence = 0;
    {
      std::lock_guard<std::mutex> lock(plan_mutex_);
      if (pending_plan_ && now >= pending_plan_->start_time)
      {
        active_plan_ = pending_plan_;
        pending_plan_.reset();
        replan_trigger_.arm(active_plan_->start_time.toSec(),
                            active_plan_->duration(), active_plan_->terminal);
      }
      if (active_plan_)
      {
        state = active_plan_->sample(now);
        terminal_complete = active_plan_->terminal && !active_plan_->hold &&
                            (now - active_plan_->start_time).toSec() >= active_plan_->duration();
        if (terminal_complete)
        {
          completed_target_sequence = active_plan_->target_sequence;
        }
        else
        {
          progress_triggered = replan_trigger_.shouldTrigger(now.toSec());
          if (progress_triggered)
          {
            ROS_DEBUG("Whole-body trajectory reached the replanning ratio at %.3f.", now.toSec());
          }
        }
        if (state.valid)
        {
          last_command_ = state;
        }
        if (terminal_complete)
        {
          active_plan_.reset();
          replan_trigger_.reset();
        }
      }
    }
    if (!state.valid)
    {
      retryPlanningIfPending();
      return;
    }
    if (terminal_complete)
    {
      state.tail_velocity.setZero();
      state.tail_acceleration.setZero();
      state.yaw_rate = 0.0;
      state.joint_velocities = Eigen::VectorXd::Zero(state.joint_positions.size());
    }
    publishFullStateTarget(state, now);
    bool completed_current_target = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      executed_history_.append(state.tail_position);
      if (terminal_complete && target_received_ && completed_target_sequence == target_sequence_)
      {
        goal_latched_.store(true);
        retry_pending_.store(false);
        completed_current_target = true;
      }
    }
    if (completed_current_target)
    {
      ROS_INFO("Whole-body target reached; full-state command output stopped.");
    }
    if (progress_triggered)
    {
      requestPlanning();
    }
    else
    {
      retryPlanningIfPending();
    }
  }

  void publishFullStateTarget(const CommandState& state, const ros::Time& stamp)
  {
    const RootCommandKinematics root = tailFluToRootLinkCommand(
        state.tail_position, state.tail_velocity, state.yaw, state.yaw_rate,
        model_info_->linkLength());

    aerial_robot_msgs::FullStateTarget message;
    message.header.stamp = stamp;
    message.header.frame_id = config_.shared.common.worldFrameId;
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
    message.joint_state.name = model_info_->jointNames();
    message.joint_state.position.resize(static_cast<size_t>(state.joint_positions.size()));
    message.joint_state.velocity.resize(static_cast<size_t>(state.joint_velocities.size()));
    for (int index = 0; index < state.joint_positions.size(); ++index)
    {
      message.joint_state.position[static_cast<size_t>(index)] = state.joint_positions(index);
      message.joint_state.velocity[static_cast<size_t>(index)] = state.joint_velocities(index);
    }

    geometry_msgs::PoseStamped root_target;
    root_target.header = message.header;
    root_target.pose.position.x = state.tail_position.x();
    root_target.pose.position.y = state.tail_position.y();
    root_target.pose.position.z = state.tail_position.z();
    root_target.pose.orientation.z = std::sin(0.5 * state.yaw);
    root_target.pose.orientation.w = std::cos(0.5 * state.yaw);

    full_state_pub_.publish(message);
    root_target_pub_.publish(root_target);
  }

  WholeBodyPlannerConfig config_;
  ros::NodeHandle nh_;
  PlanningEnvironment environment_;
  gcopter_planner::PlannerRosInterface ros_interface_;
  pluginlib::ClassLoader<aerial_robot_model::RobotModel> robot_model_loader_;
  std::vector<boost::shared_ptr<Dragon::HydrusLikeRobotModel>> robot_models_;
  std::unique_ptr<DragonModelInfo> model_info_;
  std::vector<std::shared_ptr<multilink_copilot::StabilityEvaluator>> stability_evaluators_;
  std::vector<std::unique_ptr<JointTrajectoryPlanner>> joint_planners_;
  TrajectoryHistory executed_history_;
  DragonCollisionGeometry collision_geometry_;
  CandidateDiagnosticsPublisher diagnostics_;
  TrajectoryReplanTrigger replan_trigger_;

  ros::Subscriber map_sub_;
  ros::Subscriber target_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher full_state_pub_;
  ros::Publisher root_target_pub_;
  ros::Timer command_timer_;
  ros::Timer publisher_guard_timer_;
  std::string full_state_topic_;

  std::mutex map_mutex_;

  std::mutex state_mutex_;
  CommandState latest_odom_state_;
  bool odom_received_ = false;
  bool joints_received_ = false;
  bool target_received_ = false;
  uint64_t target_sequence_ = 0;
  Eigen::Vector3d target_ = Eigen::Vector3d::Zero();
  Eigen::VectorXd current_joints_;

  std::mutex plan_mutex_;
  std::shared_ptr<const ActivePlan> active_plan_;
  std::shared_ptr<const ActivePlan> pending_plan_;
  CommandState last_command_;
  std::atomic<bool> planning_in_progress_{false};
  std::atomic<bool> planning_requested_{false};
  std::atomic<bool> retry_pending_{false};
  std::atomic<bool> planning_shutdown_{false};
  std::atomic<bool> goal_latched_{false};
  std::mutex planning_request_mutex_;
  std::condition_variable planning_request_cv_;
  std::thread planning_worker_;
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
