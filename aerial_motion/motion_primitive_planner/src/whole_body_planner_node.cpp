#include <motion_primitive_planner/whole_body_planner.h>
#include <motion_primitive_planner/whole_body_planner_node.h>

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
//! It absorbs the last candidate's sampled collision check and diagnostics.
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
  RootAttitude attitude;
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
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
    return hold ? std::numeric_limits<double>::infinity() : joint_plan.duration;
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
    const double time = std::max(0.0, std::min((stamp - start_time).toSec(),
                                               joint_plan.duration));
    const double root_time = std::max(
        0.0, std::min(root_trajectory.getTotalDuration(),
                      time - joint_plan.root_translation_delay));
    state.valid = true;
    state.tail_position = root_trajectory.getPos(root_time);
    if (time + kEpsilon < joint_plan.root_translation_delay)
    {
      state.tail_velocity.setZero();
      state.tail_acceleration.setZero();
    }
    else
    {
      state.tail_velocity = root_trajectory.getVel(root_time);
      state.tail_acceleration = root_trajectory.getAcc(root_time);
    }
    state.attitude = joint_plan.attitude(time);
    state.angular_velocity = joint_plan.angularVelocity(time);
    state.joint_positions = joint_plan.jointPositions(time);
    state.joint_velocities = joint_plan.jointVelocities(time);
    return state;
  }
};

}  // namespace

class WholeBodyPlannerNode
{
public:
  WholeBodyPlannerNode(const WholeBodyPlannerConfig& config, ros::NodeHandle& nh)
    : config_(config)
    , nh_(nh)
    , environment_(config.shared)
    , ros_interface_(config.shared.common, nh_)
    , robot_model_loader_("aerial_robot_model", "aerial_robot_model::RobotModel")
    , executed_history_(config.joint.follower)
    , replan_trigger_(config.shared.replan_trigger_ratio)
  {
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("candidate_markers", 1, true);
    selected_candidate_pub_ = nh_.advertise<std_msgs::Int32>("selected_candidate", 1, true);
    selected_min_fc_pub_ = nh_.advertise<std_msgs::Float64>("selected_min_fc_rp", 1, true);
    selected_joint_motion_pub_ = nh_.advertise<std_msgs::Float64>("selected_joint_motion", 1, true);
    initializeRobotModels();
    map_sub_ = nh_.subscribe("voxelmap/occupied", 1, &WholeBodyPlannerNode::mapCallback, this,
                             ros::TransportHints().tcpNoDelay());
    target_sub_ = nh_.subscribe("target", 1, &WholeBodyPlannerNode::targetCallback, this,
                                ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh_.subscribe("odom", 1, &WholeBodyPlannerNode::odomCallback, this,
                              ros::TransportHints().tcpNoDelay());
    joint_state_sub_ = nh_.subscribe("joint_states", 1, &WholeBodyPlannerNode::jointStateCallback, this,
                                     ros::TransportHints().tcpNoDelay());
    full_state_topic_ = nh_.resolveName("full_state_target");
    if (hasConflictingFullStatePublisher())
    {
      throw std::runtime_error("Another node already publishes " + full_state_topic_);
    }
    full_state_pub_ = nh_.advertise<aerial_robot_msgs::FullStateTarget>("full_state_target", 10);
    root_target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("root/target_pose", 10);
    command_timer_ = nh_.createTimer(ros::Duration(1.0 / config_.joint.follower.command_hz),
                                     &WholeBodyPlannerNode::commandTimerCallback, this);
    publisher_guard_timer_ = nh_.createTimer(ros::Duration(1.0),
                                             &WholeBodyPlannerNode::publisherGuardTimerCallback, this);
    planning_worker_ = std::thread(&WholeBodyPlannerNode::planningWorker, this);
    ROS_INFO("Whole-body motion primitive planner ready: candidates=%d, fc_rp_min>=%.2f, "
             "collision radius=%.2f m, replan_ratio=%.2f.",
             config_.shared.primitive.candidate_count, config_.stability.fc_rp_min_threshold,
             config_.shared.common.dilateRadius, config_.shared.replan_trigger_ratio);
  }

  ~WholeBodyPlannerNode()
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
    }

    model_info_.reset(new DragonModelInfo(robot_models_.front()));
    planner_.reset(new WholeBodyPlanner(config_, model_info_->collisionGeometry(),
                                         stability_evaluators_));
    current_joints_ = Eigen::VectorXd::Zero(model_info_->jointCount());
  }

  void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& message)
  {
    std::vector<Eigen::Vector3d> occupied_voxel_centers;
    std::string error;
    if (!ros_interface_.pointCloudToWorld(*message, occupied_voxel_centers, &error))
    {
      if (error != "point-cloud transform is unavailable")
      {
        ROS_WARN_THROTTLE(1.0, "Invalid occupied voxel map: %s", error.c_str());
      }
      return;
    }
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      environment_.replaceMap(occupied_voxel_centers);
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
      latest_odom_state_.attitude = rootAttitudeFromQuaternion(message->pose.pose.orientation);
      latest_odom_state_.angular_velocity = Eigen::Vector3d(message->twist.twist.angular.x,
                                                            message->twist.twist.angular.y,
                                                            message->twist.twist.angular.z);
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
      const Trajectory<5>& root = candidate.scaled_root.getPieceNum() > 0
                                      ? candidate.scaled_root : candidate.root.trajectory;
      if (root.getPieceNum() <= 0)
      {
        continue;
      }
      visualization_msgs::Marker marker;
      marker.header.frame_id = config_.shared.common.worldFrameId;
      marker.header.stamp = stamp;
      marker.ns = "whole_body_root_candidates";
      marker.id = static_cast<int>(index);
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.015;
      marker.color = candidateColor(candidate.status, static_cast<int>(index) == selected);
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
    const JointPlanResult* joints = selected >= 0 ? &candidates[static_cast<size_t>(selected)].joints : nullptr;
    std_msgs::Float64 value;
    value.data = joints ? joints->minimum_fc_rp : std::numeric_limits<double>::quiet_NaN();
    selected_min_fc_pub_.publish(value);
    value.data = joints ? joints->joint_motion : std::numeric_limits<double>::quiet_NaN();
    selected_joint_motion_pub_.publish(value);
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
    const NominalJointContext nominal_context = makeNominalJointContext(history, *model_info_);
    const ros::Time joint_planning_deadline =
        activation_time - ros::Duration(kActivationSafetyMargin);
    const WholeBodyPlanResult result = planner_->plan(
        batch, occupancy, start.joint_positions, start.attitude, nominal_context,
        joint_planning_deadline);
    const std::vector<WholeBodyCandidate>& candidates = result.candidates;
    const int selected = result.selected;
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
             "joint_motion=%.3f rad, tracking_rms=%.3f m, tracking_max=%.3f m, "
             "scale=%.2f.",
             selected, candidates.size(), selected_candidate.joints.duration,
             selected_candidate.joints.minimum_fc_rp, selected_candidate.joints.joint_motion,
             selected_candidate.joints.tracking_error_rms,
             selected_candidate.joints.tracking_error_max,
             selected_candidate.joints.time_scale);
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
      const Eigen::Quaterniond quaternion(linkRotation(state.attitude));
      stability_evaluators_.front()->setRootLinkRotation(KDL::Rotation::Quaternion(
          quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()));
      multilink_copilot::StabilityMetrics metrics;
      if (!stability_evaluators_.front()->evaluate(state.joint_positions, metrics) || !metrics.safe)
      {
        ROS_ERROR_THROTTLE(1.0, "Refusing to publish an unstable startup hold command.");
        return;
      }
    }
    state.tail_velocity.setZero();
    state.tail_acceleration.setZero();
    state.angular_velocity.setZero();
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
      state.angular_velocity.setZero();
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
        state.tail_position, state.tail_velocity, fluRotation(state.attitude),
        state.angular_velocity,
        model_info_->linkLength());

    aerial_robot_msgs::FullStateTarget message;
    message.header.stamp = stamp;
    message.header.frame_id = config_.shared.common.worldFrameId;
    message.root_state.header = message.header;
    message.root_state.child_frame_id = config_.root_child_frame_id;
    message.root_state.pose.pose.position.x = root.position.x();
    message.root_state.pose.pose.position.y = root.position.y();
    message.root_state.pose.pose.position.z = root.position.z();
    message.root_state.pose.pose.orientation.x = root.orientation.x();
    message.root_state.pose.pose.orientation.y = root.orientation.y();
    message.root_state.pose.pose.orientation.z = root.orientation.z();
    message.root_state.pose.pose.orientation.w = root.orientation.w();
    message.root_state.twist.twist.linear.x = root.linear_velocity.x();
    message.root_state.twist.twist.linear.y = root.linear_velocity.y();
    message.root_state.twist.twist.linear.z = root.linear_velocity.z();
    message.root_state.twist.twist.angular.x = root.angular_velocity.x();
    message.root_state.twist.twist.angular.y = root.angular_velocity.y();
    message.root_state.twist.twist.angular.z = root.angular_velocity.z();
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
    const Eigen::Quaterniond tail_orientation(fluRotation(state.attitude));
    root_target.pose.orientation.x = tail_orientation.x();
    root_target.pose.orientation.y = tail_orientation.y();
    root_target.pose.orientation.z = tail_orientation.z();
    root_target.pose.orientation.w = tail_orientation.w();

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
  std::unique_ptr<WholeBodyPlanner> planner_;
  TrajectoryHistory executed_history_;
  TrajectoryReplanTrigger replan_trigger_;

  ros::Subscriber map_sub_;
  ros::Subscriber target_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher full_state_pub_;
  ros::Publisher root_target_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher selected_candidate_pub_;
  ros::Publisher selected_min_fc_pub_;
  ros::Publisher selected_joint_motion_pub_;
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
    motion_primitive_planner::WholeBodyPlannerNode planner(config, nh);
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
