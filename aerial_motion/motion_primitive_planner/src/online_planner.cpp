#include <motion_primitive_planner/planner_config.h>
#include <motion_primitive_planner/root_candidate_evaluator.h>

#include <dragon/model/hydrus_like_robot_model.h>
#include <gcopter/planner_common.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_loader.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace motion_primitive_planner
{
namespace
{
constexpr char kRobotModelPlugin[] = "dragon/hydrus_like_robot_model";
}  // namespace

class OnlinePlanner
{
public:
  OnlinePlanner(const RootPlannerConfig& config, ros::NodeHandle& nh)
    : config_(config)
    , nh_(nh)
    , environment_(config.shared)
    , ros_interface_(config.shared.common, nh_)
    , robot_model_loader_("aerial_robot_model", "aerial_robot_model::RobotModel")
    , executed_history_(config.follower)
    , diagnostics_(nh_, config.shared.common.worldFrameId,
                   "motion_primitive_candidates", 0.0125)
  {
    initializeRobotModel();
    ROS_INFO("Motion primitive map starts as free space and will be updated when point clouds arrive.");
    map_sub_ = nh_.subscribe("pcl_topic", 1, &OnlinePlanner::mapCallback, this,
                             ros::TransportHints().tcpNoDelay());
    target_sub_ = nh_.subscribe("target", 1, &OnlinePlanner::targetCallback, this,
                                ros::TransportHints().tcpNoDelay());
    joint_state_sub_ = nh_.subscribe("joint_states", 1, &OnlinePlanner::jointStateCallback, this,
                                     ros::TransportHints().tcpNoDelay());
    executed_command_sub_ = nh_.subscribe("executed_command", 10,
                                          &OnlinePlanner::executedCommandCallback, this,
                                          ros::TransportHints().tcpNoDelay());
    ros_interface_.visualizer().clearTrajectory();
    timer_ = nh_.createTimer(ros::Duration(1.0 / config_.shared.replan_hz),
                             &OnlinePlanner::timerCallback, this);
    ROS_INFO("Motion primitive planner ready: N=%d, horizon=%.2f m, fc_rp_min>=%.2f, "
             "baselink_tilt<=%.2f, Copilot projection fallback=%s.",
             config_.shared.primitive.candidate_count, config_.shared.planning_horizon,
             config_.stability.fc_rp_min_threshold, config_.stability.max_baselink_tilt,
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
    model_info_.reset(new DragonModelInfo(robot_model_));
    current_joints_ = Eigen::VectorXd::Zero(model_info_->jointCount());
    stability_evaluator_ =
        std::make_shared<multilink_copilot::StabilityEvaluator>(robot_model_, config_.stability);
    candidate_evaluator_.reset(new RootCandidateEvaluator(
        config_.follower, config_.prediction_dt, config_.shared.primitive.max_velocity,
        config_.allow_copilot_stability_projection_fallback, *model_info_,
        stability_evaluator_, environment_));
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
    environment_.updateMap(points);
  }

  void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& message)
  {
    const Eigen::Vector3d requested(message->pose.position.x, message->pose.position.y,
                                    config_.shared.common.resolveTargetHeight(*message));
    target_ = environment_.clampTarget(
        requested, config_.shared.common.dilateRadius + environment_.voxelScale());
    const bool clamped = (target_ - requested).norm() > 1e-3;
    target_received_ = true;
    goal_latched_ = false;
    ros_interface_.visualizer().visualizeStartGoal(target_, 0.05, 1);
    ROS_INFO("Received motion-primitive goal [%.2f, %.2f, %.2f]%s.",
             target_.x(), target_.y(), target_.z(), clamped ? " (clamped to map)" : "");
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& message)
  {
    if (model_info_->readCompleteJointState(*message, current_joints_))
    {
      joints_received_ = true;
    }
  }

  void executedCommandCallback(const geometry_msgs::PoseStamped::ConstPtr& message)
  {
    executed_history_.append(Eigen::Vector3d(message->pose.position.x,
                                             message->pose.position.y,
                                             message->pose.position.z));
    executed_yaw_ = yawFromQuaternion(message->pose.orientation);
    executed_command_received_ = true;
  }

  bool activeTrajectory(double now) const
  {
    return trajectory_.getPieceNum() > 0 &&
           now - trajectory_stamp_ < trajectory_.getTotalDuration();
  }

  void publishDiagnostics(const std::vector<Candidate>& candidates, int selected)
  {
    std::vector<CandidateVisualization> visualizations;
    visualizations.reserve(candidates.size());
    for (const Candidate& candidate : candidates)
    {
      visualizations.push_back({&candidate.trajectory, candidate.status});
    }
    SelectedCandidateMetrics metrics;
    if (selected >= 0)
    {
      const Candidate& candidate = candidates[static_cast<size_t>(selected)];
      metrics.minimum_fc_rp = candidate.min_fc_rp;
      metrics.joint_motion = candidate.joint_motion;
    }
    diagnostics_.publish(visualizations, selected, metrics);
    ros_interface_.visualizer().clearTrajectory();
  }

  void timerCallback(const ros::TimerEvent&)
  {
    if (!target_received_ || !ros_interface_.odomReceived() || !joints_received_ || goal_latched_)
    {
      return;
    }
    const double handover = ros::Time::now().toSec();
    RootState start;
    start.position = ros_interface_.latestPosition();
    if (activeTrajectory(handover))
    {
      const double time = std::max(
          0.0, std::min(handover - trajectory_stamp_, trajectory_.getTotalDuration()));
      start.position = trajectory_.getPos(time);
      start.velocity = trajectory_.getVel(time);
      start.acceleration = trajectory_.getAcc(time);
    }
    if ((start.position - target_).norm() <= config_.shared.goal_tolerance)
    {
      goal_latched_ = true;
      return;
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
          ROS_INFO("Root-link local planning completed in %.1f ms.", elapsed_ms);
        }
      }

      bool enabled_;
      std::chrono::steady_clock::time_point start_;
    } timing_logger(config_.verbose);

    PrimitiveBatch batch = environment_.generate(start, target_);
    if (!batch.success())
    {
      if (batch.failure == PrimitiveBatchFailure::kStartCollision)
      {
        ROS_WARN_THROTTLE(1.0, "Motion primitive start is in collision.");
      }
      return;
    }

    const double start_yaw = config_.follower.publish_yaw_command && executed_command_received_
                                 ? executed_yaw_
                                 : yawFromQuaternion(ros_interface_.latestOrientation());
    const NominalJointContext nominal_context =
        makeNominalJointContext(executed_history_, *model_info_);
    for (Candidate& candidate : batch.candidates)
    {
      candidate_evaluator_->evaluate(candidate, nominal_context, current_joints_, start_yaw);
    }
    for (size_t index = 0; index < batch.candidates.size(); ++index)
    {
      const Candidate& candidate = batch.candidates[index];
      ROS_DEBUG("Primitive %zu: %s, min_fc=%.3f, joint_motion=%.3f, %s", index,
                candidateStatusName(candidate.status), candidate.min_fc_rp,
                candidate.joint_motion, candidate.detail.c_str());
    }
    const int selected = selectBestCandidate(
        batch.candidates, config_.allow_copilot_stability_projection_fallback);
    const bool selected_requires_projection =
        selected >= 0 && batch.candidates[static_cast<size_t>(selected)].requires_stability_projection;
    if (selected >= 0)
    {
      batch.candidates[static_cast<size_t>(selected)].status = CandidateStatus::kSelected;
    }
    publishDiagnostics(batch.candidates, selected);
    if (selected < 0)
    {
      ROS_WARN("All %zu primitives rejected; keeping the previous trajectory.",
               batch.candidates.size());
      return;
    }
    const Candidate& selected_candidate = batch.candidates[static_cast<size_t>(selected)];
    if (selected_requires_projection)
    {
      ROS_WARN("No nominally stable primitive was available; selecting primitive %d "
               "(nominal min fc_rp %.3f, joint motion %.3f rad) and relying on "
               "multilink_copilot to project the joint target to a stable configuration.",
               selected, selected_candidate.min_fc_rp, selected_candidate.joint_motion);
    }
    trajectory_ = selected_candidate.trajectory;
    trajectory_stamp_ = handover;
    ros_interface_.publishTrajectory(trajectory_, handover);
    goal_latched_ = batch.terminal;
    ROS_INFO("Selected primitive %d/%zu (length %.2f m, min fc_rp %.3f).",
             selected, batch.candidates.size(), selected_candidate.path_length,
             selected_candidate.min_fc_rp);
    ros_interface_.visualizer().visualizeStartGoal(start.position, 0.05, 0);
    ros_interface_.visualizer().visualizeStartGoal(target_, 0.05, 1);
  }

  RootPlannerConfig config_;
  ros::NodeHandle nh_;
  PlanningEnvironment environment_;
  gcopter_planner::PlannerRosInterface ros_interface_;
  pluginlib::ClassLoader<aerial_robot_model::RobotModel> robot_model_loader_;
  boost::shared_ptr<Dragon::HydrusLikeRobotModel> robot_model_;
  std::unique_ptr<DragonModelInfo> model_info_;
  std::shared_ptr<multilink_copilot::StabilityEvaluator> stability_evaluator_;
  std::unique_ptr<RootCandidateEvaluator> candidate_evaluator_;
  TrajectoryHistory executed_history_;
  CandidateDiagnosticsPublisher diagnostics_;

  ros::Subscriber map_sub_;
  ros::Subscriber target_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber executed_command_sub_;
  ros::Timer timer_;

  double executed_yaw_ = 0.0;
  bool executed_command_received_ = false;
  bool target_received_ = false;
  bool joints_received_ = false;
  bool goal_latched_ = false;
  Eigen::Vector3d target_ = Eigen::Vector3d::Zero();
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
    motion_primitive_planner::RootPlannerConfig config(ros::NodeHandle("~"));
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
