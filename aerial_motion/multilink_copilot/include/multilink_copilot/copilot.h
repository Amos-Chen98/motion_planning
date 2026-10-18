// -*- mode: c++ -*-
#ifndef MULTILINK_COPILOT_COPILOT_H
#define MULTILINK_COPILOT_COPILOT_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <aerial_robot_msgs/FullStateTarget.h>
#include <visualization_msgs/MarkerArray.h>
#include <dragon/model/hydrus_like_robot_model.h>
#include <multilink_copilot/follow_the_leader.h>
#include <multilink_copilot/stability_evaluator.h>
#include <multilink_copilot/joint_seed_selection.h>
#include <std_msgs/Time.h>
#include <pluginlib/class_loader.h>
#include <kdl/jntarray.hpp>
#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace multilink_copilot
{

class CopilotPlanner
{
public:
  struct UpdateResult
  {
    std::string status = "no_input";
    std::string source = "none";
    bool fallback = false;
    bool geometry_warmup = false;
    double total_ms = 0.0;
    double feasibility_ms = 0.0;
    double planning_ms = 0.0;  // Legacy timing boundary, excluding history and final gate.
    int projection_calls = 0;
    int qp_solves = 0;
    int projection_iterations = 0;
    aerial_robot_msgs::FullStateTarget target;
    bool published() const { return status == "published"; }
  };

  explicit CopilotPlanner(bool start_ros_runtime = true);
  // Synchronous execution uses the same stateful update as the ROS timer.
  UpdateResult step(const geometry_msgs::PoseStamped& root_tail_pose,
                    const Eigen::VectorXd& measured_joints);
  const std::vector<std::string>& jointNames() const { return link_joint_names_; }
  int linkCount() const { return link_num_; }
  StabilityEvaluator& stabilityEvaluator() { return *stability_evaluator_; }

  ~CopilotPlanner() = default;

  using StabilityMetrics = multilink_copilot::StabilityMetrics;

  using StableCandidateSource = JointSeedSource;

private:
  friend class CopilotStepTest;
  UpdateResult updateOnce();
  std::string accepted_source_ = "none";
  bool used_fallback_ = false;
  // ROS handles
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber target_pose_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher full_state_target_pub_;
  ros::Publisher trajectory_viz_pub_;
  ros::Publisher stability_fc_rp_min_pub_;
  ros::Publisher stability_overlap_clearance_pub_;
  ros::Timer control_timer_;
  ros::Timer stability_debug_timer_;
  ros::Publisher first_skip_pub_;
  bool publish_skip_event_ = false;
  ros::Time first_skip_stamp_;
  void recordFirstSkip(const geometry_msgs::Pose& root_target_pose);

  // Robot model
  pluginlib::ClassLoader<aerial_robot_model::RobotModel> robot_model_loader_;
  boost::shared_ptr<Dragon::HydrusLikeRobotModel> dragon_robot_model_;
  std::unique_ptr<StabilityEvaluator> stability_evaluator_;
  
  // Robot parameters
  int link_num_;
  double link_length_;
  std::vector<std::string> link_joint_names_;
  std::vector<int> link_joint_indices_;
  int link_joint_num_;
  std::vector<int> pitch_joint_local_indices_;
  std::vector<int> yaw_joint_local_indices_;
  std::unordered_map<std::string, int> link_joint_name_to_local_index_;
  
  // Trajectory tracking
  std::deque<TrajectoryPoint> trajectory_buffer_;
  double total_arc_length_;
  bool trajectory_initialized_;
  Eigen::Vector3d last_recorded_position_;
  
  // Control parameters
  double trajectory_sample_interval_;
  double trajectory_buffer_max_length_;
  double control_loop_rate_;
  bool snake_mode_enabled_;
  std::string target_pose_frame_type_;  // "FLU" or "LINK"
  bool publish_only_on_significant_root_motion_;
  double publish_root_translation_threshold_;
  double publish_root_rotation_threshold_;
  double snake_ik_singularity_xz_norm_threshold_;
  double max_joint_step_before_publish_;
  double max_baselink_tilt_before_publish_;
  bool publish_stability_metrics_;
  bool verbose_;
  int stability_qp_max_iterations_;
  double stability_qp_joint_step_limit_;
  double stability_qp_regularization_;
  double stability_qp_convergence_tol_;
  double stability_qp_feasibility_tol_;
  bool stability_check_fc_t_;
  double stability_fc_rp_min_thre_;
  double stability_fc_t_min_thre_;
  double stability_static_thrust_min_;
  double stability_static_thrust_max_;
  double stability_overlap_min_clearance_;
  int stability_candidate_history_size_;
  int stability_candidate_max_repairs_;
  
  // Current state
  geometry_msgs::PoseStamped::ConstPtr latest_target_pose_;  // Interpreted as the first-link tail target pose.
  Eigen::VectorXd latest_measured_link_joint_positions_;
  bool has_latest_measured_link_joint_positions_;
  Eigen::VectorXd latest_published_joint_positions_;
  bool has_latest_published_joint_positions_;
  std::deque<Eigen::VectorXd> stable_joint_history_;
  bool stability_debug_timer_started_;
  geometry_msgs::Pose last_published_root_pose_;
  bool has_last_published_root_pose_;
  Eigen::Vector3d root_pos_world_;
  Eigen::Vector3d link1_tail_pos_world_;
  std::vector<Eigen::Vector3d> latest_snake_targets_;
  
  // Callbacks
  void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
  void controlTimerCallback(const ros::TimerEvent& event);
  void stabilityDebugTimerCallback(const ros::TimerEvent& event);
  
  // Initialization
  void initializeRobotModel();
  void loadParameters();
  
  // Trajectory management
  void updateTrajectoryBuffer(const Eigen::Vector3d& link1_tail_position, const Eigen::Vector3d& root_position);
  bool prepareTrajectoryData();
  
  // Snake motion computation
  std::vector<Eigen::Vector3d> computeSnakeTargetPositions();
  std::vector<Eigen::Vector3d> computeWarmupTargetPositions();
  Eigen::VectorXd computeJointAnglesFromSnakeTarget(const std::vector<Eigen::Vector3d>& target_positions);
  Eigen::VectorXd computeWarmupNominalJointPositions(const std::vector<Eigen::Vector3d>& target_positions);
  
  // Stability check
  bool checkStability(const Eigen::VectorXd& joint_positions, bool report_result = true);
  void updateRobotModelForTargetConfiguration(const KDL::JntArray& joint_positions);
  bool computeStableJointPositions(const Eigen::VectorXd& nominal_joint_positions,
                                   Eigen::VectorXd& stable_joint_positions);
  bool solveStableJointQp(const Eigen::VectorXd& desired_joint_positions,
                          const Eigen::VectorXd& reference_joint_positions,
                          Eigen::VectorXd& stable_joint_positions,
                          bool allow_unstable_seed = false);
  bool evaluateStability(const Eigen::VectorXd& joint_positions, StabilityMetrics& metrics);
  bool satisfiesSafeStability(const StabilityMetrics& metrics) const;
  std::string describeStabilityViolations(const StabilityMetrics& metrics) const;
  double computeStabilityViolationScore(const StabilityMetrics& metrics) const;
  void rememberStableJointPositions(const Eigen::VectorXd& stable_joint_positions);
  bool areSimilarJointPositions(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs) const;
  const char* describeStableCandidateSource(StableCandidateSource source) const;
  bool enforceStableJointCommandContinuity(const Eigen::VectorXd& measured_joint_positions,
                                           Eigen::VectorXd& stable_joint_positions,
                                           StableCandidateSource& accepted_source);
  void restoreRobotModelToLinkJointPositions(const Eigen::VectorXd& joint_positions);
  
  // Helper functions
  KDL::JntArray buildUpdatedJointPositions(const Eigen::VectorXd& joint_positions) const;
  Eigen::VectorXd clampLinkJointPositions(const Eigen::VectorXd& joint_positions) const;
  Eigen::VectorXd getCurrentLinkJointPositions() const;
  double getReferenceJointPosition(int local_joint_index) const;
  Eigen::VectorXd buildDefaultReferenceJointPositions() const;
  geometry_msgs::Pose convertLink1TailPoseToRootPose(const geometry_msgs::Pose& pose) const;
  Eigen::Vector3d getLink1TailPositionFromPose(const geometry_msgs::Pose& pose) const;
  Eigen::Vector3d getRootPositionFromLink1TailPose(const geometry_msgs::Pose& pose) const;
  bool shouldPublishFullStateTarget(const geometry_msgs::Pose& root_target_pose) const;
  bool passesFullStateTargetSafetyChecks(const geometry_msgs::Pose& root_target_pose,
                                         const Eigen::VectorXd& joint_positions) const;
  void recordPublishedRootPose(const geometry_msgs::Pose& root_target_pose);
  void publishStabilityMetrics(const StabilityMetrics& metrics);
  void publishCurrentStabilityMetrics();
  
  // Visualization
  visualization_msgs::MarkerArray getTrajectoryVisualization();
};

}  // namespace multilink_copilot

#endif  // MULTILINK_COPILOT_COPILOT_H
