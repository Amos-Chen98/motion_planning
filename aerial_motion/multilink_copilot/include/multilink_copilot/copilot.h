// -*- mode: c++ -*-
#ifndef MULTILINK_COPILOT_COPILOT_H
#define MULTILINK_COPILOT_COPILOT_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <aerial_robot_msgs/FullStateTarget.h>
#include <visualization_msgs/MarkerArray.h>
#include <dragon/model/hydrus_like_robot_model.h>
#include <pluginlib/class_loader.h>
#include <kdl/jntarray.hpp>
#include <Eigen/Dense>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

namespace multilink_copilot
{

struct TrajectoryPoint
{
  Eigen::Vector3d position;
};

class CopilotPlanner
{
public:
  CopilotPlanner();
  ~CopilotPlanner() = default;

  struct StabilityMetrics
  {
    bool safe = false;
    double fc_rp_min = 0.0;
    double fc_t_min = 0.0;
    double static_thrust_min = 0.0;
    double static_thrust_max = 0.0;
    double overlap_clearance = 0.0;
  };

  enum class StableCandidateSource
  {
    kCurrentMeasured,
    kLatestStable,
    kStableHistory,
    kDesiredSeed,
    kProjected,
    kRepair
  };

  struct StableHistoryEntry
  {
    Eigen::VectorXd joint_positions;
    Eigen::Vector3d target_direction = Eigen::Vector3d::UnitX();
    ros::Time stamp;
    StableCandidateSource source = StableCandidateSource::kProjected;
  };

  struct StableCandidate
  {
    Eigen::VectorXd joint_positions;
    StabilityMetrics metrics;
    StableCandidateSource source = StableCandidateSource::kCurrentMeasured;
    double target_direction_angle = 0.0;
    double desired_distance = 0.0;
    double measured_distance = 0.0;
    double violation_score = 0.0;
  };

private:
  // ROS handles
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber target_pose_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher full_state_target_pub_;
  ros::Publisher trajectory_viz_pub_;
  ros::Timer control_timer_;
  ros::Timer stability_debug_timer_;

  // Robot model
  pluginlib::ClassLoader<aerial_robot_model::RobotModel> robot_model_loader_;
  boost::shared_ptr<Dragon::HydrusLikeRobotModel> dragon_robot_model_;
  
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
  int stability_candidate_top_k_;
  int stability_candidate_max_repairs_;
  
  // Current state
  geometry_msgs::PoseStamped::ConstPtr latest_target_pose_;  // Interpreted as the first-link tail target pose.
  Eigen::VectorXd latest_measured_link_joint_positions_;
  bool has_latest_measured_link_joint_positions_;
  Eigen::VectorXd latest_published_joint_positions_;
  bool has_latest_published_joint_positions_;
  Eigen::VectorXd latest_stable_joint_positions_;
  bool has_latest_stable_joint_positions_;
  std::deque<StableHistoryEntry> stable_joint_history_;
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
  Eigen::VectorXd computeWarmupJointPositions(const std::vector<Eigen::Vector3d>& target_positions);
  
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
  std::vector<StableCandidate> buildStableCandidates(const Eigen::VectorXd& desired_joint_positions,
                                                     const Eigen::VectorXd& measured_joint_positions,
                                                     const Eigen::Vector3d& current_target_direction);
  bool tryGetStableReferenceJointPositions(const std::vector<StableCandidate>& candidate_pool,
                                           Eigen::VectorXd& stable_reference,
                                           StableCandidateSource& reference_source);
  bool tryRepairStableJointPositions(const Eigen::VectorXd& desired_joint_positions,
                                     const std::vector<StableCandidate>& candidate_pool,
                                     Eigen::VectorXd& stable_joint_positions,
                                     StableCandidateSource& repaired_source);
  StableCandidate buildStableCandidate(const Eigen::VectorXd& raw_joint_positions,
                                       StableCandidateSource source,
                                       const Eigen::VectorXd& desired_joint_positions,
                                       const Eigen::VectorXd& measured_joint_positions,
                                       double target_direction_angle = 0.0);
  std::vector<StableHistoryEntry> getNearestStableHistoryEntries(const Eigen::VectorXd& desired_joint_positions,
                                                                 const Eigen::VectorXd& measured_joint_positions,
                                                                 const Eigen::Vector3d& current_target_direction) const;
  void rememberStableJointPositions(const Eigen::VectorXd& stable_joint_positions, StableCandidateSource source);
  Eigen::Vector3d getCurrentTargetDirection() const;
  double computeTargetDirectionAngle(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) const;
  bool areSimilarJointPositions(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs) const;
  const char* describeStableCandidateSource(StableCandidateSource source) const;
  void restoreRobotModelToLinkJointPositions(const Eigen::VectorXd& joint_positions);
  
  // Helper functions
  KDL::JntArray buildUpdatedJointPositions(const Eigen::VectorXd& joint_positions) const;
  Eigen::VectorXd clampLinkJointPositions(const Eigen::VectorXd& joint_positions) const;
  Eigen::VectorXd getCurrentLinkJointPositions() const;
  Eigen::VectorXd buildDefaultReferenceJointPositions() const;
  geometry_msgs::Pose convertLink1TailPoseToRootPose(const geometry_msgs::Pose& pose) const;
  Eigen::Vector3d getLink1TailPositionFromPose(const geometry_msgs::Pose& pose) const;
  Eigen::Vector3d getRootPositionFromLink1TailPose(const geometry_msgs::Pose& pose) const;
  bool shouldPublishFullStateTarget(const geometry_msgs::Pose& root_target_pose) const;
  void recordPublishedRootPose(const geometry_msgs::Pose& root_target_pose);
  
  // Visualization
  visualization_msgs::MarkerArray getTrajectoryVisualization();
};

}  // namespace multilink_copilot

#endif  // MULTILINK_COPILOT_COPILOT_H
