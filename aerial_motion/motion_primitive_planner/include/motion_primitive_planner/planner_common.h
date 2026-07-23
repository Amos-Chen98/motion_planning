// -*- mode: c++ -*-
#ifndef MOTION_PRIMITIVE_PLANNER_PLANNER_COMMON_H
#define MOTION_PRIMITIVE_PLANNER_PLANNER_COMMON_H

#include <motion_primitive_planner/planner_core.h>

#include <dragon/model/hydrus_like_robot_model.h>
#include <gcopter/planner_common.hpp>
#include <multilink_copilot/follow_the_leader.h>
#include <multilink_copilot/stability_evaluator.h>

#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace motion_primitive_planner
{

struct FollowerConfig
{
  double command_hz = 40.0;
  double trajectory_sample_interval = 0.05;
  double trajectory_buffer_max_length = 10.0;
  double ik_singularity_threshold = 0.10;
  double max_yaw_rate = 1.5;
  bool publish_yaw_command = true;

  static FollowerConfig fromRos(const ros::NodeHandle& private_nh);
  void validateOrThrow() const;
};

struct SharedPlannerConfig
{
  gcopter_planner::CommonPlannerConfig common;
  PrimitiveConfig primitive;
  double replan_hz = 2.0;
  bool use_accumulated_map = true;
  double goal_tolerance = 0.2;
  double planning_horizon = 3.0;

  SharedPlannerConfig() = default;
  explicit SharedPlannerConfig(const ros::NodeHandle& private_nh);
  void validateOrThrow() const;
};

multilink_copilot::StabilityConfig loadStabilityConfig(const ros::NodeHandle& private_nh);

double yawFromQuaternion(const Eigen::Quaterniond& quaternion);
double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion);
double advanceYaw(double current_yaw, const Eigen::Vector3d& velocity, double dt,
                  const FollowerConfig& config);

class DragonModelInfo
{
public:
  explicit DragonModelInfo(const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& model);

  bool readCompleteJointState(const sensor_msgs::JointState& message,
                              Eigen::VectorXd& joint_positions) const;

  int linkNum() const { return link_num_; }
  double linkLength() const { return link_length_; }
  int jointCount() const { return static_cast<int>(link_joint_names_.size()); }
  const std::vector<std::string>& jointNames() const { return link_joint_names_; }
  const std::vector<int>& linkJointIndices() const { return link_joint_indices_; }
  const std::vector<int>& pitchJointIndices() const { return pitch_joint_indices_; }
  const std::vector<int>& yawJointIndices() const { return yaw_joint_indices_; }
  DragonCollisionGeometry collisionGeometry() const;

private:
  int link_num_ = 0;
  double link_length_ = 0.0;
  std::vector<std::string> link_joint_names_;
  std::vector<int> link_joint_indices_;
  std::vector<int> pitch_joint_indices_;
  std::vector<int> yaw_joint_indices_;
  std::unordered_map<std::string, int> joint_name_to_index_;
};

class TrajectoryHistory
{
public:
  TrajectoryHistory() = default;
  explicit TrajectoryHistory(const FollowerConfig& config);

  bool append(const Eigen::Vector3d& position);
  bool append(const Eigen::Vector3d& position, double sample_interval, double maximum_length);

  const std::deque<multilink_copilot::TrajectoryPoint>& points() const { return points_; }
  double arcLength() const { return arc_length_; }
  double sampleInterval() const { return sample_interval_; }
  double maximumLength() const { return maximum_length_; }

private:
  double sample_interval_ = 0.05;
  double maximum_length_ = 10.0;
  std::deque<multilink_copilot::TrajectoryPoint> points_;
  double arc_length_ = 0.0;
};

struct NominalJointContext
{
  TrajectoryHistory executed_history;
  int link_num = 0;
  double link_length = 0.0;
  std::vector<int> pitch_joint_indices;
  std::vector<int> yaw_joint_indices;
};

NominalJointContext makeNominalJointContext(const TrajectoryHistory& history,
                                            const DragonModelInfo& model);

struct NominalJointSample
{
  double time = 0.0;
  double yaw = 0.0;
  Eigen::Vector3d root_position = Eigen::Vector3d::Zero();
  Eigen::VectorXd joints;
  bool history_changed = false;
};

class NominalJointPredictor
{
public:
  explicit NominalJointPredictor(const FollowerConfig& config) : config_(config)
  {
    config_.validateOrThrow();
  }

  std::vector<NominalJointSample> predict(const Trajectory<5>& root_trajectory,
                                          const NominalJointContext& context,
                                          const Eigen::VectorXd& start_joints,
                                          double start_yaw,
                                          double sample_dt) const;

private:
  FollowerConfig config_;
};

struct RootState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
};

enum class PrimitiveBatchFailure
{
  kNone,
  kStartCollision,
  kRouteSearchFailed,
  kLocalRouteEmpty
};

struct PrimitiveBatch
{
  PrimitiveBatchFailure failure = PrimitiveBatchFailure::kNone;
  std::string detail;
  std::vector<Eigen::Vector3d> local_route;
  Eigen::Vector3d local_target = Eigen::Vector3d::Zero();
  bool terminal = false;
  std::vector<Candidate> candidates;

  bool success() const { return failure == PrimitiveBatchFailure::kNone; }
};

class PlanningEnvironment
{
public:
  explicit PlanningEnvironment(const SharedPlannerConfig& config);

  void updateMap(const std::vector<Eigen::Vector3d>& points);
  Eigen::Vector3d clampTarget(const Eigen::Vector3d& requested, double clearance) const;
  PrimitiveBatch generate(const RootState& start, const Eigen::Vector3d& target);

  bool occupied(const Eigen::Vector3d& point) const;
  double voxelScale() const;
  Eigen::Vector3d mapOrigin() const;
  Eigen::Vector3d mapCorner() const;
  std::shared_ptr<const gcopter_planner::PlannerBackend> occupancySnapshot() const;

  static Eigen::Vector3d truncateRoute(const std::vector<Eigen::Vector3d>& full_route,
                                       double horizon,
                                       std::vector<Eigen::Vector3d>& local_route);

private:
  void rebuildMap();
  std::vector<Eigen::Vector3i> occupiedVoxels(
      const gcopter_planner::PlannerBackend& backend) const;

  SharedPlannerConfig config_;
  std::shared_ptr<gcopter_planner::PlannerBackend> backend_;
  PrimitiveGenerator generator_;
  std::unordered_set<long> occupied_voxel_keys_;
};

struct CandidateVisualization
{
  const Trajectory<5>* trajectory = nullptr;
  CandidateStatus status = CandidateStatus::kUnevaluated;
};

struct SelectedCandidateMetrics
{
  double minimum_fc_rp = std::numeric_limits<double>::quiet_NaN();
  double joint_motion = std::numeric_limits<double>::quiet_NaN();
};

std_msgs::ColorRGBA candidateColor(CandidateStatus status, bool selected);

class CandidateDiagnosticsPublisher
{
public:
  CandidateDiagnosticsPublisher(ros::NodeHandle& nh, const std::string& world_frame_id,
                                const std::string& marker_namespace, double marker_width);

  void publish(const std::vector<CandidateVisualization>& candidates, int selected,
               const SelectedCandidateMetrics& metrics) const;

private:
  std::string world_frame_id_;
  std::string marker_namespace_;
  double marker_width_ = 0.015;
  ros::Publisher marker_pub_;
  ros::Publisher selected_candidate_pub_;
  ros::Publisher selected_min_fc_pub_;
  ros::Publisher selected_joint_motion_pub_;
};

}  // namespace motion_primitive_planner

#endif  // MOTION_PRIMITIVE_PLANNER_PLANNER_COMMON_H
