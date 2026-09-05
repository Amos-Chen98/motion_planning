// -*- mode: c++ -*-
#ifndef MOTION_PRIMITIVE_PLANNER_DRAGON_GEOMETRY_H
#define MOTION_PRIMITIVE_PLANNER_DRAGON_GEOMETRY_H

#include <motion_primitive_planner/planner_config.h>

#include <dragon/model/hydrus_like_robot_model.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Geometry>

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace motion_primitive_planner
{

struct RootAttitude
{
  double yaw = 0.0;
  double pitch = 0.0;
};

struct WholeBodyConfiguration
{
  Eigen::Vector3d link1_tail = Eigen::Vector3d::Zero();
  Eigen::Matrix3d root_link_rotation = Eigen::Matrix3d::Identity();
  Eigen::VectorXd joint_positions;
};

struct DragonCollisionGeometry
{
  int link_num = 0;
  double link_length = 0.0;
  std::vector<int> pitch_joint_indices;
  std::vector<int> yaw_joint_indices;
};

struct RootCommandKinematics
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

double yawFromQuaternion(const Eigen::Quaterniond& quaternion);
double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion);
RootAttitude rootAttitudeFromQuaternion(const Eigen::Quaterniond& quaternion);
RootAttitude rootAttitudeFromQuaternion(const geometry_msgs::Quaternion& quaternion);
RootAttitude tangentAttitude(const Eigen::Vector3d& velocity,
                             const RootAttitude& fallback);
RootAttitude advanceRootAttitude(const RootAttitude& current,
                                 const Eigen::Vector3d& velocity,
                                 double dt,
                                 const FollowerConfig& config,
                                 bool command_pitch);
RootAttitude interpolateRootAttitude(const RootAttitude& start,
                                     const RootAttitude& goal,
                                     double ratio);
Eigen::Matrix3d fluRotation(const RootAttitude& attitude);
Eigen::Matrix3d linkRotation(const RootAttitude& attitude);
Eigen::Vector3d worldAngularVelocity(const RootAttitude& attitude,
                                     double yaw_rate,
                                     double pitch_rate);
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

RootCommandKinematics tailFluToRootLinkCommand(const Eigen::Vector3d& tail_position,
                                                const Eigen::Vector3d& tail_velocity,
                                                const Eigen::Matrix3d& tail_flu_rotation,
                                                const Eigen::Vector3d& angular_velocity,
                                                double link_length);

std::vector<Eigen::Vector3d> linkEndpoints(const Eigen::Vector3d& link1_tail,
                                           const Eigen::Matrix3d& root_link_rotation,
                                           const Eigen::VectorXd& joint_positions,
                                           const std::vector<int>& pitch_joint_indices,
                                           const std::vector<int>& yaw_joint_indices,
                                           int link_num,
                                           double link_length);

bool bodyCollides(const std::vector<Eigen::Vector3d>& endpoints,
                  double sample_spacing,
                  const std::function<bool(const Eigen::Vector3d&)>& occupied);

bool wholeBodyCollides(const WholeBodyConfiguration& configuration,
                       const DragonCollisionGeometry& geometry,
                       double sample_spacing,
                       const std::function<bool(const Eigen::Vector3d&)>& occupied);

double shortestYawDelta(double start_yaw, double end_yaw);

}  // namespace motion_primitive_planner

#endif  // MOTION_PRIMITIVE_PLANNER_DRAGON_GEOMETRY_H
