#include <geo_robot_model/dragon_stability_monitor.h>

#include <dragon/model/hydrus_like_robot_model.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <pluginlib/class_loader.h>
#include <std_msgs/Float64.h>

#include <boost/bind/bind.hpp>

#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace geo_robot_model
{
namespace
{
constexpr double kDefaultPublishRate = 40.0;
constexpr double kQuaternionNormTolerance = 1.0e-9;
constexpr char kRobotModelPluginName[] = "dragon/hydrus_like_robot_model";

bool finiteQuaternion(const geometry_msgs::Quaternion& quaternion)
{
  return std::isfinite(quaternion.x) && std::isfinite(quaternion.y) && std::isfinite(quaternion.z) &&
         std::isfinite(quaternion.w);
}

bool finitePosition(const geometry_msgs::Point& position)
{
  return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z);
}
}  // namespace

DragonStabilityEvaluator::DragonStabilityEvaluator()
{
  robot_model_loader_.reset(
      new pluginlib::ClassLoader<aerial_robot_model::RobotModel>("aerial_robot_model",
                                                                 "aerial_robot_model::RobotModel"));
  const auto loaded_model = robot_model_loader_->createInstance(kRobotModelPluginName);
  robot_model_ = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(loaded_model);
  if (!robot_model_)
  {
    throw std::runtime_error("Failed to load dragon/hydrus_like_robot_model");
  }

  link_joint_names_ = robot_model_->getLinkJointNames();
  const std::vector<int>& model_indices = robot_model_->getLinkJointIndices();
  if (link_joint_names_.empty() || link_joint_names_.size() != model_indices.size())
  {
    throw std::runtime_error("DRAGON robot model does not expose a valid link-joint mapping");
  }

  for (size_t index = 0; index < link_joint_names_.size(); ++index)
  {
    link_joint_indices_.emplace(link_joint_names_.at(index), model_indices.at(index));
  }
}

DragonStabilityEvaluator::~DragonStabilityEvaluator() = default;

bool DragonStabilityEvaluator::evaluate(const geometry_msgs::PoseStamped& root_pose,
                                        const sensor_msgs::JointState& joint_state, double& fc_rp_min)
{
  if (!finitePosition(root_pose.pose.position) || !finiteQuaternion(root_pose.pose.orientation) ||
      joint_state.name.size() != joint_state.position.size())
  {
    return false;
  }

  const geometry_msgs::Quaternion& quaternion = root_pose.pose.orientation;
  const double quaternion_norm =
      std::sqrt(quaternion.x * quaternion.x + quaternion.y * quaternion.y + quaternion.z * quaternion.z +
                quaternion.w * quaternion.w);
  if (!std::isfinite(quaternion_norm) || quaternion_norm < kQuaternionNormTolerance)
  {
    return false;
  }

  KDL::JntArray full_joints = robot_model_->getJointPositions();
  if (full_joints.rows() != robot_model_->getTree().getNrOfJoints())
  {
    full_joints.resize(robot_model_->getTree().getNrOfJoints());
    full_joints.data.setZero();
  }

  std::unordered_set<std::string> observed_link_joints;
  for (size_t index = 0; index < joint_state.name.size(); ++index)
  {
    const auto mapping = link_joint_indices_.find(joint_state.name.at(index));
    if (mapping == link_joint_indices_.end())
    {
      continue;
    }

    const double position = joint_state.position.at(index);
    if (!std::isfinite(position) || !observed_link_joints.insert(mapping->first).second)
    {
      return false;
    }
    if (mapping->second < 0 || mapping->second >= static_cast<int>(full_joints.rows()))
    {
      return false;
    }
    full_joints(mapping->second) = position;
  }

  if (observed_link_joints.size() != link_joint_names_.size())
  {
    return false;
  }

  const KDL::Rotation root_rotation =
      KDL::Rotation::Quaternion(quaternion.x / quaternion_norm, quaternion.y / quaternion_norm,
                                quaternion.z / quaternion_norm, quaternion.w / quaternion_norm);
  const KDL::Frame root_to_baselink =
      robot_model_->forwardKinematics<KDL::Frame>(robot_model_->getBaselinkName(), full_joints);
  robot_model_->setCogDesireOrientation(root_rotation * root_to_baselink.M);
  robot_model_->updateRobotModel(full_joints);

  const double evaluated_fc_rp_min = robot_model_->getFeasibleControlRollPitchMin();
  if (!std::isfinite(evaluated_fc_rp_min))
  {
    return false;
  }

  fc_rp_min = evaluated_fc_rp_min;
  return true;
}

DragonStabilityMonitor::DragonStabilityMonitor(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh)
  , pnh_(pnh)
  , root_pose_sub_(nh_, "root/pose", 10)
  , joint_state_sub_(nh_, "joint_states", 10)
  , synchronizer_(SyncPolicy(10), root_pose_sub_, joint_state_sub_)
  , latest_fc_rp_min_(0.0)
  , has_valid_metric_(false)
{
  double publish_rate = kDefaultPublishRate;
  pnh_.param("publish_rate", publish_rate, kDefaultPublishRate);
  if (!std::isfinite(publish_rate) || publish_rate <= 0.0)
  {
    ROS_WARN("Invalid stability publish_rate %.6f; using %.1f Hz", publish_rate, kDefaultPublishRate);
    publish_rate = kDefaultPublishRate;
  }

  fc_rp_min_pub_ = nh_.advertise<std_msgs::Float64>("stability/fc_rp_min", 1);
  synchronizer_.registerCallback(
      boost::bind(&DragonStabilityMonitor::synchronizedStateCallback, this, boost::placeholders::_1,
                  boost::placeholders::_2));
  publish_timer_ =
      nh_.createTimer(ros::Duration(1.0 / publish_rate), &DragonStabilityMonitor::publishTimerCallback, this);

  ROS_INFO("DRAGON stability monitor is ready and publishing stability/fc_rp_min at %.1f Hz", publish_rate);
}

void DragonStabilityMonitor::synchronizedStateCallback(
    const geometry_msgs::PoseStamped::ConstPtr& root_pose, const sensor_msgs::JointState::ConstPtr& joint_state)
{
  double fc_rp_min = 0.0;
  try
  {
    if (!evaluator_.evaluate(*root_pose, *joint_state, fc_rp_min))
    {
      has_valid_metric_ = false;
      ROS_WARN_THROTTLE(1.0, "Ignoring invalid synchronized DRAGON root pose and joint state");
      return;
    }
  }
  catch (const std::exception& error)
  {
    has_valid_metric_ = false;
    ROS_ERROR_THROTTLE(1.0, "Failed to evaluate DRAGON stability: %s", error.what());
    return;
  }

  latest_fc_rp_min_ = fc_rp_min;
  has_valid_metric_ = true;
}

void DragonStabilityMonitor::publishTimerCallback(const ros::TimerEvent&)
{
  if (!has_valid_metric_)
  {
    return;
  }

  std_msgs::Float64 message;
  message.data = latest_fc_rp_min_;
  fc_rp_min_pub_.publish(message);
}

}  // namespace geo_robot_model
