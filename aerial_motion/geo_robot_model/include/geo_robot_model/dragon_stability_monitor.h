#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <boost/shared_ptr.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace aerial_robot_model
{
class RobotModel;
}

namespace Dragon
{
class HydrusLikeRobotModel;
}

namespace pluginlib
{
template <class T>
class ClassLoader;
}

namespace geo_robot_model
{

class DragonStabilityEvaluator
{
public:
  DragonStabilityEvaluator();
  ~DragonStabilityEvaluator();

  bool evaluate(const geometry_msgs::PoseStamped& root_pose, const sensor_msgs::JointState& joint_state,
                double& fc_rp_min);

private:
  std::unique_ptr<pluginlib::ClassLoader<aerial_robot_model::RobotModel>> robot_model_loader_;
  boost::shared_ptr<Dragon::HydrusLikeRobotModel> robot_model_;
  std::unordered_map<std::string, int> link_joint_indices_;
  std::vector<std::string> link_joint_names_;
};

class DragonStabilityMonitor
{
public:
  DragonStabilityMonitor(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

private:
  using SyncPolicy =
      message_filters::sync_policies::ExactTime<geometry_msgs::PoseStamped, sensor_msgs::JointState>;

  void synchronizedStateCallback(const geometry_msgs::PoseStamped::ConstPtr& root_pose,
                                 const sensor_msgs::JointState::ConstPtr& joint_state);
  void publishTimerCallback(const ros::TimerEvent& event);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  DragonStabilityEvaluator evaluator_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> root_pose_sub_;
  message_filters::Subscriber<sensor_msgs::JointState> joint_state_sub_;
  message_filters::Synchronizer<SyncPolicy> synchronizer_;
  ros::Publisher fc_rp_min_pub_;
  ros::Timer publish_timer_;
  double latest_fc_rp_min_;
  bool has_valid_metric_;
};

}  // namespace geo_robot_model
