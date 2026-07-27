#include <aerial_robot_msgs/FullStateTarget.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace motion_primitive_planner
{
namespace
{
class WholeBodyNodeIntegration : public ::testing::Test
{
protected:
  void fullStateCallback(const aerial_robot_msgs::FullStateTarget::ConstPtr& message)
  {
    if (!commands_.empty() && commands_.back().joint_state.position.size() == message->joint_state.position.size())
    {
      for (size_t index = 0; index < message->joint_state.position.size(); ++index)
      {
        maximum_joint_step_ = std::max(
            maximum_joint_step_,
            std::abs(message->joint_state.position[index] - commands_.back().joint_state.position[index]));
      }
    }
    commands_.push_back(*message);
    last_command_wall_time_ = ros::WallTime::now();
    if (commands_.size() > 500)
    {
      commands_.erase(commands_.begin(), commands_.begin() + 100);
    }
    const double root_speed = std::sqrt(
        std::pow(message->root_state.twist.twist.linear.x, 2) +
        std::pow(message->root_state.twist.twist.linear.y, 2) +
        std::pow(message->root_state.twist.twist.linear.z, 2) +
        std::pow(message->root_state.twist.twist.angular.z, 2));
    bool stopped = root_speed < 1e-6;
    for (const double velocity : message->joint_state.velocity)
    {
      stopped = stopped && std::abs(velocity) < 1e-6;
    }
    last_command_stopped_ = stopped;
  }

  void rootTargetCallback(const geometry_msgs::PoseStamped::ConstPtr& message)
  {
    root_targets_.push_back(*message);
    if (root_targets_.size() > 500)
    {
      root_targets_.erase(root_targets_.begin(), root_targets_.begin() + 100);
    }
  }

  void minimumFcCallback(const std_msgs::Float64::ConstPtr& message)
  {
    if (std::isfinite(message->data))
    {
      selected_minimum_fc_rp_ = message->data;
      received_selection_ = true;
      ++selection_count_;
    }
  }

  static std::vector<std::string> fullStatePublishers()
  {
    XmlRpc::XmlRpcValue request;
    XmlRpc::XmlRpcValue response;
    XmlRpc::XmlRpcValue payload;
    request[0] = ros::this_node::getName();
    std::vector<std::string> result;
    if (!ros::master::execute("getSystemState", request, response, payload, false) || payload.size() < 1)
    {
      return result;
    }
    const XmlRpc::XmlRpcValue& publishers = payload[0];
    for (int index = 0; index < publishers.size(); ++index)
    {
      if (static_cast<std::string>(publishers[index][0]) != "/dragon/full_state_target")
      {
        continue;
      }
      for (int node = 0; node < publishers[index][1].size(); ++node)
      {
        result.push_back(static_cast<std::string>(publishers[index][1][node]));
      }
    }
    return result;
  }

  std::vector<aerial_robot_msgs::FullStateTarget> commands_;
  std::vector<geometry_msgs::PoseStamped> root_targets_;
  double maximum_joint_step_ = 0.0;
  double selected_minimum_fc_rp_ = 0.0;
  ros::WallTime last_command_wall_time_;
  int selection_count_ = 0;
  bool received_selection_ = false;
  bool last_command_stopped_ = false;
};

TEST_F(WholeBodyNodeIntegration, StopsAfterEachGoalAndAcceptsASecondGoal)
{
  ros::NodeHandle nh;
  const ros::Subscriber command_subscriber = nh.subscribe<aerial_robot_msgs::FullStateTarget>(
      "/dragon/full_state_target", 200,
      [this](const aerial_robot_msgs::FullStateTarget::ConstPtr& message) { fullStateCallback(message); });
  const ros::Subscriber root_target_subscriber = nh.subscribe<geometry_msgs::PoseStamped>(
      "/dragon/root/target_pose", 200,
      [this](const geometry_msgs::PoseStamped::ConstPtr& message) { rootTargetCallback(message); });
  const ros::Subscriber fc_subscriber = nh.subscribe<std_msgs::Float64>(
      "/dragon/selected_min_fc_rp", 10,
      [this](const std_msgs::Float64::ConstPtr& message) { minimumFcCallback(message); });
  const ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>("/dragon/root/flu_odom", 10);
  const ros::Subscriber root_tail_subscriber = nh.subscribe<geometry_msgs::PoseStamped>(
      "/dragon/root/tail_pose", 10,
      [odom_publisher](const geometry_msgs::PoseStamped::ConstPtr& root_tail) {
        nav_msgs::Odometry odometry;
        odometry.header = root_tail->header;
        odometry.child_frame_id = "dragon/root_tail";
        odometry.pose.pose.position = root_tail->pose.position;
        const geometry_msgs::Quaternion& link = root_tail->pose.orientation;
        odometry.pose.pose.orientation.x = link.y;
        odometry.pose.pose.orientation.y = -link.x;
        odometry.pose.pose.orientation.z = link.w;
        odometry.pose.pose.orientation.w = -link.z;
        odom_publisher.publish(odometry);
      });
  const ros::Publisher goal_publisher = nh.advertise<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 1, true);

  const auto make_goal = [](double x, double y) {
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "world";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 1.0;
    goal.pose.orientation.w = 1.0;
    return goal;
  };
  geometry_msgs::PoseStamped first_goal = make_goal(1.2, 0.0);
  geometry_msgs::PoseStamped second_goal = make_goal(1.8, 0.0);

  const ros::WallTime start = ros::WallTime::now();
  ros::WallRate rate(100.0);
  bool first_goal_published = false;
  bool first_goal_stopped = false;
  bool second_goal_published = false;
  bool second_goal_started = false;
  bool second_goal_stopped = false;
  size_t commands_after_first_goal = 0;
  int selections_after_first_goal = 0;
  while (ros::ok() && (ros::WallTime::now() - start).toSec() < 30.0)
  {
    const ros::WallTime now = ros::WallTime::now();
    if (!first_goal_published && (now - start).toSec() > 0.5)
    {
      first_goal.header.stamp = ros::Time::now();
      goal_publisher.publish(first_goal);
      first_goal_published = true;
    }
    ros::spinOnce();
    const bool output_silent = !last_command_wall_time_.isZero() &&
                               (now - last_command_wall_time_).toSec() >= 0.40;
    if (!first_goal_stopped && received_selection_ && commands_.size() >= 20 && output_silent)
    {
      first_goal_stopped = true;
      commands_after_first_goal = commands_.size();
      selections_after_first_goal = selection_count_;
    }
    if (first_goal_stopped && !second_goal_published)
    {
      second_goal.header.stamp = ros::Time::now();
      goal_publisher.publish(second_goal);
      second_goal_published = true;
    }
    if (second_goal_published && commands_.size() > commands_after_first_goal)
    {
      second_goal_started = true;
    }
    if (second_goal_started && selection_count_ > selections_after_first_goal && output_silent)
    {
      second_goal_stopped = true;
      break;
    }
    rate.sleep();
  }

  ASSERT_TRUE(first_goal_stopped);
  ASSERT_TRUE(second_goal_published);
  ASSERT_TRUE(second_goal_started);
  ASSERT_TRUE(second_goal_stopped);
  ASSERT_TRUE(received_selection_);
  ASSERT_GT(commands_.size(), commands_after_first_goal);
  EXPECT_GE(selected_minimum_fc_rp_ + 1e-4, 3.2);
  EXPECT_LE(maximum_joint_step_, 0.1001);
  const size_t rate_window = std::min<size_t>(40, commands_.size() - commands_after_first_goal - 1);
  ASSERT_GT(rate_window, 10u);
  const double elapsed = (commands_.back().header.stamp -
                          commands_[commands_.size() - 1 - rate_window].header.stamp).toSec();
  ASSERT_GT(elapsed, 0.0);
  EXPECT_NEAR(static_cast<double>(rate_window) / elapsed, 40.0, 0.5);

  const aerial_robot_msgs::FullStateTarget& terminal = commands_.back();
  EXPECT_EQ(terminal.header.frame_id, "world");
  EXPECT_EQ(terminal.root_state.child_frame_id, "root");
  EXPECT_EQ(terminal.joint_state.name.size(), 6u);
  EXPECT_EQ(terminal.joint_state.velocity.size(), 6u);
  EXPECT_TRUE(last_command_stopped_);
  ASSERT_EQ(root_targets_.size(), commands_.size());
  const geometry_msgs::PoseStamped& terminal_root_target = root_targets_.back();
  EXPECT_EQ(terminal_root_target.header.stamp, terminal.header.stamp);
  EXPECT_EQ(terminal_root_target.header.frame_id, terminal.header.frame_id);
  EXPECT_NEAR(terminal_root_target.pose.position.x, second_goal.pose.position.x, 0.02);
  EXPECT_NEAR(terminal_root_target.pose.position.y, second_goal.pose.position.y, 0.02);
  EXPECT_NEAR(terminal_root_target.pose.position.z, second_goal.pose.position.z, 0.02);
  const double terminal_yaw = std::atan2(
      2.0 * terminal.root_state.pose.pose.orientation.w * terminal.root_state.pose.pose.orientation.z,
      1.0 - 2.0 * std::pow(terminal.root_state.pose.pose.orientation.z, 2));
  const double terminal_tail_yaw = std::atan2(
      2.0 * terminal_root_target.pose.orientation.w * terminal_root_target.pose.orientation.z,
      1.0 - 2.0 * std::pow(terminal_root_target.pose.orientation.z, 2));
  EXPECT_NEAR(std::abs(std::remainder(terminal_yaw - terminal_tail_yaw, 2.0 * M_PI)), M_PI, 1e-6);
  EXPECT_NEAR(terminal.root_state.pose.pose.position.x + 0.5255 * std::cos(terminal_yaw),
              second_goal.pose.position.x, 0.02);
  EXPECT_NEAR(terminal.root_state.pose.pose.position.y + 0.5255 * std::sin(terminal_yaw),
              second_goal.pose.position.y, 0.02);

  const size_t stopped_command_count = commands_.size();
  const ros::WallTime silence_check_start = ros::WallTime::now();
  while (ros::ok() && (ros::WallTime::now() - silence_check_start).toSec() < 0.50)
  {
    ros::spinOnce();
    rate.sleep();
  }
  EXPECT_EQ(commands_.size(), stopped_command_count);

  const std::vector<std::string> publishers = fullStatePublishers();
  ASSERT_EQ(publishers.size(), 1u);
  EXPECT_EQ(publishers.front(), "/dragon/whole_body_motion_primitive_planner");
  ros::V_string nodes;
  ASSERT_TRUE(ros::master::getNodes(nodes));
  for (const std::string& node : nodes)
  {
    EXPECT_EQ(node.find("traj_server"), std::string::npos);
    EXPECT_EQ(node.find("copilot_planner"), std::string::npos);
  }
}
}  // namespace
}  // namespace motion_primitive_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "whole_body_node_integration_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
