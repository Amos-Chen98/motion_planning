#include <gcopter/PolyTraj.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float64.h>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace motion_primitive_planner
{
namespace
{
class RootNodeIntegration : public ::testing::Test
{
protected:
  void trajectoryCallback(const gcopter::PolyTraj::ConstPtr& message)
  {
    trajectories_.push_back(*message);
  }

  void commandCallback(const geometry_msgs::PoseStamped::ConstPtr& message)
  {
    commands_.push_back(*message);
  }

  void minimumFcCallback(const std_msgs::Float64::ConstPtr& message)
  {
    selected_minimum_fc_rp_ = message->data;
    received_selection_ = std::isfinite(message->data);
  }

  static sensor_msgs::PointCloud2 blockingCloud()
  {
    std::vector<Eigen::Vector3d> points;
    for (double x = -1.0; x <= 2.0; x += 0.10)
    {
      for (double y = -0.3; y <= 0.3; y += 0.15)
      {
        for (double z = 0.7; z <= 1.3; z += 0.15)
        {
          points.emplace_back(x, y, z);
        }
      }
    }
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = "world";
    cloud.header.stamp = ros::Time::now();
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const Eigen::Vector3d& point : points)
    {
      *x = static_cast<float>(point.x());
      *y = static_cast<float>(point.y());
      *z = static_cast<float>(point.z());
      ++x;
      ++y;
      ++z;
    }
    return cloud;
  }

  std::vector<gcopter::PolyTraj> trajectories_;
  std::vector<geometry_msgs::PoseStamped> commands_;
  double selected_minimum_fc_rp_ = 0.0;
  bool received_selection_ = false;
};

TEST_F(RootNodeIntegration, PublishesTrajectoryAndKeepsItWhenReplanningFails)
{
  ros::NodeHandle nh;
  const ros::Subscriber trajectory_subscriber = nh.subscribe<gcopter::PolyTraj>(
      "/planning/trajectory", 10,
      [this](const gcopter::PolyTraj::ConstPtr& message) { trajectoryCallback(message); });
  const ros::Subscriber command_subscriber = nh.subscribe<geometry_msgs::PoseStamped>(
      "/dragon/root/target_pose", 200,
      [this](const geometry_msgs::PoseStamped::ConstPtr& message) { commandCallback(message); });
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
  const ros::Publisher goal_publisher =
      nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);
  const ros::Publisher cloud_publisher =
      nh.advertise<sensor_msgs::PointCloud2>("/root_planner_test/cloud", 1, true);

  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "world";
  goal.pose.position.y = 0.3;
  goal.pose.position.z = 1.0;
  goal.pose.orientation.w = 1.0;

  ros::WallRate rate(100.0);
  const ros::WallTime start = ros::WallTime::now();
  bool goal_published = false;
  while (ros::ok() && trajectories_.empty() && (ros::WallTime::now() - start).toSec() < 15.0)
  {
    if (!goal_published && (ros::WallTime::now() - start).toSec() > 0.75)
    {
      goal.header.stamp = ros::Time::now();
      goal_publisher.publish(goal);
      goal_published = true;
    }
    ros::spinOnce();
    rate.sleep();
  }
  ASSERT_TRUE(goal_published);
  ASSERT_FALSE(trajectories_.empty());
  ASSERT_TRUE(received_selection_);
  EXPECT_GT(trajectories_.front().durations.size(), 0u);

  const size_t trajectory_count = trajectories_.size();
  const size_t command_count = commands_.size();
  sensor_msgs::PointCloud2 cloud = blockingCloud();
  cloud.header.stamp = ros::Time::now();
  cloud_publisher.publish(cloud);
  const ros::WallTime map_wait = ros::WallTime::now();
  while (ros::ok() && (ros::WallTime::now() - map_wait).toSec() < 0.5)
  {
    ros::spinOnce();
    rate.sleep();
  }

  goal.pose.position.y = -0.6;
  goal.header.stamp = ros::Time::now();
  goal_publisher.publish(goal);
  const ros::WallTime failure_wait = ros::WallTime::now();
  while (ros::ok() && (ros::WallTime::now() - failure_wait).toSec() < 1.5)
  {
    ros::spinOnce();
    rate.sleep();
  }
  EXPECT_EQ(trajectories_.size(), trajectory_count);
  EXPECT_GT(commands_.size(), command_count);
}
}  // namespace
}  // namespace motion_primitive_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "root_node_integration_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
