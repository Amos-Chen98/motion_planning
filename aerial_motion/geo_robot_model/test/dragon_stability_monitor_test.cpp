#include <geo_robot_model/dragon_stability_monitor.h>

#include <dragon/model/hydrus_like_robot_model.h>
#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace geo_robot_model
{
namespace
{
constexpr double kHalfPi = 1.5707963267948966;

geometry_msgs::PoseStamped rootPose()
{
  geometry_msgs::PoseStamped message;
  message.header.frame_id = "world";
  message.pose.orientation.w = 1.0;
  return message;
}

sensor_msgs::JointState linkJointState(const std::vector<double>& positions)
{
  sensor_msgs::JointState message;
  message.name = {
    "joint1_pitch", "joint1_yaw", "joint2_pitch", "joint2_yaw", "joint3_pitch", "joint3_yaw",
  };
  message.position = positions;
  return message;
}

double directCanonicalMetric(const geometry_msgs::PoseStamped& root_pose, const sensor_msgs::JointState& joint_state)
{
  pluginlib::ClassLoader<aerial_robot_model::RobotModel> loader("aerial_robot_model",
                                                                "aerial_robot_model::RobotModel");
  const auto base_model = loader.createInstance("dragon/hydrus_like_robot_model");
  const auto model = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(base_model);
  if (!model)
  {
    throw std::runtime_error("Failed to load canonical DRAGON robot model");
  }

  KDL::JntArray full_joints = model->getJointPositions();
  if (full_joints.rows() != model->getTree().getNrOfJoints())
  {
    full_joints.resize(model->getTree().getNrOfJoints());
    full_joints.data.setZero();
  }
  const std::map<std::string, uint32_t>& joint_index_map = model->getJointIndexMap();
  for (size_t index = 0; index < joint_state.name.size(); ++index)
  {
    const auto mapping = joint_index_map.find(joint_state.name.at(index));
    if (mapping != joint_index_map.end())
    {
      full_joints(mapping->second) = joint_state.position.at(index);
    }
  }

  const geometry_msgs::Quaternion& quaternion = root_pose.pose.orientation;
  const double norm =
      std::sqrt(quaternion.x * quaternion.x + quaternion.y * quaternion.y + quaternion.z * quaternion.z +
                quaternion.w * quaternion.w);
  const KDL::Rotation root_rotation =
      KDL::Rotation::Quaternion(quaternion.x / norm, quaternion.y / norm, quaternion.z / norm, quaternion.w / norm);
  const KDL::Frame root_to_baselink =
      model->forwardKinematics<KDL::Frame>(model->getBaselinkName(), full_joints);
  model->setCogDesireOrientation(root_rotation * root_to_baselink.M);
  model->updateRobotModel(full_joints);
  return model->getFeasibleControlRollPitchMin();
}

TEST(DragonStabilityEvaluatorTest, MatchesCanonicalModelAndAcceptsShuffledNames)
{
  DragonStabilityEvaluator evaluator;
  const geometry_msgs::PoseStamped root_pose = rootPose();
  const sensor_msgs::JointState square = linkJointState({0.0, kHalfPi, 0.0, kHalfPi, 0.0, kHalfPi});

  double metric = 0.0;
  ASSERT_TRUE(evaluator.evaluate(root_pose, square, metric));
  EXPECT_NEAR(metric, directCanonicalMetric(root_pose, square), 1.0e-9);

  sensor_msgs::JointState shuffled = square;
  std::reverse(shuffled.name.begin(), shuffled.name.end());
  std::reverse(shuffled.position.begin(), shuffled.position.end());
  double shuffled_metric = 0.0;
  ASSERT_TRUE(evaluator.evaluate(root_pose, shuffled, shuffled_metric));
  EXPECT_NEAR(shuffled_metric, metric, 1.0e-9);
}

TEST(DragonStabilityEvaluatorTest, RejectsIncompleteOrNonFiniteState)
{
  DragonStabilityEvaluator evaluator;
  geometry_msgs::PoseStamped root_pose = rootPose();
  sensor_msgs::JointState joints = linkJointState({0.0, kHalfPi, 0.0, kHalfPi, 0.0, kHalfPi});
  double metric = 0.0;

  sensor_msgs::JointState incomplete = joints;
  incomplete.name.pop_back();
  incomplete.position.pop_back();
  EXPECT_FALSE(evaluator.evaluate(root_pose, incomplete, metric));

  sensor_msgs::JointState non_finite = joints;
  non_finite.position.front() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(evaluator.evaluate(root_pose, non_finite, metric));

  sensor_msgs::JointState duplicate = joints;
  duplicate.name.back() = duplicate.name.front();
  EXPECT_FALSE(evaluator.evaluate(root_pose, duplicate, metric));

  root_pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(evaluator.evaluate(root_pose, joints, metric));

  root_pose = rootPose();
  root_pose.pose.position.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(evaluator.evaluate(root_pose, joints, metric));
}

TEST(DragonStabilityEvaluatorTest, StraightConfigurationHasLowerMargin)
{
  DragonStabilityEvaluator evaluator;
  const geometry_msgs::PoseStamped root_pose = rootPose();
  const sensor_msgs::JointState square = linkJointState({0.0, kHalfPi, 0.0, kHalfPi, 0.0, kHalfPi});
  const sensor_msgs::JointState straight = linkJointState({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  double square_metric = 0.0;
  double straight_metric = 0.0;
  ASSERT_TRUE(evaluator.evaluate(root_pose, square, square_metric));
  ASSERT_TRUE(evaluator.evaluate(root_pose, straight, straight_metric));
  EXPECT_GT(square_metric, straight_metric + 1.0);
}

class DragonStabilityMonitorTopicTest : public testing::Test
{
protected:
  void SetUp() override
  {
    root_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("root/pose", 10);
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
    metric_sub_ =
        nh_.subscribe("stability/fc_rp_min", 100, &DragonStabilityMonitorTopicTest::metricCallback, this);
    ASSERT_TRUE(waitUntil([this]() {
      return root_pose_pub_.getNumSubscribers() > 0 && joint_state_pub_.getNumSubscribers() > 0 &&
             metric_sub_.getNumPublishers() > 0;
    }, 5.0));
  }

  bool waitUntil(const std::function<bool()>& predicate, double timeout_seconds)
  {
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(timeout_seconds);
    while (ros::ok() && ros::WallTime::now() < deadline)
    {
      if (predicate())
      {
        return true;
      }
      ros::WallDuration(0.01).sleep();
    }
    return predicate();
  }

  void metricCallback(const std_msgs::Float64::ConstPtr& message)
  {
    std::lock_guard<std::mutex> lock(metric_mutex_);
    metric_values_.push_back(message->data);
    metric_times_.push_back(ros::WallTime::now());
  }

  size_t metricCount()
  {
    std::lock_guard<std::mutex> lock(metric_mutex_);
    return metric_values_.size();
  }

  double latestMetric()
  {
    std::lock_guard<std::mutex> lock(metric_mutex_);
    return metric_values_.empty() ? std::numeric_limits<double>::quiet_NaN() : metric_values_.back();
  }

  void clearMetrics()
  {
    std::lock_guard<std::mutex> lock(metric_mutex_);
    metric_values_.clear();
    metric_times_.clear();
  }

  void publishFor(sensor_msgs::JointState joints, double duration_seconds)
  {
    geometry_msgs::PoseStamped root_pose = rootPose();
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(duration_seconds);
    ros::WallRate rate(100.0);
    while (ros::ok() && ros::WallTime::now() < deadline)
    {
      const ros::Time stamp = ros::Time::now();
      root_pose.header.stamp = stamp;
      joints.header.stamp = stamp;
      root_pose_pub_.publish(root_pose);
      joint_state_pub_.publish(joints);
      rate.sleep();
    }
  }

  double observedPublishRate()
  {
    std::lock_guard<std::mutex> lock(metric_mutex_);
    if (metric_times_.size() < 2)
    {
      return 0.0;
    }
    return static_cast<double>(metric_times_.size() - 1) / (metric_times_.back() - metric_times_.front()).toSec();
  }

  ros::NodeHandle nh_;
  ros::Publisher root_pose_pub_;
  ros::Publisher joint_state_pub_;
  ros::Subscriber metric_sub_;
  std::mutex metric_mutex_;
  std::vector<double> metric_values_;
  std::vector<ros::WallTime> metric_times_;
};

TEST_F(DragonStabilityMonitorTopicTest, WaitsForStateAndPublishesAtConfiguredRate)
{
  ros::WallDuration(0.2).sleep();
  EXPECT_EQ(metricCount(), 0u);

  clearMetrics();
  publishFor(linkJointState({0.0, kHalfPi, 0.0, kHalfPi, 0.0, kHalfPi}), 1.5);
  ASSERT_TRUE(waitUntil([this]() { return metricCount() >= 20; }, 1.0));
  EXPECT_NEAR(observedPublishRate(), 20.0, 3.0);
  const double square_metric = latestMetric();
  ASSERT_TRUE(std::isfinite(square_metric));

  publishFor(linkJointState({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), 0.5);
  ASSERT_TRUE(waitUntil([this, square_metric]() {
    return std::isfinite(latestMetric()) && latestMetric() < square_metric - 1.0;
  }, 1.0));
}

}  // namespace
}  // namespace geo_robot_model

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dragon_stability_monitor_test");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  spinner.stop();
  return result;
}
