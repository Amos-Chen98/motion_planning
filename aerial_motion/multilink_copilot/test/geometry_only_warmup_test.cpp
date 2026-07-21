#include <aerial_robot_msgs/FullStateTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

namespace multilink_copilot
{
namespace
{

class GeometryOnlyWarmupTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    target_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("root/target_pose", 1);
    command_subscriber_ = nh_.subscribe("full_state_target", 10,
                                        &GeometryOnlyWarmupTest::commandCallback, this);
    ASSERT_TRUE(waitUntil([this]() { return target_publisher_.getNumSubscribers() > 0; }, 5.0));
  }

  geometry_msgs::PoseStamped target(double x, double y) const
  {
    geometry_msgs::PoseStamped message;
    message.header.frame_id = "world";
    message.pose.position.x = x;
    message.pose.position.y = y;
    message.pose.position.z = 1.0;
    message.pose.orientation.w = 1.0;
    return message;
  }

  void publishFor(geometry_msgs::PoseStamped message, double seconds)
  {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < deadline)
    {
      message.header.stamp = ros::Time::now();
      target_publisher_.publish(message);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  template <typename Predicate>
  bool waitUntil(Predicate predicate, double timeout_seconds)
  {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout_seconds);
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (predicate())
      {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return predicate();
  }

  size_t commandCount() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return command_count_;
  }

  aerial_robot_msgs::FullStateTarget latestCommand() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_command_;
  }

  void commandCallback(const aerial_robot_msgs::FullStateTarget::ConstPtr& message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_command_ = *message;
    ++command_count_;
  }

  ros::NodeHandle nh_;
  ros::Publisher target_publisher_;
  ros::Subscriber command_subscriber_;
  mutable std::mutex mutex_;
  aerial_robot_msgs::FullStateTarget latest_command_;
  size_t command_count_ = 0;
};

TEST_F(GeometryOnlyWarmupTest, UnpublishedNominalDoesNotReplaceMorphologyReference)
{
  publishFor(target(0.0, 0.0), 0.25);
  ASSERT_TRUE(waitUntil([this]() { return commandCount() >= 1; }, 2.0));
  const size_t initial_command_count = commandCount();
  const aerial_robot_msgs::FullStateTarget initial_command = latestCommand();
  ASSERT_EQ(initial_command.joint_state.position.size(), 6u);
  for (const double position : initial_command.joint_state.position)
  {
    EXPECT_NEAR(position, 0.0, 1e-6);
  }

  publishFor(target(0.15, 0.20), 0.25);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(commandCount(), initial_command_count);

  publishFor(target(0.60, 0.0), 0.25);
  ASSERT_TRUE(waitUntil([this, initial_command_count]() {
    return commandCount() > initial_command_count;
  }, 2.0));
  const aerial_robot_msgs::FullStateTarget next_command = latestCommand();
  ASSERT_EQ(next_command.joint_state.position.size(), 6u);
  for (const double position : next_command.joint_state.position)
  {
    EXPECT_NEAR(position, 0.0, 1e-6);
  }
}

}  // namespace
}  // namespace multilink_copilot

int main(int argc, char** argv)
{
  ros::init(argc, argv, "geometry_only_warmup_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  spinner.stop();
  return result;
}
