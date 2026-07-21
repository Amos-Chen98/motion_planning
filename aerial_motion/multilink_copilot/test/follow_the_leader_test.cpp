#include <multilink_copilot/follow_the_leader.h>

#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <vector>

namespace multilink_copilot
{
namespace
{

TEST(FollowTheLeaderWarmup, SinglePointHistoryReconstructsMeasuredMorphology)
{
  const std::vector<int> pitch_indices = {0, 2, 4};
  const std::vector<int> yaw_indices = {1, 3, 5};
  Eigen::VectorXd measured_joints = Eigen::VectorXd::Zero(6);
  measured_joints(yaw_indices[0]) = M_PI_2;
  measured_joints(yaw_indices[1]) = M_PI_2;
  measured_joints(yaw_indices[2]) = M_PI_2;
  const Eigen::Vector3d root_tail = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d root_rotation = Eigen::Matrix3d::Identity();
  const std::deque<TrajectoryPoint> history = {{root_tail}};

  const auto nominal_history = follow_the_leader::prependCurrentBodyMorphology(
      history, root_tail, root_rotation, measured_joints, pitch_indices, yaw_indices, 4, 1.0);
  const auto targets = follow_the_leader::computeTargetPositions(
      nominal_history, root_tail, 4, 1.0);
  ASSERT_EQ(targets.size(), 3u);

  const Eigen::VectorXd recovered_joints = follow_the_leader::computeJointAngles(
      targets, root_tail, root_rotation, pitch_indices, yaw_indices, 6, 0.1, measured_joints);
  EXPECT_TRUE(recovered_joints.isApprox(measured_joints, 1e-12));
}

TEST(FollowTheLeaderWarmup, PartialHistoryTransitionsIntoMeasuredMorphology)
{
  const std::vector<int> pitch_indices = {0, 2, 4};
  const std::vector<int> yaw_indices = {1, 3, 5};
  Eigen::VectorXd measured_joints = Eigen::VectorXd::Zero(6);
  measured_joints(yaw_indices[0]) = M_PI_2;
  measured_joints(yaw_indices[1]) = M_PI_2;
  measured_joints(yaw_indices[2]) = M_PI_2;
  const Eigen::Vector3d initial_root_tail = Eigen::Vector3d::Zero();
  const std::deque<TrajectoryPoint> short_history = {
      {initial_root_tail}, {Eigen::Vector3d(0.6, 0.0, 0.0)}};

  const auto nominal_history = follow_the_leader::prependCurrentBodyMorphology(
      short_history, initial_root_tail, Eigen::Matrix3d::Identity(), measured_joints,
      pitch_indices, yaw_indices, 4, 1.0);
  const auto targets = follow_the_leader::computeTargetPositions(
      nominal_history, short_history.back().position, 4, 1.0);
  ASSERT_EQ(targets.size(), 3u);
  EXPECT_TRUE(targets.front().isApprox(Eigen::Vector3d(0.0, 0.8, 0.0), 1e-12));
  EXPECT_NEAR((targets.front() - short_history.back().position).norm(), 1.0, 1e-12);
}

TEST(FollowTheLeaderWarmup, CompleteHistoryDoesNotUsePrependedMorphology)
{
  const std::vector<int> pitch_indices = {0, 2, 4};
  const std::vector<int> yaw_indices = {1, 3, 5};
  Eigen::VectorXd measured_joints = Eigen::VectorXd::Zero(6);
  measured_joints(yaw_indices[0]) = M_PI_2;
  measured_joints(yaw_indices[1]) = M_PI_2;
  measured_joints(yaw_indices[2]) = M_PI_2;
  std::deque<TrajectoryPoint> complete_history;
  for (int index = 0; index <= 30; ++index)
  {
    complete_history.push_back({Eigen::Vector3d(0.1 * index, 0.0, 0.0)});
  }
  const Eigen::Vector3d current_root_tail = complete_history.back().position;
  const Eigen::Matrix3d root_rotation = follow_the_leader::rotationAroundZ(M_PI);

  const auto nominal_history = follow_the_leader::prependCurrentBodyMorphology(
      complete_history, current_root_tail, root_rotation, measured_joints,
      pitch_indices, yaw_indices, 4, 1.0);
  const auto raw_targets = follow_the_leader::computeTargetPositions(
      complete_history, current_root_tail, 4, 1.0);
  const auto augmented_targets = follow_the_leader::computeTargetPositions(
      nominal_history, current_root_tail, 4, 1.0);

  ASSERT_EQ(raw_targets.size(), augmented_targets.size());
  for (size_t index = 0; index < raw_targets.size(); ++index)
  {
    EXPECT_TRUE(raw_targets[index].isApprox(augmented_targets[index], 1e-12));
  }
}

TEST(FollowTheLeaderWarmup, InvalidLinkConfigurationProducesNoTargets)
{
  const std::deque<TrajectoryPoint> history = {{Eigen::Vector3d::Zero()}};
  const Eigen::VectorXd measured_joints = Eigen::VectorXd::Zero(6);
  const auto nominal_history = follow_the_leader::prependCurrentBodyMorphology(
      history, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(), measured_joints,
      {0, 2, 4}, {1, 3, 5}, 1, 1.0);
  EXPECT_TRUE(follow_the_leader::computeTargetPositions(
      nominal_history, Eigen::Vector3d::Zero(), 1, 1.0).empty());
}

}  // namespace
}  // namespace multilink_copilot

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
