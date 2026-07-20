#include <motion_primitive_planner/planner_core.h>

#include <multilink_copilot/follow_the_leader.h>

#include <gtest/gtest.h>

#include <cmath>
#include <deque>

namespace motion_primitive_planner
{
namespace
{
Eigen::Matrix3d endpointState(const Eigen::Vector3d& position)
{
  Eigen::Matrix3d state = Eigen::Matrix3d::Zero();
  state.col(0) = position;
  return state;
}

TEST(PrimitiveGenerator, ProducesConfiguredDistinctCandidatesWithSharedBoundaryState)
{
  const std::vector<Eigen::Vector3d> route = {
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(1.0, 0.0, 1.0), Eigen::Vector3d(2.0, 0.0, 1.0)};
  for (const int count : {1, 2, 9})
  {
    PrimitiveConfig config;
    config.candidate_count = count;
    config.max_offset = 0.5;
    config.max_velocity = 0.5;
    config.cruise_velocity = 0.3;
    const PrimitiveGenerator generator(config);
    const std::vector<Candidate> candidates =
        generator.generate(route, endpointState(route.front()), endpointState(route.back()));
    ASSERT_EQ(candidates.size(), static_cast<size_t>(count));
    for (const Candidate& candidate : candidates)
    {
      ASSERT_GT(candidate.trajectory.getPieceNum(), 0);
      const double duration = candidate.trajectory.getTotalDuration();
      EXPECT_TRUE(candidate.trajectory.getPos(0.0).isApprox(route.front(), 1e-6));
      EXPECT_TRUE(candidate.trajectory.getPos(duration).isApprox(route.back(), 1e-6));
      EXPECT_LT(candidate.trajectory.getVel(0.0).norm(), 1e-6);
      EXPECT_LT(candidate.trajectory.getVel(duration).norm(), 1e-6);
      EXPECT_LT(candidate.trajectory.getAcc(0.0).norm(), 1e-6);
      EXPECT_LT(candidate.trajectory.getAcc(duration).norm(), 1e-6);
      EXPECT_LE(candidate.trajectory.getMaxVelRate(), config.max_velocity * (1.0 + 1e-6));
    }
    if (count > 1)
    {
      const Eigen::Vector3d nominal_mid = candidates.front().trajectory.getPos(
          0.5 * candidates.front().trajectory.getTotalDuration());
      const Eigen::Vector3d offset_mid = candidates[1].trajectory.getPos(
          0.5 * candidates[1].trajectory.getTotalDuration());
      EXPECT_GT((offset_mid - nominal_mid).norm(), 0.05);
    }
  }
}

TEST(PrimitiveGenerator, InsertsAnInteriorPointForShortRoutes)
{
  const std::vector<Eigen::Vector3d> route = {
      Eigen::Vector3d(0.0, 0.0, 0.5), Eigen::Vector3d(0.3, 0.0, 0.5)};
  const std::vector<Eigen::Vector3d> prepared = PrimitiveGenerator::prepareRoute(route);
  ASSERT_EQ(prepared.size(), 3u);
  EXPECT_TRUE(prepared[1].isApprox(0.5 * (route[0] + route[1])));

  PrimitiveConfig config;
  config.candidate_count = 3;
  config.max_velocity = 0.2;
  config.cruise_velocity = 0.15;
  const PrimitiveGenerator generator(config);
  const std::vector<Candidate> candidates =
      generator.generate(route, endpointState(route.front()), endpointState(route.back()));
  ASSERT_EQ(candidates.size(), 3u);
  for (const Candidate& candidate : candidates)
  {
    EXPECT_GT(candidate.trajectory.getPieceNum(), 0);
    EXPECT_LE(candidate.trajectory.getMaxVelRate(), config.max_velocity * (1.0 + 1e-6));
  }
}

TEST(WholeBodyCollision, DetectsTrailingLinkWhenRootTailIsClear)
{
  const Eigen::VectorXd joints = Eigen::VectorXd::Zero(6);
  const std::vector<int> pitch = {0, 2, 4};
  const std::vector<int> yaw = {1, 3, 5};
  const Eigen::Matrix3d root_rotation =
      multilink_copilot::follow_the_leader::rotationAroundZ(M_PI);
  const std::vector<Eigen::Vector3d> endpoints =
      linkEndpoints(Eigen::Vector3d::Zero(), root_rotation, joints, pitch, yaw, 4, 1.0);
  ASSERT_EQ(endpoints.size(), 5u);
  const auto occupied = [](const Eigen::Vector3d& point) {
    return (point - Eigen::Vector3d(-2.5, 0.0, 0.0)).norm() < 0.08;
  };
  EXPECT_FALSE(occupied(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(bodyCollides(endpoints, 0.05, occupied));
}

TEST(FollowTheLeaderGeometry, CurvedHistoryProducesCurvedNominalShape)
{
  std::deque<multilink_copilot::TrajectoryPoint> straight;
  std::deque<multilink_copilot::TrajectoryPoint> curved;
  for (int index = 0; index <= 40; ++index)
  {
    const double s = 0.1 * index;
    straight.push_back({Eigen::Vector3d(s, 0.0, 0.0)});
    const double angle = 0.25 * s;
    curved.push_back({Eigen::Vector3d(4.0 * std::sin(angle), 4.0 * (1.0 - std::cos(angle)), 0.0)});
  }
  const std::vector<int> pitch = {0, 2, 4};
  const std::vector<int> yaw = {1, 3, 5};
  const Eigen::Matrix3d root_rotation =
      multilink_copilot::follow_the_leader::rotationAroundZ(M_PI);
  const Eigen::VectorXd reference = Eigen::VectorXd::Zero(6);
  const auto joint_angles = [&](const std::deque<multilink_copilot::TrajectoryPoint>& history) {
    const Eigen::Vector3d tail = history.back().position;
    const auto targets = multilink_copilot::follow_the_leader::computeTargetPositions(
        history, tail, root_rotation.col(0), 4, 1.0, false);
    return multilink_copilot::follow_the_leader::computeJointAngles(
        targets, tail, root_rotation, pitch, yaw, 6, 0.1, reference);
  };
  EXPECT_LT(joint_angles(straight).norm(), 1e-6);
  EXPECT_GT(joint_angles(curved).norm(), 0.2);
}

TEST(CandidateSelector, RanksOnlyFeasibleCandidatesByLengthThenJerk)
{
  std::vector<Candidate> candidates(5);
  candidates[0].status = CandidateStatus::kCollision;
  candidates[0].path_length = 1.0;
  candidates[1].status = CandidateStatus::kStability;
  candidates[1].path_length = 0.5;
  candidates[2].status = CandidateStatus::kFeasible;
  candidates[2].path_length = 2.0;
  candidates[2].jerk_energy = 10.0;
  candidates[3].status = CandidateStatus::kFeasible;
  candidates[3].path_length = 1.5;
  candidates[3].jerk_energy = 20.0;
  candidates[4].status = CandidateStatus::kFeasible;
  candidates[4].path_length = 1.5;
  candidates[4].jerk_energy = 5.0;
  EXPECT_EQ(selectBestCandidate(candidates), 4);
  for (Candidate& candidate : candidates)
  {
    candidate.status = CandidateStatus::kCollision;
  }
  EXPECT_EQ(selectBestCandidate(candidates), -1);
}
}  // namespace
}  // namespace motion_primitive_planner

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
