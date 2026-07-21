#include <motion_primitive_planner/planner_core.h>
#include <motion_primitive_planner/joint_trajectory_planner.h>

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

TEST(PrimitiveGenerator, UsesTwoOffsetScalesForTheDefaultNineCandidates)
{
  const std::vector<Eigen::Vector3d> route = {
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(1.0, 0.0, 1.0), Eigen::Vector3d(2.0, 0.0, 1.0)};
  PrimitiveConfig config;
  config.candidate_count = 9;
  config.max_offset = 0.8;
  config.max_velocity = 0.5;
  config.cruise_velocity = 0.3;
  const PrimitiveGenerator generator(config);
  const std::vector<Candidate> candidates =
      generator.generate(route, endpointState(route.front()), endpointState(route.back()));
  ASSERT_EQ(candidates.size(), 9u);

  const Eigen::Vector3d nominal_mid =
      candidates[0].trajectory.getPos(0.5 * candidates[0].trajectory.getTotalDuration());
  const Eigen::Vector3d inner_mid =
      candidates[1].trajectory.getPos(0.5 * candidates[1].trajectory.getTotalDuration());
  const Eigen::Vector3d outer_mid =
      candidates[5].trajectory.getPos(0.5 * candidates[5].trajectory.getTotalDuration());
  const double inner_offset = (inner_mid - nominal_mid).norm();
  const double outer_offset = (outer_mid - nominal_mid).norm();
  EXPECT_GT(inner_offset, 0.1);
  EXPECT_GT(outer_offset, 1.5 * inner_offset);
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
        history, tail, 4, 1.0);
    return multilink_copilot::follow_the_leader::computeJointAngles(
        targets, tail, root_rotation, pitch, yaw, 6, 0.1, reference);
  };
  EXPECT_LT(joint_angles(straight).norm(), 1e-6);
  EXPECT_GT(joint_angles(curved).norm(), 0.2);
}

TEST(FollowTheLeaderGeometry, ShortHistoryContinuesAlongCurrentBodyMorphology)
{
  const std::vector<int> pitch = {0, 2, 4};
  const std::vector<int> yaw = {1, 3, 5};
  Eigen::VectorXd current_joints = Eigen::VectorXd::Zero(6);
  current_joints(yaw[0]) = M_PI_2;
  current_joints(yaw[1]) = M_PI_2;
  current_joints(yaw[2]) = M_PI_2;
  const Eigen::Vector3d root_tail = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d root_rotation = Eigen::Matrix3d::Identity();
  const std::deque<multilink_copilot::TrajectoryPoint> history = {{root_tail}};

  const auto combined = multilink_copilot::follow_the_leader::prependCurrentBodyMorphology(
      history, root_tail, root_rotation, current_joints, pitch, yaw, 4, 1.0);
  ASSERT_EQ(combined.size(), 4u);
  EXPECT_TRUE(combined[0].position.isApprox(Eigen::Vector3d(-1.0, 0.0, 0.0), 1e-12));
  EXPECT_TRUE(combined[1].position.isApprox(Eigen::Vector3d(-1.0, 1.0, 0.0), 1e-12));
  EXPECT_TRUE(combined[2].position.isApprox(Eigen::Vector3d(0.0, 1.0, 0.0), 1e-12));
  EXPECT_TRUE(combined[3].position.isApprox(root_tail, 1e-12));

  const auto targets = multilink_copilot::follow_the_leader::computeTargetPositions(
      combined, root_tail, 4, 1.0);
  ASSERT_EQ(targets.size(), 3u);
  EXPECT_TRUE(targets[0].isApprox(Eigen::Vector3d(0.0, 1.0, 0.0), 1e-12));
  EXPECT_TRUE(targets[1].isApprox(Eigen::Vector3d(-1.0, 1.0, 0.0), 1e-12));
  EXPECT_TRUE(targets[2].isApprox(Eigen::Vector3d(-1.0, 0.0, 0.0), 1e-12));

  const Eigen::VectorXd recovered = multilink_copilot::follow_the_leader::computeJointAngles(
      targets, root_tail, root_rotation, pitch, yaw, 6, 0.1, current_joints);
  EXPECT_TRUE(recovered.isApprox(current_joints, 1e-12));
}

TEST(FollowTheLeaderGeometry, PartialRootHistoryTransitionsIntoCurrentBodyMorphology)
{
  const std::vector<int> pitch = {0, 2, 4};
  const std::vector<int> yaw = {1, 3, 5};
  Eigen::VectorXd current_joints = Eigen::VectorXd::Zero(6);
  current_joints(yaw[0]) = M_PI_2;
  current_joints(yaw[1]) = M_PI_2;
  current_joints(yaw[2]) = M_PI_2;
  const Eigen::Vector3d initial_root_tail = Eigen::Vector3d::Zero();
  const std::deque<multilink_copilot::TrajectoryPoint> short_history = {
      {initial_root_tail}, {Eigen::Vector3d(0.6, 0.0, 0.0)}};

  const auto combined = multilink_copilot::follow_the_leader::prependCurrentBodyMorphology(
      short_history, initial_root_tail, Eigen::Matrix3d::Identity(), current_joints,
      pitch, yaw, 4, 1.0);
  const auto targets = multilink_copilot::follow_the_leader::computeTargetPositions(
      combined, short_history.back().position, 4, 1.0);
  ASSERT_EQ(targets.size(), 3u);
  EXPECT_TRUE(targets.front().isApprox(Eigen::Vector3d(0.0, 0.8, 0.0), 1e-12));
  EXPECT_NEAR((targets.front() - short_history.back().position).norm(), 1.0, 1e-12);
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

TEST(CandidateSelector, UsesCopilotProjectionOnlyWhenNoNominallyStableCandidateExists)
{
  std::vector<Candidate> candidates(3);
  candidates[0].status = CandidateStatus::kStabilityProjection;
  candidates[0].requires_stability_projection = true;
  candidates[0].min_fc_rp = 3.0;
  candidates[0].path_length = 1.0;
  candidates[1].status = CandidateStatus::kStabilityProjection;
  candidates[1].requires_stability_projection = true;
  candidates[1].min_fc_rp = 3.1;
  candidates[1].path_length = 2.0;
  candidates[2].status = CandidateStatus::kFeasible;
  candidates[2].min_fc_rp = 3.2;
  candidates[2].path_length = 3.0;

  EXPECT_EQ(selectBestCandidate(candidates, true), 2);

  candidates[2].status = CandidateStatus::kStability;
  EXPECT_EQ(selectBestCandidate(candidates), -1);
  EXPECT_EQ(selectBestCandidate(candidates, true), 0);
}

TEST(CandidateSelector, RanksProjectionFallbackByLengthJointMotionMarginAndJerk)
{
  std::vector<Candidate> candidates(4);
  for (Candidate& candidate : candidates)
  {
    candidate.status = CandidateStatus::kStabilityProjection;
    candidate.requires_stability_projection = true;
    candidate.min_fc_rp = 3.0;
  }
  candidates[0].min_fc_rp = 2.9;
  candidates[0].path_length = 2.0;
  candidates[1].path_length = 1.5;
  candidates[1].jerk_energy = 1.0;
  candidates[2].min_fc_rp = 2.8;
  candidates[2].path_length = 1.0;
  candidates[2].joint_motion = 2.0;
  candidates[2].jerk_energy = 2.0;
  candidates[3].path_length = 1.0;
  candidates[3].joint_motion = 1.0;
  candidates[3].jerk_energy = 1.0;

  EXPECT_EQ(selectBestCandidate(candidates, true), 3);

  candidates[3].requires_stability_projection = false;
  EXPECT_EQ(selectBestCandidate(candidates, true), 2);
}

TEST(WholeBodyCandidateSelector, PrioritizesClearanceThenDurationJointMotionAndJerk)
{
  std::vector<WholeBodyCandidateScore> candidates(6);
  candidates[0] = {false, 10.0, 1.0, 1.0, 1.0};
  candidates[1] = {true, 0.30, 1.0, 1.0, 1.0};
  candidates[2] = {true, 0.40, 5.0, 5.0, 5.0};
  candidates[3] = {true, 0.40, 4.0, 5.0, 5.0};
  candidates[4] = {true, 0.40, 4.0, 4.0, 5.0};
  candidates[5] = {true, 0.40, 4.0, 4.0, 3.0};
  EXPECT_EQ(selectBestWholeBodyCandidate(candidates), 5);

  candidates[1].minimum_clearance = std::numeric_limits<double>::infinity();
  candidates[2].minimum_clearance = std::numeric_limits<double>::infinity();
  EXPECT_EQ(selectBestWholeBodyCandidate(candidates), 1);
}

TEST(FullStateConversion, ConvertsFluTailPoseAndTwistToRootLinkOrigin)
{
  const RootCommandKinematics command = tailFluToRootLinkCommand(
      Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(0.4, -0.2, 0.1), 0.0, 0.5, 2.0);
  EXPECT_NEAR(command.yaw, M_PI, 1e-12);
  EXPECT_NEAR(command.yaw_rate, 0.5, 1e-12);
  EXPECT_TRUE(command.position.isApprox(Eigen::Vector3d(3.0, 2.0, 3.0), 1e-12));
  EXPECT_TRUE(command.linear_velocity.isApprox(Eigen::Vector3d(0.4, 0.8, 0.1), 1e-12));
}

TEST(JointPlanResult, InterpolatesSynchronizedPositionVelocityAndYaw)
{
  JointPlanResult result;
  result.success = true;
  result.duration = 2.0;
  result.joint_waypoints = {{0.0, Eigen::Vector2d(0.0, 0.0)},
                            {2.0, Eigen::Vector2d(2.0, -4.0)}};
  result.yaw_waypoints = {{0.0, -0.5}, {2.0, 0.5}};
  EXPECT_TRUE(result.jointPositions(1.0).isApprox(Eigen::Vector2d(1.0, -2.0), 1e-12));
  EXPECT_TRUE(result.jointVelocities(1.0).isApprox(Eigen::Vector2d(1.0, -2.0), 1e-12));
  EXPECT_NEAR(result.yaw(1.0), 0.0, 1e-12);
  EXPECT_NEAR(result.yawRate(1.0), 0.5, 1e-12);
  EXPECT_TRUE(result.jointVelocities(2.0).isZero(1e-12));
  EXPECT_NEAR(result.yawRate(2.0), 0.0, 1e-12);
}
}  // namespace
}  // namespace motion_primitive_planner

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
