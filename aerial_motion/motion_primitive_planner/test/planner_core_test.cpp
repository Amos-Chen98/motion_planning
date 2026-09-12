#include "test_octomap.h"
#include <motion_primitive_planner/octomap_message.h>
#include <octomap_msgs/conversions.h>
#include <motion_primitive_planner/whole_body_planner.h>
#include <motion_primitive_planner/whole_body_planner_node.h>
#include <motion_primitive_planner/joint_trajectory_planner.h>
#include <motion_primitive_planner/root_primitive_generator.h>

#include <multilink_copilot/follow_the_leader.h>

#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <limits>

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

Trajectory<5> linearTrajectory(const Eigen::Vector3d& start,
                               const Eigen::Vector3d& velocity,
                               double duration)
{
  Piece<5>::CoefficientMat coefficients = Piece<5>::CoefficientMat::Zero();
  coefficients.col(5) = start;
  coefficients.col(4) = velocity;
  return Trajectory<5>(
      std::vector<double>{duration},
      std::vector<Piece<5>::CoefficientMat>{coefficients});
}

SharedPlannerConfig validSharedConfig()
{
  SharedPlannerConfig config;
  config.common.worldFrameId = "world";
  config.common.dilateRadius = 0.0;
  config.common.voxelWidth = 0.1;
  config.common.mapBound = {-2.0, 2.0, -2.0, 2.0, 0.0, 3.0};
  config.common.timeoutRRT = 0.1;
  config.common.maxVelMag = 0.5;
  config.primitive.candidate_count = 3;
  config.primitive.max_offset = 0.4;
  config.primitive.max_velocity = 0.5;
  config.primitive.cruise_velocity = 0.3;
  config.primitive.minimum_piece_duration = 0.2;
  config.planning_horizon = 1.5;
  config.validateOrThrow();
  return config;
}

TEST(SharedPlannerConfig, RejectsInvalidSharedAndFollowerParameters)
{
  SharedPlannerConfig shared = validSharedConfig();
  shared.goal_tolerance = 0.0;
  EXPECT_THROW(shared.validateOrThrow(), std::invalid_argument);

  shared = validSharedConfig();
  shared.replan_trigger_ratio = 0.0;
  EXPECT_THROW(shared.validateOrThrow(), std::invalid_argument);
  shared.replan_trigger_ratio = 1.0;
  EXPECT_THROW(shared.validateOrThrow(), std::invalid_argument);
  shared.replan_trigger_ratio = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(shared.validateOrThrow(), std::invalid_argument);

  FollowerConfig follower;
  follower.command_hz = 0.0;
  EXPECT_THROW(follower.validateOrThrow(), std::invalid_argument);
  follower = FollowerConfig();
  follower.max_angular_vel = -0.1;
  EXPECT_THROW(follower.validateOrThrow(), std::invalid_argument);
}

TEST(TrajectoryReplanTrigger, FiresOnceAtConfiguredExecutionRatio)
{
  TrajectoryReplanTrigger trigger(0.5);
  trigger.arm(10.0, 4.0, false);
  EXPECT_TRUE(trigger.armed());
  EXPECT_FALSE(trigger.triggered());
  EXPECT_FALSE(trigger.shouldTrigger(11.999));
  EXPECT_TRUE(trigger.shouldTrigger(12.0));
  EXPECT_TRUE(trigger.triggered());
  EXPECT_FALSE(trigger.shouldTrigger(13.0));

  trigger.arm(20.0, 10.0, false);
  EXPECT_FALSE(trigger.shouldTrigger(24.999));
  EXPECT_TRUE(trigger.shouldTrigger(25.0));

  trigger.arm(30.0, 2.0, true);
  EXPECT_FALSE(trigger.armed());
  EXPECT_TRUE(trigger.terminal());
  EXPECT_FALSE(trigger.shouldTrigger(32.0));
}

TEST(TrajectoryReplanTrigger, RejectsInvalidRatiosAndIntervals)
{
  EXPECT_THROW(TrajectoryReplanTrigger(0.0), std::invalid_argument);
  EXPECT_THROW(TrajectoryReplanTrigger(1.0), std::invalid_argument);
  EXPECT_THROW(TrajectoryReplanTrigger(std::numeric_limits<double>::infinity()),
               std::invalid_argument);

  TrajectoryReplanTrigger trigger;
  EXPECT_THROW(trigger.arm(0.0, 0.0, false), std::invalid_argument);
  EXPECT_THROW(trigger.arm(std::numeric_limits<double>::quiet_NaN(), 1.0, false),
               std::invalid_argument);
}

TEST(PrimitiveGenerator, ProducesSinglePieceNominalAndOffsetCandidatesWithSharedBoundaryState)
{
  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d target(2.0, 0.0, 1.0);
  for (const int count : {1, 2, 9})
  {
    PrimitiveConfig config;
    config.candidate_count = count;
    config.max_offset = 0.5;
    config.max_velocity = 0.5;
    config.cruise_velocity = 0.3;
    const PrimitiveGenerator generator(config);
    const std::vector<Candidate> candidates =
        generator.generate(endpointState(start), endpointState(target));
    ASSERT_EQ(candidates.size(), static_cast<size_t>(count));
    EXPECT_EQ(candidates.front().trajectory.getPieceNum(), 1);
    for (const Candidate& candidate : candidates)
    {
      ASSERT_GT(candidate.trajectory.getPieceNum(), 0);
      const double duration = candidate.trajectory.getTotalDuration();
      EXPECT_TRUE(candidate.trajectory.getPos(0.0).isApprox(start, 1e-6));
      EXPECT_TRUE(candidate.trajectory.getPos(duration).isApprox(target, 1e-6));
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

TEST(PrimitiveGenerator, PreservesNonzeroEndpointPvaOnTheDirectCandidate)
{
  PrimitiveConfig config;
  config.candidate_count = 1;
  config.max_velocity = 0.5;
  config.cruise_velocity = 0.3;
  const PrimitiveGenerator generator(config);
  Eigen::Matrix3d initial_state = Eigen::Matrix3d::Zero();
  initial_state.col(0) = Eigen::Vector3d(0.0, 0.0, 1.0);
  initial_state.col(1) = Eigen::Vector3d(0.05, 0.04, 0.0);
  initial_state.col(2) = Eigen::Vector3d(0.01, -0.01, 0.0);
  Eigen::Matrix3d final_state = Eigen::Matrix3d::Zero();
  final_state.col(0) = Eigen::Vector3d(1.0, 0.0, 1.0);
  final_state.col(1) = Eigen::Vector3d(0.0, 0.10, 0.0);

  const Candidate candidate = generator.generate(initial_state, final_state).front();
  ASSERT_EQ(candidate.trajectory.getPieceNum(), 1);
  const double duration = candidate.trajectory.getTotalDuration();
  EXPECT_TRUE(candidate.trajectory.getPos(0.0).isApprox(initial_state.col(0), 1e-9));
  EXPECT_TRUE(candidate.trajectory.getVel(0.0).isApprox(initial_state.col(1), 1e-9));
  EXPECT_TRUE(candidate.trajectory.getAcc(0.0).isApprox(initial_state.col(2), 1e-9));
  EXPECT_TRUE(candidate.trajectory.getPos(duration).isApprox(final_state.col(0), 1e-9));
  EXPECT_TRUE(candidate.trajectory.getVel(duration).isApprox(final_state.col(1), 1e-9));
  EXPECT_LT(candidate.trajectory.getAcc(duration).norm(), 1e-6);
  EXPECT_GT(std::abs(candidate.trajectory.getPos(0.1 * duration).y()), 1e-4);
  EXPECT_LE(candidate.trajectory.getMaxVelRate(), config.max_velocity * (1.0 + 1e-6));
}

TEST(PrimitiveGenerator, LimitsOffsetsForShortChords)
{
  const Eigen::Vector3d start(0.0, 0.0, 0.5);
  const Eigen::Vector3d target(0.3, 0.0, 0.5);
  PrimitiveConfig config;
  config.candidate_count = 3;
  config.max_offset = 0.8;
  config.max_velocity = 0.2;
  config.cruise_velocity = 0.15;
  const std::vector<Candidate> candidates =
      PrimitiveGenerator(config).generate(endpointState(start), endpointState(target));
  ASSERT_EQ(candidates.size(), 3u);
  const Eigen::Vector3d nominal_mid =
      candidates[0].trajectory.getPos(0.5 * candidates[0].trajectory.getTotalDuration());
  for (size_t index = 1; index < candidates.size(); ++index)
  {
    ASSERT_EQ(candidates[index].trajectory.getPieceNum(), 2);
    const Eigen::Vector3d offset_mid =
        candidates[index].trajectory.getPos(0.5 * candidates[index].trajectory.getTotalDuration());
    EXPECT_NEAR((offset_mid - nominal_mid).norm(), 0.12, 1e-6);
  }
}

TEST(PrimitiveGenerator, UsesTwoOffsetScalesForTheDefaultNineCandidates)
{
  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d target(2.0, 0.0, 1.0);
  PrimitiveConfig config;
  config.candidate_count = 9;
  config.max_offset = 0.8;
  config.max_velocity = 0.5;
  config.cruise_velocity = 0.3;
  const PrimitiveGenerator generator(config);
  const std::vector<Candidate> candidates =
      generator.generate(endpointState(start), endpointState(target));
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

TEST(PrimitiveGenerator, UsesDeterministicOffsetsForVerticalChords)
{
  const Eigen::Vector3d start(0.0, 0.0, 0.5);
  const Eigen::Vector3d target(0.0, 0.0, 1.5);
  PrimitiveConfig config;
  config.candidate_count = 2;
  config.max_offset = 0.2;
  config.max_velocity = 0.5;
  config.cruise_velocity = 0.3;
  const std::vector<Candidate> candidates =
      PrimitiveGenerator(config).generate(endpointState(start), endpointState(target));
  ASSERT_EQ(candidates.size(), 2u);
  ASSERT_EQ(candidates[1].trajectory.getPieceNum(), 2);
  const Eigen::Vector3d offset_mid =
      candidates[1].trajectory.getPos(0.5 * candidates[1].trajectory.getTotalDuration());
  EXPECT_NEAR(offset_mid.x(), 0.0, 1e-9);
  EXPECT_NEAR(offset_mid.y(), -config.max_offset, 1e-9);
  EXPECT_NEAR(offset_mid.z(), 1.0, 1e-9);
}

TEST(PrimitiveGenerator, RejectsNonfiniteStatesAndNearZeroChords)
{
  PrimitiveConfig config;
  config.candidate_count = 2;
  config.max_velocity = 0.5;
  config.cruise_velocity = 0.3;
  const PrimitiveGenerator generator(config);

  Eigen::Matrix3d invalid = endpointState(Eigen::Vector3d::Zero());
  invalid(0, 1) = std::numeric_limits<double>::quiet_NaN();
  for (const Candidate& candidate :
       generator.generate(invalid, endpointState(Eigen::Vector3d::UnitX())))
  {
    EXPECT_EQ(candidate.status, CandidateStatus::kGenerationFailed);
  }
  for (const Candidate& candidate :
       generator.generate(endpointState(Eigen::Vector3d::Zero()),
                          endpointState(Eigen::Vector3d(1e-4, 0.0, 0.0))))
  {
    EXPECT_EQ(candidate.status, CandidateStatus::kGenerationFailed);
  }
}

TEST(WholeBodyCollision, UsesTheShortestYawDeltaAcrossTheWrapBoundary)
{
  EXPECT_NEAR(shortestYawDelta(M_PI - 0.1, -M_PI + 0.1), 0.2, 1e-12);
  EXPECT_NEAR(shortestYawDelta(-M_PI + 0.1, M_PI - 0.1), -0.2, 1e-12);
}

TEST(RootAttitudePredictor, UsesUniformFixedTimeSamplesIncludingEndpoints)
{
  FollowerConfig follower;
  const auto samples_for = [&](double duration, double sample_dt) {
    return predictRootAttitudes(
        linearTrajectory(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), duration),
        RootAttitude{}, follower, sample_dt, true);
  };
  const auto divisible = samples_for(1.0, 0.25);
  ASSERT_EQ(divisible.size(), 5u);
  for (size_t index = 0; index < divisible.size(); ++index)
    EXPECT_NEAR(divisible[index].time, 0.25 * index, 1e-12);
  const auto non_divisible = samples_for(1.0, 0.30);
  ASSERT_EQ(non_divisible.size(), 5u);
  EXPECT_DOUBLE_EQ(non_divisible.front().time, 0.0);
  EXPECT_DOUBLE_EQ(non_divisible.back().time, 1.0);
  const double interval = non_divisible[1].time - non_divisible[0].time;
  EXPECT_LE(interval, 0.30);
  for (size_t index = 2; index < non_divisible.size(); ++index)
    EXPECT_NEAR(non_divisible[index].time - non_divisible[index - 1].time, interval, 1e-12);
  const auto zero_duration = samples_for(0.0, 0.25);
  ASSERT_EQ(zero_duration.size(), 1u);
  EXPECT_DOUBLE_EQ(zero_duration.front().time, 0.0);
}

TEST(RootAttitudePredictor, PreservesRateLimitsOffsetsSwitchesAndZeroSpeedTerminal)
{
  FollowerConfig follower;
  follower.max_angular_vel = 1.0;
  const RootAttitude start{0.2, 0.1};
  Piece<5>::CoefficientMat coefficients = Piece<5>::CoefficientMat::Zero();
  coefficients(1, 4) = 2.0;
  coefficients(1, 3) = -1.0;
  coefficients(2, 4) = 2.0;
  coefficients(2, 3) = -1.0;
  const Trajectory<5> root({1.0}, {coefficients});
  const auto samples = predictRootAttitudes(root, start, follower, 0.1, true, 0.5, 2.0);
  ASSERT_EQ(samples.size(), 6u);
  EXPECT_DOUBLE_EQ(samples.front().time, 2.0);
  EXPECT_DOUBLE_EQ(samples.back().time, 2.5);
  EXPECT_DOUBLE_EQ(samples.front().attitude.yaw, start.yaw);
  EXPECT_DOUBLE_EQ(samples.front().attitude.pitch, start.pitch);
  EXPECT_NEAR(samples[1].attitude.yaw, 0.3, 1e-12);
  EXPECT_NEAR(samples[1].attitude.pitch, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(samples.back().attitude.yaw, samples[4].attitude.yaw);
  EXPECT_DOUBLE_EQ(samples.back().attitude.pitch, samples[4].attitude.pitch);
  for (size_t index = 1; index < samples.size(); ++index)
  {
    const double dt = samples[index].time - samples[index - 1].time;
    EXPECT_LE(std::abs(shortestYawDelta(samples[index - 1].attitude.yaw,
                                       samples[index].attitude.yaw)), dt + 1e-12);
    EXPECT_LE(std::abs(samples[index].attitude.pitch - samples[index - 1].attitude.pitch), dt + 1e-12);
  }
  follower.publish_yaw_command = false;
  const auto disabled = predictRootAttitudes(root, start, follower, 0.1, false);
  ASSERT_FALSE(disabled.empty());
  EXPECT_DOUBLE_EQ(disabled.back().attitude.yaw, start.yaw);
  EXPECT_DOUBLE_EQ(disabled.back().attitude.pitch, start.pitch);
  EXPECT_TRUE(predictRootAttitudes(root, start, follower, 0.1, true, 0.0, 0.0,
                                   std::chrono::steady_clock::now()).empty());
}

class TerminalJointTarget : public ::testing::Test
{
protected:
  DragonKinematicGeometry geometry{4, 1.0, {0, 2, 4}, {1, 3, 5}};
  WholeBodyConfiguration aligned{Eigen::Vector3d::Zero(), linkRotation(RootAttitude{}),
                                  Eigen::VectorXd::Zero(6)};

  void useTwoLinks()
  {
    geometry = {2, 1.0, {0}, {1}};
    aligned.joint_positions = Eigen::VectorXd::Zero(2);
  }

  TerminalJointTargetResult generate(const Trajectory<5>& root, double start_time = 0.0,
                                     RootAttitude terminal = RootAttitude{}) const
  {
    return computeTerminalJointTarget(root, start_time, terminal, aligned, geometry, 0.1);
  }

  void expectLinkLengthsAndKinematics(const TerminalJointTargetResult& result,
                                      const Eigen::Vector3d& root_tail,
                                      RootAttitude terminal = RootAttitude{}) const
  {
    ASSERT_TRUE(result.success) << result.detail;
    ASSERT_EQ(result.target_positions.size(), static_cast<size_t>(geometry.link_num - 1));
    const auto endpoints = linkEndpoints(root_tail, linkRotation(terminal), result.joints,
        geometry.pitch_joint_indices, geometry.yaw_joint_indices, geometry.link_num, geometry.link_length);
    Eigen::Vector3d head = root_tail;
    for (size_t index = 0; index < result.target_positions.size(); ++index)
    {
      EXPECT_NEAR((result.target_positions[index] - head).norm(), geometry.link_length, 1.1e-6);
      EXPECT_LE((endpoints[index + 2] - result.target_positions[index]).norm(), 5e-6);
      head = result.target_positions[index];
    }
  }
};

TEST_F(TerminalJointTarget, StraightCurveProducesOrderedTailTargets)
{
  const auto root = linearTrajectory(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 4.0);
  const auto result = generate(root);
  ASSERT_TRUE(result.success) << result.detail;
  expectLinkLengthsAndKinematics(result, root.getPos(4.0));
  for (int index = 0; index < 3; ++index)
    EXPECT_TRUE(result.target_positions[index].isApprox(Eigen::Vector3d(3.0 - index, 0.0, 0.0), 4e-6));
  EXPECT_LT(result.joints.norm(), 1e-6);
}

TEST_F(TerminalJointTarget, CurvedMincoTargetsLieOnTheAnalyticCurve)
{
  Piece<5>::CoefficientMat coefficients = Piece<5>::CoefficientMat::Zero();
  coefficients(0, 4) = 1.0;
  coefficients(1, 3) = 0.2;
  const Trajectory<5> root({4.0}, {coefficients});
  const auto result = generate(root);
  ASSERT_TRUE(result.success) << result.detail;
  expectLinkLengthsAndKinematics(result, root.getPos(4.0));
  double previous_x = 4.0;
  for (const auto& position : result.target_positions)
  {
    EXPECT_LT(position.x(), previous_x);
    EXPECT_GT(position.x(), 0.0);
    EXPECT_NEAR(position.y(), 0.2 * position.x() * position.x(), 1e-10);
    previous_x = position.x();
  }
}

TEST_F(TerminalJointTarget, IsInvariantToPolynomialSplittingAndTimeScaling)
{
  Piece<5>::CoefficientMat coefficients = Piece<5>::CoefficientMat::Zero();
  coefficients(0, 4) = 1.0;
  coefficients(1, 3) = 0.2;
  const Trajectory<5> whole({4.0}, {coefficients});
  auto shifted = coefficients;
  shifted(0, 5) = 1.2;
  shifted(1, 4) = 0.48;
  shifted(1, 5) = 0.288;
  const Trajectory<5> split({1.2, 2.8}, {coefficients, shifted});
  auto slowed = coefficients;
  slowed(0, 4) *= 0.5;
  slowed(1, 3) *= 0.25;
  const Trajectory<5> rescaled({8.0}, {slowed});
  const auto expected = generate(whole);
  ASSERT_TRUE(expected.success) << expected.detail;
  for (const auto& root : {split, rescaled})
  {
    const auto result = generate(root);
    ASSERT_TRUE(result.success) << result.detail;
    EXPECT_TRUE(result.joints.isApprox(expected.joints, 5e-6));
    for (int index = 0; index < 3; ++index)
      EXPECT_TRUE(result.target_positions[index].isApprox(expected.target_positions[index], 5e-6));
  }
}

TEST_F(TerminalJointTarget, HandlesPieceAndBodyJunctionIntersections)
{
  auto first = linearTrajectory(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 1.0);
  auto second = linearTrajectory(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX(), 1.0);
  const Trajectory<5> root({1.0, 1.0}, {first[0].getCoeffMat(), second[0].getCoeffMat()});
  const auto result = generate(root);
  ASSERT_TRUE(result.success) << result.detail;
  expectLinkLengthsAndKinematics(result, root.getPos(2.0));
  EXPECT_NEAR(result.target_positions[0].x(), 1.0, 1e-6);
  EXPECT_NEAR(result.target_positions[1].norm(), 0.0, 2e-6);
  EXPECT_NEAR(result.target_positions[2].x(), -1.0, 3e-6);
}

TEST_F(TerminalJointTarget, ShortCurveUsesAlignedBodyMorphology)
{
  useTwoLinks();
  const auto root = linearTrajectory(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 0.2);
  const auto straight = generate(root);
  ASSERT_TRUE(straight.success) << straight.detail;
  EXPECT_NEAR(straight.target_positions[0].x(), -0.8, 1e-6);
  aligned.joint_positions(1) = M_PI_2;
  const auto bent = generate(root);
  ASSERT_TRUE(bent.success) << bent.detail;
  EXPECT_NEAR(bent.target_positions[0].x(), 0.0, 1e-6);
  EXPECT_NEAR(bent.target_positions[0].y(), -std::sqrt(0.96), 1e-6);
  expectLinkLengthsAndKinematics(bent, root.getPos(0.2));
}

TEST_F(TerminalJointTarget, MovingHandoverExcludesRootPrefix)
{
  useTwoLinks();
  const auto root = linearTrajectory(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 2.0);
  aligned.link1_tail = root.getPos(1.5);
  aligned.root_link_rotation = Eigen::Matrix3d::Identity();
  aligned.joint_positions(1) = M_PI_2;
  const auto result = generate(root, 1.5);
  ASSERT_TRUE(result.success) << result.detail;
  EXPECT_NEAR(result.target_positions[0].x(), 1.5, 1e-6);
  EXPECT_NEAR(result.target_positions[0].y(), std::sqrt(0.75), 1e-6);
  expectLinkLengthsAndKinematics(result, root.getPos(2.0));
}

TEST_F(TerminalJointTarget, FindsTangencyWithoutASignChange)
{
  useTwoLinks();
  Piece<5>::CoefficientMat coefficients = Piece<5>::CoefficientMat::Zero();
  coefficients(0, 4) = 4.0;
  coefficients(0, 3) = -4.0;
  const auto result = generate(Trajectory<5>({1.0}, {coefficients}));
  ASSERT_TRUE(result.success) << result.detail;
  EXPECT_TRUE(result.target_positions[0].isApprox(Eigen::Vector3d::UnitX(), 1e-6));
}

TEST_F(TerminalJointTarget, QuinticIntersectionsMatchAnIndependentBackwardScan)
{
  useTwoLinks();
  // Deterministic, spatially curved degree-five cases exercise the full
  // degree-ten distance equation, independently of the stationary-root solver.
  for (int example = 0; example < 24; ++example)
  {
    Piece<5>::CoefficientMat coefficients = Piece<5>::CoefficientMat::Zero();
    for (int power = 1; power <= 5; ++power)
    {
      coefficients(0, 5 - power) = 0.8 + 0.3 * std::cos(example + 2.0 * power);
      coefficients(1, 5 - power) = std::sin(0.7 * example + power);
      coefficients(2, 5 - power) = 0.5 * std::cos(0.3 * example - power);
    }
    const Trajectory<5> root({1.0}, {coefficients});
    const auto result = generate(root);
    ASSERT_TRUE(result.success) << example << ": " << result.detail;
    const Eigen::Vector3d head = root.getPos(1.0);
    double right = 1.0;
    double left = right;
    for (int sample = 1; sample <= 10000; ++sample)
    {
      left = 1.0 - sample / 10000.0;
      if ((root.getPos(left) - head).norm() >= geometry.link_length) break;
      right = left;
    }
    ASSERT_LT(left, right);
    for (int iteration = 0; iteration < 50; ++iteration)
    {
      const double middle = (left + right) / 2.0;
      if ((root.getPos(middle) - head).norm() >= geometry.link_length) left = middle;
      else right = middle;
    }
    EXPECT_LE((result.target_positions[0] - root.getPos((left + right) / 2.0)).norm(), 2e-6);
  }
}

TEST_F(TerminalJointTarget, ChoosesLatestOfMultipleCurveIntersections)
{
  useTwoLinks();
  Piece<5>::CoefficientMat coefficients = Piece<5>::CoefficientMat::Zero();
  coefficients(0, 4) = 8.0;
  coefficients(0, 3) = -8.0;
  coefficients(1, 4) = -8.0;
  coefficients(1, 3) = 24.0;
  coefficients(1, 2) = -16.0;
  const auto result = generate(Trajectory<5>({1.0}, {coefficients}));
  ASSERT_TRUE(result.success) << result.detail;
  EXPECT_NEAR(result.target_positions[0].norm(), 1.0, 1e-6);
  EXPECT_GT(result.target_positions[0].x(), 0.0);
  EXPECT_GT(result.target_positions[0].y(), 0.0);
}

TEST_F(TerminalJointTarget, NoIntersectionUsesClosestDistanceIncludingStationaryPoints)
{
  useTwoLinks();
  aligned.root_link_rotation = Eigen::AngleAxisd(std::atan2(0.6, 0.8), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Piece<5>::CoefficientMat coefficients = Piece<5>::CoefficientMat::Zero();
  coefficients(0, 4) = 0.8;
  coefficients(1, 4) = 2.8;
  coefficients(1, 3) = -2.8;
  const Trajectory<5> root({1.0}, {coefficients});
  const auto result = generate(root);
  ASSERT_TRUE(result.success) << result.detail;
  const double chosen_distance = (result.target_positions[0] - root.getPos(1.0)).norm();
  EXPECT_GT(chosen_distance, 0.8);
  EXPECT_LT(chosen_distance, 1.0);
  EXPECT_GT(result.target_positions[0].y(), 0.1);
  // Independent dense oracle for the closest residual when the whole trace
  // lies inside the sphere. The body segment's maximum distance is 0.8 m.
  double greatest_distance = 0.8;
  for (int sample = 0; sample <= 10000; ++sample)
    greatest_distance = std::max(greatest_distance,
        (root.getPos(sample / 10000.0) - root.getPos(1.0)).norm());
  EXPECT_NEAR(chosen_distance, greatest_distance, 1e-6);
}

TEST_F(TerminalJointTarget, SingularPitchUsesAlignedJointsWithoutTemporalPrediction)
{
  useTwoLinks();
  const auto root = linearTrajectory(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1.0);
  aligned.joint_positions << 0.37, M_PI_2;
  const auto first = generate(root);
  ASSERT_TRUE(first.success) << first.detail;
  EXPECT_DOUBLE_EQ(first.joints(0), 0.37);
  aligned.joint_positions(0) = -0.21;
  const auto second = generate(root);
  ASSERT_TRUE(second.success) << second.detail;
  EXPECT_DOUBLE_EQ(second.joints(0), -0.21);
  EXPECT_TRUE(first.target_positions[0].isApprox(second.target_positions[0], 1e-6));
}

TEST_F(TerminalJointTarget, RejectsMalformedInputsAndExpiredBudget)
{
  const auto root = linearTrajectory(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 4.0);
  EXPECT_FALSE(generate(root, 4.1).success);
  EXPECT_FALSE(generate(root, 0.0, RootAttitude{NAN, 0.0}).success);
  EXPECT_FALSE(computeTerminalJointTarget(root, 0.0, {}, aligned, geometry, 0.1,
                                          std::chrono::steady_clock::now()).success);
  auto coefficients = root[0].getCoeffMat();
  coefficients(0, 0) = NAN;
  EXPECT_FALSE(generate(Trajectory<5>({4.0}, {coefficients})).success);
  auto shifted = root[0].getCoeffMat();
  shifted(0, 5) = 9.0;
  EXPECT_FALSE(generate(Trajectory<5>({1.0, 1.0}, {root[0].getCoeffMat(), shifted})).success);
  aligned.link1_tail.x() = 0.1;
  EXPECT_FALSE(generate(root).success);
  aligned.link1_tail.setZero();
  geometry.yaw_joint_indices[0] = geometry.pitch_joint_indices[0];
  EXPECT_FALSE(generate(root).success);
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

TEST(WholeBodyCandidateSelector, BalancesDurationAndJointMotionThenUsesDeterministicTies)
{
  std::vector<WholeBodyCandidateScore> candidates(6);
  candidates[0] = {false, 0.5, 1.0, 1.0};
  candidates[1] = {true, 3.0, 1.0, 1.0};
  candidates[2] = {true, 2.0, 5.0, 5.0};
  candidates[3] = {true, 2.0, 4.0, 5.0};
  candidates[4] = {true, 2.0, 4.0, 3.0};
  candidates[5] = {true, 1.5, 10.0, 10.0};
  EXPECT_EQ(selectBestWholeBodyCandidate(candidates, 0.25), 4);

  candidates[4].joint_motion = 5.0;
  EXPECT_EQ(selectBestWholeBodyCandidate(candidates, 0.25), 3);

  candidates[3].joint_motion = 5.0;
  EXPECT_EQ(selectBestWholeBodyCandidate(candidates, 0.25), 1);

  for (WholeBodyCandidateScore& candidate : candidates)
  {
    candidate.feasible = false;
  }
  EXPECT_EQ(selectBestWholeBodyCandidate(candidates), -1);
}

TEST(WholeBodyCandidateSelector, RejectsTheBagDerivedRedundantFold)
{
  std::vector<WholeBodyCandidateScore> candidates(2);
  candidates[0] = {true, 5.051, 8.508, 1.0};
  candidates[1] = {true, 5.200, 0.500, 2.0};

  EXPECT_EQ(selectBestWholeBodyCandidate(candidates, 0.25), 1);
  EXPECT_EQ(selectBestWholeBodyCandidate(candidates, 0.0), 0);
  EXPECT_EQ(selectBestWholeBodyCandidate(candidates, -1.0), -1);
}

TEST(WholeBodyCandidateSelector, ChargesWorkspaceTrackingError)
{
  std::vector<WholeBodyCandidateScore> candidates(2);
  candidates[0] = {true, 5.921, 4.410, 1.0, 0.605};
  candidates[1] = {true, 5.921, 4.642, 2.0, 0.432};

  EXPECT_EQ(selectBestWholeBodyCandidate(candidates, 0.25, 0.0), 0);
  EXPECT_EQ(selectBestWholeBodyCandidate(candidates, 0.25, 6.0), 1);
  EXPECT_EQ(selectBestWholeBodyCandidate(candidates, 0.25, -1.0), -1);
}

TEST(FullStateConversion, ConvertsFluTailPoseAndTwistToRootLinkOrigin)
{
  const RootAttitude attitude{0.0, -M_PI / 6.0};
  const Eigen::Matrix3d tail_rotation = fluRotation(attitude);
  const Eigen::Vector3d angular_velocity = worldAngularVelocity(attitude, 0.5, -0.2);
  const RootCommandKinematics command = tailFluToRootLinkCommand(
      Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(0.4, -0.2, 0.1),
      tail_rotation, angular_velocity, 2.0);
  const Eigen::Matrix3d link_rotation = linkRotation(attitude);
  const Eigen::Vector3d link_direction = link_rotation.col(0);
  EXPECT_TRUE(command.orientation.toRotationMatrix().isApprox(link_rotation, 1e-12));
  EXPECT_TRUE(command.angular_velocity.isApprox(angular_velocity, 1e-12));
  EXPECT_TRUE(command.position.isApprox(
      Eigen::Vector3d(1.0, 2.0, 3.0) - 2.0 * link_direction, 1e-12));
  EXPECT_TRUE(command.linear_velocity.isApprox(
      Eigen::Vector3d(0.4, -0.2, 0.1) -
          2.0 * angular_velocity.cross(link_direction), 1e-12));
}

TEST(JointPlanResult, InterpolatesSynchronizedPositionVelocityAndAttitude)
{
  JointPlanResult result;
  result.success = true;
  result.duration = 2.0;
  result.joint_waypoints = {{0.0, Eigen::Vector2d(0.0, 0.0)},
                            {2.0, Eigen::Vector2d(2.0, -4.0)}};
  result.attitude_waypoints = {{0.0, RootAttitude{-0.5, -0.2}},
                               {2.0, RootAttitude{0.5, 0.2}}};
  EXPECT_TRUE(result.jointPositions(1.0).isApprox(Eigen::Vector2d(1.0, -2.0), 1e-12));
  EXPECT_TRUE(result.jointVelocities(1.0).isApprox(Eigen::Vector2d(1.0, -2.0), 1e-12));
  EXPECT_NEAR(result.yaw(1.0), 0.0, 1e-12);
  EXPECT_NEAR(result.yawRate(1.0), 0.5, 1e-12);
  EXPECT_NEAR(result.pitch(1.0), 0.0, 1e-12);
  EXPECT_NEAR(result.pitchRate(1.0), 0.2, 1e-12);
  EXPECT_TRUE(result.angularVelocity(1.0).isApprox(Eigen::Vector3d(0.0, 0.2, 0.5), 1e-12));
  EXPECT_TRUE(result.jointVelocities(2.0).isZero(1e-12));
  EXPECT_NEAR(result.yawRate(2.0), 0.0, 1e-12);
  EXPECT_NEAR(result.pitchRate(2.0), 0.0, 1e-12);
}

TEST(JointPlanResult, InterpolatesYawAcrossWrapBoundaryOnShortestArc)
{
  JointPlanResult result;
  result.success = true;
  result.duration = 2.0;
  result.attitude_waypoints = {{0.0, RootAttitude{M_PI - 0.1, 0.0}},
                               {2.0, RootAttitude{-M_PI + 0.1, 0.0}}};

  EXPECT_NEAR(result.yaw(1.0), M_PI, 1e-12);
  EXPECT_NEAR(result.yawRate(1.0), 0.1, 1e-12);

  result.attitude_waypoints = {{0.0, RootAttitude{-M_PI + 0.1, 0.0}},
                               {2.0, RootAttitude{M_PI - 0.1, 0.0}}};
  EXPECT_NEAR(result.yaw(1.0), -M_PI, 1e-12);
  EXPECT_NEAR(result.yawRate(1.0), -0.1, 1e-12);
}

TEST(JointPlanResult, HandlesEmptyAndSingleWaypointSequences)
{
  JointPlanResult result;
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const double time : {-infinity, -1.0, 1.0, 2.0, infinity, nan})
  {
    EXPECT_EQ(result.jointPositions(time).size(), 0);
    EXPECT_EQ(result.jointVelocities(time).size(), 0);
    EXPECT_DOUBLE_EQ(result.yaw(time), 0.0);
    EXPECT_DOUBLE_EQ(result.pitch(time), 0.0);
    EXPECT_TRUE(result.angularVelocity(time).isZero());
    EXPECT_DOUBLE_EQ(result.pitchRate(time), 0.0);
  }
  result.joint_waypoints = {{1.0, Eigen::Vector2d(2.0, -3.0)}};
  result.attitude_waypoints = {{1.0, RootAttitude{0.3, -0.2}}};
  for (const double time : {-infinity, -1.0, 1.0, 2.0, infinity, nan})
  {
    EXPECT_TRUE(result.jointPositions(time).isApprox(Eigen::Vector2d(2.0, -3.0), 1e-12));
    EXPECT_TRUE(result.jointVelocities(time).isZero());
    EXPECT_DOUBLE_EQ(result.yaw(time), 0.3);
    EXPECT_DOUBLE_EQ(result.pitch(time), -0.2);
    EXPECT_TRUE(result.angularVelocity(time).isZero());
    EXPECT_DOUBLE_EQ(result.pitchRate(time), 0.0);
  }
}

TEST(JointPlanResult, PreservesOneSidedValuesAtRepeatedWaypointTimes)
{
  JointPlanResult result;
  for (const auto& sample : std::vector<std::pair<double, double>>{
           {1.0, 2.0}, {1.0, 4.0}, {2.0, 10.0}, {2.0, 12.0}, {4.0, 16.0}, {4.0, 18.0}})
  {
    result.joint_waypoints.push_back({sample.first, Eigen::VectorXd::Constant(1, sample.second)});
    result.attitude_waypoints.push_back({sample.first, {0.1 * sample.second, -0.05 * sample.second}});
  }
  struct Expected
  {
    double time;
    double position;
    double velocity;
  };
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const Expected& expected : std::vector<Expected>{
           {-infinity, 2.0, 0.0}, {0.0, 2.0, 0.0}, {1.0, 2.0, 6.0},
           {1.5, 7.0, 6.0}, {2.0, 12.0, 2.0}, {3.0, 14.0, 2.0},
           {4.0, 18.0, 0.0}, {5.0, 18.0, 0.0}, {infinity, 18.0, 0.0}, {nan, 18.0, 0.0}})
  {
    SCOPED_TRACE(expected.time);
    EXPECT_NEAR(result.jointPositions(expected.time)(0), expected.position, 1e-12);
    EXPECT_NEAR(result.jointVelocities(expected.time)(0), expected.velocity, 1e-12);
    EXPECT_NEAR(result.yaw(expected.time), 0.1 * expected.position, 1e-12);
    EXPECT_NEAR(result.pitch(expected.time), -0.05 * expected.position, 1e-12);
    EXPECT_NEAR(result.yawRate(expected.time), 0.1 * expected.velocity, 1e-12);
    EXPECT_NEAR(result.pitchRate(expected.time), -0.05 * expected.velocity, 1e-12);
    const Eigen::Vector3d angular_velocity(
        0.05 * expected.velocity * std::sin(0.1 * expected.position),
        -0.05 * expected.velocity * std::cos(0.1 * expected.position),
        0.1 * expected.velocity);
    EXPECT_TRUE(result.angularVelocity(expected.time).isApprox(angular_velocity, 1e-12));
  }
}

TEST(JointPlanResult, PreservesTheShortIntervalThreshold)
{
  for (const double duration : {0.5e-9, 1e-9, 2e-9})
  {
    SCOPED_TRACE(duration);
    JointPlanResult result;
    result.joint_waypoints = {{0.0, Eigen::VectorXd::Zero(1)},
                              {duration, Eigen::VectorXd::Constant(1, 2.0)}};
    result.attitude_waypoints = {{0.0, {0.0, 0.0}}, {duration, {0.4, -0.2}}};
    const double ratio = duration > 1e-9 ? 0.25 : 1.0;
    const double inverse_duration = duration > 1e-9 ? 1.0 / duration : 0.0;
    EXPECT_DOUBLE_EQ(result.jointPositions(0.25 * duration)(0), 2.0 * ratio);
    EXPECT_DOUBLE_EQ(result.jointVelocities(0.25 * duration)(0), 2.0 * inverse_duration);
    EXPECT_DOUBLE_EQ(result.yaw(0.25 * duration), 0.4 * ratio);
    EXPECT_DOUBLE_EQ(result.pitch(0.25 * duration), -0.2 * ratio);
    EXPECT_DOUBLE_EQ(result.yawRate(0.25 * duration), 0.4 * inverse_duration);
    EXPECT_DOUBLE_EQ(result.pitchRate(0.25 * duration), -0.2 * inverse_duration);
    EXPECT_TRUE(result.jointVelocities(duration).isZero());
    EXPECT_TRUE(result.angularVelocity(duration).isZero());
  }
}

TEST(JointPlanResult, PitchRateDoesNotEvaluateInvalidYaw)
{
  JointPlanResult result;
  result.attitude_waypoints = {
      {0.0, {std::numeric_limits<double>::quiet_NaN(), 0.0}}, {2.0, {0.0, 0.4}}};
  EXPECT_DOUBLE_EQ(result.pitchRate(1.0), 0.2);
  EXPECT_THROW(result.angularVelocity(1.0), std::invalid_argument);
}

TEST(TrajectoryHistory, SamplesAndTrimsByArcLength)
{
  FollowerConfig config;
  config.trajectory_sample_interval = 0.1;
  config.trajectory_buffer_max_length = 0.25;
  TrajectoryHistory history(config);
  EXPECT_TRUE(history.append(Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_FALSE(history.append(Eigen::Vector3d(0.05, 0.0, 0.0)));
  EXPECT_TRUE(history.append(Eigen::Vector3d(0.15, 0.0, 0.0)));
  EXPECT_TRUE(history.append(Eigen::Vector3d(0.30, 0.0, 0.0)));
  ASSERT_EQ(history.points().size(), 2u);
  EXPECT_TRUE(history.points().front().position.isApprox(Eigen::Vector3d(0.15, 0.0, 0.0)));
  EXPECT_NEAR(history.arcLength(), 0.15, 1e-12);
}

TEST(FollowerYaw, RespectsRateLimitAndPublishSwitch)
{
  FollowerConfig config;
  config.max_angular_vel = 1.0;
  EXPECT_NEAR(advanceYaw(0.0, Eigen::Vector3d(0.0, 1.0, 0.0), 0.1, config), 0.1, 1e-12);
  config.publish_yaw_command = false;
  EXPECT_NEAR(advanceYaw(0.4, Eigen::Vector3d(0.0, 1.0, 0.0), 1.0, config), 0.4, 1e-12);
}

TEST(RootAttitude, ConvertsThreeDimensionalTangentAndHandlesDegenerateVelocity)
{
  const RootAttitude fallback{0.7, 0.2};
  RootAttitude target = tangentAttitude(Eigen::Vector3d(0.0, 2.0, 0.0), fallback);
  EXPECT_NEAR(target.yaw, M_PI_2, 1e-12);
  EXPECT_NEAR(target.pitch, 0.0, 1e-12);

  target = tangentAttitude(Eigen::Vector3d(1.0, 0.0, 1.0), fallback);
  EXPECT_NEAR(target.yaw, 0.0, 1e-12);
  EXPECT_NEAR(target.pitch, -M_PI_4, 1e-12);

  target = tangentAttitude(Eigen::Vector3d(0.0, 0.0, 1.0), fallback);
  EXPECT_NEAR(target.yaw, fallback.yaw, 1e-12);
  EXPECT_NEAR(target.pitch, -M_PI_2, 1e-12);

  target = tangentAttitude(Eigen::Vector3d(1e-4, 0.0, 0.0), fallback);
  EXPECT_NEAR(target.yaw, fallback.yaw, 1e-12);
  EXPECT_NEAR(target.pitch, fallback.pitch, 1e-12);
}

TEST(RootAttitude, AdvancesYawAndPitchWithSharedAngularVelocityLimit)
{
  FollowerConfig config;
  config.max_angular_vel = 0.5;
  const RootAttitude advanced = advanceRootAttitude(
      RootAttitude{0.0, 0.0}, Eigen::Vector3d(0.0, 1.0, 1.0), 0.1,
      config, true);
  EXPECT_NEAR(advanced.yaw, 0.05, 1e-12);
  EXPECT_NEAR(advanced.pitch, -0.05, 1e-12);
}

TEST(Joint1PriorityAllocation, KeepsDownstreamEndpointsFixedBeforeSaturation)
{
  Eigen::VectorXd start = Eigen::VectorXd::Zero(6);
  start(0) = 0.4;
  start(1) = 0.3;
  start(3) = -0.2;
  start(5) = 0.5;
  const std::vector<double> lower(6, -2.0);
  const std::vector<double> upper(6, 2.0);
  const std::vector<int> pitch_indices{0, 2, 4};
  const std::vector<int> yaw_indices{1, 3, 5};
  const Eigen::Vector3d tail(0.2, -0.3, 1.1);

  Eigen::VectorXd yaw_start_joints = start;
  yaw_start_joints(0) = 0.0;
  const RootAttitude yaw_start{0.0, 0.0};
  const RootAttitude yaw_goal{0.2, 0.0};
  const Eigen::VectorXd yaw_joints =
      JointTrajectoryPlanner::joint1PriorityConfiguration(
          yaw_start_joints, 0, 1, lower, upper, yaw_start, yaw_goal, 1.0);
  EXPECT_NEAR(yaw_joints(1), yaw_start_joints(1) - 0.2, 1e-12);
  for (int index : {0, 2, 3, 4, 5})
  {
    EXPECT_DOUBLE_EQ(yaw_joints(index), yaw_start_joints(index));
  }
  const std::vector<Eigen::Vector3d> yaw_before = linkEndpoints(
      tail, linkRotation(yaw_start), yaw_start_joints,
      pitch_indices, yaw_indices, 4, 0.6);
  const std::vector<Eigen::Vector3d> yaw_after = linkEndpoints(
      tail, linkRotation(yaw_goal), yaw_joints, pitch_indices, yaw_indices, 4, 0.6);
  ASSERT_EQ(yaw_before.size(), yaw_after.size());
  for (size_t index = 2; index < yaw_before.size(); ++index)
  {
    EXPECT_TRUE(yaw_after[index].isApprox(yaw_before[index], 1e-12));
  }

  Eigen::VectorXd pitch_start_joints = start;
  pitch_start_joints(1) = 0.0;
  const RootAttitude pitch_start{0.0, -0.1};
  const RootAttitude pitch_goal{0.0, 0.2};
  const Eigen::VectorXd pitch_joints =
      JointTrajectoryPlanner::joint1PriorityConfiguration(
          pitch_start_joints, 0, 1, lower, upper,
          pitch_start, pitch_goal, 1.0);
  EXPECT_NEAR(pitch_joints(0), pitch_start_joints(0) + 0.3, 1e-12);
  for (int index : {1, 2, 3, 4, 5})
  {
    EXPECT_DOUBLE_EQ(pitch_joints(index), pitch_start_joints(index));
  }
  const std::vector<Eigen::Vector3d> pitch_before = linkEndpoints(
      tail, linkRotation(pitch_start), pitch_start_joints,
      pitch_indices, yaw_indices, 4, 0.6);
  const std::vector<Eigen::Vector3d> pitch_after = linkEndpoints(
      tail, linkRotation(pitch_goal), pitch_joints,
      pitch_indices, yaw_indices, 4, 0.6);
  ASSERT_EQ(pitch_before.size(), pitch_after.size());
  for (size_t index = 2; index < pitch_before.size(); ++index)
  {
    EXPECT_TRUE(pitch_after[index].isApprox(pitch_before[index], 1e-12));
  }
}

TEST(Joint1PriorityAllocation, SaturatesEachAxisAndLeavesOtherJointsUnchanged)
{
  Eigen::VectorXd start = Eigen::VectorXd::Zero(6);
  start(0) = 0.4;
  start(1) = 0.2;
  const std::vector<double> lower(6, -0.5);
  const std::vector<double> upper(6, 0.5);
  const RootAttitude attitude_start{0.0, 0.0};
  const RootAttitude attitude_goal{1.0, 0.4};

  const Eigen::VectorXd halfway =
      JointTrajectoryPlanner::joint1PriorityConfiguration(
          start, 0, 1, lower, upper, attitude_start, attitude_goal, 0.5);
  EXPECT_NEAR(halfway(0), 0.5, 1e-12);
  EXPECT_NEAR(halfway(1), -0.3, 1e-12);

  const Eigen::VectorXd complete =
      JointTrajectoryPlanner::joint1PriorityConfiguration(
          start, 0, 1, lower, upper, attitude_start, attitude_goal, 1.0);
  EXPECT_NEAR(complete(0), 0.5, 1e-12);
  EXPECT_NEAR(complete(1), -0.5, 1e-12);
  for (int index = 2; index < complete.size(); ++index)
  {
    EXPECT_DOUBLE_EQ(complete(index), start(index));
  }
}

TEST(PlanningEnvironment, ClampsTargetsInsideTheConfiguredMap)
{
  PlanningEnvironment environment(validSharedConfig());
  constexpr double clearance = 0.2;
  const Eigen::Vector3d clamped =
      environment.clampTarget(Eigen::Vector3d(20.0, -20.0, 20.0), clearance);
  EXPECT_TRUE((clamped.array() >= (environment.mapOrigin().array() + clearance)).all());
  EXPECT_TRUE((clamped.array() <= (environment.mapCorner().array() - clearance)).all());
}

TEST(PlanningEnvironment, AppliesConfigurableLocalTargetVelocity)
{
  RootState start;
  start.position = Eigen::Vector3d(-0.5, 0.0, 1.0);
  const Eigen::Vector3d target(1.5, 0.0, 1.0);

  SharedPlannerConfig stopped_config = validSharedConfig();
  stopped_config.planning_horizon = 0.5;
  EXPECT_TRUE(stopped_config.zero_local_target_vel);
  PlanningEnvironment stopped_environment(stopped_config);
  const PrimitiveBatch stopped_batch = stopped_environment.generate(start, target);
  ASSERT_TRUE(stopped_batch.success()) << stopped_batch.detail;
  ASSERT_FALSE(stopped_batch.terminal);
  for (const Candidate& candidate : stopped_batch.candidates)
  {
    ASSERT_GT(candidate.trajectory.getPieceNum(), 0);
    EXPECT_LT(candidate.trajectory.getVel(candidate.trajectory.getTotalDuration()).norm(), 1e-6);
  }

  SharedPlannerConfig cruise_config = stopped_config;
  cruise_config.zero_local_target_vel = false;
  PlanningEnvironment cruise_environment(cruise_config);
  const PrimitiveBatch cruise_batch = cruise_environment.generate(start, target);
  ASSERT_TRUE(cruise_batch.success()) << cruise_batch.detail;
  ASSERT_FALSE(cruise_batch.terminal);
  const Eigen::Vector3d tangent =
      cruise_batch.local_route.back() -
      cruise_batch.local_route[cruise_batch.local_route.size() - 2];
  ASSERT_GT(tangent.norm(), 1e-6);
  const Eigen::Vector3d expected_velocity =
      cruise_config.primitive.cruise_velocity * tangent.normalized();
  for (const Candidate& candidate : cruise_batch.candidates)
  {
    ASSERT_GT(candidate.trajectory.getPieceNum(), 0);
    EXPECT_TRUE(candidate.trajectory.getVel(candidate.trajectory.getTotalDuration())
                    .isApprox(expected_velocity, 1e-6));
  }
}

TEST(PlanningEnvironment, TruncatesRoutesAndKeepsGlobalTargetVelocityZero)
{
  const std::vector<Eigen::Vector3d> route = {
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(1.0, 0.0, 1.0),
      Eigen::Vector3d(2.0, 0.0, 1.0)};
  std::vector<Eigen::Vector3d> local;
  const Eigen::Vector3d local_target = PlanningEnvironment::truncateRoute(route, 1.5, local);
  ASSERT_EQ(local.size(), 3u);
  EXPECT_TRUE(local_target.isApprox(Eigen::Vector3d(1.5, 0.0, 1.0), 1e-12));

  RootState start;
  start.position = Eigen::Vector3d(-0.5, 0.0, 1.0);
  for (const bool zero_local_target_vel : {true, false})
  {
    SharedPlannerConfig config = validSharedConfig();
    config.planning_horizon = 3.0;
    config.zero_local_target_vel = zero_local_target_vel;
    PlanningEnvironment environment(config);
    const PrimitiveBatch batch = environment.generate(start, Eigen::Vector3d(0.5, 0.0, 1.0));
    ASSERT_TRUE(batch.success()) << batch.detail;
    EXPECT_GE(batch.route_search_timing.first_exact_solution_ms, 0.0);
    EXPECT_GE(batch.route_search_timing.total_ms, batch.route_search_timing.first_exact_solution_ms);
    EXPECT_TRUE(batch.terminal);
    ASSERT_EQ(batch.candidates.size(), 3u);
    for (const Candidate& candidate : batch.candidates)
    {
      ASSERT_GT(candidate.trajectory.getPieceNum(), 0);
      EXPECT_TRUE(candidate.trajectory.getPos(0.0).isApprox(start.position, 1e-6));
      EXPECT_LT(candidate.trajectory.getVel(candidate.trajectory.getTotalDuration()).norm(), 1e-6);
    }
  }
}

TEST(CandidateDiagnostics, UsesOneSharedStatusColorMapping)
{
  const std_msgs::ColorRGBA selected = candidateColor(CandidateStatus::kCollision, true);
  EXPECT_FLOAT_EQ(selected.r, 0.0f);
  EXPECT_FLOAT_EQ(selected.g, 1.0f);
  const std_msgs::ColorRGBA feasible = candidateColor(CandidateStatus::kFeasible, false);
  EXPECT_FLOAT_EQ(feasible.g, 1.0f);
  EXPECT_FLOAT_EQ(feasible.b, 1.0f);
  const std_msgs::ColorRGBA joint_failure =
      candidateColor(CandidateStatus::kJointPlanningFailed, false);
  EXPECT_FLOAT_EQ(joint_failure.r, 1.0f);
  EXPECT_FLOAT_EQ(joint_failure.g, 0.5f);
}

}  // namespace
}  // namespace motion_primitive_planner

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(OctomapMessage, FullProbabilitiesEmptyAndInvalidStreams)
{
  octomap::OcTree source(.1);
  source.updateNode(octomap::point3d(.15, .15, .15), true);
  octomap_msgs::Octomap message;
  ASSERT_TRUE(octomap_msgs::fullMapToMsg(source, message));
  message.header.frame_id = "grid";
  auto tree = motion_primitive_planner::readOctomap(message);
  ASSERT_NE(tree->search(.15, .15, .15), nullptr);
  EXPECT_DOUBLE_EQ(tree->search(.15, .15, .15)->getOccupancy(), source.search(.15, .15, .15)->getOccupancy());
  auto invalid = message;
  invalid.data.pop_back();
  EXPECT_THROW(motion_primitive_planner::readOctomap(invalid), std::invalid_argument);
  invalid = message;
  invalid.data.push_back(0);
  EXPECT_THROW(motion_primitive_planner::readOctomap(invalid), std::invalid_argument);
  invalid = message;
  invalid.binary = true;
  EXPECT_THROW(motion_primitive_planner::readOctomap(invalid), std::invalid_argument);
  invalid = message;
  invalid.id = "ColorOcTree";
  EXPECT_THROW(motion_primitive_planner::readOctomap(invalid), std::invalid_argument);
  invalid = message;
  invalid.resolution = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(motion_primitive_planner::readOctomap(invalid), std::invalid_argument);
  message.data.clear();
  EXPECT_EQ(motion_primitive_planner::readOctomap(message)->size(), 0u);
  EXPECT_GT(tree->size(), 0u);
}

TEST(OctomapPlanningBounds, OutsideTreeAndPartialCoarseLeafAreAccepted)
{
  auto config = motion_primitive_planner::validSharedConfig();
  config.common.voxelWidth = .1;
  config.common.dilateRadius = 0;
  config.common.mapBound = {-2, 2.13, -2, 2, 0, 3};
  motion_primitive_planner::PlanningEnvironment environment(config);
  const auto before = environment.snapshot();
  const auto origin = before->route->mapOrigin();
  auto tree = std::make_shared<octomap::OcTree>(.1);
  for (int x = 0; x < 2; ++x) for (int y = 0; y < 2; ++y) for (int z = 0; z < 2; ++z)
    tree->updateNode(octomap::point3d(4 + (x + .5) * .1, 2 + (y + .5) * .1, 1 + (z + .5) * .1), true, true);
  tree->updateNode(octomap::point3d(6.05, 2.05, 1.05), true, true);
  tree->updateInnerOccupancy();
  tree->prune();
  auto transform = Eigen::Isometry3d::Identity();
  transform.translation() = origin;
  environment.replaceMap(tree, transform, origin, before->route->mapCorner());
  const auto after = environment.snapshot();
  EXPECT_TRUE(after->route->query(origin + Eigen::Vector3d(4.05, 2.05, 1.05)));
  EXPECT_FALSE(before->route->query(origin + Eigen::Vector3d(4.05, 2.05, 1.05)));
  EXPECT_TRUE(after->route->query(origin + Eigen::Vector3d(6.05, 2.05, 1.05)));
  EXPECT_EQ(tree->getNumLeafNodes(), 2u);
}
