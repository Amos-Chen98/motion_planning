#include <motion_primitive_planner/planner_core.h>
#include <motion_primitive_planner/joint_trajectory_planner.h>
#include <motion_primitive_planner/planner_common.h>
#include <motion_primitive_planner/root_candidate_evaluator.h>

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

SharedPlannerConfig validSharedConfig(bool accumulated = true)
{
  SharedPlannerConfig config;
  config.common.worldFrameId = "world";
  config.common.dilateRadius = 0.0;
  config.common.voxelWidth = 0.1;
  config.common.mapBound = {-2.0, 2.0, -2.0, 2.0, 0.0, 3.0};
  config.common.timeoutRRT = 0.1;
  config.common.maxVelMag = 0.5;
  config.common.maxBdrMag = 2.1;
  config.common.maxTiltAngle = 1.05;
  config.common.gravAcc = 9.8;
  config.common.weightT = 20.0;
  config.common.chiVec = {1.0e4, 1.0e4, 1.0e4, 1.0e4};
  config.common.smoothingEps = 1.0e-2;
  config.common.integralIntervs = 16;
  config.common.relCostTol = 1.0e-5;
  config.primitive.candidate_count = 3;
  config.primitive.max_offset = 0.4;
  config.primitive.max_velocity = 0.5;
  config.primitive.cruise_velocity = 0.3;
  config.primitive.minimum_piece_duration = 0.2;
  config.use_accumulated_map = accumulated;
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

TEST(WholeBodyCollision, DetectsTrailingLinkWhenRootTailIsClear)
{
  WholeBodyConfiguration configuration;
  configuration.root_link_rotation =
      multilink_copilot::follow_the_leader::rotationAroundZ(M_PI);
  configuration.joint_positions = Eigen::VectorXd::Zero(6);
  DragonCollisionGeometry geometry;
  geometry.link_num = 4;
  geometry.link_length = 1.0;
  geometry.pitch_joint_indices = {0, 2, 4};
  geometry.yaw_joint_indices = {1, 3, 5};
  const auto occupied = [](const Eigen::Vector3d& point) {
    return (point - Eigen::Vector3d(-2.5, 0.0, 0.0)).norm() < 0.08;
  };
  EXPECT_FALSE(occupied(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(wholeBodyCollides(configuration, geometry, 0.05, occupied));
}

TEST(WholeBodyCollision, UsesOneInstantaneousConfigurationCheckerForEveryLink)
{
  WholeBodyConfiguration configuration;
  configuration.root_link_rotation =
      multilink_copilot::follow_the_leader::rotationAroundZ(M_PI);
  configuration.joint_positions = Eigen::VectorXd::Zero(6);
  DragonCollisionGeometry geometry;
  geometry.link_num = 4;
  geometry.link_length = 1.0;
  geometry.pitch_joint_indices = {0, 2, 4};
  geometry.yaw_joint_indices = {1, 3, 5};

  const auto free = [](const Eigen::Vector3d&) { return false; };
  EXPECT_FALSE(wholeBodyCollides(configuration, geometry, 0.05, free));
  for (const double link_midpoint : {0.5, -0.5, -1.5, -2.5})
  {
    const auto occupied = [link_midpoint](const Eigen::Vector3d& point) {
      return (point - Eigen::Vector3d(link_midpoint, 0.0, 0.0)).norm() < 0.01;
    };
    EXPECT_TRUE(wholeBodyCollides(configuration, geometry, 0.05, occupied));
  }
  for (const double endpoint : {1.0, -3.0})
  {
    const auto occupied = [endpoint](const Eigen::Vector3d& point) {
      return (point - Eigen::Vector3d(endpoint, 0.0, 0.0)).norm() < 0.01;
    };
    EXPECT_TRUE(wholeBodyCollides(configuration, geometry, 0.05, occupied));
  }
}

TEST(WholeBodyCollision, TreatsInvalidGeometryAsCollisionAndStopsAtFirstHit)
{
  WholeBodyConfiguration configuration;
  configuration.joint_positions = Eigen::VectorXd::Zero(6);
  DragonCollisionGeometry invalid_geometry;
  int query_count = 0;
  const auto occupied = [&query_count](const Eigen::Vector3d&) {
    ++query_count;
    return true;
  };
  EXPECT_TRUE(wholeBodyCollides(configuration, invalid_geometry, 0.05, occupied));
  EXPECT_EQ(query_count, 0);

  DragonCollisionGeometry geometry;
  geometry.link_num = 4;
  geometry.link_length = 1.0;
  geometry.pitch_joint_indices = {0, 2, 4};
  geometry.yaw_joint_indices = {1, 3, 5};
  EXPECT_TRUE(wholeBodyCollides(configuration, geometry, 0.05, occupied));
  EXPECT_EQ(query_count, 1);
}

TEST(WholeBodyCollision, TreatsAConfigurationOutsideTheMapAsCollision)
{
  PlanningEnvironment environment(validSharedConfig());
  const std::shared_ptr<const gcopter_planner::PlannerBackend> occupancy =
      environment.occupancySnapshot();
  WholeBodyConfiguration configuration;
  configuration.link1_tail = Eigen::Vector3d(-1.0, 0.0, 1.0);
  configuration.root_link_rotation =
      multilink_copilot::follow_the_leader::rotationAroundZ(M_PI);
  configuration.joint_positions = Eigen::VectorXd::Zero(6);
  DragonCollisionGeometry geometry;
  geometry.link_num = 4;
  geometry.link_length = 1.0;
  geometry.pitch_joint_indices = {0, 2, 4};
  geometry.yaw_joint_indices = {1, 3, 5};
  const auto occupied = [&occupancy](const Eigen::Vector3d& point) {
    return occupancy->query(point);
  };
  EXPECT_TRUE(wholeBodyCollides(configuration, geometry, 0.05, occupied));
}

TEST(WholeBodyCollision, ComputesAdaptiveTemporalSubdivisionsFromRootAndAngularMotion)
{
  const Eigen::VectorXd no_joint_motion = Eigen::VectorXd::Zero(0);
  EXPECT_EQ(wholeBodyMotionSubdivisionCount(
                0.0, 0.0, 0.0, 0.0, no_joint_motion, 0.125),
            1);
  EXPECT_EQ(wholeBodyMotionSubdivisionCount(
                0.375, 1.0, 0.0, 0.0, no_joint_motion, 0.125),
            3);
  EXPECT_EQ(wholeBodyMotionSubdivisionCount(
                0.0, 0.0, 2.0, 0.25, no_joint_motion, 0.125),
            4);

  Eigen::VectorXd joint_delta(2);
  joint_delta << 0.25, -0.125;
  EXPECT_EQ(wholeBodyMotionSubdivisionCount(
                0.0, 0.0, 2.0, 0.0, joint_delta, 0.125),
            6);
  EXPECT_EQ(wholeBodyMotionSubdivisionCount(
                0.125, 1.0, 0.0, 0.0, no_joint_motion, 0.125),
            1);
  EXPECT_EQ(wholeBodyMotionSubdivisionCount(
                0.126, 1.0, 0.0, 0.0, no_joint_motion, 0.125),
            2);
}

TEST(WholeBodyCollision, UsesTheShortestYawDeltaAcrossTheWrapBoundary)
{
  EXPECT_NEAR(shortestYawDelta(M_PI - 0.1, -M_PI + 0.1), 0.2, 1e-12);
  EXPECT_NEAR(shortestYawDelta(-M_PI + 0.1, M_PI - 0.1), -0.2, 1e-12);
}

TEST(RootCandidateCollision, DetectsIntermediateRootTranslation)
{
  const Trajectory<5> trajectory =
      linearTrajectory(Eigen::Vector3d(0.0, 0.0, 1.0),
                       Eigen::Vector3d(0.0, 1.0, 0.0), 1.0);
  NominalJointSample start;
  start.time = 0.0;
  start.yaw = 0.0;
  start.root_position = trajectory.getPos(start.time);
  NominalJointSample end = start;
  end.time = 1.0;
  end.root_position = trajectory.getPos(end.time);

  DragonCollisionGeometry geometry;
  geometry.link_num = 1;
  geometry.link_length = 1.0;
  const Eigen::Vector3d obstacle(0.0, 0.5, 1.0);
  const auto occupied = [&obstacle](const Eigen::Vector3d& point) {
    return (point - obstacle).norm() < 0.02;
  };

  EXPECT_TRUE(nominalWholeBodyIntervalCollides(
      trajectory, start, end, 1.0, geometry, 0.05, occupied));
}

TEST(RootCandidateCollision, DetectsIntermediateShortestPathYawRotation)
{
  const Trajectory<5> trajectory =
      linearTrajectory(Eigen::Vector3d(0.0, 0.0, 1.0),
                       Eigen::Vector3d::Zero(), 1.0);
  NominalJointSample start;
  start.time = 0.0;
  start.yaw = 0.0;
  start.root_position = trajectory.getPos(start.time);
  NominalJointSample end = start;
  end.time = 1.0;
  end.yaw = M_PI_2;

  DragonCollisionGeometry geometry;
  geometry.link_num = 1;
  geometry.link_length = 1.0;
  const Eigen::Vector3d obstacle(0.5 / std::sqrt(2.0),
                                 0.5 / std::sqrt(2.0), 1.0);
  const auto occupied = [&obstacle](const Eigen::Vector3d& point) {
    return (point - obstacle).norm() < 0.03;
  };

  EXPECT_TRUE(nominalWholeBodyIntervalCollides(
      trajectory, start, end, 0.0, geometry, 0.05, occupied));
}

TEST(RootCandidateCollision, DetectsIntermediateJointRotation)
{
  const Trajectory<5> trajectory =
      linearTrajectory(Eigen::Vector3d(0.0, 0.0, 1.0),
                       Eigen::Vector3d::Zero(), 1.0);
  NominalJointSample start;
  start.time = 0.0;
  start.yaw = 0.0;
  start.root_position = trajectory.getPos(start.time);
  start.joints = Eigen::Vector2d::Zero();
  NominalJointSample end = start;
  end.time = 1.0;
  end.joints(1) = M_PI_2;

  DragonCollisionGeometry geometry;
  geometry.link_num = 2;
  geometry.link_length = 1.0;
  geometry.pitch_joint_indices = {0};
  geometry.yaw_joint_indices = {1};
  const Eigen::Vector3d obstacle(-0.5 / std::sqrt(2.0),
                                 -0.5 / std::sqrt(2.0), 1.0);
  const auto occupied = [&obstacle](const Eigen::Vector3d& point) {
    return (point - obstacle).norm() < 0.03;
  };

  EXPECT_TRUE(nominalWholeBodyIntervalCollides(
      trajectory, start, end, 0.0, geometry, 0.05, occupied));
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

TEST(NominalJointPredictor, ExposesRetracedHistoryBranchDiscontinuity)
{
  FollowerConfig follower;
  follower.publish_yaw_command = false;
  TrajectoryHistory history(follower);
  for (int index = 0; index <= 50; ++index)
  {
    history.append(Eigen::Vector3d(-2.5 + 0.05 * index, 0.0, 1.0));
  }
  NominalJointContext context;
  context.executed_history = history;
  context.link_num = 4;
  context.link_length = 0.5255;
  context.pitch_joint_indices = {0, 2, 4};
  context.yaw_joint_indices = {1, 3, 5};

  PrimitiveConfig primitive;
  primitive.candidate_count = 1;
  primitive.max_velocity = 1.0;
  primitive.cruise_velocity = 0.25;
  Eigen::Matrix3d initial_state = Eigen::Matrix3d::Zero();
  initial_state.col(0) = Eigen::Vector3d(0.0, 0.0, 1.0);
  Eigen::Matrix3d final_state = Eigen::Matrix3d::Zero();
  final_state.col(0) = Eigen::Vector3d(-3.0, 0.0, 1.0);
  const Candidate candidate =
      PrimitiveGenerator(primitive).generate(initial_state, final_state).front();

  const std::vector<NominalJointSample> samples =
      NominalJointPredictor(follower).predict(
          candidate.trajectory, context, Eigen::VectorXd::Zero(6), 0.0, 0.05);
  ASSERT_GT(samples.size(), 2u);
  double maximum_joint_step = 0.0;
  for (size_t index = 1; index < samples.size(); ++index)
  {
    maximum_joint_step = std::max(
        maximum_joint_step,
        (samples[index].joints - samples[index - 1].joints).norm());
  }
  EXPECT_GT(maximum_joint_step, 3.0);
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

TEST(JointPlanResult, InterpolatesYawAcrossWrapBoundaryOnShortestArc)
{
  JointPlanResult result;
  result.success = true;
  result.duration = 2.0;
  result.yaw_waypoints = {{0.0, M_PI - 0.1}, {2.0, -M_PI + 0.1}};

  EXPECT_NEAR(result.yaw(1.0), M_PI, 1e-12);
  EXPECT_NEAR(result.yawRate(1.0), 0.1, 1e-12);

  result.yaw_waypoints = {{0.0, -M_PI + 0.1}, {2.0, M_PI - 0.1}};
  EXPECT_NEAR(result.yaw(1.0), -M_PI, 1e-12);
  EXPECT_NEAR(result.yawRate(1.0), -0.1, 1e-12);
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
  config.max_yaw_rate = 1.0;
  EXPECT_NEAR(advanceYaw(0.0, Eigen::Vector3d(0.0, 1.0, 0.0), 0.1, config), 0.1, 1e-12);
  config.publish_yaw_command = false;
  EXPECT_NEAR(advanceYaw(0.4, Eigen::Vector3d(0.0, 1.0, 0.0), 1.0, config), 0.4, 1e-12);
}

TEST(NominalJointPredictor, PreservesHandoverStateAndUsesSharedFollowerGeometry)
{
  PrimitiveConfig primitive;
  primitive.candidate_count = 1;
  primitive.max_velocity = 0.5;
  primitive.cruise_velocity = 0.3;
  const std::vector<Eigen::Vector3d> route = {
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.5, 0.0, 1.0)};
  const Candidate candidate = PrimitiveGenerator(primitive).generate(
      endpointState(route.front()), endpointState(route.back())).front();

  FollowerConfig follower;
  TrajectoryHistory history(follower);
  history.append(route.front());
  NominalJointContext context;
  context.executed_history = history;
  context.link_num = 4;
  context.link_length = 1.0;
  context.pitch_joint_indices = {0, 2, 4};
  context.yaw_joint_indices = {1, 3, 5};
  Eigen::VectorXd start = Eigen::VectorXd::Zero(6);
  start(1) = M_PI_2;
  start(3) = M_PI_2;
  start(5) = M_PI_2;

  const std::vector<NominalJointSample> samples =
      NominalJointPredictor(follower).predict(candidate.trajectory, context, start, 0.0, 0.05);
  ASSERT_GT(samples.size(), 2u);
  EXPECT_DOUBLE_EQ(samples.front().time, 0.0);
  EXPECT_TRUE(samples.front().joints.isApprox(start, 1e-12));
  EXPECT_TRUE(samples.back().joints.allFinite());
}

TEST(PlanningEnvironment, AccumulatesOrReplacesMapVoxels)
{
  PlanningEnvironment accumulated(validSharedConfig(true));
  const Eigen::Vector3d first(-0.5, 0.0, 1.0);
  const Eigen::Vector3d second(0.5, 0.0, 1.0);
  accumulated.updateMap({first});
  accumulated.updateMap({second});
  EXPECT_TRUE(accumulated.occupied(first));
  EXPECT_TRUE(accumulated.occupied(second));

  PlanningEnvironment latest(validSharedConfig(false));
  latest.updateMap({first});
  latest.updateMap({second});
  EXPECT_FALSE(latest.occupied(first));
  EXPECT_TRUE(latest.occupied(second));
}

TEST(PlanningEnvironment, PreservesImmutableOccupancySnapshotsAcrossMapUpdates)
{
  PlanningEnvironment environment(validSharedConfig(false));
  const Eigen::Vector3d first(-0.5, 0.0, 1.0);
  const Eigen::Vector3d second(0.5, 0.0, 1.0);
  environment.updateMap({first});
  const std::shared_ptr<const gcopter_planner::PlannerBackend> first_snapshot =
      environment.occupancySnapshot();
  environment.updateMap({second});
  const std::shared_ptr<const gcopter_planner::PlannerBackend> second_snapshot =
      environment.occupancySnapshot();

  EXPECT_TRUE(first_snapshot->query(first));
  EXPECT_FALSE(first_snapshot->query(second));
  EXPECT_FALSE(second_snapshot->query(first));
  EXPECT_TRUE(second_snapshot->query(second));
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
  const std_msgs::ColorRGBA joint_limit = candidateColor(CandidateStatus::kJointLimit, false);
  EXPECT_FLOAT_EQ(joint_limit.r, 1.0f);
  EXPECT_FLOAT_EQ(joint_limit.b, 1.0f);
}

TEST(RootStabilityFallback, AcceptsOnlyAnIsolatedFcRpViolation)
{
  multilink_copilot::StabilityConfig config;
  config.fc_rp_min_threshold = 3.5;
  config.check_fc_t = true;
  config.fc_t_min_threshold = 0.05;
  config.static_thrust_min = 2.0;
  config.static_thrust_max = 20.0;
  config.overlap_min_clearance = 0.01;
  config.max_baselink_tilt = 1.2;
  config.feasibility_tolerance = 1.0e-6;

  multilink_copilot::StabilityMetrics metrics;
  metrics.fc_rp_min = 3.0;
  metrics.fc_t_min = 0.10;
  metrics.static_thrust_min = 5.0;
  metrics.static_thrust_max = 10.0;
  metrics.overlap_clearance = 0.10;
  metrics.baselink_tilt = 0.5;
  EXPECT_TRUE(isOnlyFcRpViolation(metrics, config));

  metrics.baselink_tilt = 1.3;
  EXPECT_FALSE(isOnlyFcRpViolation(metrics, config));
  metrics.baselink_tilt = 0.5;
  metrics.static_thrust_min = 1.0;
  EXPECT_FALSE(isOnlyFcRpViolation(metrics, config));
  metrics.static_thrust_min = 5.0;
  metrics.fc_t_min = 0.01;
  EXPECT_FALSE(isOnlyFcRpViolation(metrics, config));
  metrics.fc_t_min = 0.10;
  metrics.fc_rp_min = 4.0;
  EXPECT_FALSE(isOnlyFcRpViolation(metrics, config));
}
}  // namespace
}  // namespace motion_primitive_planner

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
