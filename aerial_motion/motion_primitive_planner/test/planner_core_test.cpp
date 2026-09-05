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

TEST(WholeBodyCollision, UsesTheShortestYawDeltaAcrossTheWrapBoundary)
{
  EXPECT_NEAR(shortestYawDelta(M_PI - 0.1, -M_PI + 0.1), 0.2, 1e-12);
  EXPECT_NEAR(shortestYawDelta(-M_PI + 0.1, M_PI - 0.1), -0.2, 1e-12);
}

TEST(NominalJointPredictor, UsesUniformFixedTimeSamplesIncludingEndpoints)
{
  FollowerConfig follower;
  follower.publish_yaw_command = false;
  NominalJointContext context;
  context.link_num = 1;
  context.link_length = 1.0;
  const Eigen::VectorXd start_joints = Eigen::VectorXd::Zero(1);
  const NominalJointPredictor predictor(follower);

  const auto samples_for = [&](double duration, double sample_dt) {
    return predictor.predict(
        linearTrajectory(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), duration),
        context, start_joints, 0.0, sample_dt);
  };

  const std::vector<NominalJointSample> divisible = samples_for(1.0, 0.25);
  ASSERT_EQ(divisible.size(), 5u);
  for (size_t index = 0; index < divisible.size(); ++index)
  {
    EXPECT_NEAR(divisible[index].time, 0.25 * index, 1e-12);
  }

  const std::vector<NominalJointSample> non_divisible = samples_for(1.0, 0.30);
  ASSERT_EQ(non_divisible.size(), 5u);
  EXPECT_DOUBLE_EQ(non_divisible.front().time, 0.0);
  EXPECT_DOUBLE_EQ(non_divisible.back().time, 1.0);
  const double interval = non_divisible[1].time - non_divisible[0].time;
  EXPECT_LE(interval, 0.30);
  for (size_t index = 2; index < non_divisible.size(); ++index)
  {
    EXPECT_NEAR(non_divisible[index].time - non_divisible[index - 1].time,
                interval, 1e-12);
  }

  const std::vector<NominalJointSample> zero_duration = samples_for(0.0, 0.25);
  ASSERT_EQ(zero_duration.size(), 1u);
  EXPECT_DOUBLE_EQ(zero_duration.front().time, 0.0);
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

TEST(PlanningEnvironment, ReplacesCollisionMapSnapshots)
{
  const Eigen::Vector3d first(-0.5, 0.0, 1.0);
  const Eigen::Vector3d second(0.5, 0.0, 1.0);
  PlanningEnvironment environment(validSharedConfig());
  environment.replaceMap({first});
  EXPECT_TRUE(environment.occupied(first));
  environment.replaceMap({second});
  EXPECT_FALSE(environment.occupied(first));
  EXPECT_TRUE(environment.occupied(second));
}

TEST(PlanningEnvironment, PreservesImmutableOccupancySnapshotsAcrossMapUpdates)
{
  PlanningEnvironment environment(validSharedConfig());
  const Eigen::Vector3d first(-0.5, 0.0, 1.0);
  const Eigen::Vector3d second(0.5, 0.0, 1.0);
  environment.replaceMap({first});
  const std::shared_ptr<const gcopter_planner::PlannerBackend> first_snapshot =
      environment.occupancySnapshot();
  environment.replaceMap({second});
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
}

}  // namespace
}  // namespace motion_primitive_planner

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
