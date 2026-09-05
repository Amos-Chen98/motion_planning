// Regression coverage for global joint-space RRT planning: every free-space
// candidate batch must yield an executable whole-body trajectory, and the
// archetypal fold flip must be connected without crossing an infeasible edge.
#include <motion_primitive_planner/joint_trajectory_planner.h>
#include <motion_primitive_planner/root_primitive_generator.h>
#include <motion_primitive_planner/whole_body_planner.h>

#include <dragon/model/hydrus_like_robot_model.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace motion_primitive_planner
{
namespace
{
//! Wall-clock budget the whole-body node gives one full candidate batch.
constexpr double kBatchBudget = 0.60;

class WholeBodyJointBridge : public ::testing::Test
{
protected:
  void SetUp() override
  {
    loader_.reset(new pluginlib::ClassLoader<aerial_robot_model::RobotModel>(
        "aerial_robot_model", "aerial_robot_model::RobotModel"));
    const auto base_model = loader_->createInstance("dragon/hydrus_like_robot_model");
    model_ = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(base_model);
    ASSERT_TRUE(model_);
    evaluator_ = std::make_shared<multilink_copilot::StabilityEvaluator>(
        model_, multilink_copilot::StabilityConfig());
    info_.reset(new DragonModelInfo(model_));

    primitive_.candidate_count = 9;
    primitive_.max_offset = 0.8;
    primitive_.max_velocity = 1.0;
    primitive_.cruise_velocity = 0.8;
    primitive_.minimum_piece_duration = 0.2;
  }

  //! Uses a faster shared angular limit to keep the exhaustive bridge tests short.
  JointPlannerConfig jointConfig(unsigned int seed) const
  {
    JointPlannerConfig config;
    config.follower.command_hz = 40.0;
    config.follower.trajectory_sample_interval = 0.05;
    config.follower.trajectory_buffer_max_length = 10.0;
    config.follower.ik_singularity_threshold = 0.10;
    config.follower.max_angular_vel = 4.0;
    config.reference_dt = 0.10;
    config.planning_timeout = 0.15;
    config.validity_resolution = 0.025;
    config.max_joint_command_step = 0.10;
    config.random_seed = seed;
    return config;
  }

  bool edgeIsSafe(const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
                  double start_yaw, double goal_yaw, double resolution,
                  double& minimum_fc_rp) const
  {
    const double maximum_delta =
        start.size() > 0 ? (goal - start).cwiseAbs().maxCoeff() : 0.0;
    const int subdivisions =
        std::max(1, static_cast<int>(std::ceil(maximum_delta / resolution)));
    for (int sample = 0; sample <= subdivisions; ++sample)
    {
      const double ratio = static_cast<double>(sample) / subdivisions;
      const double yaw = start_yaw + ratio * shortestYawDelta(start_yaw, goal_yaw);
      evaluator_->setRootLinkRotation(KDL::Rotation::RotZ(yaw));
      multilink_copilot::StabilityMetrics metrics;
      if (!evaluator_->evaluate(start + ratio * (goal - start), metrics) || !metrics.safe)
      {
        return false;
      }
      minimum_fc_rp = std::min(minimum_fc_rp, metrics.fc_rp_min);
    }
    return true;
  }

  void expectTimedPathIsSafe(const JointPlanResult& result,
                             const JointPlannerConfig& config,
                             size_t first_safe_waypoint = 0) const
  {
    ASSERT_GE(result.joint_waypoints.size(), 2u);
    ASSERT_LT(first_safe_waypoint, result.joint_waypoints.size());
    for (size_t index = first_safe_waypoint + 1;
         index < result.joint_waypoints.size(); ++index)
    {
      const TimedJointWaypoint& before = result.joint_waypoints[index - 1];
      const TimedJointWaypoint& after = result.joint_waypoints[index];
      const double interval = after.time - before.time;
      ASSERT_GT(interval, 0.0);
      const double maximum_delta =
          (after.positions - before.positions).cwiseAbs().maxCoeff();
      const int subdivisions = std::max(
          {1, static_cast<int>(std::ceil(maximum_delta / config.validity_resolution)),
           static_cast<int>(std::ceil(interval / 0.01))});
      for (int sample = index == first_safe_waypoint + 1 ? 0 : 1;
           sample <= subdivisions; ++sample)
      {
        const double ratio = static_cast<double>(sample) / subdivisions;
        const double time = before.time + ratio * interval;
        const Eigen::Quaterniond rotation(result.rootLinkRotation(time));
        evaluator_->setRootLinkRotation(KDL::Rotation::Quaternion(
            rotation.x(), rotation.y(), rotation.z(), rotation.w()));
        multilink_copilot::StabilityMetrics metrics;
        ASSERT_TRUE(evaluator_->evaluate(result.jointPositions(time), metrics));
        EXPECT_TRUE(metrics.safe) << "unsafe timed path at t=" << time;
        EXPECT_GE(metrics.fc_rp_min + 1e-4,
                  evaluator_->config().fc_rp_min_threshold);
      }
    }
    for (const TimedRootAttitudeWaypoint& waypoint : result.attitude_waypoints)
    {
      if (waypoint.time + 1e-12 <
          result.joint_waypoints[first_safe_waypoint].time)
      {
        continue;
      }
      const Eigen::Quaterniond rotation(linkRotation(waypoint.attitude));
      evaluator_->setRootLinkRotation(KDL::Rotation::Quaternion(
          rotation.x(), rotation.y(), rotation.z(), rotation.w()));
      multilink_copilot::StabilityMetrics metrics;
      ASSERT_TRUE(evaluator_->evaluate(result.jointPositions(waypoint.time), metrics));
      EXPECT_TRUE(metrics.safe) << "unsafe path at attitude breakpoint t=" << waypoint.time;
    }
  }

  void expectArcLengthTiming(const JointPlanResult& result) const
  {
    ASSERT_GE(result.joint_waypoints.size(), 2u);
    size_t path_start = 0;
    if (result.root_translation_delay > 1e-9)
    {
      while (path_start + 1 < result.joint_waypoints.size() &&
             result.joint_waypoints[path_start].time + 1e-9 <
                 result.root_translation_delay)
      {
        ++path_start;
      }
      ASSERT_NEAR(result.joint_waypoints[path_start].time,
                  result.root_translation_delay, 1e-8);
    }
    ASSERT_LT(path_start, result.joint_waypoints.size() - 1);
    std::vector<double> cumulative(result.joint_waypoints.size() - path_start, 0.0);
    for (size_t index = path_start + 1; index < result.joint_waypoints.size(); ++index)
    {
      cumulative[index - path_start] = cumulative[index - path_start - 1] +
          (result.joint_waypoints[index].positions -
           result.joint_waypoints[index - 1].positions).norm();
    }
    ASSERT_GT(cumulative.back(), 1e-9);
    const double path_start_time = result.joint_waypoints[path_start].time;
    for (size_t index = path_start; index < result.joint_waypoints.size(); ++index)
    {
      const double expected_time = path_start_time +
          (result.duration - path_start_time) *
              cumulative[index - path_start] / cumulative.back();
      EXPECT_NEAR(result.joint_waypoints[index].time, expected_time, 1e-8);
    }
  }

  std::unique_ptr<pluginlib::ClassLoader<aerial_robot_model::RobotModel>> loader_;
  boost::shared_ptr<Dragon::HydrusLikeRobotModel> model_;
  std::shared_ptr<multilink_copilot::StabilityEvaluator> evaluator_;
  std::unique_ptr<DragonModelInfo> info_;
  PrimitiveConfig primitive_;
};

class WholeBodyBatchPlanner : public WholeBodyJointBridge
{
protected:
  void SetUp() override
  {
    WholeBodyJointBridge::SetUp();
    ASSERT_FALSE(HasFatalFailure());
    config_.reset(new WholeBodyPlannerConfig(ros::NodeHandle("~")));
    config_->shared.primitive = primitive_;
    config_->shared.primitive.candidate_count = 3;
    config_->shared.common.dilateRadius = 0.0;
    config_->joint = jointConfig(1);
    environment_.reset(new PlanningEnvironment(config_->shared));

    std::vector<std::shared_ptr<multilink_copilot::StabilityEvaluator>> evaluators;
    for (int index = 0; index < config_->shared.primitive.candidate_count; ++index)
    {
      const auto model = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(
          loader_->createInstance("dragon/hydrus_like_robot_model"));
      ASSERT_TRUE(model);
      evaluators.push_back(std::make_shared<multilink_copilot::StabilityEvaluator>(
          model, config_->stability));
    }
    planner_.reset(new WholeBodyPlanner(*config_, info_->collisionGeometry(), evaluators));
    start_.position = Eigen::Vector3d(0.0, 0.0, 2.0);
    start_joints_.resize(6);
    start_joints_ << 0.0, M_PI_2, 0.0, M_PI_2, 0.0, M_PI_2;
    TrajectoryHistory history(config_->joint.follower);
    history.append(start_.position);
    context_ = makeNominalJointContext(history, *info_);
    batch_ = environment_->generate(start_, start_.position + Eigen::Vector3d::UnitX());
    ASSERT_TRUE(batch_.success()) << batch_.detail;
    ASSERT_EQ(batch_.candidates.size(), 3u);
    for (const Candidate& candidate : batch_.candidates)
    {
      ASSERT_NE(candidate.status, CandidateStatus::kGenerationFailed) << candidate.detail;
    }
  }

  std::unique_ptr<WholeBodyPlannerConfig> config_;
  std::unique_ptr<PlanningEnvironment> environment_;
  std::unique_ptr<WholeBodyPlanner> planner_;
  RootState start_;
  Eigen::VectorXd start_joints_;
  NominalJointContext context_;
  PrimitiveBatch batch_;
};

TEST_F(WholeBodyBatchPlanner, SelectsExecutableCandidateUsingTheCapturedMap)
{
  const auto occupancy = environment_->occupancySnapshot();
  environment_->replaceMap({start_.position});
  ASSERT_TRUE(environment_->occupied(start_.position));
  ASSERT_FALSE(occupancy->query(start_.position));

  const WholeBodyPlanResult result = planner_->plan(
      batch_, occupancy, start_joints_, RootAttitude{}, context_,
      ros::Time::now() + ros::Duration(2.0));
  ASSERT_EQ(result.candidates.size(), batch_.candidates.size());
  ASSERT_GE(result.selected, 0);
  ASSERT_LT(static_cast<size_t>(result.selected), result.candidates.size());
  const WholeBodyCandidate& selected = result.candidates[result.selected];
  EXPECT_EQ(selected.status, CandidateStatus::kSelected);
  ASSERT_TRUE(selected.joints.success) << selected.detail;
  EXPECT_GT(selected.joints.duration, 0.0);
  EXPECT_GE(selected.joints.minimum_fc_rp + 1e-4, config_->stability.fc_rp_min_threshold);
  EXPECT_TRUE(selected.joints.jointPositions(0.0).isApprox(start_joints_, 1e-8));
  EXPECT_TRUE(selected.scaled_root.getPos(0.0).isApprox(start_.position, 1e-8));
  EXPECT_TRUE(selected.scaled_root.getPos(selected.scaled_root.getTotalDuration())
                  .isApprox(batch_.local_target, 1e-8));
  EXPECT_NEAR(selected.joints.duration,
              selected.scaled_root.getTotalDuration() + selected.joints.root_translation_delay,
              1e-8);

  const auto cost = [this](const WholeBodyCandidate& candidate) {
    return candidate.joints.duration +
        config_->joint_motion_cost_weight * candidate.joints.joint_motion +
        config_->tracking_error_cost_weight * candidate.joints.tracking_error_rms;
  };
  int selected_count = 0;
  for (const WholeBodyCandidate& candidate : result.candidates)
  {
    selected_count += candidate.status == CandidateStatus::kSelected;
    if (candidate.status == CandidateStatus::kFeasible)
    {
      EXPECT_LE(cost(selected), cost(candidate) + 1e-6);
    }
  }
  EXPECT_EQ(selected_count, 1);
}

TEST_F(WholeBodyBatchPlanner, RejectsUnevaluatedCandidatesAfterTheSharedDeadline)
{
  batch_.candidates.front().status = CandidateStatus::kGenerationFailed;
  batch_.candidates.front().detail = "generation failed before joint planning";
  const WholeBodyPlanResult result = planner_->plan(
      batch_, environment_->occupancySnapshot(), start_joints_, RootAttitude{}, context_,
      ros::Time::now() - ros::Duration(1.0));
  EXPECT_EQ(result.selected, -1);
  ASSERT_EQ(result.candidates.size(), batch_.candidates.size());
  EXPECT_EQ(result.candidates.front().status, CandidateStatus::kGenerationFailed);
  EXPECT_EQ(result.candidates.front().detail, batch_.candidates.front().detail);
  for (size_t index = 1; index < result.candidates.size(); ++index)
  {
    const WholeBodyCandidate& candidate = result.candidates[index];
    EXPECT_EQ(candidate.status, CandidateStatus::kJointPlanningFailed);
    EXPECT_EQ(candidate.detail, "whole-body planning budget exhausted");
    EXPECT_FALSE(candidate.joints.success);
    EXPECT_TRUE(candidate.joints.joint_waypoints.empty());
    EXPECT_EQ(candidate.scaled_root.getPieceNum(), 0);
  }
}

// Targets behind the robot force the nominal follow-the-leader terminal shape
// across the infeasible near-straight band.  At least one global-RRT plan in
// each root-candidate batch must fit the node's shared online budget.
TEST_F(WholeBodyJointBridge, PlansEveryFreeSpaceTargetDirection)
{
  const Eigen::Vector3d start_position(0.0, 0.0, 1.0);
  Eigen::VectorXd start_joints(6);
  start_joints << 0.0, M_PI_2, 0.0, M_PI_2, 0.0, M_PI_2;
  const double start_yaw = 0.0;

  PrimitiveGenerator generator(primitive_);
  TrajectoryHistory history(jointConfig(1).follower);
  history.append(start_position);
  const NominalJointContext context = makeNominalJointContext(history, *info_);

  int total = 0;
  int planned = 0;
  double slowest_batch = 0.0;
  std::string first_failure;
  for (int direction = 0; direction < 24; ++direction)
  {
    const double angle = 2.0 * M_PI * direction / 24.0;
    for (const double distance : {1.0, 3.0})
    {
      const Eigen::Vector3d target =
          start_position + distance * Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0);
      Eigen::Matrix3d initial_state = Eigen::Matrix3d::Zero();
      initial_state.col(0) = start_position;
      Eigen::Matrix3d final_state = Eigen::Matrix3d::Zero();
      final_state.col(0) = target;
      const std::vector<Candidate> candidates = generator.generate(initial_state, final_state);

      ++total;
      bool success = false;
      const auto batch_start = std::chrono::steady_clock::now();
      for (size_t index = 0; index < candidates.size(); ++index)
      {
        if (candidates[index].status == CandidateStatus::kGenerationFailed)
        {
          continue;
        }
        // Emulate the shared wall-clock budget the whole-body node applies.
        const double remaining =
            kBatchBudget - std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - batch_start).count();
        if (remaining <= 0.0)
        {
          break;
        }
        JointTrajectoryPlanner planner(jointConfig(1 + static_cast<unsigned int>(index)), evaluator_);
        const JointPlanResult result = planner.plan(candidates[index].trajectory, context,
                                                    start_joints, start_yaw, remaining);
        // The node evaluates the whole batch before it picks a candidate, so the
        // batch cost and not the single-candidate cost is what has to fit.
        if (result.success)
        {
          success = true;
          EXPECT_GE(result.minimum_fc_rp + 1e-4, evaluator_->config().fc_rp_min_threshold);
          EXPECT_GT(result.duration, 0.0);
        }
        else if (first_failure.empty())
        {
          first_failure = result.detail;
        }
      }
      slowest_batch = std::max(
          slowest_batch,
          std::chrono::duration<double>(std::chrono::steady_clock::now() - batch_start).count());
      if (success)
      {
        ++planned;
      }
      else
      {
        std::printf("no whole-body joint plan for direction %.0f deg at %.1f m\n",
                    angle * 180.0 / M_PI, distance);
      }
    }
  }
  std::printf("planned %d/%d free-space targets, slowest batch %.0f ms\n", planned, total,
              1000.0 * slowest_batch);
  EXPECT_EQ(planned, total) << first_failure;
  EXPECT_LT(slowest_batch, 1.5 * kBatchBudget);
}

TEST_F(WholeBodyJointBridge, AllocatesCompoundYawPitchBeforeStationaryTranslation)
{
  const Eigen::Vector3d start_position(0.0, 0.0, 1.0);
  const Eigen::Vector3d target_position(2.0, 0.5, 1.4);
  Eigen::VectorXd start_joints(6);
  start_joints << 0.0, M_PI_2, 0.0, M_PI_2, 0.0, M_PI_2;
  JointPlannerConfig config = jointConfig(71);
  config.planning_timeout = 1.0;
  TrajectoryHistory history(config.follower);
  history.append(start_position);
  const NominalJointContext context = makeNominalJointContext(history, *info_);
  Eigen::Matrix3d initial_state = Eigen::Matrix3d::Zero();
  initial_state.col(0) = start_position;
  Eigen::Matrix3d final_state = Eigen::Matrix3d::Zero();
  final_state.col(0) = target_position;
  const std::vector<Candidate> candidates =
      PrimitiveGenerator(primitive_).generate(initial_state, final_state);

  JointPlanResult result;
  for (size_t index = 0; index < candidates.size() && !result.success; ++index)
  {
    JointTrajectoryPlanner planner(config, evaluator_);
    result = planner.plan(candidates[index].trajectory, context, start_joints,
                          RootAttitude(), 1.0);
  }
  ASSERT_TRUE(result.success) << result.detail;
  ASSERT_GT(result.root_translation_delay, 0.0);
  const RootAttitude allocated = result.attitude(result.root_translation_delay);
  EXPECT_GT(std::abs(allocated.yaw), 1e-3);
  EXPECT_GT(std::abs(allocated.pitch), 1e-3);

  for (const TimedJointWaypoint& waypoint : result.joint_waypoints)
  {
    if (waypoint.time > result.root_translation_delay + 1e-9)
    {
      break;
    }
    for (int index = 2; index < waypoint.positions.size(); ++index)
    {
      EXPECT_NEAR(waypoint.positions(index), start_joints(index), 1e-10);
    }
  }
  const std::vector<double>& lower = model_->getLinkJointLowerLimits();
  const std::vector<double>& upper = model_->getLinkJointUpperLimits();
  const Eigen::VectorXd expected_half =
      JointTrajectoryPlanner::joint1PriorityConfiguration(
          start_joints, 0, 1, lower, upper, RootAttitude(), allocated, 0.5);
  EXPECT_TRUE(result.jointPositions(0.5 * result.root_translation_delay)
                  .isApprox(expected_half, 1e-9));

  const int samples = 100;
  for (int sample = 0; sample < samples; ++sample)
  {
    const double time = result.root_translation_delay *
                        (static_cast<double>(sample) + 0.5) / samples;
    EXPECT_LE(std::abs(result.yawRate(time)), config.follower.max_angular_vel + 1e-9);
    EXPECT_LE(std::abs(result.pitchRate(time)), config.follower.max_angular_vel + 1e-9);
  }
  for (size_t index = 1; index < result.joint_waypoints.size(); ++index)
  {
    if (result.joint_waypoints[index].time > result.root_translation_delay + 1e-9)
    {
      break;
    }
    const TimedJointWaypoint& before = result.joint_waypoints[index - 1];
    const TimedJointWaypoint& after = result.joint_waypoints[index];
    const double interval = after.time - before.time;
    ASSERT_GT(interval, 0.0);
    const double maximum_rate =
        (after.positions - before.positions).cwiseAbs().maxCoeff() / interval;
    EXPECT_LE(maximum_rate, config.follower.max_angular_vel + 1e-9);
    EXPECT_LE(maximum_rate / config.follower.command_hz,
              config.max_joint_command_step + 1e-9);
  }
}

TEST_F(WholeBodyJointBridge, MovingRetargetKeepsRootBoundaryStateWithoutTimeScaling)
{
  const Eigen::Vector3d start_position(0.0, 0.0, 1.0);
  Eigen::VectorXd start_joints(6);
  start_joints << 0.0, M_PI_2, 0.0, M_PI_2, 0.0, M_PI_2;
  JointPlannerConfig config = jointConfig(83);
  config.planning_timeout = 1.0;
  TrajectoryHistory history(config.follower);
  history.append(start_position);
  const NominalJointContext context = makeNominalJointContext(history, *info_);
  Eigen::Matrix3d initial_state = Eigen::Matrix3d::Zero();
  initial_state.col(0) = start_position;
  initial_state.col(1) = Eigen::Vector3d(0.2, 0.0, 0.0);
  initial_state.col(2) = Eigen::Vector3d(0.05, 0.0, 0.0);
  Eigen::Matrix3d final_state = Eigen::Matrix3d::Zero();
  final_state.col(0) = Eigen::Vector3d(3.0, 0.8, 1.3);
  const std::vector<Candidate> candidates =
      PrimitiveGenerator(primitive_).generate(initial_state, final_state);

  JointPlanResult result;
  const Candidate* selected = nullptr;
  for (size_t index = 0; index < candidates.size() && !result.success; ++index)
  {
    JointTrajectoryPlanner planner(config, evaluator_);
    result = planner.plan(candidates[index].trajectory, context, start_joints,
                          RootAttitude(), 1.0);
    if (result.success)
    {
      selected = &candidates[index];
    }
  }
  ASSERT_TRUE(result.success) << result.detail;
  ASSERT_NE(selected, nullptr);
  EXPECT_DOUBLE_EQ(result.root_translation_delay, 0.0);
  EXPECT_DOUBLE_EQ(result.time_scale, 1.0);
  EXPECT_TRUE(selected->trajectory.getPos(0.0).isApprox(initial_state.col(0), 1e-9));
  EXPECT_TRUE(selected->trajectory.getVel(0.0).isApprox(initial_state.col(1), 1e-9));
  EXPECT_TRUE(selected->trajectory.getAcc(0.0).isApprox(initial_state.col(2), 1e-9));
}

// A straight interpolation between the mirrored folds crosses the infeasible
// near-straight band.  The test intentionally makes no assumptions about the
// topology or waypoint count of the RRT detour.
TEST_F(WholeBodyJointBridge, GlobalRrtConnectsFoldFlipWithFullySafeEdges)
{
  Eigen::VectorXd positive(6);
  Eigen::VectorXd negative(6);
  positive << 0.0, M_PI_2, 0.0, M_PI_2, 0.0, M_PI_2;
  negative << 0.0, -M_PI_2, 0.0, -M_PI_2, 0.0, -M_PI_2;

  JointPlannerConfig config = jointConfig(7);
  JointTrajectoryPlanner planner(config, evaluator_);
  double direct_minimum = std::numeric_limits<double>::infinity();
  ASSERT_FALSE(edgeIsSafe(positive, negative, M_PI, M_PI,
                          config.validity_resolution, direct_minimum));

  std::vector<Eigen::VectorXd> path;
  double minimum_fc_rp = 0.0;
  std::string failure;
  ASSERT_TRUE(planner.planStableConnection(positive, negative, M_PI, path, minimum_fc_rp, &failure))
      << failure;
  ASSERT_GT(path.size(), 2u);
  EXPECT_TRUE(path.front().isApprox(positive, 1e-9));
  EXPECT_TRUE(path.back().isApprox(negative, 1e-9));
  EXPECT_GE(minimum_fc_rp + 1e-4, evaluator_->config().fc_rp_min_threshold);
  for (size_t index = 1; index < path.size(); ++index)
  {
    double edge_minimum = std::numeric_limits<double>::infinity();
    ASSERT_TRUE(edgeIsSafe(path[index - 1], path[index], M_PI, M_PI,
                           config.validity_resolution, edge_minimum))
        << "unsafe RRT edge " << index;
  }
}

// The bridge must never be reported as safe without full-resolution validation.
TEST_F(WholeBodyJointBridge, RejectsAConnectionToAnInfeasibleEndpoint)
{
  Eigen::VectorXd positive(6);
  Eigen::VectorXd straight = Eigen::VectorXd::Zero(6);
  positive << 0.0, M_PI_2, 0.0, M_PI_2, 0.0, M_PI_2;
  evaluator_->setRootLinkRotation(KDL::Rotation::RotZ(M_PI));
  multilink_copilot::StabilityMetrics metrics;
  ASSERT_TRUE(evaluator_->evaluate(straight, metrics));
  ASSERT_FALSE(metrics.safe) << "the straight shape is expected to be flight-infeasible";

  JointTrajectoryPlanner planner(jointConfig(3), evaluator_);
  std::vector<Eigen::VectorXd> path;
  double minimum_fc_rp = 0.0;
  std::string failure;
  EXPECT_FALSE(planner.planStableConnection(positive, straight, M_PI, path, minimum_fc_rp, &failure));
  EXPECT_TRUE(path.empty());
  EXPECT_FALSE(failure.empty());
}

TEST_F(WholeBodyJointBridge, TrackingCostSelectsTheMinimumComputedPlanCost)
{
  const Eigen::Vector3d history_start(0.280, 0.986, 0.769);
  const Eigen::Vector3d start_position(2.554, 1.008, 0.500);
  const Eigen::Vector3d target_position(-0.345, 0.256, 0.677);
  Eigen::VectorXd start_joints(6);
  start_joints << -0.0856703, 0.206497, -0.0231212, 0.334580,
                  -0.0212780, 0.133864;
  const double start_yaw = -0.0186;

  JointPlannerConfig config = jointConfig(1);
  config.planning_timeout = 1.0;
  TrajectoryHistory history(config.follower);
  for (int index = 0; index <= 80; ++index)
  {
    const double ratio = static_cast<double>(index) / 80.0;
    history.append(history_start + ratio * (start_position - history_start));
  }
  const NominalJointContext context = makeNominalJointContext(history, *info_);

  Eigen::Matrix3d initial_state = Eigen::Matrix3d::Zero();
  initial_state.col(0) = start_position;
  Eigen::Matrix3d final_state = Eigen::Matrix3d::Zero();
  final_state.col(0) = target_position;
  const std::vector<Candidate> candidates =
      PrimitiveGenerator(primitive_).generate(initial_state, final_state);
  ASSERT_EQ(candidates.size(), 9u);
  std::vector<JointPlanResult> results;
  std::vector<WholeBodyCandidateScore> scores;
  results.reserve(candidates.size());
  scores.reserve(candidates.size());
  for (size_t index = 0; index < candidates.size(); ++index)
  {
    config.random_seed = 1 + static_cast<unsigned int>(index);
    JointTrajectoryPlanner planner(config, evaluator_);
    results.push_back(planner.plan(candidates[index].trajectory, context,
                                   start_joints, start_yaw, 1.0));
    const JointPlanResult& result = results.back();
    scores.push_back({result.success, result.duration, result.joint_motion,
                      candidates[index].jerk_energy, result.tracking_error_rms});
  }

  constexpr double kJointMotionWeight = 0.25;
  constexpr double kTrackingErrorWeight = 6.0;
  const int winner = selectBestWholeBodyCandidate(
      scores, kJointMotionWeight, kTrackingErrorWeight);
  ASSERT_GE(winner, 0);
  const WholeBodyCandidateScore& selected = scores[static_cast<size_t>(winner)];
  const double selected_cost =
      selected.duration + kJointMotionWeight * selected.joint_motion +
      kTrackingErrorWeight * selected.tracking_error_rms;
  for (size_t index = 0; index < scores.size(); ++index)
  {
    const WholeBodyCandidateScore& score = scores[index];
    if (!score.feasible)
    {
      continue;
    }
    const double cost = score.duration + kJointMotionWeight * score.joint_motion +
                        kTrackingErrorWeight * score.tracking_error_rms;
    EXPECT_LE(selected_cost, cost + 1e-9) << "candidate " << index;
  }
  EXPECT_TRUE(std::isfinite(results[static_cast<size_t>(winner)].tracking_error_rms));
  EXPECT_TRUE(std::isfinite(results[static_cast<size_t>(winner)].tracking_error_max));
}

TEST_F(WholeBodyJointBridge, ProjectsTerminalAndMapsGlobalRrtAcrossTheRootHorizon)
{
  const Eigen::Vector3d start_position(0.0, 0.0, 1.0);
  const Eigen::Vector3d target_position(0.0, -3.0, 1.0);
  Eigen::VectorXd start_joints(6);
  start_joints << 0.0, M_PI_2, 0.0, M_PI_2, 0.0, M_PI_2;

  JointPlannerConfig config = jointConfig(31);
  config.planning_timeout = 1.0;
  TrajectoryHistory history(config.follower);
  history.append(start_position);
  const NominalJointContext context = makeNominalJointContext(history, *info_);
  Eigen::Matrix3d initial_state = Eigen::Matrix3d::Zero();
  initial_state.col(0) = start_position;
  Eigen::Matrix3d final_state = Eigen::Matrix3d::Zero();
  final_state.col(0) = target_position;
  const Candidate candidate =
      PrimitiveGenerator(primitive_).generate(initial_state, final_state).front();

  const std::vector<NominalJointSample> nominal =
      NominalJointPredictor(config.follower).predict(
          candidate.trajectory, context, start_joints, 0.0, config.reference_dt);
  ASSERT_GE(nominal.size(), 2u);
  evaluator_->setRootLinkRotation(KDL::Rotation::RotZ(nominal.back().yaw + M_PI));
  multilink_copilot::StabilityMetrics nominal_terminal_metrics;
  ASSERT_TRUE(evaluator_->evaluate(nominal.back().joints, nominal_terminal_metrics));
  ASSERT_FALSE(nominal_terminal_metrics.safe);
  Eigen::VectorXd projected_terminal;
  ASSERT_TRUE(evaluator_->projectToSafe(
      nominal.back().joints, start_joints, projected_terminal, false));

  JointTrajectoryPlanner planner(config, evaluator_);
  const JointPlanResult result =
      planner.plan(candidate.trajectory, context, start_joints, 0.0, 1.0);
  ASSERT_TRUE(result.success) << result.detail;
  ASSERT_GE(result.joint_waypoints.size(), 2u);
  EXPECT_TRUE(result.joint_waypoints.front().positions.isApprox(start_joints, 1e-9));
  EXPECT_TRUE(result.joint_waypoints.back().positions.isApprox(projected_terminal, 1e-8));
  EXPECT_NEAR(result.joint_waypoints.front().time, 0.0, 1e-12);
  EXPECT_NEAR(result.joint_waypoints.back().time, result.duration, 1e-9);
  expectArcLengthTiming(result);

  for (size_t index = 1; index < result.joint_waypoints.size(); ++index)
  {
    const TimedJointWaypoint& before = result.joint_waypoints[index - 1];
    const TimedJointWaypoint& after = result.joint_waypoints[index];
    const double duration = after.time - before.time;
    ASSERT_GT(duration, 0.0);
    const double maximum_rate =
        (after.positions - before.positions).cwiseAbs().maxCoeff() / duration;
    EXPECT_LE(maximum_rate, config.follower.max_angular_vel + 1e-9);
    EXPECT_LE(maximum_rate / config.follower.command_hz,
              config.max_joint_command_step + 1e-9);
  }
  EXPECT_TRUE(std::isfinite(result.tracking_error_rms));
  EXPECT_TRUE(std::isfinite(result.tracking_error_max));
  expectTimedPathIsSafe(result, config);
}

TEST_F(WholeBodyJointBridge, RepairsMeasuredStartAndPreservesTheCommandHandover)
{
  Eigen::VectorXd safe_fold(6);
  safe_fold << 0.0, M_PI_2, 0.0, M_PI_2, 0.0, M_PI_2;

  // Find a deterministic configuration just outside the feasible set.  Starting
  // close to its boundary gives the generic QP an unstable seed that it can
  // repair without supplying any hand-crafted positive/negative fold reference.
  Eigen::VectorXd measured_start;
  Eigen::VectorXd repaired_start;
  bool repairable_start_found = false;
  evaluator_->setRootLinkRotation(KDL::Rotation::RotZ(M_PI));
  for (int step = 1; step <= 100 && !repairable_start_found; ++step)
  {
    const double ratio = 1.0 - static_cast<double>(step) / 100.0;
    const Eigen::VectorXd candidate = ratio * safe_fold;
    multilink_copilot::StabilityMetrics metrics;
    ASSERT_TRUE(evaluator_->evaluate(candidate, metrics));
    if (!metrics.safe &&
        evaluator_->projectToSafe(candidate, candidate, repaired_start, true))
    {
      measured_start = candidate;
      repairable_start_found = true;
    }
  }
  ASSERT_TRUE(repairable_start_found);
  multilink_copilot::StabilityMetrics repaired_metrics;
  ASSERT_TRUE(evaluator_->evaluate(repaired_start, repaired_metrics));
  ASSERT_TRUE(repaired_metrics.safe);

  const Eigen::Vector3d start_position(0.0, 0.0, 1.0);
  const Eigen::Vector3d target_position(1.0, 0.0, 1.0);
  JointPlannerConfig config = jointConfig(43);
  config.planning_timeout = 1.0;
  TrajectoryHistory history(config.follower);
  history.append(start_position);
  const NominalJointContext context = makeNominalJointContext(history, *info_);
  Eigen::Matrix3d initial_state = Eigen::Matrix3d::Zero();
  initial_state.col(0) = start_position;
  Eigen::Matrix3d final_state = Eigen::Matrix3d::Zero();
  final_state.col(0) = target_position;
  const Candidate candidate =
      PrimitiveGenerator(primitive_).generate(initial_state, final_state).front();

  JointTrajectoryPlanner planner(config, evaluator_);
  const JointPlanResult result =
      planner.plan(candidate.trajectory, context, measured_start, 0.0, 1.0);
  ASSERT_TRUE(result.success) << result.detail;
  ASSERT_GE(result.joint_waypoints.size(), 3u);
  EXPECT_TRUE(result.joint_waypoints.front().positions.isApprox(measured_start, 1e-9));
  EXPECT_NEAR(result.joint_waypoints.front().time, 0.0, 1e-12);

  size_t repaired_index = result.joint_waypoints.size();
  for (size_t index = 1; index < result.joint_waypoints.size(); ++index)
  {
    if (result.joint_waypoints[index].positions.isApprox(repaired_start, 1e-8))
    {
      repaired_index = index;
      break;
    }
  }
  ASSERT_LT(repaired_index, result.joint_waypoints.size());
  EXPECT_GT(result.joint_waypoints[repaired_index].time, 0.0);
  expectArcLengthTiming(result);
  expectTimedPathIsSafe(result, config, repaired_index);
}

TEST_F(WholeBodyJointBridge, RejectsCandidateWhenTerminalProjectionFails)
{
  const Eigen::Vector3d start_position(0.0, 0.0, 1.0);
  Eigen::VectorXd start_joints(6);
  start_joints << 0.0, M_PI_2, 0.0, M_PI_2, 0.0, M_PI_2;

  JointPlannerConfig config = jointConfig(47);
  config.planning_timeout = 1.0;
  // A deliberately non-finite terminal velocity makes the terminal stability
  // frame invalid and forces the shared QP to reject its otherwise safe
  // reference. This exercises the candidate-level failure path.
  TrajectoryHistory history(config.follower);
  history.append(start_position);
  const NominalJointContext context = makeNominalJointContext(history, *info_);

  Piece<5>::CoefficientMat coefficients = Piece<5>::CoefficientMat::Zero();
  coefficients.col(5) = start_position;
  const double maximum = std::numeric_limits<double>::max();
  coefficients(0, 4) = maximum;
  coefficients(0, 3) = -maximum;
  coefficients(0, 2) = maximum;
  const Trajectory<5> malformed_root(
      std::vector<double>{1.0},
      std::vector<Piece<5>::CoefficientMat>{coefficients});

  JointTrajectoryPlanner planner(config, evaluator_);
  const JointPlanResult result =
      planner.plan(malformed_root, context, start_joints, 0.0, 1.0);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.detail, "failed to build nominal follow-the-leader joint samples");
}
}  // namespace
}  // namespace motion_primitive_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "whole_body_joint_bridge_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
