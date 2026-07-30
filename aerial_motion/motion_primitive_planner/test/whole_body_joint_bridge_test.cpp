// Regression coverage for the joint-space bridge search: every free-space target
// direction must yield an executable whole-body joint trajectory, and the
// archetypal fold flip must be repaired by single-joint moves.
#include <motion_primitive_planner/joint_trajectory_planner.h>
#include <motion_primitive_planner/planner_common.h>

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

  //! Mirrors config/whole_body_motion_primitive_planner.yaml.
  JointPlannerConfig jointConfig(unsigned int seed) const
  {
    JointPlannerConfig config;
    config.follower.command_hz = 40.0;
    config.follower.trajectory_sample_interval = 0.05;
    config.follower.trajectory_buffer_max_length = 10.0;
    config.follower.ik_singularity_threshold = 0.10;
    config.follower.max_yaw_rate = 1.5;
    config.reference_dt = 0.10;
    config.planning_timeout = 0.15;
    config.bridge_timeout = 0.12;
    config.anchor_backoff_time = 0.50;
    config.validity_resolution = 0.025;
    config.max_joint_velocity = 4.0;
    config.max_joint_command_step = 0.10;
    config.random_seed = seed;
    return config;
  }

  std::unique_ptr<pluginlib::ClassLoader<aerial_robot_model::RobotModel>> loader_;
  boost::shared_ptr<Dragon::HydrusLikeRobotModel> model_;
  std::shared_ptr<multilink_copilot::StabilityEvaluator> evaluator_;
  std::unique_ptr<DragonModelInfo> info_;
  PrimitiveConfig primitive_;
};

// Targets behind the robot force the follow-the-leader shape through the
// infeasible near-straight band; before the anchor back-off and the structured
// detour search these made every candidate fail and the node hover-hold.
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

// A straight interpolation between the two mirrored folds crosses the infeasible
// near-straight band, so the bridge has to re-fold one joint at a time.
TEST_F(WholeBodyJointBridge, BridgesTheFoldFlipWithSingleJointMoves)
{
  Eigen::VectorXd positive(6);
  Eigen::VectorXd negative(6);
  positive << 0.0, M_PI_2, 0.0, M_PI_2, 0.0, M_PI_2;
  negative << 0.0, -M_PI_2, 0.0, -M_PI_2, 0.0, -M_PI_2;

  JointPlannerConfig config = jointConfig(7);
  JointTrajectoryPlanner planner(config, evaluator_);
  std::vector<Eigen::VectorXd> path;
  double minimum_fc_rp = 0.0;
  std::string failure;
  ASSERT_TRUE(planner.planStableConnection(positive, negative, M_PI, path, minimum_fc_rp, &failure))
      << failure;
  ASSERT_EQ(path.size(), 4u);
  EXPECT_TRUE(path.front().isApprox(positive, 1e-9));
  EXPECT_TRUE(path.back().isApprox(negative, 1e-9));
  EXPECT_GE(minimum_fc_rp + 1e-4, evaluator_->config().fc_rp_min_threshold);
  for (size_t index = 1; index < path.size(); ++index)
  {
    const Eigen::VectorXd delta = (path[index] - path[index - 1]).cwiseAbs();
    int moved = 0;
    for (int joint = 0; joint < delta.size(); ++joint)
    {
      moved += delta(joint) > 1e-9 ? 1 : 0;
    }
    EXPECT_EQ(moved, 1) << "bridge leg " << index << " is not a single-joint fold";
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

TEST_F(WholeBodyJointBridge, TrackingCostPrefersCompactTurnaround)
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

  const int duration_and_joint_winner =
      selectBestWholeBodyCandidate(scores, 0.25, 0.0);
  const int tracking_aware_winner =
      selectBestWholeBodyCandidate(scores, 0.25, 6.0);
  ASSERT_GE(duration_and_joint_winner, 0);
  ASSERT_GE(tracking_aware_winner, 0);
  EXPECT_NE(tracking_aware_winner, duration_and_joint_winner);
  EXPECT_LT(results[static_cast<size_t>(tracking_aware_winner)].tracking_error_rms, 0.5);
  EXPECT_LT(results[static_cast<size_t>(tracking_aware_winner)].tracking_error_rms,
            0.8 * results[static_cast<size_t>(duration_and_joint_winner)].tracking_error_rms);
  EXPECT_LT(results[static_cast<size_t>(tracking_aware_winner)].tracking_error_max,
            0.8 * results[static_cast<size_t>(duration_and_joint_winner)].tracking_error_max);
}

TEST_F(WholeBodyJointBridge, TerminalProjectionReachesSafeFoldBeforeRootHorizon)
{
  const Eigen::Vector3d start_position(0.0, 0.0, 1.0);
  const Eigen::Vector3d target_position(0.0, -3.0, 1.0);
  Eigen::VectorXd start_joints(6);
  start_joints << 0.0, M_PI_2, 0.0, M_PI_2, 0.0, M_PI_2;

  JointPlannerConfig config = jointConfig(31);
  config.planning_timeout = 1.0;
  config.bridge_timeout = 0.6;
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
      planner.plan(candidate.trajectory, context, start_joints, 0.0, 1.0);
  ASSERT_TRUE(result.success) << result.detail;
  ASSERT_GE(result.joint_waypoints.size(), 3u);
  size_t motion_end = 0;
  for (size_t index = 1; index < result.joint_waypoints.size(); ++index)
  {
    const TimedJointWaypoint& before = result.joint_waypoints[index - 1];
    const TimedJointWaypoint& after = result.joint_waypoints[index];
    const double duration = after.time - before.time;
    ASSERT_GT(duration, 0.0);
    const double maximum_rate =
        (after.positions - before.positions).cwiseAbs().maxCoeff() / duration;
    EXPECT_LE(maximum_rate, config.max_joint_velocity + 1e-9);
    EXPECT_LE(maximum_rate / config.follower.command_hz,
              config.max_joint_command_step + 1e-9);
    if (!(after.positions - before.positions).isZero(1e-9))
    {
      motion_end = index;
    }
  }
  ASSERT_GT(motion_end, 0u);
  EXPECT_LT(result.joint_waypoints[motion_end].time, 0.3 * result.duration);
  EXPECT_TRUE(result.joint_waypoints.back().positions.isApprox(
      result.joint_waypoints[motion_end].positions, 1e-12));
  EXPECT_NEAR(result.joint_waypoints.back().time, result.duration, 1e-9);
  EXPECT_TRUE(std::isfinite(result.tracking_error_rms));
  EXPECT_TRUE(std::isfinite(result.tracking_error_max));

  const int stability_samples =
      std::max(1, static_cast<int>(std::ceil(result.duration / 0.01)));
  for (int sample = 0; sample <= stability_samples; ++sample)
  {
    const double time =
        result.duration * static_cast<double>(sample) / stability_samples;
    evaluator_->setRootLinkRotation(KDL::Rotation::RotZ(result.yaw(time) + M_PI));
    multilink_copilot::StabilityMetrics metrics;
    ASSERT_TRUE(evaluator_->evaluate(result.jointPositions(time), metrics));
    EXPECT_TRUE(metrics.safe) << "unsafe recovery at t=" << time;
    EXPECT_GE(metrics.fc_rp_min + 1e-4,
              evaluator_->config().fc_rp_min_threshold);
  }
}
}  // namespace
}  // namespace motion_primitive_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "whole_body_joint_bridge_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
