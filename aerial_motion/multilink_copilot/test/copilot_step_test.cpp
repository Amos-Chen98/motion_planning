#include <gtest/gtest.h>
#include <multilink_copilot/copilot.h>
#include <tf/transform_datatypes.h>
#include <ros/console.h>

namespace multilink_copilot
{
class CopilotStepTest : public ::testing::Test
{
protected:
  std::unique_ptr<CopilotPlanner> planner;
  Eigen::VectorXd measured;
  geometry_msgs::PoseStamped pose;
  void SetUp() override
  {
    planner.reset(new CopilotPlanner(false));
    pose.pose.position.z = 1;
    pose.pose.orientation.w = 1; // FLU
    planner->stability_evaluator_->setRootLinkRotation(KDL::Rotation::RotZ(M_PI));
    bool found = false;
    for (int block = 1; block < 4 && !found; ++block)
      for (int a = 1; a <= 15 && !found; ++a)
        for (int sign : {1, -1})
        {
          measured = Eigen::VectorXd::Zero(planner->link_joint_num_);
          for (int j = 0; j < 3; ++j)
            measured(planner->yaw_joint_local_indices_[j]) = sign * (j / block % 2 ? -1 : 1) * a * 0.1;
          StabilityMetrics metrics;
          if (planner->stability_evaluator_->evaluate(measured, metrics) && metrics.safe)
          { found = true; break; }
        }
    ASSERT_TRUE(found);
  }
  Eigen::VectorXd publishedState() { return planner->latest_published_joint_positions_; }
  bool hasPublished() { return planner->has_latest_published_joint_positions_; }
  void impossibleFeasibility()
  {
    auto config = planner->stability_evaluator_->config();
    config.fc_rp_min_threshold = 1e9;
    planner->stability_evaluator_.reset(new StabilityEvaluator(planner->dragon_robot_model_, config));
  }
  void forceFinalTiltRejection() { planner->max_baselink_tilt_before_publish_ = 1e-12; }
};

TEST_F(CopilotStepTest, PublishedThenFilteredRetainsPublishedState)
{
  auto first = planner->step(pose, measured);
  ASSERT_TRUE(first.published()) << first.status;
  measured = Eigen::Map<const Eigen::VectorXd>(first.target.joint_state.position.data(), measured.size());
  const auto before = publishedState();
  const auto second = planner->step(pose, measured);
  EXPECT_EQ(second.status, "filtered");
  EXPECT_TRUE(publishedState().isApprox(before));
  EXPECT_GE(second.total_ms, second.feasibility_ms);
  EXPECT_GE(first.total_ms, first.planning_ms);
}

TEST_F(CopilotStepTest, FailedProjectionDoesNotCommitPublication)
{
  impossibleFeasibility();
  const auto result = planner->step(pose, measured);
  EXPECT_EQ(result.status, "feasibility_failed");
  EXPECT_FALSE(hasPublished());
  EXPECT_GT(result.projection_calls, 0);
  EXPECT_GT(result.qp_solves, 0);
  EXPECT_GE(result.total_ms, result.feasibility_ms);
}

TEST_F(CopilotStepTest, FinalSafetyGateDoesNotCommitPublication)
{
  forceFinalTiltRejection();
  pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.01, 0.0, 0.0);
  const auto result = planner->step(pose, measured);
  EXPECT_EQ(result.status, "safety_rejected");
  EXPECT_FALSE(hasPublished());
  EXPECT_GE(result.total_ms, result.feasibility_ms);
}

TEST_F(CopilotStepTest, RejectsWrongMeasuredDimension)
{
  EXPECT_THROW(planner->step(pose, Eigen::VectorXd::Zero(1)), std::invalid_argument);
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "copilot_step_test");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
  ros::console::notifyLoggerLevelsChanged();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
