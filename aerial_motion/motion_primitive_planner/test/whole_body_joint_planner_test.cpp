#include <motion_primitive_planner/joint_trajectory_planner.h>

#include <dragon/model/hydrus_like_robot_model.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace motion_primitive_planner
{
namespace
{
class WholeBodyJointPlannerIntegration : public ::testing::Test
{
protected:
  void SetUp() override
  {
    loader_.reset(new pluginlib::ClassLoader<aerial_robot_model::RobotModel>(
        "aerial_robot_model", "aerial_robot_model::RobotModel"));
    const auto base_model = loader_->createInstance("dragon/hydrus_like_robot_model");
    model_ = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(base_model);
    ASSERT_TRUE(model_);

    multilink_copilot::StabilityConfig stability_config;
    stability_config.fc_rp_min_threshold = 3.2;
    stability_config.static_thrust_min = 2.0;
    stability_config.static_thrust_max = 30.0;
    stability_config.overlap_min_clearance = 0.01;
    stability_config.max_baselink_tilt = 1.2;
    evaluator_ = std::make_shared<multilink_copilot::StabilityEvaluator>(model_, stability_config);
    evaluator_->setRootLinkRotation(KDL::Rotation::RotZ(M_PI));
  }

  bool edgeIsSafe(const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
                  double& minimum_fc_rp) const
  {
    const int subdivisions = std::max(
        1, static_cast<int>(std::ceil((goal - start).cwiseAbs().maxCoeff() / 0.025)));
    for (int sample = 0; sample <= subdivisions; ++sample)
    {
      const double ratio = static_cast<double>(sample) / subdivisions;
      multilink_copilot::StabilityMetrics metrics;
      if (!evaluator_->evaluate(start + ratio * (goal - start), metrics) || !metrics.safe)
      {
        return false;
      }
      minimum_fc_rp = std::min(minimum_fc_rp, metrics.fc_rp_min);
    }
    return true;
  }

  std::unique_ptr<pluginlib::ClassLoader<aerial_robot_model::RobotModel>> loader_;
  boost::shared_ptr<Dragon::HydrusLikeRobotModel> model_;
  std::shared_ptr<multilink_copilot::StabilityEvaluator> evaluator_;
};

TEST_F(WholeBodyJointPlannerIntegration, RejectsLinearYawFlipAndFindsStableDetour)
{
  Eigen::VectorXd positive(6);
  Eigen::VectorXd negative(6);
  positive << 0.0, M_PI_2, 0.0, M_PI_2, 0.0, M_PI_2;
  negative << 0.0, -M_PI_2, 0.0, -M_PI_2, 0.0, -M_PI_2;

  double linear_minimum = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(edgeIsSafe(positive, negative, linear_minimum));

  Eigen::VectorXd first_fold = positive;
  Eigen::VectorXd second_fold = positive;
  first_fold(1) = -M_PI_2;
  second_fold(1) = -M_PI_2;
  second_fold(3) = -M_PI_2;
  const std::vector<Eigen::VectorXd> known_detour = {positive, first_fold, second_fold, negative};
  double known_minimum = std::numeric_limits<double>::infinity();
  for (size_t index = 1; index < known_detour.size(); ++index)
  {
    ASSERT_TRUE(edgeIsSafe(known_detour[index - 1], known_detour[index], known_minimum));
  }
  EXPECT_GE(known_minimum + 1e-4, 3.2);

  JointPlannerConfig planner_config;
  planner_config.planning_timeout = 1.0;
  planner_config.validity_resolution = 0.025;
  planner_config.random_seed = 11;
  JointTrajectoryPlanner planner(planner_config, evaluator_);
  std::vector<Eigen::VectorXd> planned_detour;
  double planned_minimum = 0.0;
  std::string failure;
  ASSERT_TRUE(planner.planStableConnection(positive, negative, M_PI, planned_detour,
                                           planned_minimum, &failure)) << failure;
  ASSERT_GT(planned_detour.size(), 2u);
  EXPECT_GE(planned_minimum + 1e-4, 3.2);
  for (size_t index = 1; index < planned_detour.size(); ++index)
  {
    double edge_minimum = std::numeric_limits<double>::infinity();
    ASSERT_TRUE(edgeIsSafe(planned_detour[index - 1], planned_detour[index], edge_minimum));
  }
}

TEST_F(WholeBodyJointPlannerIntegration, MatchesLegacyMetricsAndProjectsToSafeConfiguration)
{
  const auto legacy_base = loader_->createInstance("dragon/hydrus_like_robot_model");
  const auto legacy_model = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(legacy_base);
  ASSERT_TRUE(legacy_model);
  const std::vector<int> joint_indices = legacy_model->getLinkJointIndices();
  const std::vector<Eigen::VectorXd> configurations = {
      (Eigen::VectorXd(6) << 0.0, M_PI_2, 0.0, M_PI_2, 0.0, M_PI_2).finished(),
      (Eigen::VectorXd(6) << 0.0, -M_PI_2, 0.0, M_PI_2, 0.0, M_PI_2).finished(),
      (Eigen::VectorXd(6) << 0.0, -M_PI_2, 0.0, -M_PI_2, 0.0, -M_PI_2).finished()};
  for (const Eigen::VectorXd& joints : configurations)
  {
    multilink_copilot::StabilityMetrics shared_metrics;
    ASSERT_TRUE(evaluator_->evaluate(joints, shared_metrics));

    KDL::JntArray full_joints = legacy_model->getJointPositions();
    if (full_joints.rows() != legacy_model->getTree().getNrOfJoints())
    {
      full_joints.resize(legacy_model->getTree().getNrOfJoints());
    }
    for (int index = 0; index < joints.size(); ++index)
    {
      full_joints(joint_indices[static_cast<size_t>(index)]) = joints(index);
    }
    const KDL::Frame root_to_baselink = legacy_model->forwardKinematics<KDL::Frame>(
        legacy_model->getBaselinkName(), full_joints);
    legacy_model->setCogDesireOrientation(KDL::Rotation::RotZ(M_PI) * root_to_baselink.M);
    legacy_model->updateRobotModel(full_joints);
    legacy_model->updateJacobians(full_joints, false);

    EXPECT_NEAR(shared_metrics.fc_rp_min, legacy_model->getFeasibleControlRollPitchMin(), 1e-8);
    EXPECT_NEAR(shared_metrics.fc_t_min, legacy_model->getFeasibleControlTMin(), 1e-8);
    EXPECT_NEAR(shared_metrics.static_thrust_min, legacy_model->getStaticThrust().minCoeff(), 1e-8);
    EXPECT_NEAR(shared_metrics.static_thrust_max, legacy_model->getStaticThrust().maxCoeff(), 1e-8);
    EXPECT_NEAR(shared_metrics.overlap_clearance,
                legacy_model->getClosestRotorDist() - 2.0 * legacy_model->getEdfRadius(), 1e-8);
  }

  const Eigen::VectorXd desired = Eigen::VectorXd::Zero(6);
  const Eigen::VectorXd reference = configurations.front();
  Eigen::VectorXd projected;
  ASSERT_TRUE(evaluator_->projectToSafe(desired, reference, projected));
  multilink_copilot::StabilityMetrics projected_metrics;
  ASSERT_TRUE(evaluator_->evaluate(projected, projected_metrics));
  EXPECT_TRUE(projected_metrics.safe);
  EXPECT_GE(projected_metrics.fc_rp_min + 1e-4, 3.2);
  EXPECT_LT((projected - desired).norm(), (reference - desired).norm());
}

TEST_F(WholeBodyJointPlannerIntegration, RejectsJointLimitViolationsInTheSharedEvaluator)
{
  Eigen::VectorXd outside_limits = Eigen::VectorXd::Zero(6);
  outside_limits(0) = 10.0;
  multilink_copilot::StabilityMetrics metrics;
  EXPECT_FALSE(evaluator_->evaluate(outside_limits, metrics));
}

TEST_F(WholeBodyJointPlannerIntegration, ExtractsSharedDragonJointMetadataAndCompleteStates)
{
  const DragonModelInfo info(model_);
  EXPECT_EQ(info.linkNum(), 4);
  EXPECT_GT(info.linkLength(), 0.0);
  ASSERT_EQ(info.jointNames().size(), 6u);
  EXPECT_EQ(info.pitchJointIndices(), (std::vector<int>{0, 2, 4}));
  EXPECT_EQ(info.yawJointIndices(), (std::vector<int>{1, 3, 5}));

  sensor_msgs::JointState complete;
  for (int index = info.jointCount() - 1; index >= 0; --index)
  {
    complete.name.push_back(info.jointNames()[static_cast<size_t>(index)]);
    complete.position.push_back(0.1 * index);
  }
  Eigen::VectorXd joints = Eigen::VectorXd::Constant(info.jointCount(), -1.0);
  ASSERT_TRUE(info.readCompleteJointState(complete, joints));
  for (int index = 0; index < joints.size(); ++index)
  {
    EXPECT_NEAR(joints(index), 0.1 * index, 1e-12);
  }

  sensor_msgs::JointState incomplete = complete;
  incomplete.name.pop_back();
  incomplete.position.pop_back();
  const Eigen::VectorXd previous = joints;
  EXPECT_FALSE(info.readCompleteJointState(incomplete, joints));
  EXPECT_TRUE(joints.isApprox(previous, 1e-12));
}
}  // namespace
}  // namespace motion_primitive_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "whole_body_joint_planner_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
