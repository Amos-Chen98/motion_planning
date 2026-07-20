#include <gtest/gtest.h>

#include <multilink_copilot/joint_command_continuity.h>

#include <limits>

namespace multilink_copilot
{
namespace
{

TEST(JointCommandContinuity, LeavesSmallStepUnchanged)
{
  const Eigen::Vector3d reference(0.0, -0.2, 0.3);
  const Eigen::Vector3d requested(0.05, -0.25, 0.22);
  Eigen::VectorXd limited;
  bool was_limited = true;

  ASSERT_TRUE(limitJointPositionStep(reference, requested, 0.1, limited, was_limited));
  EXPECT_FALSE(was_limited);
  EXPECT_TRUE(limited.isApprox(requested));
}

TEST(JointCommandContinuity, UniformlyScalesLargeStep)
{
  const Eigen::Vector3d reference(0.0, -0.3, 0.2);
  const Eigen::Vector3d requested(0.4, 0.5, -0.2);
  Eigen::VectorXd limited;
  bool was_limited = false;

  ASSERT_TRUE(limitJointPositionStep(reference, requested, 0.1, limited, was_limited));
  EXPECT_TRUE(was_limited);
  EXPECT_NEAR((limited - reference).cwiseAbs().maxCoeff(), 0.1, 1e-12);
  EXPECT_TRUE((limited - reference).normalized().isApprox((requested - reference).normalized()));
}

TEST(JointCommandContinuity, NonPositiveLimitDisablesLimiting)
{
  const Eigen::Vector2d reference(-1.0, 1.0);
  const Eigen::Vector2d requested(1.0, -1.0);
  Eigen::VectorXd limited;
  bool was_limited = true;

  ASSERT_TRUE(limitJointPositionStep(reference, requested, 0.0, limited, was_limited));
  EXPECT_FALSE(was_limited);
  EXPECT_TRUE(limited.isApprox(requested));
}

TEST(JointCommandContinuity, RejectsMismatchedOrNonFiniteInput)
{
  Eigen::VectorXd limited;
  bool was_limited = false;

  EXPECT_FALSE(limitJointPositionStep(Eigen::Vector2d::Zero(), Eigen::Vector3d::Zero(), 0.1, limited, was_limited));

  Eigen::Vector2d non_finite = Eigen::Vector2d::Zero();
  non_finite(1) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(limitJointPositionStep(Eigen::Vector2d::Zero(), non_finite, 0.1, limited, was_limited));
}

}  // namespace
}  // namespace multilink_copilot

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
