#include "gcopter/planner_common.hpp"

#include <gtest/gtest.h>

#include <sensor_msgs/PointField.h>

#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

gcopter_planner::CommonPlannerConfig validConfig()
{
    gcopter_planner::CommonPlannerConfig config;
    config.worldFrameId = "world";
    config.dilateRadius = 0.5;
    config.voxelWidth = 0.25;
    config.mapBound = {-5.0, 5.0, -4.0, 4.0, 0.0, 3.0};
    config.timeoutRRT = 0.02;
    config.maxVelMag = 2.0;
    config.maxBdrMag = 2.1;
    config.maxTiltAngle = 1.05;
    config.gravAcc = 9.8;
    config.weightT = 20.0;
    config.chiVec = {1.0e4, 1.0e4, 1.0e4, 1.0e4};
    config.smoothingEps = 1.0e-2;
    config.integralIntervs = 16;
    config.relCostTol = 1.0e-5;
    config.targetHeight = 1.0;
    return config;
}

sensor_msgs::PointField field(const std::string &name,
                              const uint32_t offset)
{
    sensor_msgs::PointField result;
    result.name = name;
    result.offset = offset;
    result.datatype = sensor_msgs::PointField::FLOAT32;
    result.count = 1;
    return result;
}

void writeFloat(sensor_msgs::PointCloud2 &cloud,
                const size_t pointIndex,
                const uint32_t offset,
                const float value)
{
    std::memcpy(cloud.data.data() + pointIndex * cloud.point_step + offset,
                &value, sizeof(value));
}

} // namespace

TEST(CommonPlannerConfigTest, ValidatesRequiredValues)
{
    gcopter_planner::CommonPlannerConfig config = validConfig();
    EXPECT_TRUE(config.validationError().empty());
    EXPECT_NO_THROW(config.validateOrThrow());

    config.mapBound.pop_back();
    EXPECT_EQ("MapBound must contain six finite values",
              config.validationError());

    config = validConfig();
    config.chiVec.resize(3);
    EXPECT_EQ("ChiVec must contain four finite, non-negative weights",
              config.validationError());

    config = validConfig();
    config.voxelWidth = 0.0;
    EXPECT_EQ("VoxelWidth must be finite and positive",
              config.validationError());
}

TEST(CommonPlannerConfigTest, ResolvesTargetHeightModes)
{
    gcopter_planner::CommonPlannerConfig config = validConfig();
    geometry_msgs::PoseStamped target;
    target.pose.position.z = 2.25;

    config.fixTargetHeight = false;
    EXPECT_DOUBLE_EQ(2.25, config.resolveTargetHeight(target));

    config.fixTargetHeight = true;
    config.targetHeight = 1.75;
    EXPECT_DOUBLE_EQ(1.75, config.resolveTargetHeight(target));
}

TEST(PointCloudDecoderTest, SupportsReorderedFieldsPaddingAndInvalidPoints)
{
    sensor_msgs::PointCloud2 cloud;
    cloud.height = 1;
    cloud.width = 3;
    cloud.is_bigendian = false;
    cloud.is_dense = false;
    cloud.fields = {
        field("z", 0),
        field("intensity", 4),
        field("x", 8),
        field("y", 12)};
    cloud.point_step = 20;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step);

    writeFloat(cloud, 0, 0, 3.0F);
    writeFloat(cloud, 0, 8, 1.0F);
    writeFloat(cloud, 0, 12, 2.0F);
    writeFloat(cloud, 1, 0, 6.0F);
    writeFloat(cloud, 1, 8, 4.0F);
    writeFloat(cloud, 1, 12, 5.0F);
    writeFloat(cloud, 2, 0, 9.0F);
    writeFloat(cloud, 2, 8, std::numeric_limits<float>::quiet_NaN());
    writeFloat(cloud, 2, 12, 8.0F);

    std::vector<Eigen::Vector3d> points;
    std::string error;
    ASSERT_TRUE(gcopter_planner::PlannerRosInterface::decodePointCloud(
        cloud, points, &error))
        << error;
    ASSERT_EQ(2U, points.size());
    EXPECT_TRUE(points[0].isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
    EXPECT_TRUE(points[1].isApprox(Eigen::Vector3d(4.0, 5.0, 6.0)));
}

TEST(PointCloudDecoderTest, RejectsMissingCoordinateField)
{
    sensor_msgs::PointCloud2 cloud;
    cloud.height = 1;
    cloud.width = 1;
    cloud.fields = {field("x", 0), field("z", 4)};
    cloud.point_step = 8;
    cloud.row_step = 8;
    cloud.data.resize(8);

    std::vector<Eigen::Vector3d> points;
    std::string error;
    EXPECT_FALSE(gcopter_planner::PlannerRosInterface::decodePointCloud(
        cloud, points, &error));
    EXPECT_FALSE(error.empty());
}

TEST(TrajectoryMessageTest, SerializesDurationsAndCoefficients)
{
    Piece<5>::CoefficientMat coefficients =
        Piece<5>::CoefficientMat::Zero();
    for (int axis = 0; axis < 3; ++axis)
    {
        for (int column = 0; column < 6; ++column)
        {
            coefficients(axis, column) = axis * 10 + column;
        }
    }

    Trajectory<5> trajectory;
    trajectory.emplace_back(1.25, coefficients);
    const gcopter::PolyTraj message =
        gcopter_planner::PlannerRosInterface::trajectoryMessage(
            trajectory, ros::Time(12.5), 7);

    EXPECT_EQ(5, message.order);
    EXPECT_EQ(7, message.traj_id);
    EXPECT_DOUBLE_EQ(12.5, message.start_time.toSec());
    ASSERT_EQ(1U, message.durations.size());
    EXPECT_DOUBLE_EQ(1.25, message.durations[0]);
    ASSERT_EQ(6U, message.coef_x.size());
    ASSERT_EQ(6U, message.coef_y.size());
    ASSERT_EQ(6U, message.coef_z.size());
    for (int column = 0; column < 6; ++column)
    {
        EXPECT_DOUBLE_EQ(column, message.coef_x[column]);
        EXPECT_DOUBLE_EQ(10 + column, message.coef_y[column]);
        EXPECT_DOUBLE_EQ(20 + column, message.coef_z[column]);
    }
}

TEST(TrajectoryScalingTest, PreservesPathAndReducesVelocity)
{
    Piece<5>::CoefficientMat coefficients =
        Piece<5>::CoefficientMat::Zero();
    coefficients(0, 4) = 2.0;

    Trajectory<5> trajectory;
    trajectory.emplace_back(1.0, coefficients);
    const Trajectory<5> scaled =
        gcopter_planner::PlannerBackend::timeScaledTrajectory(
            trajectory, 2.0);

    ASSERT_EQ(1, scaled.getPieceNum());
    EXPECT_DOUBLE_EQ(2.0, scaled.getTotalDuration());
    EXPECT_TRUE(scaled.getPos(1.0).isApprox(trajectory.getPos(0.5)));
    EXPECT_NEAR(trajectory.getMaxVelRate() / 2.0,
                scaled.getMaxVelRate(), 1.0e-9);
    EXPECT_THROW(
        gcopter_planner::PlannerBackend::timeScaledTrajectory(
            trajectory, 0.0),
        std::invalid_argument);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
