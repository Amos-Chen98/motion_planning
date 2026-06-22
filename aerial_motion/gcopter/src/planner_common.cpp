#include "gcopter/planner_common.hpp"

#include "gcopter/gcopter.hpp"
#include "gcopter/sfc_gen.hpp"
#include "misc/tf_utils.hpp"

#include <sensor_msgs/point_cloud2_iterator.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace gcopter_planner
{
namespace
{

bool isFinite(const double value)
{
    return std::isfinite(value);
}

bool allFinite(const std::vector<double> &values)
{
    return std::all_of(values.begin(), values.end(),
                       [](const double value)
                       { return std::isfinite(value); });
}

} // namespace

CommonPlannerConfig::CommonPlannerConfig(const ros::NodeHandle &nhPriv)
{
    nhPriv.param<std::string>("WorldFrameId", worldFrameId, "world");
    nhPriv.param("DilateRadius", dilateRadius, 0.0);
    nhPriv.param("VoxelWidth", voxelWidth, 0.0);
    nhPriv.getParam("MapBound", mapBound);
    nhPriv.param("TimeoutRRT", timeoutRRT, 0.0);
    nhPriv.param("MaxVelMag", maxVelMag, 0.0);
    nhPriv.param("MaxBdrMag", maxBdrMag, 0.0);
    nhPriv.param("MaxTiltAngle", maxTiltAngle, 0.0);
    nhPriv.param("GravAcc", gravAcc, 0.0);
    nhPriv.param("WeightT", weightT, 0.0);
    nhPriv.getParam("ChiVec", chiVec);
    nhPriv.param("SmoothingEps", smoothingEps, 0.0);
    nhPriv.param("IntegralIntervs", integralIntervs, 0);
    nhPriv.param("RelCostTol", relCostTol, 0.0);
    nhPriv.param("UseFixedTargetHeight", useFixedTargetHeight, false);
    nhPriv.param("TargetHeight", targetHeight, 1.0);
    nhPriv.param("UseTargetZ", useTargetZ, false);
    nhPriv.param("ShowPolytopeCorridor", showPolytopeCorridor, true);
}

std::string CommonPlannerConfig::validationError() const
{
    if (worldFrameId.empty())
    {
        return "WorldFrameId must not be empty";
    }
    if (mapBound.size() != 6 || !allFinite(mapBound))
    {
        return "MapBound must contain six finite values";
    }
    if (!(mapBound[0] < mapBound[1] &&
          mapBound[2] < mapBound[3] &&
          mapBound[4] < mapBound[5]))
    {
        return "MapBound must satisfy min < max on every axis";
    }
    if (!isFinite(voxelWidth) || voxelWidth <= 0.0)
    {
        return "VoxelWidth must be finite and positive";
    }
    for (int axis = 0; axis < 3; ++axis)
    {
        if ((mapBound[2 * axis + 1] - mapBound[2 * axis]) / voxelWidth < 1.0)
        {
            return "MapBound must contain at least one voxel on every axis";
        }
    }
    if (!isFinite(dilateRadius) || dilateRadius < 0.0)
    {
        return "DilateRadius must be finite and non-negative";
    }
    if (!isFinite(timeoutRRT) || timeoutRRT <= 0.0)
    {
        return "TimeoutRRT must be finite and positive";
    }
    if (!isFinite(maxVelMag) || maxVelMag <= 0.0 ||
        !isFinite(maxBdrMag) || maxBdrMag <= 0.0 ||
        !isFinite(maxTiltAngle) || maxTiltAngle <= 0.0)
    {
        return "MaxVelMag, MaxBdrMag, and MaxTiltAngle must be finite and positive";
    }
    if (!isFinite(gravAcc) || gravAcc <= 0.0)
    {
        return "GravAcc must be finite and positive";
    }
    if (!isFinite(weightT) || weightT <= 0.0)
    {
        return "WeightT must be finite and positive";
    }
    if (chiVec.size() != 4 || !allFinite(chiVec) ||
        std::any_of(chiVec.begin(), chiVec.end(),
                    [](const double value)
                    { return value < 0.0; }))
    {
        return "ChiVec must contain four finite, non-negative weights";
    }
    if (!isFinite(smoothingEps) || smoothingEps <= 0.0)
    {
        return "SmoothingEps must be finite and positive";
    }
    if (integralIntervs <= 0)
    {
        return "IntegralIntervs must be positive";
    }
    if (!isFinite(relCostTol) || relCostTol <= 0.0)
    {
        return "RelCostTol must be finite and positive";
    }
    if (!isFinite(targetHeight))
    {
        return "TargetHeight must be finite";
    }

    return std::string();
}

void CommonPlannerConfig::validateOrThrow() const
{
    const std::string error = validationError();
    if (!error.empty())
    {
        throw std::invalid_argument(error);
    }
}

double CommonPlannerConfig::resolveTargetHeight(
    const geometry_msgs::PoseStamped &msg) const
{
    if (useFixedTargetHeight)
    {
        return targetHeight;
    }
    if (useTargetZ)
    {
        return msg.pose.position.z;
    }

    return mapBound[4] + dilateRadius +
           std::fabs(msg.pose.orientation.z) *
               (mapBound[5] - mapBound[4] - 2.0 * dilateRadius);
}

PlannerBackend::PlannerBackend(const CommonPlannerConfig &config)
    : config_(config),
      dilateVoxelRadius_(0)
{
    config_.validateOrThrow();

    const Eigen::Vector3i size(
        static_cast<int>((config_.mapBound[1] - config_.mapBound[0]) /
                         config_.voxelWidth),
        static_cast<int>((config_.mapBound[3] - config_.mapBound[2]) /
                         config_.voxelWidth),
        static_cast<int>((config_.mapBound[5] - config_.mapBound[4]) /
                         config_.voxelWidth));
    const Eigen::Vector3d origin(config_.mapBound[0],
                                 config_.mapBound[2],
                                 config_.mapBound[4]);
    voxelMap_ = voxel_map::VoxelMap(size, origin, config_.voxelWidth);
    dilateVoxelRadius_ =
        static_cast<int>(std::ceil(config_.dilateRadius / voxelMap_.getScale()));
}

void PlannerBackend::setMapPoints(
    const std::vector<Eigen::Vector3d> &points)
{
    voxelMap_.clear();
    for (const Eigen::Vector3d &point : points)
    {
        voxelMap_.setOccupied(point);
    }
    voxelMap_.dilate(dilateVoxelRadius_);
}

void PlannerBackend::setMapVoxels(
    const std::vector<Eigen::Vector3i> &voxelIds)
{
    voxelMap_.clear();
    for (const Eigen::Vector3i &voxelId : voxelIds)
    {
        voxelMap_.setOccupied(voxelId);
    }
    voxelMap_.dilate(dilateVoxelRadius_);
}

bool PlannerBackend::query(const Eigen::Vector3d &position) const
{
    return voxelMap_.query(position);
}

double PlannerBackend::voxelScale() const
{
    return voxelMap_.getScale();
}

Eigen::Vector3i PlannerBackend::mapSize() const
{
    return voxelMap_.getSize();
}

Eigen::Vector3d PlannerBackend::mapOrigin() const
{
    return voxelMap_.getOrigin();
}

Eigen::Vector3d PlannerBackend::mapCorner() const
{
    return voxelMap_.getCorner();
}

long PlannerBackend::voxelKey(const Eigen::Vector3d &position) const
{
    const Eigen::Vector3i id = voxelMap_.posD2I(position);
    const Eigen::Vector3i size = voxelMap_.getSize();
    if (id(0) < 0 || id(1) < 0 || id(2) < 0 ||
        id(0) >= size(0) || id(1) >= size(1) || id(2) >= size(2))
    {
        return -1;
    }

    return static_cast<long>(id(0)) +
           static_cast<long>(size(0)) *
               (static_cast<long>(id(1)) +
                static_cast<long>(size(1)) * static_cast<long>(id(2)));
}

Eigen::Vector3i PlannerBackend::voxelIdFromKey(const long key) const
{
    const Eigen::Vector3i size = voxelMap_.getSize();
    const long xy = static_cast<long>(size(0)) * static_cast<long>(size(1));
    const int z = static_cast<int>(key / xy);
    const long remainder = key - static_cast<long>(z) * xy;
    const int y = static_cast<int>(remainder / size(0));
    const int x =
        static_cast<int>(remainder - static_cast<long>(y) * size(0));
    return Eigen::Vector3i(x, y, z);
}

Eigen::Vector3d PlannerBackend::clampInsideMap(
    const Eigen::Vector3d &point,
    const double clearance) const
{
    const Eigen::Vector3d offset =
        Eigen::Vector3d::Constant(std::max(0.0, clearance));
    const Eigen::Vector3d lower = voxelMap_.getOrigin() + offset;
    const Eigen::Vector3d upper = voxelMap_.getCorner() - offset;
    return point.cwiseMax(lower).cwiseMin(upper);
}

bool PlannerBackend::searchPath(
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &goal,
    std::vector<Eigen::Vector3d> &route)
{
    route.clear();
    try
    {
        sfc_gen::planPath<voxel_map::VoxelMap>(
            start, goal,
            voxelMap_.getOrigin(), voxelMap_.getCorner(),
            &voxelMap_, config_.timeoutRRT, route);
    }
    catch (const std::exception &exception)
    {
        ROS_WARN_THROTTLE(1.0, "Path search threw an exception: %s",
                          exception.what());
        return false;
    }

    if (route.size() <= 1)
    {
        ROS_WARN_THROTTLE(1.0, "RRT did not produce a usable route.");
        return false;
    }
    return true;
}

bool PlannerBackend::buildCorridor(
    const std::vector<Eigen::Vector3d> &route,
    std::vector<Eigen::MatrixX4d> &hPolys)
{
    hPolys.clear();
    if (route.size() <= 1)
    {
        ROS_WARN_THROTTLE(1.0,
                          "Cannot generate a corridor for a route with fewer than two points.");
        return false;
    }

    try
    {
        std::vector<Eigen::Vector3d> surfacePoints;
        voxelMap_.getSurf(surfacePoints);
        sfc_gen::convexCover(route, surfacePoints,
                             voxelMap_.getOrigin(), voxelMap_.getCorner(),
                             7.0, 3.0, hPolys);
        sfc_gen::shortCut(hPolys);
    }
    catch (const std::exception &exception)
    {
        ROS_WARN_THROTTLE(1.0, "Corridor generation threw an exception: %s",
                          exception.what());
        return false;
    }

    if (hPolys.empty())
    {
        ROS_WARN_THROTTLE(1.0,
                          "Failed to generate a safe flight corridor.");
        return false;
    }
    return true;
}

bool PlannerBackend::optimizeTrajectory(
    const Eigen::Matrix3d &initialState,
    const Eigen::Matrix3d &finalState,
    const std::vector<Eigen::MatrixX4d> &hPolys,
    Trajectory<5> &trajectory,
    const std::string &plannerLabel)
{
    if (hPolys.empty())
    {
        ROS_WARN_THROTTLE(1.0, "GCOPTER received an empty flight corridor.");
        return false;
    }

    Eigen::VectorXd magnitudeBounds(3);
    magnitudeBounds << config_.maxVelMag,
        config_.maxBdrMag,
        config_.maxTiltAngle;

    Eigen::VectorXd penaltyWeights(4);
    penaltyWeights << config_.chiVec[0],
        config_.chiVec[1],
        config_.chiVec[2],
        config_.chiVec[3];

    gcopter::GCOPTER_PolytopeSFC gcopter;
    try
    {
        if (!gcopter.setup(config_.weightT,
                           initialState, finalState,
                           hPolys, INFINITY,
                           config_.smoothingEps,
                           config_.integralIntervs,
                           magnitudeBounds,
                           penaltyWeights,
                           config_.gravAcc))
        {
            ROS_WARN_THROTTLE(1.0, "GCOPTER setup failed.");
            return false;
        }

        trajectory.clear();
        const double cost =
            gcopter.optimize(trajectory, config_.relCostTol);
        if (!std::isfinite(cost))
        {
            ROS_WARN_THROTTLE(1.0, "GCOPTER optimization failed.");
            return false;
        }
    }
    catch (const std::exception &exception)
    {
        ROS_WARN_THROTTLE(1.0,
                          "GCOPTER optimization threw an exception: %s",
                          exception.what());
        return false;
    }
    if (trajectory.getPieceNum() <= 0)
    {
        ROS_WARN_THROTTLE(1.0, "GCOPTER produced an empty trajectory.");
        return false;
    }

    return enforceVelocityLimit(trajectory, plannerLabel);
}

Trajectory<5> PlannerBackend::timeScaledTrajectory(
    const Trajectory<5> &input,
    const double scale)
{
    if (!std::isfinite(scale) || scale <= 0.0)
    {
        throw std::invalid_argument(
            "Trajectory time scale must be finite and positive");
    }

    Trajectory<5> output;
    output.reserve(input.getPieceNum());
    for (const Piece<5> &piece : input)
    {
        Piece<5>::CoefficientMat coefficients = piece.getCoeffMat();
        double coefficientScale = 1.0;
        for (int power = 1; power <= 5; ++power)
        {
            coefficientScale *= scale;
            coefficients.col(5 - power) /= coefficientScale;
        }
        output.emplace_back(piece.getDuration() * scale, coefficients);
    }
    return output;
}

bool PlannerBackend::enforceVelocityLimit(
    Trajectory<5> &trajectory,
    const std::string &plannerLabel) const
{
    const double optimizedMaxVelocity = trajectory.getMaxVelRate();
    if (!std::isfinite(optimizedMaxVelocity))
    {
        ROS_WARN_THROTTLE(
            1.0, "Optimized %s trajectory has an invalid velocity bound.",
            plannerLabel.c_str());
        return false;
    }

    if (optimizedMaxVelocity <= config_.maxVelMag)
    {
        return true;
    }

    constexpr double velocityMargin = 0.99;
    const bool significantViolation =
        optimizedMaxVelocity > config_.maxVelMag * 1.01;
    const double timeScale =
        optimizedMaxVelocity / (velocityMargin * config_.maxVelMag);
    trajectory = timeScaledTrajectory(trajectory, timeScale);

    const double scaledMaxVelocity = trajectory.getMaxVelRate();
    if (!std::isfinite(scaledMaxVelocity) ||
        scaledMaxVelocity > config_.maxVelMag * (1.0 + 1.0e-6))
    {
        ROS_WARN_THROTTLE(
            1.0, "Failed to enforce the %s trajectory velocity limit.",
            plannerLabel.c_str());
        return false;
    }

    if (significantViolation)
    {
        ROS_WARN_THROTTLE(
            1.0,
            "Time-scaled %s trajectory by %.3f: max velocity %.3f -> %.3f m/s "
            "(limit %.3f m/s).",
            plannerLabel.c_str(), timeScale,
            optimizedMaxVelocity, scaledMaxVelocity, config_.maxVelMag);
    }
    else
    {
        ROS_DEBUG("Time-scaled %s trajectory by %.3f to preserve the %.3f m/s "
                  "velocity limit.",
                  plannerLabel.c_str(), timeScale, config_.maxVelMag);
    }

    return true;
}

PlannerRosInterface::PlannerRosInterface(
    const CommonPlannerConfig &config,
    ros::NodeHandle &nh)
    : config_(config),
      nh_(nh),
      tfListener_(tfBuffer_),
      visualizer_(nh_, config_.worldFrameId),
      latestPosition_(Eigen::Vector3d::Zero()),
      odomReceived_(false),
      trajectoryId_(0)
{
    odomSub_ = nh_.subscribe(
        "odom", 1, &PlannerRosInterface::odomCallback, this,
        ros::TransportHints().tcpNoDelay());
    trajectoryPub_ = nh_.advertise<gcopter::PolyTraj>("trajectory", 10);
}

bool PlannerRosInterface::odomReceived() const
{
    return odomReceived_;
}

const Eigen::Vector3d &PlannerRosInterface::latestPosition() const
{
    return latestPosition_;
}

std::string PlannerRosInterface::resolvedOdomTopic() const
{
    return nh_.resolveName("odom");
}

bool PlannerRosInterface::pointCloudToWorld(
    const sensor_msgs::PointCloud2 &msg,
    std::vector<Eigen::Vector3d> &pointsWorld,
    std::string *error) const
{
    Eigen::Isometry3d worldCloud;
    if (!tf_utils::resolveToWorld(tfBuffer_, config_.worldFrameId,
                                  msg.header.frame_id, worldCloud))
    {
        if (error != nullptr)
        {
            *error = "point-cloud transform is unavailable";
        }
        return false;
    }

    std::vector<Eigen::Vector3d> pointsCloud;
    if (!decodePointCloud(msg, pointsCloud, error))
    {
        return false;
    }

    pointsWorld.clear();
    pointsWorld.reserve(pointsCloud.size());
    for (const Eigen::Vector3d &point : pointsCloud)
    {
        pointsWorld.push_back(worldCloud * point);
    }
    return true;
}

bool PlannerRosInterface::decodePointCloud(
    const sensor_msgs::PointCloud2 &msg,
    std::vector<Eigen::Vector3d> &points,
    std::string *error)
{
    points.clear();
    try
    {
        sensor_msgs::PointCloud2ConstIterator<float> iterX(msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iterY(msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iterZ(msg, "z");

        points.reserve(static_cast<size_t>(msg.width) *
                       static_cast<size_t>(msg.height));
        for (; iterX != iterX.end(); ++iterX, ++iterY, ++iterZ)
        {
            if (std::isfinite(*iterX) &&
                std::isfinite(*iterY) &&
                std::isfinite(*iterZ))
            {
                points.emplace_back(*iterX, *iterY, *iterZ);
            }
        }
    }
    catch (const std::runtime_error &exception)
    {
        if (error != nullptr)
        {
            *error = exception.what();
        }
        return false;
    }

    return true;
}

void PlannerRosInterface::publishTrajectory(
    const Trajectory<5> &trajectory,
    const double startTime)
{
    trajectoryPub_.publish(
        trajectoryMessage(trajectory, ros::Time(startTime), trajectoryId_++));
}

gcopter::PolyTraj PlannerRosInterface::trajectoryMessage(
    const Trajectory<5> &trajectory,
    const ros::Time &startTime,
    const int trajectoryId)
{
    gcopter::PolyTraj msg;
    msg.start_time = startTime;
    msg.traj_id = trajectoryId;
    msg.order = 5;

    constexpr int coefficientsPerPiece = 6;
    const int pieceCount = trajectory.getPieceNum();
    msg.durations.reserve(pieceCount);
    msg.coef_x.reserve(pieceCount * coefficientsPerPiece);
    msg.coef_y.reserve(pieceCount * coefficientsPerPiece);
    msg.coef_z.reserve(pieceCount * coefficientsPerPiece);

    for (int pieceIndex = 0; pieceIndex < pieceCount; ++pieceIndex)
    {
        const Piece<5> &piece = trajectory[pieceIndex];
        const Piece<5>::CoefficientMat coefficients = piece.getCoeffMat();
        msg.durations.push_back(piece.getDuration());
        for (int column = 0; column < coefficientsPerPiece; ++column)
        {
            msg.coef_x.push_back(coefficients(0, column));
            msg.coef_y.push_back(coefficients(1, column));
            msg.coef_z.push_back(coefficients(2, column));
        }
    }

    return msg;
}

Visualizer &PlannerRosInterface::visualizer()
{
    return visualizer_;
}

Eigen::Quaterniond PlannerRosInterface::normalizedQuaternion(
    const geometry_msgs::Quaternion &quaternion)
{
    Eigen::Quaterniond result(quaternion.w, quaternion.x,
                              quaternion.y, quaternion.z);
    if (result.norm() < 1.0e-9)
    {
        return Eigen::Quaterniond::Identity();
    }
    return result.normalized();
}

void PlannerRosInterface::odomCallback(
    const nav_msgs::Odometry::ConstPtr &msg)
{
    const geometry_msgs::Point &position = msg->pose.pose.position;
    Eigen::Isometry3d odomBody = Eigen::Isometry3d::Identity();
    odomBody.translate(
        Eigen::Vector3d(position.x, position.y, position.z));
    odomBody.rotate(normalizedQuaternion(msg->pose.pose.orientation));

    Eigen::Isometry3d worldOdom;
    if (!tf_utils::resolveToWorld(tfBuffer_, config_.worldFrameId,
                                  msg->header.frame_id, worldOdom))
    {
        return;
    }

    latestPosition_ = (worldOdom * odomBody).translation();
    odomReceived_ = true;
}

} // namespace gcopter_planner
