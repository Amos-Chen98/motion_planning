#include "misc/visualizer.hpp"
#include "misc/tf_utils.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_listener.h>

#include <gcopter/PolyTraj.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace
{
inline Eigen::Quaterniond normalizedQuaternion(const geometry_msgs::Quaternion &q)
{
    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    if (quat.norm() < 1.0e-9)
    {
        return Eigen::Quaterniond::Identity();
    }
    return quat.normalized();
}

inline bool isFinitePoint(const float x, const float y, const float z)
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

template <int D>
Trajectory<D> timeScaledTrajectory(const Trajectory<D> &input, const double scale)
{
    Trajectory<D> output;
    output.reserve(input.getPieceNum());

    for (const auto &piece : input)
    {
        typename Piece<D>::CoefficientMat coeff = piece.getCoeffMat();
        double coefficientScale = 1.0;
        for (int power = 1; power <= D; ++power)
        {
            coefficientScale *= scale;
            coeff.col(D - power) /= coefficientScale;
        }
        output.emplace_back(piece.getDuration() * scale, coeff);
    }

    return output;
}
} // namespace

struct OnlinePlannerConfig
{
    std::string worldFrameId;

    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double gravAcc;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;
    double commandHz;
    bool publishYawCommand;
    double replanHz;
    bool useAccumulatedMap;
    bool useFixedTargetHeight;
    double targetHeight;
    bool useTargetZ;
    bool showPolytopeCorridor;
    double goalTolerance;
    double planningHorizon;

    explicit OnlinePlannerConfig(const ros::NodeHandle &nhPriv)
    {
        nhPriv.param<std::string>("WorldFrameId", worldFrameId, "world");

        nhPriv.getParam("DilateRadius", dilateRadius);
        nhPriv.getParam("VoxelWidth", voxelWidth);
        nhPriv.getParam("MapBound", mapBound);
        nhPriv.getParam("TimeoutRRT", timeoutRRT);
        nhPriv.getParam("MaxVelMag", maxVelMag);
        nhPriv.getParam("MaxBdrMag", maxBdrMag);
        nhPriv.getParam("MaxTiltAngle", maxTiltAngle);
        nhPriv.getParam("GravAcc", gravAcc);
        nhPriv.getParam("WeightT", weightT);
        nhPriv.getParam("ChiVec", chiVec);
        nhPriv.getParam("SmoothingEps", smoothingEps);
        nhPriv.getParam("IntegralIntervs", integralIntervs);
        nhPriv.getParam("RelCostTol", relCostTol);
        nhPriv.param("CommandHz", commandHz, 40.0);
        nhPriv.param("PublishYawCommand", publishYawCommand, false);
        nhPriv.param("ReplanHz", replanHz, 2.0);
        nhPriv.param("UseAccumulatedMap", useAccumulatedMap, true);
        nhPriv.param("UseFixedTargetHeight", useFixedTargetHeight, false);
        nhPriv.param("TargetHeight", targetHeight, 1.0);
        nhPriv.param("UseTargetZ", useTargetZ, false);
        nhPriv.param("ShowPolytopeCorridor", showPolytopeCorridor, true);
        nhPriv.param("GoalTolerance", goalTolerance, 0.2);
        nhPriv.param("PlanningHorizon", planningHorizon, 6.0);
    }
};

class OnlineReplanner
{
private:
    OnlinePlannerConfig config;
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    ros::Subscriber localMapSub;
    ros::Subscriber targetSub;
    ros::Subscriber odomSub;
    ros::Publisher trajPub;
    ros::Timer replanTimer;

    voxel_map::VoxelMap voxelMap;
    Visualizer visualizer;

    std::unordered_set<long> occupiedVoxelKeys;
    int dilateVoxelRadius;

    Eigen::Isometry3d latestWorldBody;
    Eigen::Vector3d latestPositionWorld;
    bool odomReceived;
    bool mapInitialized;
    bool targetReceived;

    // Retain the published trajectory and its t=0 for C2 handover sampling.
    Trajectory<5> traj;
    double trajStamp;
    int trajId_;

    // Plan to the clamped target; keep the raw request only for diagnostics.
    Eigen::Vector3d globalTarget_;
    Eigen::Vector3d requestedTarget_;
    bool targetWasClamped_;

    // Stop replanning after committing the terminal braking trajectory.
    bool goalLatched_;

public:
    OnlineReplanner(const OnlinePlannerConfig &conf, ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          tfListener(tfBuffer),
          visualizer(nh, config.worldFrameId),
          latestWorldBody(Eigen::Isometry3d::Identity()),
          latestPositionWorld(Eigen::Vector3d::Zero()),
          odomReceived(false),
          mapInitialized(false),
          targetReceived(false),
          trajStamp(0.0),
          trajId_(0),
          globalTarget_(Eigen::Vector3d::Zero()),
          requestedTarget_(Eigen::Vector3d::Zero()),
          targetWasClamped_(false),
          goalLatched_(false)
    {
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);
        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);
        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);
        dilateVoxelRadius = static_cast<int>(std::ceil(config.dilateRadius / voxelMap.getScale()));

        localMapSub = nh.subscribe("pcl_topic", 1, &OnlineReplanner::localMapCallback, this,
                                   ros::TransportHints().tcpNoDelay());
        targetSub = nh.subscribe("target", 1, &OnlineReplanner::targetCallback, this,
                                 ros::TransportHints().tcpNoDelay());
        odomSub = nh.subscribe("odom", 1, &OnlineReplanner::odomCallback, this,
                               ros::TransportHints().tcpNoDelay());
        trajPub = nh.advertise<gcopter::PolyTraj>("trajectory", 10);

        const double replanHz = config.replanHz > 0.0 ? config.replanHz : 2.0;
        replanTimer = nh.createTimer(ros::Duration(1.0 / replanHz),
                                     &OnlineReplanner::replanTimerCallback, this);
    }

private:
    inline long voxelKeyFromPosition(const Eigen::Vector3d &pos) const
    {
        const Eigen::Vector3i id = voxelMap.posD2I(pos);
        const Eigen::Vector3i size = voxelMap.getSize();
        if (id(0) < 0 || id(1) < 0 || id(2) < 0 ||
            id(0) >= size(0) || id(1) >= size(1) || id(2) >= size(2))
        {
            return -1;
        }

        return static_cast<long>(id(0)) +
               static_cast<long>(size(0)) *
                   (static_cast<long>(id(1)) + static_cast<long>(size(1)) * static_cast<long>(id(2)));
    }

    inline Eigen::Vector3i voxelIdFromKey(const long key) const
    {
        const Eigen::Vector3i size = voxelMap.getSize();
        const long xy = static_cast<long>(size(0)) * static_cast<long>(size(1));
        const int z = static_cast<int>(key / xy);
        const long rem = key - static_cast<long>(z) * xy;
        const int y = static_cast<int>(rem / size(0));
        const int x = static_cast<int>(rem - static_cast<long>(y) * size(0));
        return Eigen::Vector3i(x, y, z);
    }

    inline void rebuildVoxelMap()
    {
        voxelMap.clear();
        for (const long key : occupiedVoxelKeys)
        {
            voxelMap.setOccupied(voxelIdFromKey(key));
        }
        voxelMap.dilate(dilateVoxelRadius);
        mapInitialized = true;
    }

    inline void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // Transform odometry into the planning frame.
        const geometry_msgs::Point &p = msg->pose.pose.position;
        Eigen::Isometry3d odomRefBody = Eigen::Isometry3d::Identity();
        odomRefBody.translate(Eigen::Vector3d(p.x, p.y, p.z));
        odomRefBody.rotate(normalizedQuaternion(msg->pose.pose.orientation));

        Eigen::Isometry3d worldOdomRef;
        if (!tf_utils::resolveToWorld(tfBuffer, config.worldFrameId,
                                      msg->header.frame_id, worldOdomRef))
        {
            return; // keep the last known pose until the transform is available
        }

        latestWorldBody = worldOdomRef * odomRefBody;
        latestPositionWorld = latestWorldBody.translation();
        odomReceived = true;
    }

    inline void localMapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        // Transform sensor points into the planning frame.
        Eigen::Isometry3d worldSensor;
        if (!tf_utils::resolveToWorld(tfBuffer, config.worldFrameId,
                                      msg->header.frame_id, worldSensor))
        {
            return; // skip this cloud rather than insert mis-registered points
        }

        if (!config.useAccumulatedMap)
        {
            occupiedVoxelKeys.clear();
        }

        size_t accepted = 0;

        try
        {
            sensor_msgs::PointCloud2ConstIterator<float> iterX(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iterY(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iterZ(*msg, "z");

            for (; iterX != iterX.end(); ++iterX, ++iterY, ++iterZ)
            {
                if (!isFinitePoint(*iterX, *iterY, *iterZ))
                {
                    continue;
                }

                const Eigen::Vector3d pointWorld = worldSensor * Eigen::Vector3d(*iterX, *iterY, *iterZ);
                const long key = voxelKeyFromPosition(pointWorld);
                if (key >= 0)
                {
                    occupiedVoxelKeys.insert(key);
                    accepted++;
                }
            }
        }
        catch (const std::runtime_error &e)
        {
            ROS_WARN("Invalid local point cloud: %s", e.what());
            return;
        }

        rebuildVoxelMap();
        ROS_DEBUG("Integrated %zu local points into %zu occupied voxels.",
                  accepted, occupiedVoxelKeys.size());
    }

    inline double getTargetHeight(const geometry_msgs::PoseStamped &msg) const
    {
        if (config.useFixedTargetHeight)
        {
            return config.targetHeight;
        }

        if (config.useTargetZ)
        {
            return msg.pose.position.z;
        }

        return config.mapBound[4] + config.dilateRadius +
               std::fabs(msg.pose.orientation.z) *
                   (config.mapBound[5] - config.mapBound[4] - 2.0 * config.dilateRadius);
    }

    inline void targetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // Clamp once and use the bounded target throughout planning.
        requestedTarget_ = Eigen::Vector3d(msg->pose.position.x,
                                           msg->pose.position.y,
                                           getTargetHeight(*msg));
        globalTarget_ = clampInsideMap(requestedTarget_);
        targetWasClamped_ = (globalTarget_ - requestedTarget_).norm() > 1.0e-3;
        targetReceived = true;

        goalLatched_ = false;

        if (targetWasClamped_)
        {
            ROS_WARN("Requested target [%.2f, %.2f, %.2f] is outside the planning volume; "
                     "clamped to [%.2f, %.2f, %.2f]. Planning to the clamped target.",
                     requestedTarget_.x(), requestedTarget_.y(), requestedTarget_.z(),
                     globalTarget_.x(), globalTarget_.y(), globalTarget_.z());
        }

        visualizer.visualizeStartGoal(globalTarget_, 0.05, 1);
        if (targetWasClamped_)
        {
            visualizer.visualizeStartGoal(requestedTarget_, 0.05, 2);
        }
        ROS_INFO("Received online replanning target in %s: [%.2f, %.2f, %.2f].",
                 config.worldFrameId.c_str(),
                 globalTarget_.x(), globalTarget_.y(), globalTarget_.z());
    }

    // Keep RRT goals inside the valid planning volume.
    inline Eigen::Vector3d clampInsideMap(const Eigen::Vector3d &pt) const
    {
        const Eigen::Vector3d offs = Eigen::Vector3d::Constant(config.dilateRadius + voxelMap.getScale());
        const Eigen::Vector3d lo = voxelMap.getOrigin() + offs;
        const Eigen::Vector3d hi = voxelMap.getCorner() - offs;
        return pt.cwiseMax(lo).cwiseMin(hi);
    }

    // Truncate the route to the local planning horizon.
    inline Eigen::Vector3d truncateRouteToHorizon(const std::vector<Eigen::Vector3d> &full,
                                                  std::vector<Eigen::Vector3d> &local) const
    {
        local.clear();
        local.push_back(full.front());

        double accumulated = 0.0;
        for (size_t i = 1; i < full.size(); ++i)
        {
            const double seg = (full[i] - full[i - 1]).norm();
            if (accumulated + seg >= config.planningHorizon)
            {
                const double remain = config.planningHorizon - accumulated;
                const double ratio = seg > 1.0e-6 ? remain / seg : 0.0;
                local.push_back(full[i - 1] + ratio * (full[i] - full[i - 1]));
                return local.back();
            }
            accumulated += seg;
            local.push_back(full[i]);
        }

        return local.back();
    }

    inline bool computeTrajectory(const Eigen::Vector3d &start,
                                  const Eigen::Vector3d &startVel,
                                  const Eigen::Vector3d &startAcc,
                                  const Eigen::Vector3d &goal,
                                  Trajectory<5> &candidateTraj,
                                  std::vector<Eigen::Vector3d> &route,
                                  std::vector<Eigen::MatrixX4d> &hPolys,
                                  bool &terminal)
    {
        // A terminal trajectory brakes at the goal or closest safe approach.
        terminal = false;

        std::vector<Eigen::Vector3d> fullRoute;
        try
        {
            sfc_gen::planPath<voxel_map::VoxelMap>(start,
                                                   goal,
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &voxelMap,
                                                   config.timeoutRRT,
                                                   fullRoute);
        }
        catch (const std::exception &e)
        {
            // Keep invalid OMPL queries from aborting the planner.
            ROS_WARN_THROTTLE(1.0, "Path search threw an exception: %s", e.what());
            return false;
        }

        if (fullRoute.size() <= 1)
        {
            ROS_WARN_THROTTLE(1.0, "RRT did not produce a usable route.");
            return false;
        }

        const Eigen::Vector3d routeEnd = fullRoute.back();

        // Optimize one local horizon.
        const Eigen::Vector3d localTarget = truncateRouteToHorizon(fullRoute, route);
        if (route.size() < 2)
        {
            ROS_WARN_THROTTLE(1.0, "Local route is too short to plan.");
            return false;
        }

        std::vector<Eigen::Vector3d> pc;
        voxelMap.getSurf(pc);
        sfc_gen::convexCover(route,
                             pc,
                             voxelMap.getOrigin(),
                             voxelMap.getCorner(),
                             7.0,
                             3.0,
                             hPolys);
        sfc_gen::shortCut(hPolys);

        if (hPolys.empty())
        {
            ROS_WARN_THROTTLE(1.0, "Failed to generate a safe flight corridor.");
            return false;
        }

        // Cruise through intermediate horizons; brake on the terminal horizon.
        Eigen::Vector3d localTargetVel = Eigen::Vector3d::Zero();
        const Eigen::Vector3d tangent = route.back() - route[route.size() - 2];
        const bool routeTruncated = (localTarget - routeEnd).norm() > 1.0e-6;
        if (routeTruncated && tangent.norm() > 1.0e-6)
        {
            localTargetVel = config.maxVelMag * tangent.normalized();
        }
        terminal = !routeTruncated;

        Eigen::Matrix3d iniState;
        Eigen::Matrix3d finState;
        iniState << start, startVel, startAcc;
        finState << localTarget, localTargetVel, Eigen::Vector3d::Zero();

        gcopter::GCOPTER_PolytopeSFC gcopter;

        Eigen::VectorXd magnitudeBounds(3);
        Eigen::VectorXd penaltyWeights(4);
        magnitudeBounds(0) = config.maxVelMag;
        magnitudeBounds(1) = config.maxBdrMag;
        magnitudeBounds(2) = config.maxTiltAngle;
        penaltyWeights(0) = config.chiVec[0];
        penaltyWeights(1) = config.chiVec[1];
        penaltyWeights(2) = config.chiVec[2];
        penaltyWeights(3) = config.chiVec[3];

        if (!gcopter.setup(config.weightT,
                           iniState,
                           finState,
                           hPolys,
                           INFINITY,
                           config.smoothingEps,
                           config.integralIntervs,
                           magnitudeBounds,
                           penaltyWeights,
                           config.gravAcc))
        {
            ROS_WARN_THROTTLE(1.0, "GCOPTER setup failed.");
            return false;
        }

        candidateTraj.clear();
        if (std::isinf(gcopter.optimize(candidateTraj, config.relCostTol)))
        {
            ROS_WARN_THROTTLE(1.0, "GCOPTER optimization failed.");
            return false;
        }

        if (candidateTraj.getPieceNum() <= 0)
        {
            return false;
        }

        // Enforce the velocity limit on the continuous polynomial.
        const double optimizedMaxVel = candidateTraj.getMaxVelRate();
        if (!std::isfinite(optimizedMaxVel) || config.maxVelMag <= 0.0)
        {
            ROS_WARN_THROTTLE(1.0, "Optimized trajectory has an invalid velocity bound.");
            return false;
        }

        if (optimizedMaxVel > config.maxVelMag)
        {
            constexpr double velocityMargin = 0.99;
            const bool significantViolation =
                optimizedMaxVel > config.maxVelMag * 1.01;
            const double timeScale =
                optimizedMaxVel / (velocityMargin * config.maxVelMag);
            candidateTraj = timeScaledTrajectory(candidateTraj, timeScale);

            const double scaledMaxVel = candidateTraj.getMaxVelRate();
            if (!std::isfinite(scaledMaxVel) ||
                scaledMaxVel > config.maxVelMag * (1.0 + 1.0e-6))
            {
                ROS_WARN_THROTTLE(1.0, "Failed to enforce the trajectory velocity limit.");
                return false;
            }

            if (significantViolation)
            {
                ROS_WARN_THROTTLE(1.0,
                                  "Time-scaled online trajectory by %.3f: max velocity "
                                  "%.3f -> %.3f m/s (limit %.3f m/s).",
                                  timeScale, optimizedMaxVel, scaledMaxVel, config.maxVelMag);
            }
            else
            {
                ROS_DEBUG("Time-scaled online trajectory by %.3f to preserve the "
                          "%.3f m/s velocity limit.",
                          timeScale, config.maxVelMag);
            }
        }

        return true;
    }

    inline void replanTimerCallback(const ros::TimerEvent &)
    {
        if (!targetReceived || !odomReceived || !mapInitialized)
        {
            return;
        }

        if (goalLatched_)
        {
            return;
        }

        // Seed replanning from the active trajectory for C2 continuity.
        const double handoverTime = ros::Time::now().toSec();
        Eigen::Vector3d start = latestPositionWorld;
        Eigen::Vector3d startVel = Eigen::Vector3d::Zero();
        Eigen::Vector3d startAcc = Eigen::Vector3d::Zero();
        if (hasActiveTraj(handoverTime))
        {
            const double tCur =
                std::max(0.0, std::min(handoverTime - trajStamp, traj.getTotalDuration()));
            start = traj.getPos(tCur);
            startVel = traj.getVel(tCur);
            startAcc = traj.getAcc(tCur);
        }

        // Avoid degenerate near-goal planning.
        if ((start - globalTarget_).norm() <= config.goalTolerance)
        {
            goalLatched_ = true;
            return;
        }

        if (voxelMap.query(start) != 0)
        {
            ROS_WARN_THROTTLE(1.0, "Current planning start is outside the map or in collision.");
            return;
        }

        Trajectory<5> candidateTraj;
        std::vector<Eigen::Vector3d> route;
        std::vector<Eigen::MatrixX4d> hPolys;

        bool terminal = false;
        const auto planningStart = std::chrono::steady_clock::now();
        const bool planningSucceeded =
            computeTrajectory(start, startVel, startAcc, globalTarget_,
                              candidateTraj, route, hPolys, terminal);
        const double planningTimeMs =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - planningStart)
                .count();
        ROS_INFO("GCOPTER online planning %s in %.3f ms.",
                 planningSucceeded ? "succeeded" : "failed",
                 planningTimeMs);

        // Keep executing the previous trajectory if replanning fails.
        if (!planningSucceeded)
        {
            return;
        }

        traj = candidateTraj;
        trajStamp = handoverTime;
        publishTrajectory(handoverTime);

        // Latch after publishing the terminal braking trajectory.
        if (terminal)
        {
            goalLatched_ = true;
            const Eigen::Vector3d endPos = traj.getPos(traj.getTotalDuration());
            if ((endPos - globalTarget_).norm() > config.goalTolerance)
            {
                ROS_WARN("Global target unreachable; stopping at the closest safe approach "
                         "[%.2f, %.2f, %.2f].",
                         endPos.x(), endPos.y(), endPos.z());
            }
        }

        visualizer.visualizeStartGoal(start, 0.05, 0);
        visualizer.visualizeStartGoal(globalTarget_, 0.05, 1);
        if (targetWasClamped_)
        {
            visualizer.visualizeStartGoal(requestedTarget_, 0.05, 2);
        }
        if (config.showPolytopeCorridor)
        {
            visualizer.visualizePolytope(hPolys);
        }
        visualizer.visualize(traj, route);
    }

    // Prefer the active trajectory over odometry for handover state sampling.
    inline bool hasActiveTraj(const double now) const
    {
        return traj.getPieceNum() > 0 && (now - trajStamp) < traj.getTotalDuration();
    }

    // Publish the optimized polynomial for asynchronous execution.
    inline void publishTrajectory(const double startTime)
    {
        gcopter::PolyTraj msg;
        msg.start_time = ros::Time(startTime);
        msg.traj_id = trajId_++;
        msg.order = 5;

        const int coefPerPiece = 6; // order + 1
        const int n = traj.getPieceNum();
        msg.durations.reserve(n);
        msg.coef_x.reserve(n * coefPerPiece);
        msg.coef_y.reserve(n * coefPerPiece);
        msg.coef_z.reserve(n * coefPerPiece);
        for (int p = 0; p < n; ++p)
        {
            const Piece<5> &piece = traj[p];
            const Piece<5>::CoefficientMat coeff = piece.getCoeffMat();
            msg.durations.push_back(piece.getDuration());
            for (int j = 0; j < coefPerPiece; ++j)
            {
                msg.coef_x.push_back(coeff(0, j));
                msg.coef_y.push_back(coeff(1, j));
                msg.coef_z.push_back(coeff(2, j));
            }
        }

        trajPub.publish(msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "online_replanning_node");
    ros::NodeHandle nh;
    OnlineReplanner replanner(OnlinePlannerConfig(ros::NodeHandle("~")), nh);
    ros::spin();
    return 0;
}
