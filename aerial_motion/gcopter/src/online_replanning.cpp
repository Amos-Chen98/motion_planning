#include "misc/visualizer.hpp"
#include "misc/tf_utils.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <algorithm>
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

inline double quaternionYaw(const Eigen::Quaterniond &q)
{
    const Eigen::Quaterniond quat = q.normalized();
    return std::atan2(2.0 * (quat.w() * quat.z() + quat.x() * quat.y()),
                      1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
}

inline bool isFinitePoint(const float x, const float y, const float z)
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
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
    double replanHz;
    bool useAccumulatedMap;
    bool useFixedTargetHeight;
    double targetHeight;
    bool useTargetZ;
    bool showPolytopeCorridor;
    double goalTolerance;
    double planningHorizon;
    double noReplanRadius;
    int stallReplanLimit;

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
        nhPriv.param("ReplanHz", replanHz, 2.0);
        nhPriv.param("UseAccumulatedMap", useAccumulatedMap, true);
        nhPriv.param("UseFixedTargetHeight", useFixedTargetHeight, false);
        nhPriv.param("TargetHeight", targetHeight, 1.0);
        nhPriv.param("UseTargetZ", useTargetZ, false);
        nhPriv.param("ShowPolytopeCorridor", showPolytopeCorridor, true);
        nhPriv.param("GoalTolerance", goalTolerance, 0.2);
        nhPriv.param("PlanningHorizon", planningHorizon, 6.0);
        nhPriv.param("NoReplanRadius", noReplanRadius, 1.0);
        nhPriv.param("StallReplanLimit", stallReplanLimit, 5);
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
    ros::Publisher commandPub;
    ros::Timer commandTimer;
    ros::Timer replanTimer;

    voxel_map::VoxelMap voxelMap;
    Visualizer visualizer;
    flatness::FlatnessMap flatmap;

    std::unordered_set<long> occupiedVoxelKeys;
    int dilateVoxelRadius;

    Eigen::Isometry3d latestWorldBody;
    Eigen::Vector3d latestPositionWorld;
    Eigen::Vector3d latestGoalWorld;
    bool odomReceived;
    bool mapInitialized;
    bool targetReceived;
    bool commandActive;

    Trajectory<5> traj;
    double trajStamp;
    double lastYaw;

    // Unreachable-goal handling (e.g. target set inside an obstacle): track the best
    // approach toward the goal and freeze once we can no longer make progress, so the
    // robot stops at the end of its last known-safe trajectory instead of looping.
    double bestGoalApproach_;
    int stalledReplans_;
    bool holdAtSafeEnd_;

public:
    OnlineReplanner(const OnlinePlannerConfig &conf, ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          tfListener(tfBuffer),
          visualizer(nh, config.worldFrameId),
          latestWorldBody(Eigen::Isometry3d::Identity()),
          latestPositionWorld(Eigen::Vector3d::Zero()),
          latestGoalWorld(Eigen::Vector3d::Zero()),
          odomReceived(false),
          mapInitialized(false),
          targetReceived(false),
          commandActive(false),
          trajStamp(0.0),
          lastYaw(0.0),
          bestGoalApproach_(std::numeric_limits<double>::infinity()),
          stalledReplans_(0),
          holdAtSafeEnd_(false)
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
        commandPub = nh.advertise<geometry_msgs::PoseStamped>("command", 10);

        flatmap.reset(config.gravAcc);

        const double commandHz = config.commandHz > 0.0 ? config.commandHz : 40.0;
        commandTimer = nh.createTimer(ros::Duration(1.0 / commandHz),
                                      &OnlineReplanner::commandTimerCallback, this);

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
        // The odom pose is expressed in msg->header.frame_id (its reference frame);
        // register that frame to the world frame so downstream planning is always
        // done in WorldFrameId, regardless of the odom source's convention.
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

        if (traj.getPieceNum() <= 0)
        {
            lastYaw = quaternionYaw(Eigen::Quaterniond(latestWorldBody.rotation()));
        }

        odomReceived = true;
    }

    inline void localMapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        // Register the cloud's own sensor frame (msg->header.frame_id) to the world
        // frame via TF, so the sensor need not coincide with the odom body frame.
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
        latestGoalWorld = Eigen::Vector3d(msg->pose.position.x,
                                          msg->pose.position.y,
                                          getTargetHeight(*msg));
        targetReceived = true;

        // A fresh target re-arms the unreachable-goal logic.
        bestGoalApproach_ = std::numeric_limits<double>::infinity();
        stalledReplans_ = 0;
        holdAtSafeEnd_ = false;

        visualizer.visualizeStartGoal(latestGoalWorld, 0.05, 1);
        ROS_INFO("Received online replanning target in %s: [%.2f, %.2f, %.2f].",
                 config.worldFrameId.c_str(),
                 latestGoalWorld.x(), latestGoalWorld.y(), latestGoalWorld.z());
    }

    // Clamp a (possibly far/out-of-bounds) point into the valid planning volume so
    // that RRT always receives an in-bounds goal. Unknown space inside the grid is
    // treated as free, hence flyable.
    inline Eigen::Vector3d clampInsideMap(const Eigen::Vector3d &pt) const
    {
        const Eigen::Vector3d offs = Eigen::Vector3d::Constant(config.dilateRadius + voxelMap.getScale());
        const Eigen::Vector3d lo = voxelMap.getOrigin() + offs;
        const Eigen::Vector3d hi = voxelMap.getCorner() - offs;
        return pt.cwiseMax(lo).cwiseMin(hi);
    }

    // Truncate a geometric route to the planning horizon (by arc length). The last
    // kept point is interpolated to land exactly at the horizon distance.
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

        // Whole route is within the horizon: local target is the route end (goal).
        return local.back();
    }

    inline bool computeTrajectory(const Eigen::Vector3d &start,
                                  const Eigen::Vector3d &startVel,
                                  const Eigen::Vector3d &startAcc,
                                  const Eigen::Vector3d &goal,
                                  Trajectory<5> &candidateTraj,
                                  std::vector<Eigen::Vector3d> &route,
                                  std::vector<Eigen::MatrixX4d> &hPolys,
                                  bool &reachedGoal)
    {
        reachedGoal = false;
        const Eigen::Vector3d plannerGoal = clampInsideMap(goal);

        std::vector<Eigen::Vector3d> fullRoute;
        try
        {
            sfc_gen::planPath<voxel_map::VoxelMap>(start,
                                                   plannerGoal,
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &voxelMap,
                                                   config.timeoutRRT,
                                                   fullRoute);
        }
        catch (const std::exception &e)
        {
            // OMPL can throw (e.g. degenerate prolate hyperspheroid) for ill-posed
            // queries; never let it abort the planner node.
            ROS_WARN_THROTTLE(1.0, "Path search threw an exception: %s", e.what());
            return false;
        }

        if (fullRoute.size() <= 1)
        {
            ROS_WARN_THROTTLE(1.0, "RRT did not produce a usable route.");
            return false;
        }

        // RRT only returns an approximate path when the goal is unreachable (e.g. set
        // inside an obstacle): its endpoint then falls short of the goal.
        const Eigen::Vector3d routeEnd = fullRoute.back();
        reachedGoal = (routeEnd - plannerGoal).norm() <= config.goalTolerance;

        // Keep only one horizon worth of the route; optimize a local trajectory.
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

        // Local target velocity. While the route is longer than the planning horizon the
        // local target is only an intermediate waypoint, so keep full cruising momentum
        // (maxVel) along the route tangent. Once the whole route fits inside the horizon
        // the local target coincides with the route's actual endpoint -- the goal, or the
        // best safe approach when the goal is unreachable -- and we command zero terminal
        // velocity, letting GCOPTER brake smoothly to a stop over the remaining
        // (<= PlanningHorizon) distance. Keying the stop on routeEnd (not the raw goal)
        // still yields a clean zero-velocity halt at the best safe approach when the goal
        // sits inside an obstacle.
        Eigen::Vector3d localTargetVel = Eigen::Vector3d::Zero();
        const Eigen::Vector3d tangent = route.back() - route[route.size() - 2];
        const bool routeTruncated = (localTarget - routeEnd).norm() > 1.0e-6;
        if (routeTruncated && tangent.norm() > 1.0e-6)
        {
            localTargetVel = config.maxVelMag * tangent.normalized();
        }

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

        return candidateTraj.getPieceNum() > 0;
    }

    inline void replanTimerCallback(const ros::TimerEvent &)
    {
        if (!targetReceived || !odomReceived || !mapInitialized)
        {
            return;
        }

        const Eigen::Vector3d goal = latestGoalWorld;

        // Arrival is judged on the real (odometry) position.
        if ((latestPositionWorld - goal).norm() <= config.goalTolerance)
        {
            commandActive = false;
            traj.clear();
            publishPoseCommand(goal, Eigen::Vector3d::Zero());
            return;
        }

        // The goal was found unreachable and we already stopped making progress: keep
        // executing the last known-safe trajectory (it ends with zero velocity at the
        // best safe approach) and stop replanning until a new target arrives.
        if (holdAtSafeEnd_)
        {
            return;
        }

        // Sample the start state from the trajectory currently being executed so the
        // new local trajectory continues its position/velocity/acceleration (C2). The
        // sampling instant tNow becomes the new trajStamp for seamless handover.
        const double tNow = ros::Time::now().toSec();
        Eigen::Vector3d start = latestPositionWorld;
        Eigen::Vector3d startVel = Eigen::Vector3d::Zero();
        Eigen::Vector3d startAcc = Eigen::Vector3d::Zero();
        if (commandActive && traj.getPieceNum() > 0)
        {
            const double tCur = std::max(0.0, std::min(tNow - trajStamp, traj.getTotalDuration()));
            start = traj.getPos(tCur);
            startVel = traj.getVel(tCur);
            startAcc = traj.getAcc(tCur);
        }

        // Final-approach no-replan zone (mirrors EGO-Planner's no_replan_thresh): once
        // we are within NoReplanRadius of the goal, keep executing the committed
        // trajectory (which already ends at the goal with zero velocity) instead of
        // replanning. Replanning here would degenerate to an (almost) start==goal
        // problem, making RRT's prolate-hyperspheroid throw and GCOPTER diverge to
        // NaN/Inf as the robot lands on the target.
        if (commandActive && traj.getPieceNum() > 0 &&
            (start - goal).norm() <= config.noReplanRadius)
        {
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

        bool reachedGoal = false;
        if (!computeTrajectory(start, startVel, startAcc, goal, candidateTraj, route, hPolys, reachedGoal))
        {
            return;
        }

        // Unreachable-goal arbitration: if the route cannot reach the goal and the
        // robot is no longer getting any closer to it, give up and hold at the safe
        // trajectory end rather than oscillating near the obstacle forever.
        if (reachedGoal)
        {
            bestGoalApproach_ = std::numeric_limits<double>::infinity();
            stalledReplans_ = 0;
        }
        else
        {
            const double distToGoal = (start - goal).norm();
            if (distToGoal < bestGoalApproach_ - 0.1)
            {
                bestGoalApproach_ = distToGoal; // still closing in on the goal
                stalledReplans_ = 0;
            }
            else if (++stalledReplans_ >= config.stallReplanLimit)
            {
                holdAtSafeEnd_ = true;
                ROS_WARN("Global target appears unreachable; holding at the safe trajectory end.");
                return; // keep the last known-safe trajectory; do not commit a new one
            }
        }

        traj = candidateTraj;
        trajStamp = tNow;
        commandActive = true;

        visualizer.visualizeStartGoal(start, 0.05, 0);
        visualizer.visualizeStartGoal(goal, 0.05, 1);
        if (config.showPolytopeCorridor)
        {
            visualizer.visualizePolytope(hPolys);
        }
        visualizer.visualize(traj, route);
    }

    inline void publishPoseCommand(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel)
    {
        if (vel(0) * vel(0) + vel(1) * vel(1) > 1.0e-6)
        {
            lastYaw = std::atan2(vel(1), vel(0));
        }

        geometry_msgs::PoseStamped poseMsg;
        poseMsg.header.frame_id = config.worldFrameId;
        poseMsg.header.stamp = ros::Time::now();

        poseMsg.pose.position.x = pos(0);
        poseMsg.pose.position.y = pos(1);
        poseMsg.pose.position.z = pos(2);
        poseMsg.pose.orientation.x = 0.0;
        poseMsg.pose.orientation.y = 0.0;
        poseMsg.pose.orientation.z = std::sin(0.5 * lastYaw);
        poseMsg.pose.orientation.w = std::cos(0.5 * lastYaw);

        commandPub.publish(poseMsg);
    }

    inline void publishDiagnostics(const double t, const Eigen::Vector3d &vel)
    {
        Eigen::Vector4d quat;
        Eigen::Vector3d omg;

        flatmap.forward(traj.getAcc(t),
                        traj.getJer(t),
                        0.0,
                        0.0,
                        quat,
                        omg);

        std_msgs::Float64 speedMsg, tiltMsg, bdrMsg;
        speedMsg.data = vel.norm();
        tiltMsg.data = std::acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)));
        bdrMsg.data = omg.norm();
        visualizer.speedPub.publish(speedMsg);
        visualizer.tiltPub.publish(tiltMsg);
        visualizer.bdrPub.publish(bdrMsg);
    }

    inline void commandTimerCallback(const ros::TimerEvent &)
    {
        if (!commandActive || traj.getPieceNum() <= 0)
        {
            return;
        }

        const double duration = traj.getTotalDuration();
        const double delta = ros::Time::now().toSec() - trajStamp;
        const bool finished = delta >= duration;
        const double t = std::max(0.0, std::min(delta, duration));

        const Eigen::Vector3d pos = traj.getPos(t);
        const Eigen::Vector3d vel = traj.getVel(t);

        if (delta > 0.0 && !finished)
        {
            publishDiagnostics(t, vel);
        }

        publishPoseCommand(pos, vel);

        if (finished)
        {
            commandActive = false;
        }
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
