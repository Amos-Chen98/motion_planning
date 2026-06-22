#include "gcopter/planner_common.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace
{

struct OnlinePlannerConfig
{
    gcopter_planner::CommonPlannerConfig common;
    double replanHz = 2.0;
    bool useAccumulatedMap = true;
    double goalTolerance = 0.2;
    double planningHorizon = 6.0;

    explicit OnlinePlannerConfig(const ros::NodeHandle &nhPriv)
        : common(nhPriv)
    {
        nhPriv.param("ReplanHz", replanHz, 2.0);
        nhPriv.param("UseAccumulatedMap", useAccumulatedMap, true);
        nhPriv.param("GoalTolerance", goalTolerance, 0.2);
        nhPriv.param("PlanningHorizon", planningHorizon, 6.0);

        common.validateOrThrow();
        if (!std::isfinite(replanHz) || replanHz <= 0.0)
        {
            throw std::invalid_argument("ReplanHz must be finite and positive");
        }
        if (!std::isfinite(goalTolerance) || goalTolerance <= 0.0)
        {
            throw std::invalid_argument(
                "GoalTolerance must be finite and positive");
        }
        if (!std::isfinite(planningHorizon) || planningHorizon <= 0.0)
        {
            throw std::invalid_argument(
                "PlanningHorizon must be finite and positive");
        }
    }
};

class OnlineReplanner
{
public:
    OnlineReplanner(const OnlinePlannerConfig &config,
                    ros::NodeHandle &nh)
        : config_(config),
          nh_(nh),
          backend_(config_.common),
          rosInterface_(config_.common, nh_),
          mapInitialized_(false),
          targetReceived_(false),
          trajectoryStamp_(0.0),
          globalTarget_(Eigen::Vector3d::Zero()),
          requestedTarget_(Eigen::Vector3d::Zero()),
          targetWasClamped_(false),
          goalLatched_(false)
    {
        localMapSub_ = nh_.subscribe(
            "pcl_topic", 1, &OnlineReplanner::localMapCallback, this,
            ros::TransportHints().tcpNoDelay());
        targetSub_ = nh_.subscribe(
            "target", 1, &OnlineReplanner::targetCallback, this,
            ros::TransportHints().tcpNoDelay());
        replanTimer_ = nh_.createTimer(
            ros::Duration(1.0 / config_.replanHz),
            &OnlineReplanner::replanTimerCallback, this);
    }

private:
    void rebuildVoxelMap()
    {
        std::vector<Eigen::Vector3i> occupiedVoxelIds;
        occupiedVoxelIds.reserve(occupiedVoxelKeys_.size());
        for (const long key : occupiedVoxelKeys_)
        {
            occupiedVoxelIds.push_back(backend_.voxelIdFromKey(key));
        }
        backend_.setMapVoxels(occupiedVoxelIds);
        mapInitialized_ = true;
    }

    void localMapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        std::vector<Eigen::Vector3d> pointsWorld;
        std::string error;
        if (!rosInterface_.pointCloudToWorld(*msg, pointsWorld, &error))
        {
            if (error != "point-cloud transform is unavailable")
            {
                ROS_WARN("Invalid local point cloud: %s", error.c_str());
            }
            return;
        }

        if (!config_.useAccumulatedMap)
        {
            occupiedVoxelKeys_.clear();
        }

        size_t accepted = 0;
        for (const Eigen::Vector3d &pointWorld : pointsWorld)
        {
            const long key = backend_.voxelKey(pointWorld);
            if (key >= 0)
            {
                occupiedVoxelKeys_.insert(key);
                ++accepted;
            }
        }

        rebuildVoxelMap();
        ROS_DEBUG("Integrated %zu local points into %zu occupied voxels.",
                  accepted, occupiedVoxelKeys_.size());
    }

    void targetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        requestedTarget_ = Eigen::Vector3d(
            msg->pose.position.x,
            msg->pose.position.y,
            config_.common.resolveTargetHeight(*msg));
        globalTarget_ = backend_.clampInsideMap(
            requestedTarget_,
            config_.common.dilateRadius + backend_.voxelScale());
        targetWasClamped_ =
            (globalTarget_ - requestedTarget_).norm() > 1.0e-3;
        targetReceived_ = true;
        goalLatched_ = false;

        if (targetWasClamped_)
        {
            ROS_WARN("Requested target [%.2f, %.2f, %.2f] is outside the planning "
                     "volume; clamped to [%.2f, %.2f, %.2f]. Planning to the "
                     "clamped target.",
                     requestedTarget_.x(), requestedTarget_.y(),
                     requestedTarget_.z(),
                     globalTarget_.x(), globalTarget_.y(), globalTarget_.z());
        }

        rosInterface_.visualizer().visualizeStartGoal(
            globalTarget_, 0.05, 1);
        if (targetWasClamped_)
        {
            rosInterface_.visualizer().visualizeStartGoal(
                requestedTarget_, 0.05, 2);
        }
        ROS_INFO("Received online replanning target in %s: [%.2f, %.2f, %.2f].",
                 config_.common.worldFrameId.c_str(),
                 globalTarget_.x(), globalTarget_.y(), globalTarget_.z());
    }

    Eigen::Vector3d truncateRouteToHorizon(
        const std::vector<Eigen::Vector3d> &fullRoute,
        std::vector<Eigen::Vector3d> &localRoute) const
    {
        localRoute.clear();
        localRoute.push_back(fullRoute.front());

        double accumulated = 0.0;
        for (size_t index = 1; index < fullRoute.size(); ++index)
        {
            const double segmentLength =
                (fullRoute[index] - fullRoute[index - 1]).norm();
            if (accumulated + segmentLength >= config_.planningHorizon)
            {
                const double remaining =
                    config_.planningHorizon - accumulated;
                const double ratio =
                    segmentLength > 1.0e-6
                        ? remaining / segmentLength
                        : 0.0;
                localRoute.push_back(
                    fullRoute[index - 1] +
                    ratio * (fullRoute[index] - fullRoute[index - 1]));
                return localRoute.back();
            }

            accumulated += segmentLength;
            localRoute.push_back(fullRoute[index]);
        }

        return localRoute.back();
    }

    bool computeTrajectory(
        const Eigen::Vector3d &start,
        const Eigen::Vector3d &startVelocity,
        const Eigen::Vector3d &startAcceleration,
        const Eigen::Vector3d &goal,
        Trajectory<5> &candidateTrajectory,
        std::vector<Eigen::Vector3d> &route,
        std::vector<Eigen::MatrixX4d> &hPolys,
        bool &terminal)
    {
        terminal = false;

        std::vector<Eigen::Vector3d> fullRoute;
        if (!backend_.searchPath(start, goal, fullRoute))
        {
            return false;
        }

        const Eigen::Vector3d routeEnd = fullRoute.back();
        const Eigen::Vector3d localTarget =
            truncateRouteToHorizon(fullRoute, route);
        if (route.size() < 2)
        {
            ROS_WARN_THROTTLE(1.0, "Local route is too short to plan.");
            return false;
        }
        if (!backend_.buildCorridor(route, hPolys))
        {
            return false;
        }

        Eigen::Vector3d localTargetVelocity = Eigen::Vector3d::Zero();
        const Eigen::Vector3d tangent =
            route.back() - route[route.size() - 2];
        const bool routeTruncated =
            (localTarget - routeEnd).norm() > 1.0e-6;
        if (routeTruncated && tangent.norm() > 1.0e-6)
        {
            localTargetVelocity =
                config_.common.maxVelMag * tangent.normalized();
        }
        terminal = !routeTruncated;

        Eigen::Matrix3d initialState;
        Eigen::Matrix3d finalState;
        initialState << start, startVelocity, startAcceleration;
        finalState << localTarget,
            localTargetVelocity,
            Eigen::Vector3d::Zero();

        return backend_.optimizeTrajectory(
            initialState, finalState, hPolys,
            candidateTrajectory, "online");
    }

    bool hasActiveTrajectory(const double now) const
    {
        return trajectory_.getPieceNum() > 0 &&
               (now - trajectoryStamp_) < trajectory_.getTotalDuration();
    }

    void replanTimerCallback(const ros::TimerEvent &)
    {
        if (!targetReceived_ ||
            !rosInterface_.odomReceived() ||
            !mapInitialized_ ||
            goalLatched_)
        {
            return;
        }

        const double handoverTime = ros::Time::now().toSec();
        Eigen::Vector3d start = rosInterface_.latestPosition();
        Eigen::Vector3d startVelocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d startAcceleration = Eigen::Vector3d::Zero();
        if (hasActiveTrajectory(handoverTime))
        {
            const double currentTime = std::max(
                0.0,
                std::min(handoverTime - trajectoryStamp_,
                         trajectory_.getTotalDuration()));
            start = trajectory_.getPos(currentTime);
            startVelocity = trajectory_.getVel(currentTime);
            startAcceleration = trajectory_.getAcc(currentTime);
        }

        if ((start - globalTarget_).norm() <= config_.goalTolerance)
        {
            goalLatched_ = true;
            return;
        }
        if (backend_.query(start))
        {
            ROS_WARN_THROTTLE(
                1.0,
                "Current planning start is outside the map or in collision.");
            return;
        }

        Trajectory<5> candidateTrajectory;
        std::vector<Eigen::Vector3d> route;
        std::vector<Eigen::MatrixX4d> hPolys;
        bool terminal = false;

        const auto planningStart = std::chrono::steady_clock::now();
        const bool succeeded = computeTrajectory(
            start, startVelocity, startAcceleration, globalTarget_,
            candidateTrajectory, route, hPolys, terminal);
        const double planningTimeMs =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - planningStart)
                .count();
        ROS_INFO("GCOPTER online planning %s in %.3f ms.",
                 succeeded ? "succeeded" : "failed", planningTimeMs);

        // Keep executing the previous trajectory when replanning fails.
        if (!succeeded)
        {
            return;
        }

        trajectory_ = candidateTrajectory;
        trajectoryStamp_ = handoverTime;
        rosInterface_.publishTrajectory(trajectory_, handoverTime);

        if (terminal)
        {
            goalLatched_ = true;
            const Eigen::Vector3d endPosition =
                trajectory_.getPos(trajectory_.getTotalDuration());
            if ((endPosition - globalTarget_).norm() >
                config_.goalTolerance)
            {
                ROS_WARN("Global target unreachable; stopping at the closest "
                         "safe approach [%.2f, %.2f, %.2f].",
                         endPosition.x(), endPosition.y(), endPosition.z());
            }
        }

        rosInterface_.visualizer().visualizeStartGoal(start, 0.05, 0);
        rosInterface_.visualizer().visualizeStartGoal(
            globalTarget_, 0.05, 1);
        if (targetWasClamped_)
        {
            rosInterface_.visualizer().visualizeStartGoal(
                requestedTarget_, 0.05, 2);
        }
        if (config_.common.showPolytopeCorridor)
        {
            rosInterface_.visualizer().visualizePolytope(hPolys);
        }
        rosInterface_.visualizer().visualize(trajectory_, route);
    }

    OnlinePlannerConfig config_;
    ros::NodeHandle nh_;
    gcopter_planner::PlannerBackend backend_;
    gcopter_planner::PlannerRosInterface rosInterface_;
    ros::Subscriber localMapSub_;
    ros::Subscriber targetSub_;
    ros::Timer replanTimer_;

    std::unordered_set<long> occupiedVoxelKeys_;
    bool mapInitialized_;
    bool targetReceived_;

    Trajectory<5> trajectory_;
    double trajectoryStamp_;

    Eigen::Vector3d globalTarget_;
    Eigen::Vector3d requestedTarget_;
    bool targetWasClamped_;
    bool goalLatched_;
};

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "online_replanning_node");
    ros::NodeHandle nh;

    try
    {
        OnlinePlannerConfig config(ros::NodeHandle("~"));
        OnlineReplanner replanner(config, nh);
        ros::spin();
    }
    catch (const std::exception &exception)
    {
        ROS_FATAL("Invalid GCOPTER online planner configuration: %s",
                  exception.what());
        return 1;
    }

    return 0;
}
