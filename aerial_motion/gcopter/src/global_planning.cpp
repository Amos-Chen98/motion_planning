#include "gcopter/planner_common.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Geometry>

#include <chrono>
#include <exception>
#include <string>
#include <vector>

namespace
{

class GlobalPlanner
{
public:
    GlobalPlanner(const gcopter_planner::CommonPlannerConfig &config,
                  ros::NodeHandle &nh)
        : config_(config),
          nh_(nh),
          backend_(config_),
          rosInterface_(config_, nh_),
          mapInitialized_(false)
    {
        mapSub_ = nh_.subscribe(
            "pcl_topic", 1, &GlobalPlanner::mapCallback, this,
            ros::TransportHints().tcpNoDelay());
        targetSub_ = nh_.subscribe(
            "target", 1, &GlobalPlanner::targetCallback, this,
            ros::TransportHints().tcpNoDelay());
    }

private:
    void mapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (mapInitialized_)
        {
            return;
        }

        std::vector<Eigen::Vector3d> pointsWorld;
        std::string error;
        if (!rosInterface_.pointCloudToWorld(*msg, pointsWorld, &error))
        {
            if (error != "point-cloud transform is unavailable")
            {
                ROS_WARN("Invalid global point cloud: %s", error.c_str());
            }
            return;
        }
        if (pointsWorld.empty())
        {
            ROS_WARN("Received a global point cloud with no finite XYZ points.");
            return;
        }

        backend_.setMapPoints(pointsWorld);
        mapInitialized_ = true;
        ROS_INFO("Initialized global voxel map from %zu finite points.",
                 pointsWorld.size());
    }

    bool plan(const Eigen::Vector3d &start,
              const Eigen::Vector3d &goal,
              const double startTime)
    {
        std::vector<Eigen::Vector3d> route;
        if (!backend_.searchPath(start, goal, route))
        {
            return false;
        }

        std::vector<Eigen::MatrixX4d> hPolys;
        if (!backend_.buildCorridor(route, hPolys))
        {
            return false;
        }

        Eigen::Matrix3d initialState;
        Eigen::Matrix3d finalState;
        initialState << route.front(),
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero();
        finalState << route.back(),
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero();

        if (!backend_.optimizeTrajectory(initialState, finalState,
                                         hPolys, trajectory_, "global"))
        {
            return false;
        }

        rosInterface_.publishTrajectory(trajectory_, startTime);
        if (config_.showPolytopeCorridor)
        {
            rosInterface_.visualizer().visualizePolytope(hPolys);
        }
        rosInterface_.visualizer().visualize(trajectory_, route);
        return true;
    }

    void targetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (!mapInitialized_)
        {
            ROS_WARN("Map is not initialized yet. Ignore target.");
            return;
        }
        if (!rosInterface_.odomReceived())
        {
            ROS_WARN("No odometry received from %s. Ignore target.",
                     rosInterface_.resolvedOdomTopic().c_str());
            return;
        }

        const double startTime = ros::Time::now().toSec();
        const Eigen::Vector3d start = rosInterface_.latestPosition();
        const Eigen::Vector3d goal(
            msg->pose.position.x,
            msg->pose.position.y,
            config_.resolveTargetHeight(*msg));

        if (backend_.query(start))
        {
            ROS_WARN("Current odometry position is outside the map or in collision. "
                     "Ignore target.");
            return;
        }
        if (backend_.query(goal))
        {
            ROS_WARN("Infeasible target position selected.");
            return;
        }

        rosInterface_.visualizer().visualizeStartGoal(start, 0.05, 0);
        rosInterface_.visualizer().visualizeStartGoal(goal, 0.05, 1);

        const auto planningStart = std::chrono::steady_clock::now();
        const bool succeeded = plan(start, goal, startTime);
        const double planningTimeMs =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - planningStart)
                .count();
        ROS_INFO("GCOPTER global planning %s in %.3f ms.",
                 succeeded ? "succeeded" : "failed", planningTimeMs);
    }

    gcopter_planner::CommonPlannerConfig config_;
    ros::NodeHandle nh_;
    gcopter_planner::PlannerBackend backend_;
    gcopter_planner::PlannerRosInterface rosInterface_;
    ros::Subscriber mapSub_;
    ros::Subscriber targetSub_;
    bool mapInitialized_;
    Trajectory<5> trajectory_;
};

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planning_node");
    ros::NodeHandle nh;

    try
    {
        gcopter_planner::CommonPlannerConfig config(ros::NodeHandle("~"));
        config.validateOrThrow();
        GlobalPlanner planner(config, nh);
        ros::spin();
    }
    catch (const std::exception &exception)
    {
        ROS_FATAL("Invalid GCOPTER global planner configuration: %s",
                  exception.what());
        return 1;
    }

    return 0;
}
