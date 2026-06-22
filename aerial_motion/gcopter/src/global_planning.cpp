#include "misc/visualizer.hpp"
#include "misc/tf_utils.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

#include <gcopter/PolyTraj.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

struct Config
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
    bool useFixedTargetHeight;
    double targetHeight;
    bool useTargetZ;
    bool showPolytopeCorridor;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.param<std::string>("WorldFrameId", worldFrameId, "world");
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("VoxelWidth", voxelWidth);
        nh_priv.getParam("MapBound", mapBound);
        nh_priv.getParam("TimeoutRRT", timeoutRRT);
        nh_priv.getParam("MaxVelMag", maxVelMag);
        nh_priv.getParam("MaxBdrMag", maxBdrMag);
        nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
        nh_priv.getParam("GravAcc", gravAcc);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("ChiVec", chiVec);
        nh_priv.getParam("SmoothingEps", smoothingEps);
        nh_priv.getParam("IntegralIntervs", integralIntervs);
        nh_priv.getParam("RelCostTol", relCostTol);
        nh_priv.param("CommandHz", commandHz, 40.0);
        nh_priv.param("PublishYawCommand", publishYawCommand, false);
        nh_priv.param("UseFixedTargetHeight", useFixedTargetHeight, false);
        nh_priv.param("TargetHeight", targetHeight, 1.0);
        nh_priv.param("UseTargetZ", useTargetZ, false);
        nh_priv.param("ShowPolytopeCorridor", showPolytopeCorridor, true);
    }
};

class GlobalPlanner
{
private:
    Config config;

    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    ros::Subscriber mapSub;
    ros::Subscriber targetSub;
    ros::Subscriber odomSub;
    ros::Publisher trajPub;

    bool mapInitialized;
    bool odomReceived;
    voxel_map::VoxelMap voxelMap;
    Visualizer visualizer;
    std::vector<Eigen::Vector3d> startGoal;

    Trajectory<5> traj;
    Eigen::Vector3d latestPosition;
    int trajId_;

public:
    GlobalPlanner(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          tfListener(tfBuffer),
          mapInitialized(false),
          odomReceived(false),
          visualizer(nh, config.worldFrameId),
          latestPosition(Eigen::Vector3d::Zero()),
          trajId_(0)
    {
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

        mapSub = nh.subscribe("pcl_topic", 1, &GlobalPlanner::mapCallBack, this,
                              ros::TransportHints().tcpNoDelay());

        targetSub = nh.subscribe("target", 1, &GlobalPlanner::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());

        odomSub = nh.subscribe("odom", 1, &GlobalPlanner::odomCallBack, this,
                               ros::TransportHints().tcpNoDelay());

        trajPub = nh.advertise<gcopter::PolyTraj>("trajectory", 10);
    }

    inline Eigen::Quaterniond normalizedQuaternion(const geometry_msgs::Quaternion &q) const
    {
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        if (quat.norm() < 1.0e-9)
        {
            return Eigen::Quaterniond::Identity();
        }
        return quat.normalized();
    }

    inline void odomCallBack(const nav_msgs::Odometry::ConstPtr &msg)
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

        const Eigen::Isometry3d worldBody = worldOdomRef * odomRefBody;
        latestPosition = worldBody.translation();
        odomReceived = true;
    }

    inline void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (!mapInitialized)
        {
            if (msg->data.empty() || msg->point_step < 3 * sizeof(float))
            {
                ROS_WARN("Received empty or invalid point cloud map.");
                return;
            }

            // Transform the map into the planning frame.
            Eigen::Isometry3d worldCloud;
            if (!tf_utils::resolveToWorld(tfBuffer, config.worldFrameId,
                                          msg->header.frame_id, worldCloud))
            {
                return; // leave uninitialized; retry on the next cloud
            }

            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;
            const float *fdata = reinterpret_cast<const float *>(&msg->data[0]);
            for (size_t i = 0; i < total; i++)
            {
                cur = msg->point_step / sizeof(float) * i;

                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                {
                    continue;
                }
                voxelMap.setOccupied(worldCloud * Eigen::Vector3d(fdata[cur + 0],
                                                                  fdata[cur + 1],
                                                                  fdata[cur + 2]));
            }

            voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));

            mapInitialized = true;
        }
    }

    inline bool plan(const double startTime)
    {
        if (startGoal.size() == 2)
        {
            traj.clear();

            std::vector<Eigen::Vector3d> route;
            sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
                                                   startGoal[1],
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &voxelMap, config.timeoutRRT,
                                                   route);
            std::vector<Eigen::MatrixX4d> hPolys;
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

            if (route.size() > 1)
            {
                if (config.showPolytopeCorridor)
                {
                    visualizer.visualizePolytope(hPolys);
                }

                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

                gcopter::GCOPTER_PolytopeSFC gcopter;

                // Bounds: velocity, body rate, tilt; weights: position and dynamics.
                Eigen::VectorXd magnitudeBounds(3);
                Eigen::VectorXd penaltyWeights(4);
                magnitudeBounds(0) = config.maxVelMag;
                magnitudeBounds(1) = config.maxBdrMag;
                magnitudeBounds(2) = config.maxTiltAngle;
                penaltyWeights(0) = (config.chiVec)[0];
                penaltyWeights(1) = (config.chiVec)[1];
                penaltyWeights(2) = (config.chiVec)[2];
                penaltyWeights(3) = (config.chiVec)[3];
                const int quadratureRes = config.integralIntervs;

                if (!gcopter.setup(config.weightT,
                                   iniState, finState,
                                   hPolys, INFINITY,
                                   config.smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   config.gravAcc))
                {
                    return false;
                }

                if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
                {
                    return false;
                }

                if (traj.getPieceNum() > 0)
                {
                    publishTrajectory(startTime);
                    visualizer.visualize(traj, route);
                    return true;
                }
            }
        }

        return false;
    }

    inline double getTargetHeight(const geometry_msgs::PoseStamped::ConstPtr &msg) const
    {
        if (config.useFixedTargetHeight)
        {
            return config.targetHeight;
        }

        if (config.useTargetZ)
        {
            return msg->pose.position.z;
        }

        return config.mapBound[4] + config.dilateRadius +
               fabs(msg->pose.orientation.z) *
                   (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);
    }

    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (!mapInitialized)
        {
            ROS_WARN("Map is not initialized yet. Ignore target.");
            return;
        }

        if (!odomReceived)
        {
            ROS_WARN("No odometry received from %s. Ignore target.", nh.resolveName("odom").c_str());
            return;
        }

        // Anchor trajectory time to the sampled start state.
        const double t0 = ros::Time::now().toSec();
        const Eigen::Vector3d start = latestPosition;
        const Eigen::Vector3d goal(msg->pose.position.x,
                                   msg->pose.position.y,
                                   getTargetHeight(msg));

        if (voxelMap.query(start) != 0)
        {
            ROS_WARN("Current odometry position is outside the map or in collision. Ignore target.");
            return;
        }

        if (voxelMap.query(goal) != 0)
        {
            ROS_WARN("Infeasible target position selected.");
            return;
        }

        startGoal.clear();
        visualizer.visualizeStartGoal(start, 0.05, 0);
        visualizer.visualizeStartGoal(goal, 0.05, 1);
        startGoal.emplace_back(start);
        startGoal.emplace_back(goal);

        const auto planningStart = std::chrono::steady_clock::now();
        const bool planningSucceeded = plan(t0);
        const double planningTimeMs =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - planningStart)
                .count();
        ROS_INFO("GCOPTER global planning %s in %.3f ms.",
                 planningSucceeded ? "succeeded" : "failed",
                 planningTimeMs);
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
    ros::init(argc, argv, "global_planning_node");
    ros::NodeHandle nh_;

    GlobalPlanner global_planner(Config(ros::NodeHandle("~")), nh_);

    ros::spin();

    return 0;
}
