#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

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
    ros::Subscriber mapSub;
    ros::Subscriber targetSub;
    ros::Subscriber odomSub;
    ros::Publisher commandPub;

    bool mapInitialized;
    bool odomReceived;
    bool commandActive;
    voxel_map::VoxelMap voxelMap;
    Visualizer visualizer;
    std::vector<Eigen::Vector3d> startGoal;

    Trajectory<5> traj;
    double trajStamp;
    Eigen::Vector3d latestPosition;
    double lastYaw;

    ros::Timer commandTimer;
    flatness::FlatnessMap flatmap;

public:
    GlobalPlanner(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          mapInitialized(false),
          odomReceived(false),
          commandActive(false),
          visualizer(nh, config.worldFrameId),
          latestPosition(Eigen::Vector3d::Zero()),
          lastYaw(0.0)
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

        commandPub = nh.advertise<geometry_msgs::PoseStamped>("command", 10);

        flatmap.reset(config.gravAcc);

        const double hz = config.commandHz > 0.0 ? config.commandHz : 40.0;
        commandTimer = nh.createTimer(ros::Duration(1.0 / hz),
                                      &GlobalPlanner::commandTimerCallback, this);
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

    inline double yawFromRotation(const Eigen::Matrix3d &rot) const
    {
        const Eigen::Quaterniond q(rot);
        return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    }

    inline void odomCallBack(const nav_msgs::Odometry::ConstPtr &msg)
    {
        const geometry_msgs::Point &p = msg->pose.pose.position;
        Eigen::Isometry3d odomPose = Eigen::Isometry3d::Identity();
        odomPose.translate(Eigen::Vector3d(p.x, p.y, p.z));
        odomPose.rotate(normalizedQuaternion(msg->pose.pose.orientation));

        latestPosition = odomPose.translation();
        odomReceived = true;

        // Track heading from odometry only until a trajectory takes over.
        if (traj.getPieceNum() <= 0)
        {
            lastYaw = yawFromRotation(odomPose.rotation());
        }
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
                voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                     fdata[cur + 1],
                                                     fdata[cur + 2]));
            }

            voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));

            mapInitialized = true;
        }
    }

    inline void plan()
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

                // magnitudeBounds = [v_max, omg_max, theta_max]^T
                // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight]^T
                // initialize some constraint parameters
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
                    return;
                }

                if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
                {
                    return;
                }

                if (traj.getPieceNum() > 0)
                {
                    trajStamp = ros::Time::now().toSec();
                    commandActive = true;
                    visualizer.visualize(traj, route);
                }
            }
        }
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
        plan();
    }

    // Stream a position/yaw setpoint from an already-sampled trajectory state.
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

    // Publish flatness-derived diagnostics for the current trajectory state.
    inline void publishDiagnostics(const double t,
                                   const Eigen::Vector3d &vel)
    {
        Eigen::Vector4d quat;
        Eigen::Vector3d omg;

        flatmap.forward(traj.getAcc(t),
                        traj.getJer(t),
                        0.0, 0.0,
                        quat, omg);

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

        // Sample the trajectory once at the clamped time and reuse the state
        // for both the diagnostics and the pose command.
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
            // Final setpoint emitted; stop streaming until the next plan.
            commandActive = false;
        }
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
