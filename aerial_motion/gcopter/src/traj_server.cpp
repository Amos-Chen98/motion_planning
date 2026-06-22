#include "gcopter/trajectory.hpp"
#include "gcopter/flatness.hpp"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <gcopter/PolyTraj.h>

#include <Eigen/Geometry>

#include <cmath>

// Execute planner trajectories in a separate process to keep command streaming responsive.
class TrajServer
{
private:
    ros::NodeHandle nh;
    ros::Subscriber trajSub;
    ros::Subscriber odomSub;
    ros::Publisher commandPub;
    ros::Publisher speedPub;
    ros::Publisher tiltPub;
    ros::Publisher bdrPub;
    ros::Timer commandTimer;

    flatness::FlatnessMap flatmap;

    std::string worldFrameId;
    bool publishYawCommand;

    Trajectory<5> traj;
    ros::Time trajStamp;
    int trajId;
    bool receiveTraj;
    double lastYaw;

public:
    TrajServer(ros::NodeHandle &nh_)
        : nh(nh_),
          trajId(-1),
          receiveTraj(false),
          lastYaw(0.0)
    {
        ros::NodeHandle nhPriv("~");
        double gravAcc = 9.8;
        double commandHz = 40.0;
        nhPriv.param("GravAcc", gravAcc, 9.8);
        nhPriv.param("CommandHz", commandHz, 40.0);
        nhPriv.param("PublishYawCommand", publishYawCommand, false);
        nhPriv.param<std::string>("WorldFrameId", worldFrameId, "world");

        flatmap.reset(gravAcc);

        trajSub = nh.subscribe("trajectory", 10, &TrajServer::trajCallback, this,
                               ros::TransportHints().tcpNoDelay());
        odomSub = nh.subscribe("odom", 1, &TrajServer::odomCallback, this,
                               ros::TransportHints().tcpNoDelay());

        commandPub = nh.advertise<geometry_msgs::PoseStamped>("command", 10);
        speedPub = nh.advertise<std_msgs::Float64>("/visualizer/speed", 1000);
        tiltPub = nh.advertise<std_msgs::Float64>("/visualizer/tilt_angle", 1000);
        bdrPub = nh.advertise<std_msgs::Float64>("/visualizer/body_rate", 1000);

        commandHz = commandHz > 0.0 ? commandHz : 40.0;
        commandTimer = nh.createTimer(ros::Duration(1.0 / commandHz),
                                      &TrajServer::commandTimerCallback, this);
    }

private:
    inline void trajCallback(const gcopter::PolyTraj::ConstPtr &msg)
    {
        // The server executes fixed-degree Trajectory<5> messages.
        if (msg->order != 5)
        {
            ROS_WARN_THROTTLE(1.0, "Ignoring PolyTraj with order %d (expected 5).", msg->order);
            return;
        }

        // Reject out-of-order trajectories.
        if (receiveTraj && msg->traj_id < trajId)
        {
            return;
        }

        const int coefPerPiece = msg->order + 1;
        const size_t n = msg->durations.size();
        if (n == 0 ||
            msg->coef_x.size() != n * coefPerPiece ||
            msg->coef_y.size() != n * coefPerPiece ||
            msg->coef_z.size() != n * coefPerPiece)
        {
            ROS_WARN_THROTTLE(1.0, "Received malformed PolyTraj; dropping.");
            return;
        }

        Trajectory<5> newTraj;
        newTraj.reserve(n);
        for (size_t p = 0; p < n; ++p)
        {
            Piece<5>::CoefficientMat coeff;
            for (int j = 0; j < coefPerPiece; ++j)
            {
                const size_t idx = p * coefPerPiece + j;
                coeff(0, j) = msg->coef_x[idx];
                coeff(1, j) = msg->coef_y[idx];
                coeff(2, j) = msg->coef_z[idx];
            }
            newTraj.emplace_back(msg->durations[p], coeff);
        }

        traj = newTraj;
        trajStamp = msg->start_time;
        trajId = msg->traj_id;
        receiveTraj = true;
    }

    inline void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // Preserve measured yaw when trajectory yaw is disabled.
        if (!publishYawCommand || traj.getPieceNum() <= 0)
        {
            const geometry_msgs::Quaternion &q = msg->pose.pose.orientation;
            lastYaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        }
    }

    inline void commandTimerCallback(const ros::TimerEvent &)
    {
        if (!receiveTraj || traj.getPieceNum() <= 0)
        {
            return;
        }

        const double duration = traj.getTotalDuration();
        const double tCur = (ros::Time::now() - trajStamp).toSec();

        if (tCur >= duration)
        {
            // Publish the terminal setpoint once.
            publishPoseCommand(traj.getPos(duration));
            receiveTraj = false;
            return;
        }

        // Hold the start state until its timestamp.
        const double tEval = std::max(0.0, tCur);
        const Eigen::Vector3d pos = traj.getPos(tEval);
        const Eigen::Vector3d vel = traj.getVel(tEval);

        if (publishYawCommand && vel(0) * vel(0) + vel(1) * vel(1) > 1.0e-6)
        {
            lastYaw = std::atan2(vel(1), vel(0));
        }

        publishPoseCommand(pos);
        if (tCur > 0.0)
        {
            publishDiagnostics(tEval, vel);
        }
    }

    inline void publishPoseCommand(const Eigen::Vector3d &pos)
    {
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.header.frame_id = worldFrameId;
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

        flatmap.forward(traj.getAcc(t), traj.getJer(t), 0.0, 0.0, quat, omg);

        std_msgs::Float64 speedMsg, tiltMsg, bdrMsg;
        speedMsg.data = vel.norm();
        tiltMsg.data = std::acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)));
        bdrMsg.data = omg.norm();
        speedPub.publish(speedMsg);
        tiltPub.publish(tiltMsg);
        bdrPub.publish(bdrMsg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_server");
    ros::NodeHandle nh;
    TrajServer server(nh);
    ros::spin();
    return 0;
}
