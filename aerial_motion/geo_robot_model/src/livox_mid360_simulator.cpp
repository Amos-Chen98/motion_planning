#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    inline double degToRad(const double deg)
    {
        return deg * M_PI / 180.0;
    }

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
} // namespace

struct LivoxSimConfig
{
    // Frame describing the robot pose carried by the odometry message (odom child frame).
    std::string robotFrameId;
    // Lidar IMU frame in which the simulated point cloud is expressed.
    std::string lidarImuFrameId;

    double publishRate;
    double minRange;
    double maxRange;
    double horizontalFovDeg;
    double verticalMinDeg;
    double verticalMaxDeg;

    explicit LivoxSimConfig(const ros::NodeHandle &nhPriv)
    {
        nhPriv.param<std::string>("RobotFrameId", robotFrameId, "quadrotor/cog");
        nhPriv.param<std::string>("LidarImuFrameId", lidarImuFrameId, "quadrotor/lidar_imu");
        nhPriv.param("PublishRate", publishRate, 10.0);
        nhPriv.param("LivoxMinRange", minRange, 0.1);
        nhPriv.param("LivoxMaxRange", maxRange, 40.0);
        nhPriv.param("LivoxHorizontalFovDeg", horizontalFovDeg, 360.0);
        nhPriv.param("LivoxVerticalMinDeg", verticalMinDeg, -7.0);
        nhPriv.param("LivoxVerticalMaxDeg", verticalMaxDeg, 52.0);
    }
};

class LivoxMid360Simulator
{
private:
    LivoxSimConfig config;
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    ros::Subscriber mapSub;
    ros::Subscriber odomSub;
    ros::Publisher localCloudPub;
    ros::Timer publishTimer;

    std::vector<Eigen::Vector3d> globalPoints;
    Eigen::Isometry3d latestWorldRobot;

    // Cached T_robot_lidar_imu extrinsic looked up from TF.
    Eigen::Isometry3d robotLidarImu;
    ros::WallTime tfWarningStartTime;
    bool extrinsicReady;
    bool mapReceived;
    bool odomReceived;

public:
    LivoxMid360Simulator(const LivoxSimConfig &conf, ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          tfListener(tfBuffer),
          latestWorldRobot(Eigen::Isometry3d::Identity()),
          robotLidarImu(Eigen::Isometry3d::Identity()),
          tfWarningStartTime(ros::WallTime::now()),
          extrinsicReady(false),
          mapReceived(false),
          odomReceived(false)
    {
        mapSub = nh.subscribe("global_pcl_topic", 1, &LivoxMid360Simulator::mapCallback, this,
                              ros::TransportHints().tcpNoDelay());
        odomSub = nh.subscribe("odom", 1, &LivoxMid360Simulator::odomCallback, this,
                               ros::TransportHints().tcpNoDelay());
        localCloudPub = nh.advertise<sensor_msgs::PointCloud2>("local_pcl_topic", 1);

        const double hz = config.publishRate > 0.0 ? config.publishRate : 10.0;
        publishTimer = nh.createTimer(ros::Duration(1.0 / hz),
                                      &LivoxMid360Simulator::publishTimerCallback, this);
    }

private:
    // Resolve T_robot_lidar_imu from TF on every cycle so articulated platforms can
    // move the lidar relative to their odometry frame. The last valid transform is
    // retained to bridge brief TF gaps.
    inline bool updateExtrinsic()
    {
        try
        {
            const geometry_msgs::TransformStamped tf =
                tfBuffer.lookupTransform(config.robotFrameId, config.lidarImuFrameId, ros::Time(0));
            robotLidarImu = tf2::transformToEigen(tf);
            extrinsicReady = true;
        }
        catch (const tf2::TransformException &e)
        {
            // Allow the listener to receive already-published TF before warning.
            if (!extrinsicReady &&
                (ros::WallTime::now() - tfWarningStartTime).toSec() >= 1.0)
            {
                ROS_WARN_THROTTLE(2.0, "Waiting for TF %s -> %s: %s",
                                  config.robotFrameId.c_str(),
                                  config.lidarImuFrameId.c_str(),
                                  e.what());
            }
        }
        return extrinsicReady;
    }

    inline void mapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        std::vector<Eigen::Vector3d> points;
        points.reserve(static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height));

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
                points.emplace_back(*iterX, *iterY, *iterZ);
            }
        }
        catch (const std::runtime_error &e)
        {
            ROS_WARN("Invalid global map cloud: %s", e.what());
            return;
        }

        globalPoints.swap(points);
        mapReceived = true;
    }

    inline void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        const geometry_msgs::Point &p = msg->pose.pose.position;
        latestWorldRobot = Eigen::Isometry3d::Identity();
        latestWorldRobot.translate(Eigen::Vector3d(p.x, p.y, p.z));
        latestWorldRobot.rotate(normalizedQuaternion(msg->pose.pose.orientation));
        odomReceived = true;
    }

    inline bool insideLivoxFov(const Eigen::Vector3d &pointLidar) const
    {
        const double range = pointLidar.norm();
        if (range < config.minRange || range > config.maxRange)
        {
            return false;
        }

        const double verticalDeg =
            std::atan2(pointLidar.z(), std::hypot(pointLidar.x(), pointLidar.y())) * 180.0 / M_PI;
        if (verticalDeg < config.verticalMinDeg || verticalDeg > config.verticalMaxDeg)
        {
            return false;
        }

        if (config.horizontalFovDeg >= 359.999)
        {
            return true;
        }

        const double halfHorizontal = 0.5 * degToRad(config.horizontalFovDeg);
        const double horizontal = std::atan2(pointLidar.y(), pointLidar.x());
        return std::abs(horizontal) <= halfHorizontal;
    }

    inline void publishTimerCallback(const ros::TimerEvent &)
    {
        // Query TF even while other inputs are absent so standalone launches clearly
        // report the missing robot-owned transform and articulated mounts stay current.
        if (!updateExtrinsic() || !mapReceived || !odomReceived)
        {
            return;
        }

        // T_world_lidar_imu = T_world_robot * T_robot_lidar_imu.
        const Eigen::Isometry3d lidarImuWorld =
            (latestWorldRobot * robotLidarImu).inverse();

        std::vector<Eigen::Vector3d> localPoints;
        localPoints.reserve(globalPoints.size());

        for (const Eigen::Vector3d &pointWorld : globalPoints)
        {
            const Eigen::Vector3d pointLidar = lidarImuWorld * pointWorld;
            if (insideLivoxFov(pointLidar))
            {
                localPoints.emplace_back(pointLidar);
            }
        }

        sensor_msgs::PointCloud2 msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = config.lidarImuFrameId;

        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(localPoints.size());

        sensor_msgs::PointCloud2Iterator<float> iterX(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iterY(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iterZ(msg, "z");
        for (const Eigen::Vector3d &point : localPoints)
        {
            *iterX = static_cast<float>(point.x());
            *iterY = static_cast<float>(point.y());
            *iterZ = static_cast<float>(point.z());
            ++iterX;
            ++iterY;
            ++iterZ;
        }

        localCloudPub.publish(msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_mid360_simulator");
    ros::NodeHandle nh;
    LivoxMid360Simulator simulator(LivoxSimConfig(ros::NodeHandle("~")), nh);
    ros::spin();
    return 0;
}
