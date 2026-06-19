#ifndef TF_UTILS_HPP
#define TF_UTILS_HPP

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <Eigen/Geometry>

#include <string>

namespace tf_utils
{
inline Eigen::Isometry3d toIsometry(const geometry_msgs::TransformStamped &ts)
{
    const geometry_msgs::Vector3 &t = ts.transform.translation;
    const geometry_msgs::Quaternion &r = ts.transform.rotation;
    Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
    if (q.norm() < 1.0e-9)
    {
        q = Eigen::Quaterniond::Identity();
    }
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.translate(Eigen::Vector3d(t.x, t.y, t.z));
    iso.rotate(q.normalized());
    return iso;
}

// Resolve T_world_source: the rigid transform that maps a point expressed in
// `sourceFrame` into `worldFrame`, using the latest available transform
// (ros::Time(0), non-blocking). Returns identity when `sourceFrame` is empty or
// already equals `worldFrame`, so inputs already in the world frame need no TF.
// Returns false (with a throttled warning) when a genuinely-needed transform is
// unavailable, signalling the caller to skip the message.
inline bool resolveToWorld(const tf2_ros::Buffer &buffer,
                           const std::string &worldFrame,
                           const std::string &sourceFrame,
                           Eigen::Isometry3d &T_world_source)
{
    if (sourceFrame.empty())
    {
        ROS_WARN_THROTTLE(2.0, "Input has empty frame_id; assuming '%s'.",
                          worldFrame.c_str());
        T_world_source = Eigen::Isometry3d::Identity();
        return true;
    }

    if (sourceFrame == worldFrame)
    {
        T_world_source = Eigen::Isometry3d::Identity();
        return true;
    }

    try
    {
        T_world_source =
            toIsometry(buffer.lookupTransform(worldFrame, sourceFrame, ros::Time(0)));
        return true;
    }
    catch (const tf2::TransformException &e)
    {
        ROS_WARN_THROTTLE(1.0, "TF '%s' <- '%s' unavailable: %s",
                          worldFrame.c_str(), sourceFrame.c_str(), e.what());
        return false;
    }
}
} // namespace tf_utils

#endif // TF_UTILS_HPP
