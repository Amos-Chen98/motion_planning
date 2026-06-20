#ifndef TF_UTILS_HPP
#define TF_UTILS_HPP

#include <ros/ros.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Geometry>

#include <string>

namespace tf_utils
{
// Resolve T_world_source: the rigid transform that maps a point expressed in
// `sourceFrame` into `worldFrame`, using the latest available transform
// (ros::Time(0), non-blocking). Legacy ROS 1 frame IDs with a leading slash are
// aliased to their tf2-compatible names.
inline bool resolveToWorld(const tf2_ros::Buffer &buffer,
                           const std::string &worldFrame,
                           const std::string &sourceFrame,
                           Eigen::Isometry3d &T_world_source)
{
    const std::string tf2WorldFrame =
        worldFrame.compare(0, 1, "/") == 0 ? worldFrame.substr(1) : worldFrame;
    const std::string tf2SourceFrame =
        sourceFrame.compare(0, 1, "/") == 0 ? sourceFrame.substr(1) : sourceFrame;

    try
    {
        T_world_source = tf2::transformToEigen(
            buffer.lookupTransform(tf2WorldFrame, tf2SourceFrame, ros::Time(0)));
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
