#include <voxel_mapping/occupied_voxel_map.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace voxel_mapping
{
namespace
{
std::string normalizedFrameId(const std::string& frame_id)
{
  return !frame_id.empty() && frame_id.front() == '/' ? frame_id.substr(1) : frame_id;
}

VoxelMapConfig loadConfig(const ros::NodeHandle& private_nh)
{
  VoxelMapConfig config;
  private_nh.param<std::string>("WorldFrameId", config.world_frame_id, "world");
  private_nh.param("VoxelWidth", config.voxel_width, 0.0);
  private_nh.getParam("MapBound", config.map_bound);
  private_nh.param("UseAccumulatedMap", config.use_accumulated_map, true);
  config.validateOrThrow();
  return config;
}
}  // namespace

class VoxelMappingNode
{
public:
  VoxelMappingNode()
    : private_nh_("~")
    , config_(loadConfig(private_nh_))
    , map_(config_)
    , tf_listener_(tf_buffer_)
  {
    occupied_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("occupied", 1, true);
    cloud_sub_ = nh_.subscribe("cloud", 1, &VoxelMappingNode::cloudCallback, this,
                               ros::TransportHints().tcpNoDelay());
    ROS_INFO("Voxel mapping ready: frame=%s, width=%.3f m, accumulation=%s.",
             config_.world_frame_id.c_str(), config_.voxel_width,
             config_.use_accumulated_map ? "enabled" : "disabled");
  }

private:
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& message)
  {
    std::vector<Eigen::Vector3d> points;
    std::string error;
    if (!pointCloudToWorld(*message, points, error))
    {
      ROS_WARN_THROTTLE(1.0, "Cannot update voxel map: %s", error.c_str());
      return;
    }
    map_.update(points);
    publishOccupiedMap(message->header.stamp);
  }

  bool pointCloudToWorld(const sensor_msgs::PointCloud2& message,
                         std::vector<Eigen::Vector3d>& points,
                         std::string& error) const
  {
    if (message.header.frame_id.empty())
    {
      error = "point cloud frame_id is empty";
      return false;
    }

    Eigen::Isometry3d world_from_cloud = Eigen::Isometry3d::Identity();
    const std::string world_frame = normalizedFrameId(config_.world_frame_id);
    const std::string cloud_frame = normalizedFrameId(message.header.frame_id);
    if (world_frame != cloud_frame)
    {
      try
      {
        world_from_cloud = tf2::transformToEigen(
            tf_buffer_.lookupTransform(world_frame, cloud_frame, ros::Time(0)));
      }
      catch (const tf2::TransformException& exception)
      {
        error = exception.what();
        return false;
      }
    }

    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(message, "z");
      points.clear();
      points.reserve(static_cast<size_t>(message.width) * message.height);
      for (; x != x.end(); ++x, ++y, ++z)
      {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z))
        {
          points.push_back(world_from_cloud * Eigen::Vector3d(*x, *y, *z));
        }
      }
    }
    catch (const std::runtime_error& exception)
    {
      error = exception.what();
      return false;
    }
    return true;
  }

  void publishOccupiedMap(const ros::Time& source_stamp)
  {
    const std::vector<Eigen::Vector3d> centers = map_.occupiedVoxelCenters();
    sensor_msgs::PointCloud2 message;
    message.header.frame_id = config_.world_frame_id;
    message.header.stamp = source_stamp.isZero() ? ros::Time::now() : source_stamp;
    sensor_msgs::PointCloud2Modifier modifier(message);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(centers.size());
    sensor_msgs::PointCloud2Iterator<float> x(message, "x");
    sensor_msgs::PointCloud2Iterator<float> y(message, "y");
    sensor_msgs::PointCloud2Iterator<float> z(message, "z");
    for (const Eigen::Vector3d& center : centers)
    {
      *x = static_cast<float>(center.x());
      *y = static_cast<float>(center.y());
      *z = static_cast<float>(center.z());
      ++x;
      ++y;
      ++z;
    }
    message.is_dense = true;
    occupied_pub_.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  VoxelMapConfig config_;
  OccupiedVoxelMap map_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber cloud_sub_;
  ros::Publisher occupied_pub_;
};
}  // namespace voxel_mapping

int main(int argc, char** argv)
{
  ros::init(argc, argv, "voxel_mapping");
  try
  {
    voxel_mapping::VoxelMappingNode node;
    ros::spin();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL("Failed to start voxel mapping: %s", exception.what());
    return 1;
  }
  return 0;
}
