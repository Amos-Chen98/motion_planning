#include <geo_robot_model/dragon_stability_monitor.h>

#include <ros/ros.h>

#include <exception>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "geo_dragon_stability_monitor");

  try
  {
    geo_robot_model::DragonStabilityMonitor monitor(ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("Failed to initialize DRAGON stability monitor: %s", error.what());
    return 1;
  }

  return 0;
}
