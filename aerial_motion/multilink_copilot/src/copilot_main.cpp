#include <multilink_copilot/copilot.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "copilot_planner");

  try
  {
    multilink_copilot::CopilotPlanner planner;
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Exception in CopilotPlanner: %s", e.what());
    return 1;
  }

  return 0;
}
