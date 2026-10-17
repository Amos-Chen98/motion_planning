#include <dragon/model/hydrus_like_robot_model.h>
#include <aerial_robot_msgs/FullStateTarget.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <cmath>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robustness_initial_target");
  ros::NodeHandle nh;
  try
  {
    pluginlib::ClassLoader<aerial_robot_model::RobotModel> loader("aerial_robot_model", "aerial_robot_model::RobotModel");
    const auto model = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(
        loader.createInstance("dragon/hydrus_like_robot_model"));
    if (!model) throw std::runtime_error("Cannot load DRAGON model");
    KDL::JntArray zero(model->getTree().getNrOfJoints());
    zero.data.setZero();
    // Same first-link tail landmark as StateEstimator::initializeRootTailPosePublisher.
    const auto root_to_tail = model->forwardKinematics<KDL::Frame>("link2", zero);
    aerial_robot_msgs::FullStateTarget msg;
    msg.header.frame_id = msg.root_state.header.frame_id = "world";
    msg.root_state.pose.pose.position.x = 0.53 - root_to_tail.p.x();
    msg.root_state.pose.pose.position.y = -root_to_tail.p.y();
    msg.root_state.pose.pose.position.z = 1.0 - root_to_tail.p.z();
    msg.root_state.pose.pose.orientation.w = 1.0;
    msg.joint_state.name = model->getLinkJointNames();
    for (const auto& name : msg.joint_state.name)
      msg.joint_state.position.push_back(name.find("yaw") != std::string::npos ? M_PI / 2.0 : 0.0);
    const auto pub = nh.advertise<aerial_robot_msgs::FullStateTarget>("robustness/initial_target", 1, true);
    pub.publish(msg);
    ros::spin();
  }
  catch (const std::exception& e) { ROS_FATAL("%s", e.what()); return 1; }
  return 0;
}
