#pragma once

#include <multilink_copilot/copilot.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>
#include <iomanip>

// Optional observer: called only outside the timed Copilot step.
class BenchmarkVisualization
{
public:
  BenchmarkVisualization()
  {
    ros::NodeHandle nh;
    joints_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1, true);
    markers_ = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1, true);
  }

  void setReference(const std::vector<geometry_msgs::PoseStamped>& reference)
  {
    path_ = marker("reference_path", visualization_msgs::Marker::LINE_STRIP);
    path_.scale.x = 0.025;
    path_.color.r = 0.95; path_.color.g = 0.65; path_.color.b = 0.05;
    for (const auto& pose : reference) path_.points.push_back(point(pose.pose.position));
  }

  void setWaypoints(const std::vector<geometry_msgs::PoseStamped>& waypoints)
  {
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
      auto ring = marker("waypoint_ring", visualization_msgs::Marker::LINE_STRIP);
      ring.id = i;
      ring.pose = waypoints[i].pose;
      ring.scale.x = 0.025;
      ring.color.b = 1.0;
      for (int j = 0; j <= 64; ++j)
      {
        geometry_msgs::Point p;
        p.y = 0.4 * std::cos(2 * M_PI * j / 64);
        p.z = 0.4 * std::sin(2 * M_PI * j / 64);
        ring.points.push_back(p);
      }
      waypoints_.push_back(ring);
      auto axis = marker("waypoint_axis", visualization_msgs::Marker::ARROW);
      axis.id = i; axis.pose = waypoints[i].pose;
      axis.scale.x = 0.3; axis.scale.y = 0.04; axis.scale.z = 0.08;
      axis.color.b = 1.0;
      waypoints_.push_back(axis);
    }
  }

  void publish(multilink_copilot::CopilotPlanner& planner, const Eigen::VectorXd& measured,
               const geometry_msgs::Pose& accepted_root, const geometry_msgs::PoseStamped& requested,
               const multilink_copilot::CopilotPlanner::UpdateResult& result,
               double time, bool finished = false)
  {
    const auto stamp = ros::Time::now();
    auto& evaluator = planner.stabilityEvaluator();
    tf::Transform root;
    tf::poseMsgToTF(accepted_root, root);
    evaluator.setRootLinkRotation(KDL::Rotation::Quaternion(accepted_root.orientation.x,
        accepted_root.orientation.y, accepted_root.orientation.z, accepted_root.orientation.w));
    evaluator.updateRobotModel(measured);
    const auto full_joints = evaluator.buildFullJointPositions(measured);
    const auto& model = evaluator.robotModel();
    sensor_msgs::JointState state;
    state.header.stamp = stamp;
    for (const auto& entry : model->getTree().getSegments())
    {
      const auto& joint = entry.second.segment.getJoint();
      if (joint.getType() == KDL::Joint::None) continue;
      state.name.push_back(joint.getName());
      state.position.push_back(full_joints(entry.second.q_nr));
    }
    joints_.publish(state);
    transforms_.sendTransform(tf::StampedTransform(root, stamp, "world", "copilot_benchmark/root"));

    visualization_msgs::MarkerArray display;
    display.markers = waypoints_;
    path_.header.stamp = stamp;
    display.markers.push_back(path_);
    auto body = marker("commanded_body", visualization_msgs::Marker::LINE_STRIP);
    body.scale.x = 0.035;
    body.color.g = 0.8; body.color.b = 0.9;
    body.points.push_back(point(root.getOrigin()));
    for (int link = 2; link <= planner.linkCount(); ++link)
    {
      const auto frame = model->forwardKinematics<KDL::Frame>("link" + std::to_string(link), full_joints);
      body.points.push_back(point(root * tf::Vector3(frame.p.x(), frame.p.y(), frame.p.z())));
    }
    const auto tail_frame = model->forwardKinematics<KDL::Frame>("link" + std::to_string(planner.linkCount()), full_joints);
    const auto tail = tail_frame * KDL::Vector(model->getLinkLength(), 0, 0);
    body.points.push_back(point(root * tf::Vector3(tail.x(), tail.y(), tail.z())));
    display.markers.push_back(body);
    auto target = marker("requested_tail", visualization_msgs::Marker::SPHERE);
    target.pose.position = requested.pose.position;
    target.scale.x = target.scale.y = target.scale.z = 0.10;
    target.color.r = 1.0; target.color.g = 0.65;
    display.markers.push_back(target);
    auto text = marker("status", visualization_msgs::Marker::TEXT_VIEW_FACING);
    text.pose.position = accepted_root.position;
    text.pose.position.z += 0.7;
    text.scale.z = 0.13;
    text.color.r = text.color.g = text.color.b = 0.95;
    std::ostringstream description;
    description << planner.linkCount() << " links | " << (finished ? "Finished" : (result.status == "initialization" ? "Initializing" : "Measured"))
                << " | t=" << std::fixed << std::setprecision(2) << time << " s\n"
                << result.status << " | QP solves: " << result.qp_solves
                << " | update: " << result.total_ms << " ms\n"
                << "Yellow: root-tail reference | Cyan: accepted body";
    text.text = description.str();
    display.markers.push_back(text);
    markers_.publish(display);
    ros::spinOnce();
  }

private:
  static geometry_msgs::Point point(const geometry_msgs::Point& p) { return p; }
  static geometry_msgs::Point point(const tf::Vector3& p)
  {
    geometry_msgs::Point result;
    result.x = p.x(); result.y = p.y(); result.z = p.z();
    return result;
  }
  static visualization_msgs::Marker marker(const std::string& name, int type)
  {
    visualization_msgs::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = ros::Time::now();
    m.ns = name;
    m.type = type;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1;
    m.color.a = 1;
    return m;
  }
  ros::Publisher joints_, markers_;
  tf::TransformBroadcaster transforms_;
  visualization_msgs::Marker path_;
  std::vector<visualization_msgs::Marker> waypoints_;
};
