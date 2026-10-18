#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

struct BenchmarkReference
{
  std::vector<double> times;
  std::vector<geometry_msgs::PoseStamped> poses;
};

inline BenchmarkReference readBenchmarkReference(const std::string& filename, double dt)
{
  std::ifstream input(filename);
  std::string line;
  if (!input || !std::getline(input, line)) throw std::runtime_error("Cannot read reference CSV: " + filename);
  if (!line.empty() && line.back() == '\r') line.pop_back();
  if (line != "time_s,x,y,z,qx,qy,qz,qw") throw std::runtime_error("Invalid reference CSV header");
  BenchmarkReference result;
  while (std::getline(input, line))
  {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    std::istringstream row(line);
    std::string field;
    std::vector<double> values;
    while (std::getline(row, field, ','))
    {
      size_t used = 0;
      const double value = std::stod(field, &used);
      if (used != field.size() || !std::isfinite(value)) throw std::runtime_error("Invalid reference value");
      values.push_back(value);
    }
    if (values.size() != 8 || std::abs(values[0] - result.poses.size() * dt) > 1e-7)
      throw std::runtime_error("Reference must contain contiguous samples starting at t=0");
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.header.stamp.fromSec(1.0 + values[0]);
    pose.pose.position.x = values[1]; pose.pose.position.y = values[2]; pose.pose.position.z = values[3];
    tf::Quaternion q(values[4], values[5], values[6], values[7]);
    if (std::abs(q.length2() - 1.0) > 1e-6) throw std::runtime_error("Reference quaternion is not normalized");
    q.normalize();
    tf::quaternionTFToMsg(q, pose.pose.orientation);
    result.times.push_back(values[0]); result.poses.push_back(pose);
  }
  if (result.poses.empty()) throw std::runtime_error("Empty reference CSV");
  return result;
}

inline tf::Quaternion benchmarkLinkRotation(const geometry_msgs::Quaternion& flu)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(flu, q);
  return q * tf::Quaternion(0, 0, 1, 0);
}
