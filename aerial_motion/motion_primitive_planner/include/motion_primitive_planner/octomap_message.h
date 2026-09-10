#pragma once
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <memory>

namespace motion_primitive_planner
{
// Validate the native stream before deserializing it once into an immutable scene.
std::shared_ptr<const octomap::OcTree> readOctomap(const octomap_msgs::Octomap& message);
}
