#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace pcd_filter
{
struct FilterModel
{
  std::string xml;
  std::vector<std::string> collision_frames;
  size_t collision_count = 0;
};

std::string normalizedFrame(const std::string& frame);
FilterModel makeFilterModel(const std::string& source, const std::string& tf_prefix);
}  // namespace pcd_filter
