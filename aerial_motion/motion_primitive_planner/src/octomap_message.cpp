#include <motion_primitive_planner/octomap_message.h>
#include <octomap_msgs/conversions.h>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace motion_primitive_planner
{
namespace
{
// Native full OcTree data is preorder: float log-odds, byte child mask, children.
// OctoMap's reader does not stop recursion on a truncated stream.
void validateNode(const std::vector<int8_t>& data, size_t& cursor, unsigned depth)
{
  if (depth > 16 || data.size() - cursor < sizeof(float) + 1)
    throw std::invalid_argument("Truncated or overdeep full OctoMap data");
  float value;
  std::memcpy(&value, data.data() + cursor, sizeof(value));
  cursor += sizeof(value);
  if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite OctoMap occupancy");
  const uint8_t mask = static_cast<uint8_t>(data[cursor++]);
  for (unsigned child = 0; child < 8; ++child)
    if (mask & (1u << child)) validateNode(data, cursor, depth + 1);
}
}

std::shared_ptr<const octomap::OcTree> readOctomap(const octomap_msgs::Octomap& message)
{
  if (message.binary || message.id != "OcTree" || !std::isfinite(message.resolution) ||
      message.resolution <= 0 || message.header.frame_id.empty())
    throw std::invalid_argument("Expected a full OcTree message with a valid frame and resolution");
  if (!message.data.empty())
  {
    size_t cursor = 0;
    validateNode(message.data, cursor, 0);
    if (cursor != message.data.size()) throw std::invalid_argument("Trailing full OctoMap data");
  }
  std::unique_ptr<octomap::AbstractOcTree> decoded(octomap_msgs::fullMsgToMap(message));
  auto* tree = dynamic_cast<octomap::OcTree*>(decoded.get());
  if (!tree) throw std::invalid_argument("Failed to decode full OcTree");
  tree->setOccupancyThres(0.5);
  decoded.release();
  return std::shared_ptr<const octomap::OcTree>(tree);
}
}
