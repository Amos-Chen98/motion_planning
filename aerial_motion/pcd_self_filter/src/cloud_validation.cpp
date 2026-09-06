#include <pcd_self_filter/cloud_validation.h>
#include <pcd_self_filter/model_adapter.h>

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <stdexcept>

namespace pcd_self_filter
{
sensor_msgs::PointCloud2 prepareCloud(const sensor_msgs::PointCloud2& input)
{
  if (input.header.stamp.isZero())
    throw std::invalid_argument("Point cloud timestamp is zero");
  if (normalizedFrame(input.header.frame_id).empty())
    throw std::invalid_argument("Point cloud frame_id is empty");
  const uint16_t endian_probe = 1;
  const bool host_big_endian = *reinterpret_cast<const uint8_t*>(&endian_probe) == 0;
  if (input.is_bigendian != host_big_endian)
    throw std::invalid_argument("Point cloud byte order differs from the host");
  if (input.point_step == 0 || (input.width != 0 && input.height == 0) ||
      uint64_t(input.width) * input.point_step > input.row_step ||
      uint64_t(input.row_step) * input.height != input.data.size())
    throw std::invalid_argument("Invalid PointCloud2 dimensions or data size");

  std::set<std::string> names;
  std::array<int64_t, 3> offsets{{-1, -1, -1}};
  const std::array<std::string, 3> axes{{"x", "y", "z"}};
  for (const auto& field : input.fields)
  {
    const std::array<uint32_t, 9> sizes{{0, 1, 1, 2, 2, 4, 4, 4, 8}};
    if (field.datatype == 0 || field.datatype >= sizes.size() || field.count == 0 ||
        uint64_t(field.offset) + uint64_t(field.count) * sizes[field.datatype] > input.point_step ||
        !names.insert(field.name).second)
      throw std::invalid_argument("Invalid PointCloud2 field layout");
    for (size_t axis = 0; axis < axes.size(); ++axis)
      if (field.name == axes[axis])
      {
        if (field.datatype != sensor_msgs::PointField::FLOAT32 || field.count != 1)
          throw std::invalid_argument("XYZ must be scalar FLOAT32 fields");
        offsets[axis] = field.offset;
      }
  }
  for (const auto offset : offsets)
    if (offset < 0)
      throw std::invalid_argument("Point cloud is missing XYZ fields");
  const uint64_t count = uint64_t(input.width) * input.height;
  if (count * input.point_step > std::numeric_limits<uint32_t>::max())
    throw std::invalid_argument("Packed point cloud exceeds PointCloud2 row_step capacity");

  sensor_msgs::PointCloud2 output;
  output.header = input.header;
  output.header.frame_id = normalizedFrame(input.header.frame_id);
  output.fields = input.fields;
  output.point_step = input.point_step;
  output.is_bigendian = input.is_bigendian;
  output.height = 1;
  output.is_dense = true;
  output.data.reserve(count * input.point_step);
  for (uint32_t row = 0; row < input.height; ++row)
    for (uint32_t col = 0; col < input.width; ++col)
    {
      const uint8_t* point = input.data.data() + size_t(row) * input.row_step + size_t(col) * input.point_step;
      bool finite = true;
      for (const auto offset : offsets)
      {
        float value;
        std::memcpy(&value, point + offset, sizeof(value));
        finite = finite && std::isfinite(value);
      }
      if (finite)
        output.data.insert(output.data.end(), point, point + input.point_step);
    }
  output.width = output.data.size() / output.point_step;
  output.row_step = output.data.size();
  return output;
}
}  // namespace pcd_self_filter
