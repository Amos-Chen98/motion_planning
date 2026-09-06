#include <pcd_self_filter/model_adapter.h>
#include <pcd_self_filter/cloud_validation.h>

#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <urdf/model.h>

#include <cstring>
#include <limits>

namespace pcd_self_filter
{
namespace
{
const std::string model = R"(
<robot name="test">
 <link name="base">
  <visual><geometry><sphere radius="9"/></geometry></visual>
  <collision name="first"><origin xyz="1 2 3" rpy="0 0 1"/><geometry><box size="1 2 3"/></geometry></collision>
  <collision name="second"><geometry><sphere radius="0.2"/></geometry></collision>
  <collision name="mesh"><origin xyz="0.1 0.2 0.3" rpy="0.2 0.3 0.4"/><geometry><mesh filename="package://example/part.stl" scale="2 3 4"/></geometry></collision>
 </link>
 <link name="tool">
  <visual><origin xyz="0.1 0.2 0.3" rpy="0.2 0.3 0.4"/><geometry><mesh filename="package://example/part.stl" scale="2 3 4"/></geometry><material name="red"/></visual>
  <visual><geometry><cylinder length="0.5" radius="0.1"/></geometry></visual>
 </link>
 <link name="sensor"/>
 <joint name="hinge" type="revolute"><parent link="base"/><child link="tool"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
 <joint name="mount" type="fixed"><parent link="tool"/><child link="sensor"/></joint>
</robot>)";

TEST(ModelAdapter, PreservesAllCollisionsAndIgnoresVisualOnlyLinks)
{
  const auto adapted = makeFilterModel(model, "/dragon/");
  EXPECT_EQ(adapted.collision_count, 3u);
  EXPECT_EQ(adapted.collision_frames, (std::vector<std::string>{"dragon/base"}));
  urdf::Model parsed;
  ASSERT_TRUE(parsed.initString(adapted.xml));
  const auto base = parsed.getLink("dragon/base");
  ASSERT_EQ(base->collision_array.size(), 3u);
  EXPECT_EQ(base->collision_array[0]->name, "first");
  EXPECT_DOUBLE_EQ(base->collision_array[0]->origin.position.z, 3.0);
  const auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(base->collision_array[2]->geometry);
  ASSERT_TRUE(mesh);
  EXPECT_EQ(mesh->filename, "package://example/part.stl");
  EXPECT_DOUBLE_EQ(mesh->scale.x, 2.0);
  EXPECT_DOUBLE_EQ(mesh->scale.y, 3.0);
  EXPECT_DOUBLE_EQ(mesh->scale.z, 4.0);
  EXPECT_DOUBLE_EQ(base->collision_array[2]->origin.position.x, 0.1);
  EXPECT_DOUBLE_EQ(base->collision_array[2]->origin.position.y, 0.2);
  EXPECT_DOUBLE_EQ(base->collision_array[2]->origin.position.z, 0.3);
  double roll, pitch, yaw;
  base->collision_array[2]->origin.rotation.getRPY(roll, pitch, yaw);
  EXPECT_NEAR(roll, 0.2, 1e-12);
  EXPECT_NEAR(pitch, 0.3, 1e-12);
  EXPECT_NEAR(yaw, 0.4, 1e-12);
  const auto tool = parsed.getLink("dragon/tool");
  EXPECT_TRUE(tool->collision_array.empty());
  EXPECT_EQ(tool->visual_array.size(), 2u);
  EXPECT_TRUE(parsed.getLink("dragon/sensor")->collision_array.empty());
  EXPECT_EQ(parsed.getJoint("hinge")->parent_link_name, "dragon/base");
  EXPECT_EQ(parsed.getJoint("hinge")->child_link_name, "dragon/tool");

  urdf::Model original;
  ASSERT_TRUE(original.initString(model));
  EXPECT_EQ(original.getLink("base")->collision_array.size(), 3u);
  EXPECT_TRUE(original.getLink("tool")->collision_array.empty());
  EXPECT_FALSE(original.getLink("dragon/base"));
}

TEST(ModelAdapter, PrefixIsIdempotentAndEmptyPrefixIsSupported)
{
  const auto first = makeFilterModel(model, "dragon");
  const auto second = makeFilterModel(first.xml, "dragon/");
  EXPECT_EQ(first.xml, second.xml);
  EXPECT_EQ(first.collision_count, second.collision_count);
  EXPECT_EQ(first.collision_frames, second.collision_frames);
  EXPECT_EQ(makeFilterModel(model, "").collision_frames.front(), "base");
}

TEST(ModelAdapter, RejectsInvalidTreesAndEmptyGeometry)
{
  EXPECT_THROW(makeFilterModel("broken", "dragon"), std::invalid_argument);
  EXPECT_THROW(makeFilterModel("<robot name='x'><link name='empty'/></robot>", ""), std::invalid_argument);
  EXPECT_THROW(makeFilterModel("<robot name='x'><link name='x'><visual/></link></robot>", ""), std::invalid_argument);
  EXPECT_THROW(makeFilterModel("<robot name='x'><link name='x'><visual><geometry><sphere radius='1'/></geometry></visual></link></robot>", ""), std::invalid_argument);
  EXPECT_THROW(makeFilterModel("<robot name='x'><link name='x'/><joint name='j' type='fixed'><parent link='x'/><child link='absent'/></joint></robot>", ""), std::invalid_argument);
}

sensor_msgs::PointCloud2 cloud()
{
  sensor_msgs::PointCloud2 message;
  message.header.stamp = ros::Time(10, 0);
  message.header.frame_id = "/sensor";
  sensor_msgs::PointCloud2Modifier modifier(message);
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::PointField::FLOAT32);
  modifier.resize(4);
  for (size_t i = 0; i < 4; ++i)
  {
    float values[] = {float(i), 0.0f, 1.0f, float(i + 10)};
    std::memcpy(message.data.data() + i * message.point_step, values, sizeof(values));
  }
  return message;
}

TEST(CloudValidation, PacksOrganizedRowsPreservingFieldsAndRemovingNonfinitePoints)
{
  auto message = cloud();
  message.width = 2;
  message.height = 2;
  message.row_step = 40;
  auto original = message.data;
  message.data.assign(80, 0xff);
  std::memcpy(message.data.data(), original.data(), 32);
  std::memcpy(message.data.data() + 40, original.data() + 32, 32);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(message.data.data() + 16, &nan, sizeof(nan));
  const auto packed = prepareCloud(message);
  EXPECT_EQ(packed.header.stamp, message.header.stamp);
  EXPECT_EQ(packed.header.frame_id, "sensor");
  EXPECT_EQ(packed.fields, message.fields);
  EXPECT_EQ(packed.height, 1u);
  EXPECT_EQ(packed.width, 3u);
  EXPECT_EQ(packed.data.size(), 48u);
  sensor_msgs::PointCloud2ConstIterator<float> intensity(packed, "intensity");
  EXPECT_FLOAT_EQ(*intensity, 10.0f);
  EXPECT_FLOAT_EQ(*++intensity, 12.0f);
  EXPECT_FLOAT_EQ(*++intensity, 13.0f);
}

TEST(CloudValidation, RejectsUnsafeLayoutsAndUnstampedClouds)
{
  auto message = cloud();
  message.data.pop_back();
  EXPECT_THROW(prepareCloud(message), std::invalid_argument);
  message = cloud();
  message.fields[0].offset = message.point_step;
  EXPECT_THROW(prepareCloud(message), std::invalid_argument);
  message = cloud();
  message.fields[0].name = "intensity";
  EXPECT_THROW(prepareCloud(message), std::invalid_argument);
  message = cloud();
  message.fields[0].datatype = sensor_msgs::PointField::FLOAT64;
  EXPECT_THROW(prepareCloud(message), std::invalid_argument);
  message = cloud();
  message.header.stamp = ros::Time(0);
  EXPECT_THROW(prepareCloud(message), std::invalid_argument);
  message = cloud();
  message.is_bigendian = !message.is_bigendian;
  EXPECT_THROW(prepareCloud(message), std::invalid_argument);
}

TEST(CloudValidation, AcceptsValidEmptyClouds)
{
  auto message = cloud();
  sensor_msgs::PointCloud2Modifier(message).resize(0);
  EXPECT_EQ(prepareCloud(message).width, 0u);
}
}  // namespace
}  // namespace pcd_self_filter

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
