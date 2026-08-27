#include <voxel_mapping/occupied_voxel_map.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>

namespace voxel_mapping
{
namespace
{
VoxelMapConfig validConfig(const bool accumulated = true)
{
  VoxelMapConfig config;
  config.world_frame_id = "world";
  config.voxel_width = 0.1;
  config.map_bound = {-2.0, 2.0, -2.0, 2.0, 0.0, 3.0};
  config.use_accumulated_map = accumulated;
  return config;
}

bool contains(const std::vector<Eigen::Vector3d>& centers,
              const Eigen::Vector3d& expected)
{
  return std::any_of(centers.begin(), centers.end(),
                     [&expected](const Eigen::Vector3d& center) {
                       return center.isApprox(expected, 1e-12);
                     });
}

TEST(OccupiedVoxelMap, DeduplicatesHitsAndExportsVoxelCenters)
{
  OccupiedVoxelMap map(validConfig());
  map.update({Eigen::Vector3d(-0.46, 0.0, 1.0),
              Eigen::Vector3d(-0.44, 0.0, 1.0),
              Eigen::Vector3d(0.26, 0.0, 1.0),
              Eigen::Vector3d(-2.01, 0.0, 1.0),
              Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0)});

  const std::vector<Eigen::Vector3d> centers = map.occupiedVoxelCenters();
  ASSERT_EQ(centers.size(), 2u);
  EXPECT_TRUE(contains(centers, Eigen::Vector3d(-0.45, 0.05, 1.05)));
  EXPECT_TRUE(contains(centers, Eigen::Vector3d(0.25, 0.05, 1.05)));
  EXPECT_FALSE(contains(centers, Eigen::Vector3d(-0.45, 0.15, 1.05)));
}

TEST(OccupiedVoxelMap, PreservesAccumulationAndReplacementSemantics)
{
  const Eigen::Vector3d first(-0.5, 0.0, 1.0);
  const Eigen::Vector3d second(0.5, 0.0, 1.0);

  OccupiedVoxelMap accumulated(validConfig(true));
  accumulated.update({first});
  accumulated.update({second});
  EXPECT_EQ(accumulated.occupiedVoxelCount(), 2u);

  OccupiedVoxelMap latest(validConfig(false));
  latest.update({first});
  latest.update({second});
  EXPECT_EQ(latest.occupiedVoxelCount(), 1u);
  EXPECT_TRUE(contains(latest.occupiedVoxelCenters(), Eigen::Vector3d(0.55, 0.05, 1.05)));
  latest.update({});
  EXPECT_EQ(latest.occupiedVoxelCount(), 0u);
  EXPECT_TRUE(latest.occupiedVoxelCenters().empty());
}

TEST(VoxelMapConfig, RejectsInvalidGeometry)
{
  VoxelMapConfig config = validConfig();
  config.voxel_width = 0.0;
  EXPECT_THROW(OccupiedVoxelMap map(config), std::invalid_argument);
}
}  // namespace
}  // namespace voxel_mapping

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
