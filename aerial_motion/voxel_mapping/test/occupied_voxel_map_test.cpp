#include <voxel_mapping/occupied_voxel_map.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>

namespace voxel_mapping
{
namespace
{
VoxelMapConfig validConfig(const bool accumulated = true, const bool noise_filter = true)
{
  VoxelMapConfig config;
  config.world_frame_id = "world";
  config.voxel_width = 0.1;
  config.map_bound = {-2.0, 2.0, -2.0, 2.0, 0.0, 3.0};
  config.use_accumulated_map = accumulated;
  config.enable_noise_filter = noise_filter;
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
  OccupiedVoxelMap map(validConfig(true, false));
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

  OccupiedVoxelMap accumulated(validConfig(true, false));
  accumulated.update({first});
  accumulated.update({second});
  EXPECT_EQ(accumulated.occupiedVoxelCount(), 2u);

  OccupiedVoxelMap latest(validConfig(false, false));
  latest.update({first});
  latest.update({second});
  EXPECT_EQ(latest.occupiedVoxelCount(), 1u);
  EXPECT_TRUE(contains(latest.occupiedVoxelCenters(), Eigen::Vector3d(0.55, 0.05, 1.05)));
  latest.update({});
  EXPECT_EQ(latest.occupiedVoxelCount(), 0u);
  EXPECT_TRUE(latest.occupiedVoxelCenters().empty());
}

TEST(OccupiedVoxelMap, RejectsIsolatedPointsAndPairsEvenAcrossRepeatedFrames)
{
  OccupiedVoxelMap map(validConfig());
  for (int frame = 0; frame < 5; ++frame)
  {
    map.update({Eigen::Vector3d(-0.95, 0.05, 1.05),
                Eigen::Vector3d(0.05, 0.05, 1.05),
                Eigen::Vector3d(0.10, 0.05, 1.05)});
    EXPECT_EQ(map.occupiedVoxelCount(), 0u);
  }
}

TEST(OccupiedVoxelMap, UsesNeighborsAcrossVoxelBoundaries)
{
  OccupiedVoxelMap map(validConfig());
  map.update({Eigen::Vector3d(-0.04, 0.05, 1.05),
              Eigen::Vector3d(0.05, 0.05, 1.05),
              Eigen::Vector3d(0.14, 0.05, 1.05)});
  const auto centers = map.occupiedVoxelCenters();
  ASSERT_EQ(centers.size(), 3u);
  for (double x : {-0.05, 0.05, 0.15})
  {
    EXPECT_TRUE(contains(centers, Eigen::Vector3d(x, 0.05, 1.05)));
  }
}

TEST(OccupiedVoxelMap, CountsOtherPointsBeforeVoxelDeduplication)
{
  const std::vector<Eigen::Vector3d> cluster = {
      Eigen::Vector3d(0.02, 0.05, 1.05), Eigen::Vector3d(0.04, 0.05, 1.05),
      Eigen::Vector3d(0.06, 0.05, 1.05)};
  OccupiedVoxelMap map(validConfig());
  map.update(cluster);
  ASSERT_EQ(map.occupiedVoxelCount(), 1u);
  EXPECT_TRUE(contains(map.occupiedVoxelCenters(), Eigen::Vector3d(0.05, 0.05, 1.05)));

  auto config = validConfig();
  config.noise_filter_min_neighbors = 3;
  OccupiedVoxelMap stricter(config);
  stricter.update(cluster);
  EXPECT_EQ(stricter.occupiedVoxelCount(), 0u);
}

TEST(OccupiedVoxelMap, HonorsRadiusAndNeighborOverrides)
{
  const std::vector<Eigen::Vector3d> pair = {
      Eigen::Vector3d(0.05, 0.05, 1.05), Eigen::Vector3d(0.30, 0.05, 1.05)};
  auto config = validConfig();
  config.noise_filter_min_neighbors = 1;
  OccupiedVoxelMap narrow(config);
  narrow.update(pair);
  EXPECT_EQ(narrow.occupiedVoxelCount(), 0u);

  config.noise_filter_radius = 0.30;
  OccupiedVoxelMap wide(config);
  wide.update(pair);
  EXPECT_EQ(wide.occupiedVoxelCount(), 2u);
}

TEST(OccupiedVoxelMap, RetainsPlaneAndSupportedThinRodWhileRemovingNoise)
{
  std::vector<Eigen::Vector3d> points;
  for (int x = 0; x < 5; ++x)
  {
    for (int y = 0; y < 5; ++y)
    {
      points.emplace_back(0.05 + 0.1 * x, 0.05 + 0.1 * y, 1.05);
    }
  }
  for (int z = 0; z < 6; ++z)
  {
    points.emplace_back(-0.95, 0.05, 0.55 + 0.08 * z);
  }
  points.emplace_back(1.55, 1.55, 2.55);
  OccupiedVoxelMap map(validConfig());
  map.update(points);
  const auto centers = map.occupiedVoxelCenters();
  ASSERT_EQ(centers.size(), 30u);
  for (int x = 0; x < 5; ++x)
  {
    for (int y = 0; y < 5; ++y)
    {
      EXPECT_TRUE(contains(centers, Eigen::Vector3d(0.05 + 0.1 * x, 0.05 + 0.1 * y, 1.05)));
    }
  }
  for (double z : {0.55, 0.65, 0.75, 0.85, 0.95})
  {
    EXPECT_TRUE(contains(centers, Eigen::Vector3d(-0.95, 0.05, z)));
  }
  EXPECT_FALSE(contains(centers, Eigen::Vector3d(1.55, 1.55, 2.55)));
}

TEST(OccupiedVoxelMap, ExcludesInvalidAndOutOfBoundsPointsFromNeighborSupport)
{
  OccupiedVoxelMap map(validConfig());
  map.update({Eigen::Vector3d(0.05, 0.05, 0.03),
              Eigen::Vector3d(0.05, 0.05, -0.01),
              Eigen::Vector3d(0.05, 0.05, -0.02),
              Eigen::Vector3d(1.99, 0.05, 1.05),
              Eigen::Vector3d(2.00, 0.05, 1.05),
              Eigen::Vector3d(2.01, 0.05, 1.05),
              Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.05, 0.03),
              Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0.05, 0.03),
              Eigen::Vector3d(std::numeric_limits<double>::max(), 0.05, 0.03),
              Eigen::Vector3d(0.55, 0.05, 1.05),
              Eigen::Vector3d(0.56, 0.05, 1.05),
              Eigen::Vector3d(0.57, 0.05, 1.05)});
  ASSERT_EQ(map.occupiedVoxelCount(), 1u);
  EXPECT_TRUE(contains(map.occupiedVoxelCenters(), Eigen::Vector3d(0.55, 0.05, 1.05)));
}

TEST(OccupiedVoxelMap, PreservesDoublePrecisionVoxelMembership)
{
  OccupiedVoxelMap map(validConfig());
  // All three x coordinates round to 0.5f, but straddle a voxel boundary in double.
  map.update({Eigen::Vector3d(0.5 - 1e-9, 0.05, 1.05),
              Eigen::Vector3d(0.5, 0.05, 1.05),
              Eigen::Vector3d(0.5 + 1e-9, 0.05, 1.05)});
  const auto centers = map.occupiedVoxelCenters();
  ASSERT_EQ(centers.size(), 2u);
  EXPECT_TRUE(contains(centers, Eigen::Vector3d(0.45, 0.05, 1.05)));
  EXPECT_TRUE(contains(centers, Eigen::Vector3d(0.55, 0.05, 1.05)));
}

TEST(OccupiedVoxelMap, DoesNotUseHistoricalPointsAsNeighbors)
{
  OccupiedVoxelMap map(validConfig());
  map.update({Eigen::Vector3d(0.02, 0.05, 1.05),
              Eigen::Vector3d(0.04, 0.05, 1.05),
              Eigen::Vector3d(0.06, 0.05, 1.05)});
  for (int frame = 0; frame < 5; ++frame)
  {
    map.update({Eigen::Vector3d(0.12, 0.05, 1.05)});
  }
  ASSERT_EQ(map.occupiedVoxelCount(), 1u);
  EXPECT_FALSE(contains(map.occupiedVoxelCenters(), Eigen::Vector3d(0.15, 0.05, 1.05)));
}

TEST(OccupiedVoxelMap, PreservesFilteredAccumulationAndReplacementSemantics)
{
  const std::vector<Eigen::Vector3d> first = {
      Eigen::Vector3d(-0.95, 0.05, 1.05), Eigen::Vector3d(-0.94, 0.05, 1.05),
      Eigen::Vector3d(-0.93, 0.05, 1.05)};
  const std::vector<Eigen::Vector3d> second = {
      Eigen::Vector3d(0.95, 0.05, 1.05), Eigen::Vector3d(0.96, 0.05, 1.05),
      Eigen::Vector3d(0.97, 0.05, 1.05)};
  for (bool accumulate : {false, true})
  {
    OccupiedVoxelMap map(validConfig(accumulate));
    map.update({});
    EXPECT_EQ(map.occupiedVoxelCount(), 0u);
    map.update(first);
    map.update(second);
    EXPECT_EQ(map.occupiedVoxelCount(), accumulate ? 2u : 1u);
    EXPECT_TRUE(contains(map.occupiedVoxelCenters(), Eigen::Vector3d(0.95, 0.05, 1.05)));
    map.update({Eigen::Vector3d(0.05, 0.05, 1.05)});
    EXPECT_EQ(map.occupiedVoxelCount(), accumulate ? 2u : 0u);
    map.update(first);
    map.update({});
    EXPECT_EQ(map.occupiedVoxelCount(), accumulate ? 2u : 0u);
  }
}

TEST(VoxelMapConfig, RejectsInvalidNoiseFilterParameters)
{
  for (double radius : {0.0, -0.1, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()})
  {
    auto config = validConfig();
    config.noise_filter_radius = radius;
    EXPECT_THROW(OccupiedVoxelMap map(config), std::invalid_argument);
  }
  for (int neighbors : {0, -1})
  {
    auto config = validConfig();
    config.noise_filter_min_neighbors = neighbors;
    EXPECT_THROW(OccupiedVoxelMap map(config), std::invalid_argument);
  }
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
