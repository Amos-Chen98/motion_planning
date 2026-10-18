#include <gtest/gtest.h>
#include "../src/evaluation/benchmark_reference.h"
#include <unistd.h>

namespace
{
struct InputFile
{
  std::string path;
  explicit InputFile(const std::string& rows)
  {
    char pattern[] = "/tmp/copilot_reference_XXXXXX";
    const int fd = mkstemp(pattern);
    if (fd < 0) throw std::runtime_error("mkstemp failed");
    close(fd);
    path = pattern;
    std::ofstream(path) << "time_s,x,y,z,qx,qy,qz,qw\n" << rows;
  }
  ~InputFile() { unlink(path.c_str()); }
};
}

TEST(BenchmarkReference, KeepsAllSamplesAndEndpoint)
{
  InputFile file("0,0,0,1,0,0,0,1\n0.025,1,2,3,0,0,1,0\n");
  const auto result = readBenchmarkReference(file.path, .025);
  ASSERT_EQ(result.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(result.times.back(), .025);
  EXPECT_DOUBLE_EQ(result.poses.back().pose.position.z, 3);
  EXPECT_EQ(result.poses.back().header.frame_id, "world");
  EXPECT_DOUBLE_EQ(result.poses.back().header.stamp.toSec(), 1.025);
}

TEST(BenchmarkReference, RejectsGapsNonfiniteAndInvalidQuaternion)
{
  for (const auto& rows : {"0,0,0,1,0,0,0,1\n0.05,0,0,1,0,0,0,1\n",
                          "0,nan,0,1,0,0,0,1\n", "0,0,0,1,0,0,0,0\n", ""})
  {
    InputFile file(rows);
    EXPECT_THROW(readBenchmarkReference(file.path, .025), std::runtime_error);
  }
}

TEST(BenchmarkReference, ConvertsFullAttitudeWithLocalZRotation)
{
  tf::Quaternion q;
  q.setRPY(.3, -.7, 1.1);
  geometry_msgs::Quaternion message;
  tf::quaternionTFToMsg(q, message);
  const auto link = benchmarkLinkRotation(message);
  const tf::Matrix3x3 flu_matrix(q), link_matrix(link);
  for (int i = 0; i < 3; ++i)
  {
    EXPECT_NEAR(link_matrix[i][0], -flu_matrix[i][0], 1e-12);
    EXPECT_NEAR(link_matrix[i][1], -flu_matrix[i][1], 1e-12);
    EXPECT_NEAR(link_matrix[i][2], flu_matrix[i][2], 1e-12);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
