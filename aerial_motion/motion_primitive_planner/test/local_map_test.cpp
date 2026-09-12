#include <motion_primitive_planner/local_map.h>
#include <motion_primitive_planner/dragon_collision_checker.h>
#include <rog_map_msgs/validation.h>
#include <gtest/gtest.h>
#include <atomic>
#include <thread>

namespace motion_primitive_planner {
namespace {
SharedPlannerConfig config() {
  SharedPlannerConfig c;
  c.common.worldFrameId="world";c.common.voxelWidth=.1;c.common.dilateRadius=.2;
  c.common.mapBound={-6,6,-6,6,-4,4};c.common.timeoutRRT=.2;c.common.maxVelMag=.5;
  c.primitive.candidate_count=3;c.primitive.max_offset=.4;c.primitive.max_velocity=.5;
  c.primitive.cruise_velocity=.3;c.primitive.minimum_piece_duration=.2;c.planning_horizon=3;
  c.validateOrThrow();return c;
}
rog_map_msgs::LocalMap emptyMap() {
  rog_map_msgs::LocalMap m;m.header.frame_id="world";m.header.stamp=ros::Time(10);m.epoch=100;m.version=1;
  m.resolution=.1;m.inflation_steps=2;m.size={121,121,81};m.origin.x=-6;m.origin.y=-6;m.origin.z=-3;
  m.occupied_bits.resize((121*121*81+7)/8);m.inflated_bits=m.occupied_bits;return m;
}
TEST(LocalMap, RawGeometryAndImportedInflationAgreeWithLegacyRoute) {
  auto m=emptyMap();auto c=config();
  std::vector<Eigen::Vector3i> cells={{60,60,40},{0,0,0},{120,120,80}};
  for(const auto& p:cells) {
    rog_map_msgs::setBit(m.occupied_bits,p.x()+121*(p.y()+121*p.z()));
    for(int dx=-2;dx<=2;++dx)for(int dy=-2;dy<=2;++dy)for(int dz=-2;dz<=2;++dz) {
      Eigen::Vector3i q=p+Eigen::Vector3i(dx,dy,dz);
      if((q.array()<0).any()||q.x()>=121||q.y()>=121||q.z()>=81)continue;
      rog_map_msgs::setBit(m.inflated_bits,q.x()+121*(q.y()+121*q.z()));
    }
  }
  auto scene=buildLocalScene(m,c.common);
  EXPECT_EQ(scene->collision->occupiedVoxelCount(),3u);
  EXPECT_EQ(scene->epoch,100u);EXPECT_EQ(scene->map_stamp,ros::Time(10));
  gcopter_planner::RoutePlannerBackend legacy(c.common,scene->route->mapOrigin(),scene->route->mapSize());
  legacy.setMapVoxels(cells);
  for(int z=0;z<81;++z)for(int y=0;y<121;++y)for(int x=0;x<121;++x) {
    Eigen::Vector3d p=scene->route->mapOrigin()+.1*Eigen::Vector3d(x+.5,y+.5,z+.5);
    ASSERT_EQ(scene->route->query(p),legacy.query(p));
  }
  auto cleared=emptyMap();cleared.version=2;
  auto next=buildLocalScene(cleared,c.common);
  EXPECT_EQ(next->collision->occupiedVoxelCount(),0u);
  EXPECT_EQ(scene->collision->occupiedVoxelCount(),3u);
}
TEST(LocalMap, RejectsInvalidSnapshots) {
  auto m=emptyMap();auto c=config();
  auto bad=m;bad.inflated_bits.pop_back();EXPECT_THROW(buildLocalScene(bad,c.common),std::invalid_argument);
  bad=m;bad.header.frame_id="other";EXPECT_THROW(buildLocalScene(bad,c.common),std::invalid_argument);
  bad=m;bad.resolution=.2;EXPECT_THROW(buildLocalScene(bad,c.common),std::invalid_argument);
  bad=m;bad.origin.x+=.025;EXPECT_THROW(buildLocalScene(bad,c.common),std::invalid_argument);
  bad=m;bad.size={1,1,1};bad.occupied_bits={0};bad.inflated_bits={0};
  EXPECT_THROW(buildLocalScene(bad,c.common),std::invalid_argument);
  bad=m;bad.size[0]=0;EXPECT_THROW(buildLocalScene(bad,c.common),std::invalid_argument);
  bad=m;bad.epoch=0;EXPECT_THROW(buildLocalScene(bad,c.common),std::invalid_argument);
  bad=m;bad.inflation_steps=3;EXPECT_THROW(buildLocalScene(bad,c.common),std::invalid_argument);
  bad=m;bad.occupied_bits[0]=1;EXPECT_THROW(buildLocalScene(bad,c.common),std::invalid_argument);
  bad=m;bad.occupied_bits.back()=128;bad.inflated_bits.back()=128;
  EXPECT_THROW(buildLocalScene(bad,c.common),std::invalid_argument);
}
TEST(LocalMap, ConcurrentPublicationKeepsOldScenesImmutable) {
  auto c=config();c.common.dilateRadius=0;PlanningEnvironment environment(c);
  auto m=emptyMap();m.inflation_steps=0;m.size={11,11,11};
  m.occupied_bits.assign((11*11*11+7)/8,0);m.inflated_bits=m.occupied_bits;
  m.version=2;
  auto original=buildLocalScene(m,c.common);environment.replaceScene(original);
  std::atomic<bool> done{false};
  std::thread writer([&] {
    for(unsigned version=3;version<=102;++version) {
      m.version=version;m.occupied_bits[0]=version%2;m.inflated_bits=m.occupied_bits;
      environment.replaceScene(buildLocalScene(m,c.common));
    }
    done=true;
  });
  do {
    const auto scene=environment.snapshot();
    EXPECT_EQ(scene->collision->occupiedVoxelCount(),scene->version%2);
    EXPECT_EQ(scene->route->query(scene->route->mapOrigin()+Eigen::Vector3d::Constant(.05)),scene->version%2);
    EXPECT_EQ(original->collision->occupiedVoxelCount(),0u);
    std::this_thread::yield();
  } while(!done);
  writer.join();
  EXPECT_EQ(environment.snapshot()->version,102u);
}
TEST(LocalMap, DistantGoalSurvivesWindowMovement) {
  ros::Time::init();auto c=config();auto m=emptyMap();PlanningEnvironment environment(c);
  RootState start;start.position=Eigen::Vector3d(0,0,1);
  auto first=buildLocalScene(m,c.common);
  auto batch=environment.generate(start,Eigen::Vector3d(20,0,1),first);
  ASSERT_TRUE(batch.success())<<batch.detail;EXPECT_FALSE(batch.terminal);EXPECT_GT(batch.local_target.x(),1);
  m.origin.x+=6;m.version=2;auto second=buildLocalScene(m,c.common);start.position.x()=6;
  auto later=environment.generate(start,Eigen::Vector3d(20,0,1),second);
  ASSERT_TRUE(later.success())<<later.detail;EXPECT_FALSE(later.terminal);EXPECT_GT(later.local_target.x(),7);
  EXPECT_EQ(first->route->mapOrigin().x(),-6);
  environment.replaceScene(second);EXPECT_EQ(environment.snapshot(),second);
  start.position.x()=8;auto terminal=environment.generate(start,Eigen::Vector3d(9,0,1),second);
  ASSERT_TRUE(terminal.success());EXPECT_TRUE(terminal.terminal);
}
}
}

int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
