#include "test_octomap.h"
#include <motion_primitive_planner/dragon_collision_checker.h>
#include <motion_primitive_planner/root_primitive_generator.h>
#include "trajectory_collision.h"

#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <algorithm>
#include <future>
#include <limits>

namespace motion_primitive_planner
{
namespace
{
Trajectory<5> linearRoot(const Eigen::Vector3d& start, const Eigen::Vector3d& velocity,
                        double duration)
{
  Piece<5>::CoefficientMat coefficients = Piece<5>::CoefficientMat::Zero();
  coefficients.col(5) = start;
  coefficients.col(4) = velocity;
  return Trajectory<5>({duration}, {coefficients});
}

class DragonCollision : public ::testing::Test
{
protected:
  void SetUp() override
  {
    loader_.reset(new pluginlib::ClassLoader<aerial_robot_model::RobotModel>(
        "aerial_robot_model", "aerial_robot_model::RobotModel"));
    robot_ = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(
        loader_->createInstance("dragon/hydrus_like_robot_model"));
    ASSERT_TRUE(robot_);
    model_ = std::make_shared<DragonCollisionModel>(*robot_);
    checker_.reset(new DragonCollisionChecker(model_));
    configuration_.link1_tail = Eigen::Vector3d(0.0, 0.0, 1.0);
    configuration_.joint_positions = Eigen::VectorXd::Zero(6);
  }

  Eigen::Affine3d pose(const std::string& link) const
  {
    KDL::JntArray joints(robot_->getTree().getNrOfJoints());
    joints.data.setZero();
    for (size_t i = 0; i < robot_->getLinkJointIndices().size(); ++i)
      joints(robot_->getLinkJointIndices()[i]) = configuration_.joint_positions(i);
    const auto kdl_link1 = robot_->forwardKinematics<Eigen::Affine3d>("link1", joints);
    const auto kdl_link = robot_->forwardKinematics<Eigen::Affine3d>(link, joints);
    Eigen::Affine3d world_link1 = Eigen::Affine3d::Identity();
    world_link1.linear() = configuration_.root_link_rotation;
    world_link1.translation() = configuration_.link1_tail -
        robot_->getLinkLength() * configuration_.root_link_rotation.col(0);
    const auto collision = robot_->getUrdfModel().getLink(link)->collision;
    Eigen::Affine3d local = Eigen::Affine3d::Identity();
    const auto& q = collision->origin.rotation;
    local.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
    const auto& p = collision->origin.position;
    local.translation() = Eigen::Vector3d(p.x, p.y, p.z);
    return world_link1 * kdl_link1.inverse() * kdl_link * local;
  }

  std::shared_ptr<CollisionEnvironment> at(const Eigen::Vector3d& point, double width = 0.002) const
  {
    // Put the requested world point exactly at a cell center on a shifted grid.
    const Eigen::Vector3d origin = point - Eigen::Vector3d::Constant(5000.5 * width);
    return std::make_shared<TestCollisionEnvironment>(std::vector<Eigen::Vector3d>{point},
        width, origin, Eigen::Vector3i::Constant(10001));
  }

  std::shared_ptr<DragonCollisionModel> fromUrdf(const urdf::Model& urdf) const
  {
    return std::make_shared<DragonCollisionModel>(urdf, robot_->getTree(),
        robot_->getLinkJointNames(), robot_->getLinkJointIndices(), robot_->getLinkLength());
  }

  void readUrdf(urdf::Model& urdf) const
  {
    std::string xml;
    ASSERT_TRUE(ros::NodeHandle().getParam("robot_description", xml));
    ASSERT_TRUE(urdf.initString(xml));
  }

  std::unique_ptr<pluginlib::ClassLoader<aerial_robot_model::RobotModel>> loader_;
  boost::shared_ptr<Dragon::HydrusLikeRobotModel> robot_;
  std::shared_ptr<DragonCollisionModel> model_;
  std::unique_ptr<DragonCollisionChecker> checker_;
  WholeBodyConfiguration configuration_;
};

TEST_F(DragonCollision, LoadsEightPrimitivesAndCoversTheirVolumeBeyondCenterlines)
{
  ASSERT_EQ(model_->geometryCount(), 8u);
  for (int link = 1; link <= 4; ++link)
  {
    const std::string name = "link" + std::to_string(link);
    const Eigen::Vector3d box_point = pose(name) * Eigen::Vector3d(0.18, 0.05, 0.07);
    EXPECT_TRUE(checker_->collides(configuration_, *at(box_point))) << name;
    const std::string gimbal = "gimbal" + std::to_string(link) + "_roll_module";
    for (double end : {-0.18, 0.18})
    {
      const Eigen::Vector3d point = pose(gimbal) * Eigen::Vector3d(0.0, 0.0, end);
      EXPECT_TRUE(checker_->collides(configuration_, *at(point))) << gimbal << " " << end;
    }
  }
  EXPECT_FALSE(checker_->collides(configuration_, *at(Eigen::Vector3d(0.0, 0.0, 2.0))));
}

TEST_F(DragonCollision, IncludesUrdfJointOffsetsAndCollisionOrigins)
{
  // The yaw joints add 0.0515 m per segment beyond the 0.474 m link length.
  // The planner's tail convention uses the model's link length (including the offset).
  EXPECT_NEAR(robot_->getLinkLength(), 0.5255, 1e-12);
  const Eigen::Vector3d last_box_center(-0.5255 + 3.0 * (0.474 + 0.0515) + 0.237, -0.017, 1.005);
  EXPECT_TRUE(pose("link4").translation().isApprox(last_box_center, 1e-12));
  // Near the far end of the box, beyond its centerline endpoint.
  EXPECT_TRUE(checker_->collides(configuration_, *at(last_box_center + Eigen::Vector3d(0.29, 0.0, 0.0))));
  const Eigen::Vector3d gimbal_center(-0.5255 + 3.0 * (0.474 + 0.0515) + 0.237, 0.0, 1.02);
  EXPECT_TRUE(pose("gimbal4_roll_module").translation().isApprox(gimbal_center, 1e-12))
      << "actual=" << pose("gimbal4_roll_module").translation().transpose()
      << " expected=" << gimbal_center.transpose();
  EXPECT_TRUE(checker_->collides(configuration_, *at(gimbal_center + Eigen::Vector3d(0.0, 0.195, 0.0))));
}

TEST_F(DragonCollision, FollowsRootAndArticulatedKdlPoses)
{
  configuration_.link1_tail = Eigen::Vector3d(1.23, -0.71, 2.14);
  configuration_.root_link_rotation =
      (Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX())).toRotationMatrix();
  configuration_.joint_positions << 0.25, 0.7, -0.3, -0.8, 0.45, 0.5;
  for (int link = 1; link <= 4; ++link)
  {
    for (const std::string& name : {"link" + std::to_string(link),
                                    "gimbal" + std::to_string(link) + "_roll_module"})
      EXPECT_TRUE(checker_->collides(configuration_, *at(pose(name).translation()))) << name;
  }
  EXPECT_FALSE(checker_->collides(configuration_, *at(Eigen::Vector3d(0.0, 0.0, 8.0))));
}

TEST_F(DragonCollision, DetectsContactWithoutAddingAnEnvironmentMargin)
{
  configuration_.link1_tail.z() = 1.09;
  const Eigen::Vector3d origin(-4.0, -4.0, 0.0);
  const Eigen::Vector3i size(400, 400, 400);
  // link1 bottom is z=1.0; this voxel's top is z=1.0.
  TestCollisionEnvironment box_contact({Eigen::Vector3d(-0.39, -0.01, 0.99)}, 0.02, origin, size);
  EXPECT_TRUE(checker_->collides(configuration_, box_contact));
  configuration_.link1_tail.z() += 0.001;
  EXPECT_FALSE(checker_->collides(configuration_, box_contact));
  configuration_.link1_tail.z() -= 0.002;
  EXPECT_TRUE(checker_->collides(configuration_, box_contact));
  configuration_.link1_tail.z() = 1.09;
  // Gimbal end is y=0.20; the cell begins at y=0.20.
  TestCollisionEnvironment cylinder_contact({Eigen::Vector3d(-0.23, 0.21, 1.11)}, 0.02, origin, size);
  EXPECT_TRUE(checker_->collides(configuration_, cylinder_contact));
  configuration_.link1_tail.y() -= 0.001;
  EXPECT_FALSE(checker_->collides(configuration_, cylinder_contact));
}

TEST_F(DragonCollision, KeepsGimbalsCanonicalAndDoesNotModifyDynamicsState)
{
  const auto environment = at(pose("gimbal3_roll_module") * Eigen::Vector3d(0.0, 0.0, 0.19));
  ASSERT_TRUE(checker_->collides(configuration_, *environment));
  KDL::JntArray dynamics(robot_->getTree().getNrOfJoints());
  dynamics.data.setConstant(0.73);
  for (int index : robot_->getLinkJointIndices()) dynamics(index) = 0.0;
  // HydrusLike solves gimbals from the desired CoG attitude, overwriting inputs.
  // Tilt that attitude so the actual saved dynamics state has nonzero gimbals.
  robot_->setCogDesireOrientation(0.6, -0.4, 0.0);
  robot_->updateRobotModel(dynamics);
  const KDL::JntArray before = robot_->getJointPositions();
  const auto joint_indices = robot_->getJointIndexMap();
  ASSERT_GT(std::abs(before(joint_indices.at("gimbal3_roll"))), 0.1);
  ASSERT_GT(std::abs(before(joint_indices.at("gimbal3_pitch"))), 0.1);
  DragonCollisionChecker rebuilt(std::make_shared<DragonCollisionModel>(*robot_));
  EXPECT_TRUE(checker_->collides(configuration_, *environment));
  EXPECT_TRUE(rebuilt.collides(configuration_, *environment));
  EXPECT_TRUE(robot_->getJointPositions().data.isApprox(before.data, 0.0));
}

TEST_F(DragonCollision, ChecksWholeGeometryAgainstMapBoundsEvenWithNoOccupiedCells)
{
  TestCollisionEnvironment empty({}, 0.01, Eigen::Vector3d(-4.0, -4.0, 0.0), Eigen::Vector3i(800, 800, 400));
  EXPECT_FALSE(checker_->collides(configuration_, empty));
  configuration_.link1_tail.z() = 0.05;  // Centerline is in bounds; box volume is not.
  EXPECT_TRUE(checker_->collides(configuration_, empty));
  configuration_.link1_tail.z() = 1.0;
  configuration_.link1_tail.y() = -3.85;  // Only the gimbal end crosses y=-4.
  EXPECT_TRUE(checker_->collides(configuration_, empty));
  configuration_.link1_tail.y() = -3.79;
  EXPECT_FALSE(checker_->collides(configuration_, empty));
  configuration_.link1_tail = Eigen::Vector3d(3.0, 0.0, 1.0);
  EXPECT_TRUE(checker_->collides(configuration_, empty));
}

TEST_F(DragonCollision, RejectsInvalidConfigurationAndJointMappings)
{
  TestCollisionEnvironment empty({}, 0.1, Eigen::Vector3d(-4.0, -4.0, -4.0), Eigen::Vector3i::Constant(80));
  const auto valid = configuration_;
  configuration_.joint_positions.resize(5);
  EXPECT_TRUE(checker_->collides(configuration_, empty));
  configuration_ = valid;
  configuration_.joint_positions(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(checker_->collides(configuration_, empty));
  configuration_ = valid;
  configuration_.root_link_rotation(0, 0) = 2.0;
  EXPECT_TRUE(checker_->collides(configuration_, empty));
  configuration_ = valid;
  configuration_.root_link_rotation(0, 0) = -1.0;
  EXPECT_TRUE(checker_->collides(configuration_, empty));
  configuration_ = valid;
  configuration_.link1_tail.x() = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(checker_->collides(configuration_, empty));
  auto indices = robot_->getLinkJointIndices();
  indices[0] = indices[1];
  EXPECT_THROW(DragonCollisionModel(robot_->getUrdfModel(), robot_->getTree(),
      robot_->getLinkJointNames(), indices, robot_->getLinkLength()), std::invalid_argument);
  EXPECT_THROW(DragonCollisionModel(robot_->getUrdfModel(), KDL::Tree(),
      robot_->getLinkJointNames(), robot_->getLinkJointIndices(), robot_->getLinkLength()), std::invalid_argument);
}

TEST_F(DragonCollision, RejectsMissingWrongDuplicateAndInvalidPrimitives)
{
  urdf::Model urdf;
  urdf::LinkSharedPtr link;
  readUrdf(urdf);
  urdf.getLink("link1", link);
  link->collision_array.clear();
  EXPECT_THROW(fromUrdf(urdf), std::invalid_argument);
  readUrdf(urdf);
  urdf.getLink("link1", link);
  link->collision_array.front()->geometry.reset(new urdf::Mesh());
  EXPECT_THROW(fromUrdf(urdf), std::invalid_argument);
  readUrdf(urdf);
  urdf.getLink("link1", link);
  link->collision_array.push_back(link->collision_array.front());
  EXPECT_THROW(fromUrdf(urdf), std::invalid_argument);
  readUrdf(urdf);
  urdf.getLink("link1", link);
  static_cast<urdf::Box&>(*link->collision_array.front()->geometry).dim.x = -1.0;
  EXPECT_THROW(fromUrdf(urdf), std::invalid_argument);
  readUrdf(urdf);
  urdf.getLink("gimbal1_roll_module", link);
  static_cast<urdf::Cylinder&>(*link->collision_array.front()->geometry).radius = 0.0;
  EXPECT_THROW(fromUrdf(urdf), std::invalid_argument);
}

TEST_F(DragonCollision, IgnoresVisualsMeshesAndUnselectedCollisionLinks)
{
  urdf::Model urdf;
  readUrdf(urdf);
  urdf::LinkSharedPtr link;
  urdf.getLink("link1", link);
  urdf::CollisionSharedPtr mesh(new urdf::Collision());
  auto geometry = new urdf::Mesh();
  geometry->filename = "package://does_not_exist/never_load_this.stl";
  mesh->geometry.reset(geometry);
  link->collision_array.push_back(mesh);
  urdf.getLink("root", link);
  urdf::CollisionSharedPtr extra(new urdf::Collision());
  auto box = new urdf::Box();
  box->dim = urdf::Vector3(100.0, 100.0, 100.0);
  extra->geometry.reset(box);
  link->collision_array.push_back(extra);
  const auto model = fromUrdf(urdf);
  EXPECT_EQ(model->geometryCount(), 8u);
  DragonCollisionChecker checker(model);
  EXPECT_FALSE(checker.collides(configuration_, *at(Eigen::Vector3d(0.0, 0.0, 3.0))));
}

TEST_F(DragonCollision, AlignsShiftedGridAndDeduplicatesOccupiedCells)
{
  const Eigen::Vector3d origin(-3.033, -3.027, 0.013);
  const Eigen::Vector3d point = pose("link1") * Eigen::Vector3d(0.18, 0.05, 0.07);
  TestCollisionEnvironment occupied({point, point, point}, 0.01, origin, Eigen::Vector3i(800, 800, 400));
  EXPECT_EQ(occupied.occupiedVoxelCount(), 1u);
  EXPECT_TRUE(checker_->collides(configuration_, occupied));
  TestCollisionEnvironment empty({}, 0.01, origin, Eigen::Vector3i(800, 800, 400));
  EXPECT_EQ(empty.occupiedVoxelCount(), 0u);
  EXPECT_FALSE(checker_->collides(configuration_, empty));
}

TEST_F(DragonCollision, SharesImmutableEnvironmentAcrossIndependentQueryWorkers)
{
  const auto hit = at(pose("gimbal4_roll_module") * Eigen::Vector3d(0.0, 0.0, 0.18));
  const auto miss = at(Eigen::Vector3d(0.0, 0.0, 3.0));
  std::vector<std::future<bool>> workers;
  for (int thread = 0; thread < 4; ++thread)
    workers.push_back(std::async(std::launch::async, [this, hit, miss]() {
      DragonCollisionChecker checker(model_);
      for (int query = 0; query < 100; ++query)
        if (!checker.collides(configuration_, *hit) || checker.collides(configuration_, *miss)) return false;
      return true;
    }));
  for (auto& worker : workers) EXPECT_TRUE(worker.get());
}

TEST_F(DragonCollision, CapturesRawCellsWithRouteDilationAndPreservesPreviousSnapshots)
{
  SharedPlannerConfig config(ros::NodeHandle("~"));
  config.common.dilateRadius = 0.2;
  PlanningEnvironment environment(config);
  // Above the complete robot but inside the route map's dilation neighborhood.
  const Eigen::Vector3d clear_point(-0.237, 0.0, 1.23);
  replaceTestMap(environment, {clear_point});
  const auto first = environment.snapshot();
  EXPECT_TRUE(first->route->query(Eigen::Vector3d(-0.237, 0.0, 1.10)));
  EXPECT_FALSE(checker_->collides(configuration_, *first->collision));
  config.common.dilateRadius = 0.0;
  PlanningEnvironment undilated(config);
  replaceTestMap(undilated, {clear_point});
  EXPECT_FALSE(undilated.snapshot()->route->query(Eigen::Vector3d(-0.237, 0.0, 1.10)));
  EXPECT_FALSE(checker_->collides(configuration_, *undilated.snapshot()->collision));
  const Eigen::Vector3d hit = pose("gimbal2_roll_module") * Eigen::Vector3d(0.0, 0.0, 0.18);
  replaceTestMap(environment, {hit, hit});
  const auto second = environment.snapshot();
  EXPECT_TRUE(first->route->query(clear_point));
  EXPECT_FALSE(first->route->query(hit));
  EXPECT_FALSE(second->route->query(clear_point));
  EXPECT_TRUE(second->route->query(hit));
  EXPECT_EQ(second->collision->occupiedVoxelCount(), 1u);
  EXPECT_TRUE(checker_->collides(configuration_, *second->collision));
  EXPECT_FALSE(checker_->collides(configuration_, *first->collision));
  replaceTestMap(environment, {});
  EXPECT_FALSE(checker_->collides(configuration_, *environment.snapshot()->collision));
  EXPECT_TRUE(checker_->collides(configuration_, *second->collision));
}

TEST_F(DragonCollision, FullTreeFreeUnknownAndPrunedVolumesAgreeWithRoute)
{
  SharedPlannerConfig config(ros::NodeHandle("~"));
  config.common.voxelWidth = .1;
  config.common.dilateRadius = 0;
  PlanningEnvironment environment(config);
  const auto route = environment.snapshot()->route;
  const auto origin = route->mapOrigin();
  auto tree = std::make_shared<octomap::OcTree>(.1);
  // Align an eight-cell block around link1 to the octree's depth-15 grid.
  const Eigen::Vector3d center = pose("link1").translation() - origin;
  const Eigen::Vector3d lower = (center.array() / .2).floor().matrix() * .2;
  for (int x = 0; x < 2; ++x) for (int y = 0; y < 2; ++y) for (int z = 0; z < 2; ++z)
  {
    const auto p = (lower + Eigen::Vector3d(x + .5, y + .5, z + .5) * .1).eval();
    tree->updateNode(octomap::point3d(p.x(), p.y(), p.z()), true, true);
  }
  tree->updateInnerOccupancy(); tree->prune();
  ASSERT_EQ(tree->getNumLeafNodes(), 1u);
  environment.replaceMap(tree, gridTransform(origin), origin, route->mapCorner(), ros::Time(5));
  const auto occupied_scene = environment.snapshot();
  EXPECT_EQ(occupied_scene->map_stamp, ros::Time(5));
  EXPECT_TRUE(checker_->collides(configuration_, *occupied_scene->collision));
  for (int x = 0; x < 2; ++x) for (int y = 0; y < 2; ++y) for (int z = 0; z < 2; ++z)
    EXPECT_TRUE(occupied_scene->route->query(origin + lower + Eigen::Vector3d(x + .5, y + .5, z + .5) * .1));
  auto free_tree = std::make_shared<octomap::OcTree>(*tree);
  for (auto it = free_tree->begin_leafs(); it != free_tree->end_leafs(); ++it) it->setLogOdds(octomap::logodds(.4));
  free_tree->updateInnerOccupancy();
  environment.replaceMap(free_tree, gridTransform(origin), origin, route->mapCorner());
  EXPECT_FALSE(checker_->collides(configuration_, *environment.snapshot()->collision));
  EXPECT_FALSE(environment.occupied(origin + center));
  EXPECT_TRUE(checker_->collides(configuration_, *occupied_scene->collision));
  auto invalid_transform = gridTransform(origin); invalid_transform.translation().x() += .01;
  const auto before = environment.snapshot();
  EXPECT_THROW(environment.replaceMap(tree, invalid_transform, origin, route->mapCorner()), std::invalid_argument);
  EXPECT_EQ(environment.snapshot(), before);
}

TEST_F(DragonCollision, SamplesFirstAndLastSynchronizedConfigurations)
{
  JointPlanResult joints;
  joints.duration = 0.01;  // Less than one command interval: only the two endpoints.
  joints.joint_waypoints = {{0.0, configuration_.joint_positions},
                           {joints.duration, configuration_.joint_positions}};
  joints.attitude_waypoints = {{0.0, {M_PI, 0.0}}, {joints.duration, {M_PI, 0.0}}};
  configuration_.root_link_rotation = joints.rootLinkRotation(0.0);
  const auto root = linearRoot(configuration_.link1_tail, Eigen::Vector3d(0.0, 0.0, 100.0), joints.duration);
  const auto first = at(pose("link1") * Eigen::Vector3d(0.18, 0.0, 0.0));
  configuration_.link1_tail = root.getPos(joints.duration);
  const auto last = at(pose("link1") * Eigen::Vector3d(0.18, 0.0, 0.0));
  ASSERT_FALSE(checker_->collides(configuration_, *first));
  EXPECT_TRUE(trajectoryCollides(root, joints, 40.0, *first, *checker_));
  EXPECT_TRUE(trajectoryCollides(root, joints, 40.0, *last, *checker_));
  joints.duration = 0.0;
  EXPECT_TRUE(trajectoryCollides(root, joints, 40.0, *first, *checker_));
  EXPECT_FALSE(trajectoryCollides(root, joints, 40.0, *last, *checker_));
}

TEST_F(DragonCollision, SamplesJointRepairDuringRootTranslationDelay)
{
  JointPlanResult joints;
  joints.duration = 0.1;
  joints.root_translation_delay = 0.075;
  Eigen::VectorXd bent = configuration_.joint_positions;
  const auto& names = robot_->getLinkJointNames();
  const auto yaw = std::find(names.begin(), names.end(), "joint1_yaw") - names.begin();
  bent(yaw) = M_PI_2;
  joints.joint_waypoints = {{0.0, configuration_.joint_positions}, {0.025, bent},
                           {0.075, configuration_.joint_positions},
                           {0.1, configuration_.joint_positions}};
  joints.attitude_waypoints = {{0.0, {M_PI, 0.0}}, {0.1, {M_PI, 0.0}}};
  configuration_.root_link_rotation = joints.rootLinkRotation(0.025);
  configuration_.joint_positions = joints.jointPositions(0.025);
  const auto obstacle = at(pose("gimbal4_roll_module").translation());
  const auto root = linearRoot(configuration_.link1_tail, Eigen::Vector3d(0.0, 0.0, 20.0), 0.025);
  configuration_.joint_positions = joints.jointPositions(0.0);
  ASSERT_FALSE(checker_->collides(configuration_, *obstacle));
  configuration_.link1_tail = root.getPos(0.025);
  ASSERT_FALSE(checker_->collides(configuration_, *obstacle));
  EXPECT_TRUE(trajectoryCollides(root, joints, 40.0, *obstacle, *checker_));
  joints.root_translation_delay = 0.0;
  EXPECT_FALSE(trajectoryCollides(root, joints, 40.0, *obstacle, *checker_));
}

TEST_F(DragonCollision, UsesTimeScaledRootAndInterpolatedAttitude)
{
  JointPlanResult joints;
  joints.duration = 0.1;
  joints.time_scale = 4.0;
  joints.joint_waypoints = {{0.0, configuration_.joint_positions}, {0.1, configuration_.joint_positions}};
  joints.attitude_waypoints = {{0.0, {M_PI, 0.0}}, {0.1, {M_PI, 1.0}}};
  const auto original = linearRoot(configuration_.link1_tail, Eigen::Vector3d(0.0, 0.0, 40.0), 0.025);
  const auto scaled = gcopter_planner::PlannerBackend::timeScaledTrajectory(original, joints.time_scale);
  configuration_.link1_tail = scaled.getPos(0.05);
  configuration_.root_link_rotation = joints.rootLinkRotation(0.05);
  const auto obstacle = at(pose("gimbal4_roll_module").translation());
  EXPECT_TRUE(trajectoryCollides(scaled, joints, 40.0, *obstacle, *checker_));
  EXPECT_FALSE(trajectoryCollides(original, joints, 40.0, *obstacle, *checker_));
  joints.attitude_waypoints.back().attitude.pitch = 0.0;
  EXPECT_FALSE(trajectoryCollides(scaled, joints, 40.0, *obstacle, *checker_));
}
}  // namespace
}  // namespace motion_primitive_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dragon_collision_checker_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
