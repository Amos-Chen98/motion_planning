// Compile this same harness against both archived and current headers/libraries.
#include "plan_snapshot.h"
#include <pluginlib/class_loader.h>
#include <ompl/util/Console.h>
#include <ros/ros.h>

#include <chrono>
#include <fstream>
#include <stdexcept>

using namespace motion_primitive_planner;
using Clock = std::chrono::steady_clock;

struct Scenario
{
  std::string name;
  RootState start;
  Eigen::Vector3d target;
  RootAttitude attitude;
  Eigen::VectorXd joints;
  std::vector<Eigen::Vector3d> obstacles;
  PrimitiveBatch captured;
  std::shared_ptr<const gcopter_planner::PlannerBackend> occupancy;
  NominalJointContext context;
};

double elapsedMs(Clock::time_point start)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "parallel_benchmark");
  ros::NodeHandle nh("~");
  std::string output;
  nh.param<std::string>("output", output, "/tmp/planner_benchmark.json");
  std::string mode;
  nh.param<std::string>("mode", mode, "ample");
  int threads = 1;
  nh.param("threads", threads, 1);
  ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
  const auto initialization_start = Clock::now();
  WholeBodyPlannerConfig config(nh);
#ifndef MOTION_PRIMITIVE_BASELINE
  config.planning_threads = threads;
#endif
  if (mode == "ample") config.joint.planning_timeout = 5.0;
  config.shared.common.dilateRadius = 0.0;
  // Use launch-equivalent translational and angular limits for the main corpus.
  config.shared.primitive.candidate_count = 9;
  pluginlib::ClassLoader<aerial_robot_model::RobotModel> loader(
      "aerial_robot_model", "aerial_robot_model::RobotModel");
  std::vector<std::shared_ptr<multilink_copilot::StabilityEvaluator>> evaluators;
  for (int index = 0; index < config.shared.primitive.candidate_count; ++index)
  {
    const auto model = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(
        loader.createInstance("dragon/hydrus_like_robot_model"));
    if (!model) throw std::runtime_error("robot model unavailable");
    evaluators.push_back(std::make_shared<multilink_copilot::StabilityEvaluator>(model, config.stability));
  }
  DragonModelInfo info(evaluators.front()->robotModel());
  WholeBodyPlanner planner(config, info.collisionGeometry(), evaluators);
  PlanningEnvironment environment(config.shared);
  PrimitiveGenerator generator(config.shared.primitive);
  const double initialization_ms = elapsedMs(initialization_start);

  std::vector<Scenario> scenarios;
  auto scenario = [&](const std::string& name, const Eigen::Vector3d& displacement) {
    Scenario s;
    s.name = name;
    s.start.position = Eigen::Vector3d(0.0, 0.0, 2.0);
    s.target = s.start.position + displacement;
    s.joints = (Eigen::VectorXd(6) << 0.0, M_PI_2, 0.0, M_PI_2, 0.0, M_PI_2).finished();
    return s;
  };
  for (int direction = 0; direction < 24; ++direction)
    for (int distance : {1, 3})
    {
      const double angle = 2.0 * M_PI * direction / 24.0;
      scenarios.push_back(scenario("free_" + std::to_string(direction) + "_" + std::to_string(distance),
          distance * Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0)));
    }
  auto moving = scenario("moving", Eigen::Vector3d(1.0, 0.25, 0.0));
  moving.start.velocity = Eigen::Vector3d(0.25, 0.0, 0.0);
  moving.start.acceleration = Eigen::Vector3d(0.02, 0.0, 0.0);
  scenarios.push_back(moving);
  auto pitched = scenario("compound_attitude", Eigen::Vector3d(-1.0, 0.6, 0.8));
  pitched.attitude = {0.2, 0.1};
  scenarios.push_back(pitched);
  auto fold = scenario("opposite_fold", Eigen::Vector3d(-1.5, -0.6, 0.0));
  fold.joints = -fold.joints;
  scenarios.push_back(fold);
  auto obstacle = scenario("downstream_collision", Eigen::Vector3d(1.0, 0.0, 0.0));
  const auto endpoints = linkEndpoints(obstacle.start.position, linkRotation(obstacle.attitude),
      obstacle.joints, info.pitchJointIndices(), info.yawJointIndices(), info.linkNum(), info.linkLength());
  obstacle.obstacles.push_back(0.5 * (endpoints[2] + endpoints[3]));
  scenarios.push_back(obstacle);
  auto nearby = scenario("near_obstacle", Eigen::Vector3d(2.0, 0.0, 0.0));
  nearby.obstacles = {Eigen::Vector3d(1.0, 0.45, 2.0), Eigen::Vector3d(1.0, -0.45, 2.0)};
  scenarios.push_back(nearby);
  scenarios.push_back(scenario("short_chord", Eigen::Vector3d(0.12, 0.0, 0.0)));

  for (auto& s : scenarios)
  {
    environment.replaceMap(s.obstacles);
    s.occupancy = environment.occupancySnapshot();
    Eigen::Matrix3d initial, final = Eigen::Matrix3d::Zero();
    initial.col(0) = s.start.position;
    initial.col(1) = s.start.velocity;
    initial.col(2) = s.start.acceleration;
    final.col(0) = s.target;
    s.captured.candidates = generator.generate(initial, final);
    TrajectoryHistory history(config.joint.follower);
    history.append(s.start.position);
    s.context = makeNominalJointContext(history, info);
  }
  const double budget = mode == "ample" ? 60.0 : (mode == "tight" ? 0.06 : 0.60);
  // Warm the pool, model kernels and allocator without timing construction.
  for (int i = 0; i < 3; ++i)
    planner.plan(scenarios[0].captured, scenarios[0].occupancy, scenarios[0].joints,
                 scenarios[0].attitude, scenarios[0].context, ros::Time::now() + ros::Duration(budget));

  std::ofstream out(output);
  if (!out) throw std::runtime_error("cannot open benchmark output");
  out << std::setprecision(17) << "{\"threads\":" << threads << ",\"mode\":" << std::quoted(mode)
      << ",\"initialization_ms\":" << initialization_ms << ",\"scenarios\":[";
  bool first = true;
  for (const auto& s : scenarios)
  {
    const auto start = Clock::now();
    const WholeBodyPlanResult result = planner.plan(s.captured, s.occupancy, s.joints, s.attitude,
        s.context, ros::Time::now() + ros::Duration(budget));
    const double batch_ms = elapsedMs(start);
    // Include the existing route search and MINCO generation in a separate
    // end-to-end measurement. Its stochastic route is not the equivalence input.
    environment.replaceMap(s.obstacles);
    const auto full_start = Clock::now();
    const auto deadline = ros::Time::now() + ros::Duration(budget);
    const auto batch = environment.generate(s.start, s.target);
    WholeBodyPlanResult full;
    if (batch.success()) full = planner.plan(batch, environment.occupancySnapshot(), s.joints,
                                             s.attitude, s.context, deadline);
    const double total_ms = elapsedMs(full_start);
    int attempted = 0, feasible = 0, exhausted = 0;
    for (const auto& c : result.candidates)
    {
      attempted += c.root.status != CandidateStatus::kGenerationFailed &&
                   c.detail != "whole-body planning budget exhausted";
      feasible += c.status == CandidateStatus::kFeasible || c.status == CandidateStatus::kSelected;
      exhausted += c.detail == "whole-body planning budget exhausted";
    }
    if (!first) out << ',';
    first = false;
    out << "{\"name\":" << std::quoted(s.name) << ",\"batch_ms\":" << batch_ms
        << ",\"total_ms\":" << total_ms << ",\"selected\":" << result.selected
        << ",\"full_selected\":" << full.selected << ",\"attempted\":" << attempted
        << ",\"feasible\":" << feasible << ",\"exhausted\":" << exhausted
        << ",\"snapshot\":";
    test::PlanSnapshot(result).write(out);
    out << '}';
  }
  out << "]}\n";
  return 0;
}
