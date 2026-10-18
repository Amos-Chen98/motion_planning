#include <multilink_copilot/copilot.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <filesystem>
#include "benchmark_visualization.h"
#include "benchmark_reference.h"

namespace
{
bool finiteMetrics(const multilink_copilot::StabilityMetrics& m)
{
  return std::isfinite(m.fc_rp_min) && std::isfinite(m.fc_t_min) &&
         std::isfinite(m.static_thrust_min) && std::isfinite(m.static_thrust_max) &&
         std::isfinite(m.overlap_clearance) && std::isfinite(m.baselink_tilt);
}

Eigen::VectorXd initialState(multilink_copilot::CopilotPlanner& planner, const tf::Quaternion& rotation,
                             multilink_copilot::StabilityMetrics& metrics, std::ostream& audit)
{
  auto& evaluator = planner.stabilityEvaluator();
  evaluator.setRootLinkRotation(KDL::Rotation::Quaternion(rotation.x(), rotation.y(), rotation.z(), rotation.w()));
  const auto& names = planner.jointNames();
  audit << "mode,block_size,amplitude,sign,valid,safe,fc_rp,thrust_min,thrust_max,clearance,tilt\n";
  for (int mode = 0; mode < 2; ++mode)
    for (int block_size = 1; block_size < planner.linkCount(); ++block_size)
    for (int amplitude = 1; amplitude <= 15; ++amplitude)
      for (int sign : {1, -1})
      {
        Eigen::VectorXd q = Eigen::VectorXd::Zero(names.size());
        for (int segment = 1; segment < planner.linkCount(); ++segment)
        {
          const std::string name = "joint" + std::to_string(segment) + "_yaw";
          const auto iter = std::find(names.begin(), names.end(), name);
          if (iter == names.end()) throw std::runtime_error("Incomplete yaw joint map");
          q(std::distance(names.begin(), iter)) = sign * (((segment - 1) / block_size) % 2 ? -1 : 1) * amplitude * 0.1;
        }
        if (mode == 1)
        {
          Eigen::VectorXd projected;
          if (!evaluator.projectToSafe(q, q, projected, true)) continue;
          q = projected;
        }
        const bool valid = evaluator.evaluate(q, metrics);
        audit << mode << ',' << block_size << ',' << amplitude << ',' << sign << ',' << valid << ',' << metrics.safe << ','
              << metrics.fc_rp_min << ',' << metrics.static_thrust_min << ',' << metrics.static_thrust_max
              << ',' << metrics.overlap_clearance << ',' << metrics.baselink_tilt << '\n';
        if (valid && finiteMetrics(metrics) && metrics.safe) return q;
      }
  throw std::runtime_error("No feasible alternating-yaw initialization found under unchanged constraints");
}

void writeArray(std::ostream& out, const Eigen::VectorXd& q)
{
  out << '[';
  for (int i = 0; i < q.size(); ++i) { if (i) out << ','; out << q(i); }
  out << ']';
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "computation_benchmark");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
  ros::console::notifyLoggerLevelsChanged();
  try
  {
    ros::NodeHandle pnh("~");
    std::string output_dir, reference_file, waypoint_file;
    int link_count;
    double dt, playback_speed, start_delay;
    bool visualize, keep_open;
    pnh.param("visualize", visualize, false);
    pnh.param("keep_open", keep_open, true);
    pnh.param("playback_speed", playback_speed, 1.0);
    pnh.param("start_delay", start_delay, 3.0);
    pnh.param<std::string>("output_dir", output_dir, "");
    pnh.param("link_count", link_count, 4);
    pnh.param<std::string>("reference_file", reference_file, "");
    pnh.param<std::string>("waypoint_file", waypoint_file, "");
    pnh.param("dt", dt, 0.025);
    if (output_dir.empty() || reference_file.empty() ||
        (link_count < 4 || link_count > 8) || !std::isfinite(dt) || dt <= 0 ||
        !std::isfinite(playback_speed) || playback_speed <= 0 || !std::isfinite(start_delay) || start_delay < 0)
      throw std::invalid_argument("Invalid benchmark parameters");
    std::filesystem::create_directories(output_dir);
    if (std::filesystem::exists(output_dir + "/samples.csv") ||
        std::filesystem::exists(output_dir + "/initialization.json"))
      throw std::runtime_error("Output directory already contains a trial; choose a new output_dir");
    const BenchmarkReference input = readBenchmarkReference(reference_file, dt);
    std::ofstream run_settings(output_dir + "/run_settings.json");
    run_settings << "{\"visualize\":" << (visualize ? "true" : "false")
                 << ",\"link_count\":" << link_count << ",\"trajectory\":\"scenario\""
                 << ",\"playback_speed\":" << playback_speed
                 << ",\"expected_samples\":" << input.poses.size()
                 << ",\"dt\":" << dt
                 << ",\"duration_seconds\":" << input.times.back() << "}\n";
    run_settings.close();
    if (!run_settings) throw std::runtime_error("Cannot write run settings");
    multilink_copilot::CopilotPlanner planner(false);
    if (planner.linkCount() != link_count || planner.jointNames().size() != size_t(2 * (link_count - 1)))
      throw std::runtime_error("Unexpected robot dimensions");
    for (int segment = 1; segment < link_count; ++segment)
      for (const std::string suffix : {"_yaw", "_pitch"})
        if (std::count(planner.jointNames().begin(), planner.jointNames().end(),
                       "joint" + std::to_string(segment) + suffix) != 1)
          throw std::runtime_error("Invalid joint name mapping");
    const auto first = input.poses.front();
    multilink_copilot::StabilityMetrics initial_metrics;
    std::ostringstream initialization_audit;
    Eigen::VectorXd measured = initialState(planner, benchmarkLinkRotation(first.pose.orientation), initial_metrics, initialization_audit);
    std::ofstream initialization(output_dir + "/initialization.json");
    initialization << std::setprecision(17) << "{\"link_count\":" << link_count << ",\"joints\":";
    writeArray(initialization, measured);
    initialization << ",\"fc_rp_min\":" << initial_metrics.fc_rp_min
                   << ",\"thrust_min\":" << initial_metrics.static_thrust_min
                   << ",\"thrust_max\":" << initial_metrics.static_thrust_max
                   << ",\"clearance\":" << initial_metrics.overlap_clearance
                   << ",\"baselink_tilt\":" << initial_metrics.baselink_tilt << "}\n";
    initialization.close();
    std::ofstream csv(output_dir + "/samples.csv");
    if (!csv || !initialization) throw std::runtime_error("Cannot write benchmark output");
    csv << "step,time_s,geometry_warmup,total_ms,feasibility_ms,planning_ms,projection_calls,qp_solves,projection_iterations,status,source,fallback,published,output_safe\n";
    // Buffer results until timing is complete. Validation runs after each timed step.
    struct Sample { multilink_copilot::CopilotPlanner::UpdateResult result; bool safe; };
    std::vector<Sample> samples;
    samples.reserve(input.poses.size());
    std::unique_ptr<BenchmarkVisualization> viewer;
    geometry_msgs::Pose accepted_root = first.pose;
    const tf::Quaternion initial_rotation = benchmarkLinkRotation(first.pose.orientation);
    tf::quaternionTFToMsg(initial_rotation, accepted_root.orientation);
    const tf::Vector3 offset = tf::quatRotate(initial_rotation,
        tf::Vector3(planner.stabilityEvaluator().robotModel()->getLinkLength(), 0, 0));
    accepted_root.position.x -= offset.x();
    accepted_root.position.y -= offset.y();
    accepted_root.position.z -= offset.z();
    if (visualize)
    {
      viewer.reset(new BenchmarkVisualization());
      viewer->setReference(input.poses);
      if (!waypoint_file.empty()) viewer->setWaypoints(readBenchmarkReference(waypoint_file, 1.0).poses);
      std::cout << "Visualization enabled; results: " << output_dir << std::endl;
      multilink_copilot::CopilotPlanner::UpdateResult initial;
      initial.status = "initialization";
      const auto until = ros::WallTime::now() + ros::WallDuration(start_delay);
      while (ros::ok() && ros::WallTime::now() < until)
      {
        viewer->publish(planner, measured, accepted_root, first, initial, 0);
        ros::WallDuration(0.05).sleep();
      }
    }
    ros::WallRate playback_rate(playback_speed / dt);
    std::string failure;
    try
    {
      for (size_t i = 0; i < input.poses.size() && ros::ok(); ++i)
      {
        const auto& pose = input.poses[i];
        // Keep every completed update even if its audit or visualization fails.
        samples.push_back({planner.step(pose, measured), false});
        auto& sample = samples.back();
        const auto& result = sample.result;
        if (result.published())
        {
          measured = Eigen::Map<const Eigen::VectorXd>(result.target.joint_state.position.data(),
                                                      result.target.joint_state.position.size());
          multilink_copilot::StabilityMetrics metrics;
          auto& evaluator = planner.stabilityEvaluator();
          const auto rotation = benchmarkLinkRotation(pose.pose.orientation);
          evaluator.setRootLinkRotation(KDL::Rotation::Quaternion(rotation.x(), rotation.y(), rotation.z(), rotation.w()));
          sample.safe = evaluator.evaluate(measured, metrics) && finiteMetrics(metrics) && metrics.safe;
          if (!sample.safe) throw std::runtime_error("Published command failed independent feasibility audit");
          accepted_root = result.target.root_state.pose.pose;
        }
        if (viewer)
        {
          viewer->publish(planner, measured, accepted_root, pose, result, input.times[i]);
          playback_rate.sleep();
        }
      }
    }
    catch (const std::exception& error) { failure = error.what(); }
    csv << std::setprecision(12);
    for (size_t i = 0; i < samples.size(); ++i)
    {
      const auto& r = samples[i].result;
      csv << i << ',' << input.times[i] << ',' << r.geometry_warmup << ','
          << r.total_ms << ',' << r.feasibility_ms << ',' << r.planning_ms << ',' << r.projection_calls << ','
          << r.qp_solves << ',' << r.projection_iterations << ',' << r.status << ',' << r.source << ','
          << r.fallback << ',' << r.published() << ',' << samples[i].safe << '\n';
    }
    csv.close();
    if (!csv) throw std::runtime_error("Failed to flush samples");
    const bool completed = failure.empty() && samples.size() == input.poses.size();
    std::cout << (completed ? "Completed " : "Interrupted ") << link_count << " links / scenario: " << samples.size() << " updates" << std::endl;
    if (!failure.empty()) throw std::runtime_error(failure);
    if (viewer && keep_open && completed)
    {
      std::cout << "Saved results. RViz remains open; Ctrl+C to close." << std::endl;
      ros::WallRate hold_rate(20);
      while (ros::ok())
      {
        viewer->publish(planner, measured, accepted_root, input.poses[samples.size() - 1],
                        samples.back().result, input.times[samples.size() - 1], true);
        hold_rate.sleep();
      }
    }
    return completed ? 0 : 130;
  }
  catch (const std::exception& error)
  {
    std::cerr << "Computation benchmark failed: " << error.what() << '\n';
    return 1;
  }
}
