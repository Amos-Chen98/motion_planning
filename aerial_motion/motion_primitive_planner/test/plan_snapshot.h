#ifndef MOTION_PRIMITIVE_PLANNER_TEST_PLAN_SNAPSHOT_H
#define MOTION_PRIMITIVE_PLANNER_TEST_PLAN_SNAPSHOT_H

#include <motion_primitive_planner/whole_body_planner.h>
#include <iomanip>
#include <ostream>

namespace motion_primitive_planner
{
namespace test
{
//! Lossless, ordered snapshot of every result field, also usable with the
//! pre-parallelism headers/library. Shape entries delimit variable arrays.
struct PlanSnapshot
{
  std::vector<double> values;
  std::vector<std::string> details;

  template <typename Derived>
  void matrix(const Eigen::MatrixBase<Derived>& value)
  {
    values.push_back(value.rows());
    values.push_back(value.cols());
    for (int row = 0; row < value.rows(); ++row)
      for (int col = 0; col < value.cols(); ++col) values.push_back(value(row, col));
  }

  void trajectory(const Trajectory<5>& value)
  {
    values.push_back(value.getPieceNum());
    for (int piece = 0; piece < value.getPieceNum(); ++piece)
    {
      values.push_back(value[piece].getDuration());
      matrix(value[piece].getCoeffMat());
    }
  }

  explicit PlanSnapshot(const WholeBodyPlanResult& result)
  {
    values = {static_cast<double>(result.selected), static_cast<double>(result.candidates.size())};
    for (const auto& candidate : result.candidates)
    {
      const auto& joints = candidate.joints;
      details.insert(details.end(), {candidate.detail, candidate.root.detail, joints.detail});
      values.insert(values.end(), {static_cast<double>(candidate.status),
          static_cast<double>(candidate.root.status), candidate.root.jerk_energy,
          candidate.root.path_length, static_cast<double>(joints.success), joints.duration,
          joints.time_scale, joints.root_translation_delay, joints.minimum_fc_rp,
          joints.joint_motion, joints.tracking_error_rms, joints.tracking_error_max});
      trajectory(candidate.root.trajectory);
      trajectory(candidate.scaled_root);
      values.push_back(joints.joint_waypoints.size());
      for (const auto& point : joints.joint_waypoints)
      {
        values.push_back(point.time);
        matrix(point.positions);
      }
      values.push_back(joints.attitude_waypoints.size());
      for (const auto& point : joints.attitude_waypoints)
        values.insert(values.end(), {point.time, point.attitude.yaw, point.attitude.pitch});
    }
  }

  void write(std::ostream& out) const
  {
    out << std::setprecision(17) << "{\"values\":[";
    for (size_t i = 0; i < values.size(); ++i)
    {
      if (i) out << ',';
      if (std::isfinite(values[i])) out << values[i];
      else out << std::quoted(std::isnan(values[i]) ? "nan" : (values[i] > 0 ? "inf" : "-inf"));
    }
    out << "],\"details\":[";
    for (size_t i = 0; i < details.size(); ++i)
    {
      if (i) out << ',';
      out << std::quoted(details[i]);
    }
    out << "]}";
  }
};
}  // namespace test
}  // namespace motion_primitive_planner

#endif
