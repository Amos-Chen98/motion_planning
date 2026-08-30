#include <motion_primitive_planner/root_candidate_evaluator.h>

#include <Eigen/Geometry>
#include <kdl/frames.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motion_primitive_planner
{
namespace
{
constexpr double kEpsilon = 1e-6;
}

RootCandidateEvaluator::RootCandidateEvaluator(
    const FollowerConfig& follower_config, double prediction_dt,
    bool allow_stability_projection_fallback, const DragonModelInfo& model,
    const std::shared_ptr<multilink_copilot::StabilityEvaluator>& stability_evaluator,
    const PlanningEnvironment& environment)
  : follower_config_(follower_config)
  , prediction_dt_(prediction_dt)
  , allow_stability_projection_fallback_(allow_stability_projection_fallback)
  , collision_geometry_(model.collisionGeometry())
  , stability_evaluator_(stability_evaluator)
  , environment_(environment)
  , predictor_(follower_config)
{
  if (!stability_evaluator_ || !std::isfinite(prediction_dt_) || prediction_dt_ <= 0.0)
  {
    throw std::invalid_argument("Invalid root candidate evaluator configuration");
  }
}

bool isOnlyFcRpViolation(const multilink_copilot::StabilityMetrics& metrics,
                         const multilink_copilot::StabilityConfig& config)
{
  const double tolerance = config.feasibility_tolerance;
  const bool fc_rp_failed = std::isfinite(metrics.fc_rp_min) &&
                            metrics.fc_rp_min + tolerance < config.fc_rp_min_threshold;
  const bool other_metrics_finite = std::isfinite(metrics.fc_t_min) &&
                                    std::isfinite(metrics.static_thrust_min) &&
                                    std::isfinite(metrics.static_thrust_max) &&
                                    std::isfinite(metrics.overlap_clearance) &&
                                    std::isfinite(metrics.baselink_tilt);
  return fc_rp_failed && other_metrics_finite &&
         (!config.check_fc_t || metrics.fc_t_min + tolerance >= config.fc_t_min_threshold) &&
         metrics.static_thrust_min + tolerance >= config.static_thrust_min &&
         metrics.static_thrust_max - tolerance <= config.static_thrust_max &&
         metrics.overlap_clearance + tolerance >= config.overlap_min_clearance &&
         (config.max_baselink_tilt <= 0.0 ||
          metrics.baselink_tilt <= config.max_baselink_tilt + tolerance);
}

void RootCandidateEvaluator::evaluate(Candidate& candidate,
                                      const NominalJointContext& nominal_context,
                                      const Eigen::VectorXd& start_joints,
                                      double start_yaw)
{
  if (candidate.status == CandidateStatus::kGenerationFailed)
  {
    return;
  }
  const std::vector<NominalJointSample> samples = predictor_.predict(
      candidate.trajectory, nominal_context, start_joints, start_yaw,
      1.0 / follower_config_.command_hz);
  if (samples.empty())
  {
    candidate.status = CandidateStatus::kGenerationFailed;
    candidate.detail = "nominal joint prediction failed";
    return;
  }

  candidate.min_fc_rp = std::numeric_limits<double>::infinity();
  candidate.joint_motion = 0.0;
  candidate.requires_stability_projection = false;
  const std::shared_ptr<const gcopter_planner::PlannerBackend> occupancy =
      environment_.occupancySnapshot();
  const auto occupied = [&occupancy](const Eigen::Vector3d& point) {
    return occupancy->query(point);
  };
  double next_evaluation_time = 0.0;
  Eigen::VectorXd previous_joints = samples.front().joints;
  for (size_t index = 0; index < samples.size(); ++index)
  {
    const NominalJointSample& sample = samples[index];
    if (index > 0)
    {
      candidate.joint_motion += (sample.joints - previous_joints).norm();
      previous_joints = sample.joints;
    }

    WholeBodyConfiguration configuration;
    configuration.link1_tail = sample.root_position;
    configuration.root_link_rotation =
        Eigen::AngleAxisd(sample.yaw + M_PI,
                          Eigen::Vector3d::UnitZ()).toRotationMatrix();
    configuration.joint_positions = sample.joints;
    if (wholeBodyCollides(configuration, collision_geometry_,
                          0.5 * occupancy->voxelScale(), occupied))
    {
      candidate.status = CandidateStatus::kCollision;
      candidate.detail = "whole-body sampled collision";
      return;
    }

    const bool final_sample = index + 1 == samples.size();
    if (!sample.history_changed && sample.time + kEpsilon < next_evaluation_time && !final_sample)
    {
      continue;
    }
    while (next_evaluation_time <= sample.time + kEpsilon)
    {
      next_evaluation_time += prediction_dt_;
    }

    stability_evaluator_->setRootLinkRotation(KDL::Rotation::RotZ(sample.yaw + M_PI));
    multilink_copilot::StabilityMetrics metrics;
    if (!stability_evaluator_->evaluate(sample.joints, metrics))
    {
      candidate.status = CandidateStatus::kJointLimit;
      candidate.detail = "nominal joint limit";
      const std::vector<double>& lower =
          stability_evaluator_->robotModel()->getLinkJointLowerLimits();
      const std::vector<double>& upper =
          stability_evaluator_->robotModel()->getLinkJointUpperLimits();
      for (int joint = 0; joint < sample.joints.size(); ++joint)
      {
        if (joint >= static_cast<int>(lower.size()) || joint >= static_cast<int>(upper.size()) ||
            !std::isfinite(sample.joints(joint)) ||
            sample.joints(joint) < lower[static_cast<size_t>(joint)] ||
            sample.joints(joint) > upper[static_cast<size_t>(joint)])
        {
          candidate.detail += " at index " + std::to_string(joint) +
                              " (q=" + std::to_string(sample.joints(joint)) + ")";
          break;
        }
      }
      return;
    }
    candidate.min_fc_rp = std::min(candidate.min_fc_rp, metrics.fc_rp_min);
    if (metrics.safe)
    {
      continue;
    }
    if (allow_stability_projection_fallback_ &&
        isOnlyFcRpViolation(metrics, stability_evaluator_->config()))
    {
      candidate.requires_stability_projection = true;
      continue;
    }
    candidate.status = CandidateStatus::kStability;
    candidate.detail = stability_evaluator_->describeViolations(metrics);
    return;
  }

  if (candidate.requires_stability_projection)
  {
    candidate.status = CandidateStatus::kStabilityProjection;
    candidate.detail = "nominal fc_rp_min=" + std::to_string(candidate.min_fc_rp) +
                       "; downstream Copilot projection required";
  }
  else
  {
    candidate.status = CandidateStatus::kFeasible;
    candidate.detail.clear();
  }
}

}  // namespace motion_primitive_planner
