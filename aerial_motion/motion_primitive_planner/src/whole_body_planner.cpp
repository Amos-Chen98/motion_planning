#include <motion_primitive_planner/whole_body_planner.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion_primitive_planner
{

namespace
{
constexpr double kEpsilon = 1e-6;
}  // namespace

int selectBestWholeBodyCandidate(const std::vector<WholeBodyCandidateScore>& candidates,
                                 double joint_motion_cost_weight,
                                 double tracking_error_cost_weight)
{
  if (!std::isfinite(joint_motion_cost_weight) || joint_motion_cost_weight < 0.0)
  {
    return -1;
  }
  if (!std::isfinite(tracking_error_cost_weight) || tracking_error_cost_weight < 0.0)
  {
    return -1;
  }

  int best = -1;
  for (size_t index = 0; index < candidates.size(); ++index)
  {
    const WholeBodyCandidateScore& candidate = candidates[index];
    if (!candidate.feasible || !std::isfinite(candidate.duration) ||
        !std::isfinite(candidate.joint_motion) ||
        !std::isfinite(candidate.tracking_error_rms) ||
        candidate.duration < 0.0 || candidate.joint_motion < 0.0 ||
        candidate.tracking_error_rms < 0.0)
    {
      continue;
    }
    if (best < 0)
    {
      best = static_cast<int>(index);
      continue;
    }
    const WholeBodyCandidateScore& current = candidates[static_cast<size_t>(best)];
    const double candidate_cost =
        candidate.duration + joint_motion_cost_weight * candidate.joint_motion +
        tracking_error_cost_weight * candidate.tracking_error_rms;
    const double current_cost =
        current.duration + joint_motion_cost_weight * current.joint_motion +
        tracking_error_cost_weight * current.tracking_error_rms;
    bool better = candidate_cost < current_cost - kEpsilon;
    if (!better && std::abs(candidate_cost - current_cost) <= kEpsilon)
    {
      better = candidate.tracking_error_rms < current.tracking_error_rms - kEpsilon;
      if (!better &&
          std::abs(candidate.tracking_error_rms - current.tracking_error_rms) <= kEpsilon)
      {
        better = candidate.joint_motion < current.joint_motion - kEpsilon;
        if (!better && std::abs(candidate.joint_motion - current.joint_motion) <= kEpsilon)
        {
          better = candidate.duration < current.duration - kEpsilon ||
                   (std::abs(candidate.duration - current.duration) <= kEpsilon &&
                    candidate.root_jerk < current.root_jerk);
        }
      }
    }
    if (better)
    {
      best = static_cast<int>(index);
    }
  }
  return best;
}

WholeBodyPlanner::WholeBodyPlanner(
    const WholeBodyPlannerConfig& config, const DragonCollisionGeometry& geometry,
    const std::vector<std::shared_ptr<multilink_copilot::StabilityEvaluator>>& evaluators)
  : config_(config), collision_geometry_(geometry)
{
  if (evaluators.size() != static_cast<size_t>(config_.shared.primitive.candidate_count))
  {
    throw std::invalid_argument("Each root candidate requires an independent stability evaluator");
  }
  for (size_t index = 0; index < evaluators.size(); ++index)
  {
    JointPlannerConfig joint_config = config_.joint;
    joint_config.random_seed += static_cast<unsigned int>(index);
    joint_planners_.emplace_back(new JointTrajectoryPlanner(joint_config, evaluators[index]));
  }
}

WholeBodyPlanResult WholeBodyPlanner::plan(
    const PrimitiveBatch& batch,
    const std::shared_ptr<const gcopter_planner::PlannerBackend>& occupancy,
    const Eigen::VectorXd& start_joints, const RootAttitude& start_attitude,
    const NominalJointContext& nominal_context, const ros::Time& deadline)
{
  if (!occupancy || batch.candidates.size() > joint_planners_.size())
  {
    throw std::invalid_argument("Invalid whole-body candidate batch or occupancy snapshot");
  }
  WholeBodyPlanResult result;
  std::vector<WholeBodyCandidate>& candidates = result.candidates;
  candidates.resize(batch.candidates.size());
  for (size_t index = 0; index < batch.candidates.size(); ++index)
  {
    WholeBodyCandidate& candidate = candidates[index];
    candidate.root = batch.candidates[index];
    if (candidate.root.status == CandidateStatus::kGenerationFailed)
    {
      candidate.detail = candidate.root.detail;
      continue;
    }
    const double joint_planning_budget = (deadline - ros::Time::now()).toSec();
    if (joint_planning_budget <= 0.0)
    {
      candidate.status = CandidateStatus::kJointPlanningFailed;
      candidate.detail = "whole-body planning budget exhausted";
      continue;
    }
    candidate.joints = joint_planners_[index]->plan(candidate.root.trajectory, nominal_context,
                                                    start_joints, start_attitude,
                                                    joint_planning_budget);
    if (!candidate.joints.success)
    {
      candidate.status = CandidateStatus::kJointPlanningFailed;
      candidate.detail = candidate.joints.detail;
      continue;
    }
    candidate.scaled_root = gcopter_planner::PlannerBackend::timeScaledTrajectory(
        candidate.root.trajectory, candidate.joints.time_scale);
    if (wholeBodyTrajectoryCollides(candidate.scaled_root, candidate.joints, occupancy))
    {
      candidate.status = CandidateStatus::kCollision;
      candidate.detail = "whole-body sampled collision";
      continue;
    }
    candidate.status = CandidateStatus::kFeasible;
  }

  const int selected = selectBest(candidates);
  if (selected >= 0)
  {
    candidates[static_cast<size_t>(selected)].status = CandidateStatus::kSelected;
  }
  result.selected = selected;
  return result;
}

bool WholeBodyPlanner::wholeBodyTrajectoryCollides(
    const Trajectory<5>& root, const JointPlanResult& joints,
    const std::shared_ptr<const gcopter_planner::PlannerBackend>& occupancy) const
{
  const double duration = joints.duration;
  const double command_dt = 1.0 / config_.joint.follower.command_hz;
  const double spatial_resolution = 0.5 * occupancy->voxelScale();
  const auto occupied = [&occupancy](const Eigen::Vector3d& point) {
    return occupancy->query(point);
  };

  const auto collides_at_time = [&](double time) {
    WholeBodyConfiguration configuration;
    const double root_time = std::max(
        0.0, std::min(root.getTotalDuration(),
                      time - joints.root_translation_delay));
    configuration.link1_tail = root.getPos(root_time);
    configuration.root_link_rotation = joints.rootLinkRotation(time);
    configuration.joint_positions = joints.jointPositions(time);
    return wholeBodyCollides(configuration, collision_geometry_, spatial_resolution, occupied);
  };

  if (!std::isfinite(duration) || duration < 0.0)
  {
    return true;
  }
  const int sample_count = duration > kEpsilon
                               ? std::max(1, static_cast<int>(std::ceil(duration / command_dt)))
                               : 0;
  for (int sample = 0; sample <= sample_count; ++sample)
  {
    const double time = sample_count > 0
                            ? duration * static_cast<double>(sample) / sample_count
                            : 0.0;
    if (collides_at_time(time))
    {
      return true;
    }
  }
  return false;
}

int WholeBodyPlanner::selectBest(const std::vector<WholeBodyCandidate>& candidates) const
{
  std::vector<WholeBodyCandidateScore> scores;
  scores.reserve(candidates.size());
  for (const WholeBodyCandidate& candidate : candidates)
  {
    scores.push_back({candidate.status == CandidateStatus::kFeasible,
                      candidate.joints.duration,
                      candidate.joints.joint_motion,
                      candidate.root.jerk_energy,
                      candidate.joints.tracking_error_rms});
  }
  return selectBestWholeBodyCandidate(scores, config_.joint_motion_cost_weight,
                                      config_.tracking_error_cost_weight);
}

}  // namespace motion_primitive_planner
