#include <motion_primitive_planner/whole_body_planner.h>
#include "candidate_executor.h"
#include "trajectory_collision.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <set>
#ifdef __linux__
#include <sched.h>
#endif

namespace motion_primitive_planner
{

namespace
{
constexpr double kEpsilon = 1e-6;

size_t availableCpus()
{
#ifdef __linux__
  cpu_set_t cpus;
  CPU_ZERO(&cpus);
  if (sched_getaffinity(0, sizeof(cpus), &cpus) == 0 && CPU_COUNT(&cpus) > 0)
    return static_cast<size_t>(CPU_COUNT(&cpus));
#endif
  return std::max(1u, std::thread::hardware_concurrency());
}
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
    const WholeBodyPlannerConfig& config,
    const std::vector<std::shared_ptr<multilink_copilot::StabilityEvaluator>>& evaluators)
  : config_(config)
{
  if (config_.planning_threads < 0 || evaluators.empty() ||
      evaluators.size() != static_cast<size_t>(config_.shared.primitive.candidate_count))
  {
    throw std::invalid_argument("Each root candidate requires an independent stability evaluator");
  }
  std::set<const void*> models;
  for (size_t index = 0; index < evaluators.size(); ++index)
  {
    if (!evaluators[index] || !evaluators[index]->robotModel() ||
        !models.insert(evaluators[index]->robotModel().get()).second)
      throw std::invalid_argument("Each root candidate requires an independent robot model");
    JointPlannerConfig joint_config = config_.joint;
    joint_config.random_seed += static_cast<unsigned int>(index);
    joint_planners_.emplace_back(new JointTrajectoryPlanner(joint_config, evaluators[index]));
  }
  const auto collision_model = std::make_shared<DragonCollisionModel>(*evaluators.front()->robotModel());
  for (size_t index = 0; index < evaluators.size(); ++index)
    collision_checkers_.emplace_back(new DragonCollisionChecker(collision_model));
  const size_t threads = std::min(evaluators.size(), config_.planning_threads == 0
      ? availableCpus() : static_cast<size_t>(config_.planning_threads));
  executor_.reset(new CandidateExecutor(threads));
}

WholeBodyPlanner::~WholeBodyPlanner() = default;

WholeBodyPlanResult WholeBodyPlanner::plan(
    const PrimitiveBatch& batch,
    const std::shared_ptr<const PlanningSceneSnapshot>& scene,
    const Eigen::VectorXd& start_joints, const RootAttitude& start_attitude,
    const NominalJointContext& nominal_context, const ros::Time& deadline)
{
  std::lock_guard<std::mutex> plan_lock(plan_mutex_);
  if (!scene || !scene->route || !scene->collision || batch.candidates.size() > joint_planners_.size())
  {
    throw std::invalid_argument("Invalid whole-body candidate batch or scene snapshot");
  }
  WholeBodyPlanResult result;
  std::vector<WholeBodyCandidate>& candidates = result.candidates;
  candidates.resize(batch.candidates.size());
  executor_->run(batch.candidates.size(), [&](size_t index) {
    WholeBodyCandidate& candidate = candidates[index];
    candidate.root = batch.candidates[index];
    if (candidate.root.status == CandidateStatus::kGenerationFailed)
    {
      candidate.detail = candidate.root.detail;
      return;
    }
    const double joint_planning_budget = (deadline - ros::Time::now()).toSec();
    if (joint_planning_budget <= 0.0)
    {
      candidate.status = CandidateStatus::kJointPlanningFailed;
      candidate.detail = "whole-body planning budget exhausted";
      return;
    }
    candidate.joints = joint_planners_[index]->plan(candidate.root.trajectory, nominal_context,
                                                    start_joints, start_attitude,
                                                    joint_planning_budget);
    if (!candidate.joints.success)
    {
      candidate.status = CandidateStatus::kJointPlanningFailed;
      candidate.detail = candidate.joints.detail;
      return;
    }
    candidate.scaled_root = gcopter_planner::PlannerBackend::timeScaledTrajectory(
        candidate.root.trajectory, candidate.joints.time_scale);
    if (trajectoryCollides(candidate.scaled_root, candidate.joints,
                           config_.joint.follower.command_hz,
                           *scene->collision, *collision_checkers_[index]))
    {
      candidate.status = CandidateStatus::kCollision;
      candidate.detail = "whole-body sampled collision";
      return;
    }
    candidate.status = CandidateStatus::kFeasible;
  });

  const int selected = selectBest(candidates);
  if (selected >= 0)
  {
    candidates[static_cast<size_t>(selected)].status = CandidateStatus::kSelected;
  }
  result.selected = selected;
  return result;
}

bool trajectoryCollides(
    const Trajectory<5>& root, const JointPlanResult& joints, double command_hz,
    const CollisionEnvironment& environment, DragonCollisionChecker& checker)
{
  const double duration = joints.duration;
  if (!std::isfinite(command_hz) || command_hz <= 0.0 || root.getPieceNum() == 0 ||
      !std::isfinite(root.getTotalDuration()) || root.getTotalDuration() < 0.0 ||
      !std::isfinite(joints.root_translation_delay) || joints.root_translation_delay < 0.0)
    return true;
  const double command_dt = 1.0 / command_hz;
  const auto collides_at_time = [&](double time) {
    WholeBodyConfiguration configuration;
    const double root_time = std::max(
        0.0, std::min(root.getTotalDuration(),
                      time - joints.root_translation_delay));
    configuration.link1_tail = root.getPos(root_time);
    configuration.root_link_rotation = joints.rootLinkRotation(time);
    configuration.joint_positions = joints.jointPositions(time);
    return checker.collides(configuration, environment);
  };

  if (!std::isfinite(duration) || duration < 0.0 ||
      duration / command_dt >= static_cast<double>(std::numeric_limits<int>::max()))
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
