#ifndef MULTILINK_COPILOT_JOINT_SEED_SELECTION_H
#define MULTILINK_COPILOT_JOINT_SEED_SELECTION_H

#include <multilink_copilot/stability_evaluator.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace multilink_copilot
{

enum class JointSeedSource
{
  kLatestPublished,
  kCurrentMeasured,
  kStableHistory,
  kDesiredSeed,
  kProjected,
  kRepair
};

struct JointSeedEvaluation
{
  StabilityMetrics metrics;
  bool valid = false;
  double violation_score = std::numeric_limits<double>::infinity();

  bool isValid() const
  {
    return valid && std::isfinite(metrics.fc_rp_min) && std::isfinite(metrics.fc_t_min) &&
           std::isfinite(metrics.static_thrust_min) && std::isfinite(metrics.static_thrust_max) &&
           std::isfinite(metrics.overlap_clearance) && std::isfinite(metrics.baselink_tilt);
  }
};

struct JointSeedCandidate
{
  Eigen::VectorXd joint_positions;
  JointSeedSource source = JointSeedSource::kCurrentMeasured;
  JointSeedEvaluation evaluation;
  double desired_distance = 0.0;
  double measured_distance = 0.0;
};

struct JointSeedSelection
{
  std::vector<JointSeedCandidate> candidates;
  std::optional<size_t> reference_index;
};

inline int jointRepairSeedPriority(JointSeedSource source)
{
  switch (source)
  {
    case JointSeedSource::kLatestPublished: return 0;
    case JointSeedSource::kCurrentMeasured: return 1;
    case JointSeedSource::kDesiredSeed: return 2;
    case JointSeedSource::kStableHistory: return 3;
    default: return 4;
  }
}

inline bool sameJointSeed(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs)
{
  return lhs.size() == rhs.size() && (lhs.array() == rhs.array()).all();
}

// Inputs are joint-limit-clamped configurations. The evaluation cache belongs
// only to this call: historical feasibility is never reused across cycles.
inline JointSeedSelection selectJointSeed(
    const Eigen::VectorXd& desired, const Eigen::VectorXd& measured,
    const Eigen::VectorXd* latest_published, const std::deque<Eigen::VectorXd>& history,
    const JointSeedEvaluation& desired_evaluation,
    const std::function<JointSeedEvaluation(const Eigen::VectorXd&)>& evaluate)
{
  JointSeedSelection selection;
  const auto append = [&](const Eigen::VectorXd& joints, JointSeedSource source) -> std::optional<size_t> {
    if (joints.size() != measured.size() || !joints.allFinite())
      return std::nullopt;

    for (size_t index = 0; index < selection.candidates.size(); ++index)
    {
      JointSeedCandidate& existing = selection.candidates[index];
      if (sameJointSeed(existing.joint_positions, joints))
      {
        // The nominal command is added only for repair. If it was already
        // visited as history, retain its higher repair-source priority.
        if (jointRepairSeedPriority(source) < jointRepairSeedPriority(existing.source))
          existing.source = source;
        return index;
      }
    }

    JointSeedCandidate candidate;
    candidate.joint_positions = joints;
    candidate.source = source;
    candidate.desired_distance = (joints - desired).norm();
    candidate.measured_distance = (joints - measured).norm();
    candidate.evaluation = sameJointSeed(joints, desired) ? desired_evaluation : evaluate(joints);
    selection.candidates.push_back(std::move(candidate));
    return selection.candidates.size() - 1;
  };
  const auto try_reference = [&](const Eigen::VectorXd& joints, JointSeedSource source) {
    const std::optional<size_t> index = append(joints, source);
    if (index && selection.candidates[*index].evaluation.isValid() &&
        selection.candidates[*index].evaluation.metrics.safe)
    {
      selection.reference_index = index;
      return true;
    }
    return false;
  };

  if (latest_published && try_reference(*latest_published, JointSeedSource::kLatestPublished))
    return selection;
  if (try_reference(measured, JointSeedSource::kCurrentMeasured))
    return selection;

  struct RankedHistory
  {
    const Eigen::VectorXd* joints;
    double squared_distance;
  };
  std::vector<RankedHistory> ranked_history;
  ranked_history.reserve(history.size());
  // Reverse insertion order and stable sorting make exact distance ties prefer
  // newer records, including when ROS timestamps would have been identical.
  for (auto entry = history.rbegin(); entry != history.rend(); ++entry)
  {
    if (entry->size() != measured.size() || !entry->allFinite())
      continue;
    const double squared_distance = (*entry - measured).squaredNorm();
    if (std::isfinite(squared_distance))
      ranked_history.push_back({&*entry, squared_distance});
  }
  std::stable_sort(ranked_history.begin(), ranked_history.end(),
                   [](const RankedHistory& lhs, const RankedHistory& rhs) {
                     return lhs.squared_distance < rhs.squared_distance;
                   });
  for (const RankedHistory& entry : ranked_history)
  {
    if (try_reference(*entry.joints, JointSeedSource::kStableHistory))
      return selection;
  }

  append(desired, JointSeedSource::kDesiredSeed);
  return selection;
}

inline bool repairJointSeeds(
    const Eigen::VectorXd& desired, const std::vector<JointSeedCandidate>& candidates, int max_repairs,
    const std::function<bool(const Eigen::VectorXd&, Eigen::VectorXd&)>& project,
    Eigen::VectorXd& repaired)
{
  if (max_repairs <= 0)
    return false;

  std::vector<const JointSeedCandidate*> ranked;
  for (const JointSeedCandidate& candidate : candidates)
  {
    if (candidate.evaluation.isValid() && !candidate.evaluation.metrics.safe &&
        std::isfinite(candidate.evaluation.violation_score) &&
        std::isfinite(candidate.desired_distance) && std::isfinite(candidate.measured_distance))
      ranked.push_back(&candidate);
  }
  std::stable_sort(ranked.begin(), ranked.end(),
                   [](const JointSeedCandidate* lhs, const JointSeedCandidate* rhs) {
                     if (lhs->evaluation.violation_score != rhs->evaluation.violation_score)
                       return lhs->evaluation.violation_score < rhs->evaluation.violation_score;
                     if (jointRepairSeedPriority(lhs->source) != jointRepairSeedPriority(rhs->source))
                       return jointRepairSeedPriority(lhs->source) < jointRepairSeedPriority(rhs->source);
                     if (lhs->desired_distance != rhs->desired_distance)
                       return lhs->desired_distance < rhs->desired_distance;
                     return lhs->measured_distance < rhs->measured_distance;
                   });

  bool found = false;
  double best_distance = std::numeric_limits<double>::infinity();
  const size_t attempts = std::min(static_cast<size_t>(max_repairs), ranked.size());
  for (size_t index = 0; index < attempts; ++index)
  {
    Eigen::VectorXd projected;
    if (!project(ranked[index]->joint_positions, projected) ||
        projected.size() != desired.size() || !projected.allFinite())
      continue;
    const double distance = (projected - desired).norm();
    if (!found || distance < best_distance)
    {
      repaired = projected;
      best_distance = distance;
      found = true;
    }
  }
  return found;
}

inline void rememberJointSeed(std::deque<Eigen::VectorXd>& history, const Eigen::VectorXd& joints,
                               size_t capacity, double merge_tolerance)
{
  if (capacity == 0)
  {
    history.clear();
    return;
  }
  if (!history.empty() && history.back().size() == joints.size() &&
      (history.back() - joints).norm() <= merge_tolerance)
    history.back() = joints;
  else
    history.push_back(joints);
  while (history.size() > capacity)
    history.pop_front();
}

}  // namespace multilink_copilot

#endif  // MULTILINK_COPILOT_JOINT_SEED_SELECTION_H
