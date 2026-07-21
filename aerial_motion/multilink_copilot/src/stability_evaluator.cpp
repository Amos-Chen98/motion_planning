#include <multilink_copilot/stability_evaluator.h>

#include <qpOASES.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace multilink_copilot
{
namespace
{
using RowMajorMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

constexpr int kQpMaxWorkingSetRecalculations = 50;

std::vector<int> selectSmallestIndices(const Eigen::VectorXd& values, int count)
{
  const int clamped_count = std::max(0, std::min<int>(count, values.size()));
  std::vector<int> indices(values.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::partial_sort(indices.begin(), indices.begin() + clamped_count, indices.end(),
                    [&values](int lhs, int rhs) { return values(lhs) < values(rhs); });
  indices.resize(clamped_count);
  return indices;
}

Eigen::RowVectorXd extractLinkJointJacobianRow(const Eigen::MatrixXd& jacobian, int row, int joint_count)
{
  Eigen::RowVectorXd selected = Eigen::RowVectorXd::Zero(joint_count);
  if (row < 0 || row >= jacobian.rows())
  {
    return selected;
  }
  const int columns = std::max(0, std::min(joint_count, static_cast<int>(jacobian.cols()) - 6));
  if (columns > 0)
  {
    selected.head(columns) = jacobian.block(row, 6, 1, columns);
  }
  return selected;
}

Eigen::VectorXd buildLimitVector(const std::vector<double>& values, int size, double fallback)
{
  Eigen::VectorXd result = Eigen::VectorXd::Constant(size, fallback);
  for (int index = 0; index < std::min<int>(size, values.size()); ++index)
  {
    result(index) = values[static_cast<size_t>(index)];
  }
  return result;
}

bool hasNonEmptyIntersection(const Eigen::VectorXd& lower, const Eigen::VectorXd& upper, double tolerance)
{
  if (lower.size() != upper.size())
  {
    return false;
  }
  for (int index = 0; index < lower.size(); ++index)
  {
    if (lower(index) > upper(index) + tolerance)
    {
      return false;
    }
  }
  return true;
}

struct ConstraintBuilder
{
  ConstraintBuilder(int rows, int columns)
    : matrix(Eigen::MatrixXd::Zero(rows, columns))
    , lower(Eigen::VectorXd::Constant(rows, -qpOASES::INFTY))
    , upper(Eigen::VectorXd::Constant(rows, qpOASES::INFTY))
  {
  }

  void appendLower(const Eigen::RowVectorXd& jacobian, double bound)
  {
    matrix.row(row) = jacobian;
    lower(row) = bound;
    ++row;
  }

  void appendUpper(const Eigen::RowVectorXd& jacobian, double bound)
  {
    matrix.row(row) = jacobian;
    upper(row) = bound;
    ++row;
  }

  Eigen::MatrixXd matrix;
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  int row = 0;
};

void appendFcConstraints(const Eigen::VectorXd& distances,
                         const Eigen::MatrixXd& jacobian,
                         const std::vector<int>& active_indices,
                         int joint_count,
                         double threshold,
                         const Eigen::VectorXd& current,
                         ConstraintBuilder& builder)
{
  for (const int active_index : active_indices)
  {
    const Eigen::RowVectorXd row = extractLinkJointJacobianRow(jacobian, active_index, joint_count);
    builder.appendLower(row, threshold - distances(active_index) + row.dot(current));
  }
}

void appendThrustConstraints(const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& model,
                             int joint_count,
                             const StabilityConfig& config,
                             const Eigen::VectorXd& current,
                             ConstraintBuilder& builder)
{
  const Eigen::VectorXd& thrust = model->getStaticThrust();
  const Eigen::MatrixXd& jacobian = model->getLambdaJacobian();
  for (int index = 0; index < thrust.size(); ++index)
  {
    const Eigen::RowVectorXd row = extractLinkJointJacobianRow(jacobian, index, joint_count);
    builder.appendLower(row, config.static_thrust_min - thrust(index) + row.dot(current));
    builder.appendUpper(row, config.static_thrust_max - thrust(index) + row.dot(current));
  }
}

void appendOverlapConstraint(const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& model,
                             int joint_count,
                             const StabilityConfig& config,
                             const Eigen::VectorXd& current,
                             ConstraintBuilder& builder)
{
  const double clearance = model->getClosestRotorDist() - 2.0 * model->getEdfRadius();
  const Eigen::RowVectorXd row = extractLinkJointJacobianRow(model->getRotorOverlapJacobian(), 0, joint_count);
  builder.appendLower(row, config.overlap_min_clearance - clearance + row.dot(current));
}

double rotorOverlapClearance(const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& model)
{
  const std::vector<Eigen::Vector3d> edfs = model->getEdfsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> normals = model->getRotorsNormalFromCog<Eigen::Vector3d>();
  const int rotor_count = model->getRotorNum();
  if (edfs.size() < static_cast<size_t>(2 * rotor_count) ||
      normals.size() < static_cast<size_t>(rotor_count))
  {
    return -std::numeric_limits<double>::infinity();
  }
  double minimum_distance = std::numeric_limits<double>::infinity();
  for (int first = 0; first < 2 * rotor_count; ++first)
  {
    for (int second = first + 1; second < 2 * rotor_count; ++second)
    {
      if (first / 2 == second / 2)
      {
        continue;
      }
      const int high = edfs[static_cast<size_t>(first)].z() >= edfs[static_cast<size_t>(second)].z() ?
                           first : second;
      const int low = high == first ? second : first;
      const Eigen::Vector3d& upper = edfs[static_cast<size_t>(high)];
      const Eigen::Vector3d& lower = edfs[static_cast<size_t>(low)];
      const Eigen::Vector3d& normal = normals[static_cast<size_t>(high / 2)];
      if (std::abs(normal.z()) <= 1e-9)
      {
        return -std::numeric_limits<double>::infinity();
      }
      const Eigen::Vector3d projected = upper + (lower.z() - upper.z()) * normal / normal.z();
      const double distance = (projected - lower).norm() -
                              (upper.z() - lower.z()) * std::tan(model->getEdfMaxTilt());
      minimum_distance = std::min(minimum_distance, distance);
    }
  }
  return minimum_distance - 2.0 * model->getEdfRadius();
}
}  // namespace

StabilityEvaluator::StabilityEvaluator(const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& robot_model,
                                       const StabilityConfig& config)
  : robot_model_(robot_model), config_(config)
{
  if (!robot_model_)
  {
    throw std::invalid_argument("StabilityEvaluator requires a DRAGON robot model");
  }
  if (config_.qp_max_iterations <= 0 || config_.qp_joint_step_limit <= 0.0 ||
      config_.qp_regularization < 0.0 || config_.qp_convergence_tolerance <= 0.0 ||
      config_.feasibility_tolerance < 0.0 || config_.static_thrust_min > config_.static_thrust_max)
  {
    throw std::invalid_argument("Invalid stability evaluator configuration");
  }
  link_joint_indices_ = robot_model_->getLinkJointIndices();
  link_joint_num_ = static_cast<int>(link_joint_indices_.size());
  link_num_ = robot_model_->getRotorNum();
}

void StabilityEvaluator::setRootLinkRotation(const KDL::Rotation& root_link_rotation)
{
  root_link_rotation_ = root_link_rotation;
}

bool StabilityEvaluator::evaluate(const Eigen::VectorXd& joint_positions, StabilityMetrics& metrics)
{
  metrics = StabilityMetrics();
  if (!withinJointLimits(joint_positions))
  {
    return false;
  }

  const KDL::JntArray full_joints = buildFullJointPositions(joint_positions);
  const KDL::Frame root_to_baselink =
      robot_model_->forwardKinematics<KDL::Frame>(robot_model_->getBaselinkName(), full_joints);
  const KDL::Rotation world_baselink = root_link_rotation_ * root_to_baselink.M;
  robot_model_->setCogDesireOrientation(world_baselink);
  robot_model_->updateRobotModel(full_joints);
  if (!robot_model_->getExternalWrenchMap().empty())
  {
    robot_model_->updateJacobians(full_joints, false);
  }

  metrics.fc_rp_min = robot_model_->getFeasibleControlRollPitchMin();
  metrics.fc_t_min = robot_model_->getFeasibleControlTMin();
  const Eigen::VectorXd& thrust = robot_model_->getStaticThrust();
  if (thrust.size() > 0)
  {
    metrics.static_thrust_min = thrust.minCoeff();
    metrics.static_thrust_max = thrust.maxCoeff();
  }
  metrics.overlap_clearance = rotorOverlapClearance(robot_model_);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  world_baselink.GetRPY(roll, pitch, yaw);
  metrics.baselink_tilt = std::max(std::abs(roll), std::abs(pitch));
  metrics.safe = satisfiesSafeStability(metrics);
  return true;
}

bool StabilityEvaluator::satisfiesSafeStability(const StabilityMetrics& metrics) const
{
  const double tolerance = config_.feasibility_tolerance;
  if (!std::isfinite(metrics.fc_rp_min) || !std::isfinite(metrics.fc_t_min) ||
      !std::isfinite(metrics.static_thrust_min) || !std::isfinite(metrics.static_thrust_max) ||
      !std::isfinite(metrics.overlap_clearance) || !std::isfinite(metrics.baselink_tilt))
  {
    return false;
  }
  return metrics.fc_rp_min + tolerance >= config_.fc_rp_min_threshold &&
         (!config_.check_fc_t || metrics.fc_t_min + tolerance >= config_.fc_t_min_threshold) &&
         metrics.static_thrust_min + tolerance >= config_.static_thrust_min &&
         metrics.static_thrust_max - tolerance <= config_.static_thrust_max &&
         metrics.overlap_clearance + tolerance >= config_.overlap_min_clearance &&
         (config_.max_baselink_tilt <= 0.0 || metrics.baselink_tilt <= config_.max_baselink_tilt + tolerance);
}

std::string StabilityEvaluator::describeViolations(const StabilityMetrics& metrics) const
{
  std::ostringstream stream;
  const auto append = [&stream](const std::string& text) {
    if (stream.tellp() > 0)
    {
      stream << "; ";
    }
    stream << text;
  };
  const double tolerance = config_.feasibility_tolerance;
  if (metrics.fc_rp_min + tolerance < config_.fc_rp_min_threshold) append("fc_rp_min");
  if (config_.check_fc_t && metrics.fc_t_min + tolerance < config_.fc_t_min_threshold) append("fc_t_min");
  if (metrics.static_thrust_min + tolerance < config_.static_thrust_min) append("static_thrust_min");
  if (metrics.static_thrust_max - tolerance > config_.static_thrust_max) append("static_thrust_max");
  if (metrics.overlap_clearance + tolerance < config_.overlap_min_clearance) append("overlap_clearance");
  if (config_.max_baselink_tilt > 0.0 && metrics.baselink_tilt > config_.max_baselink_tilt + tolerance)
  {
    append("baselink_tilt");
  }
  return stream.tellp() > 0 ? stream.str() : std::string("none");
}

double StabilityEvaluator::violationScore(const StabilityMetrics& metrics) const
{
  const auto gap = [](double value) { return std::max(0.0, value); };
  const auto scale = [this](double value) {
    return std::max(std::abs(value), config_.feasibility_tolerance);
  };
  const double tolerance = config_.feasibility_tolerance;
  double score = gap(config_.fc_rp_min_threshold - metrics.fc_rp_min - tolerance) /
                 scale(config_.fc_rp_min_threshold);
  if (config_.check_fc_t)
  {
    score += gap(config_.fc_t_min_threshold - metrics.fc_t_min - tolerance) /
             scale(config_.fc_t_min_threshold);
  }
  score += gap(config_.static_thrust_min - metrics.static_thrust_min - tolerance) /
           scale(config_.static_thrust_min);
  score += gap(metrics.static_thrust_max - config_.static_thrust_max - tolerance) /
           scale(config_.static_thrust_max);
  score += gap(config_.overlap_min_clearance - metrics.overlap_clearance - tolerance) /
           scale(config_.overlap_min_clearance);
  if (config_.max_baselink_tilt > 0.0)
  {
    score += gap(metrics.baselink_tilt - config_.max_baselink_tilt - tolerance) /
             scale(config_.max_baselink_tilt);
  }
  return score;
}

bool StabilityEvaluator::projectToSafe(const Eigen::VectorXd& desired_joint_positions,
                                       const Eigen::VectorXd& reference_joint_positions,
                                       Eigen::VectorXd& stable_joint_positions,
                                       bool allow_unstable_seed)
{
  const Eigen::VectorXd desired = clampJointPositions(desired_joint_positions);
  if (link_joint_num_ <= 0)
  {
    stable_joint_positions = desired;
    StabilityMetrics metrics;
    return evaluate(stable_joint_positions, metrics) && metrics.safe;
  }

  Eigen::VectorXd current = clampJointPositions(reference_joint_positions);
  StabilityMetrics current_metrics;
  if (!evaluate(current, current_metrics) || (!allow_unstable_seed && !current_metrics.safe))
  {
    return false;
  }

  Eigen::VectorXd best = current;
  bool has_best = current_metrics.safe;
  double best_error = has_best ? (best - desired).norm() : std::numeric_limits<double>::infinity();
  const Eigen::VectorXd lower = buildLimitVector(robot_model_->getLinkJointLowerLimits(), link_joint_num_, -M_PI);
  const Eigen::VectorXd upper = buildLimitVector(robot_model_->getLinkJointUpperLimits(), link_joint_num_, M_PI);

  for (int iteration = 0; iteration < config_.qp_max_iterations; ++iteration)
  {
    StabilityMetrics iteration_metrics;
    if (!evaluate(current, iteration_metrics))
    {
      break;
    }
    const Eigen::VectorXd current_vector = current;
    const double error = (current_vector - desired).norm();
    if (iteration_metrics.safe && (!has_best || error < best_error))
    {
      best = current;
      best_error = error;
      has_best = true;
    }
    if (iteration_metrics.safe && error <= config_.qp_convergence_tolerance)
    {
      stable_joint_positions = current;
      return true;
    }

    if (robot_model_->getExternalWrenchMap().empty())
    {
      robot_model_->updateJacobians(buildFullJointPositions(current), false);
    }

    const std::vector<int> fc_rp_indices =
        selectSmallestIndices(robot_model_->getFeasibleControlRollPitchDists(), link_num_);
    const std::vector<int> fc_t_indices =
        config_.check_fc_t ? selectSmallestIndices(robot_model_->getFeasibleControlTDists(), link_num_) :
                             std::vector<int>();
    const int constraint_count = static_cast<int>(fc_rp_indices.size() + fc_t_indices.size()) +
                                 2 * robot_model_->getStaticThrust().size() + 1;
    ConstraintBuilder builder(constraint_count, link_joint_num_);
    appendFcConstraints(robot_model_->getFeasibleControlRollPitchDists(),
                        robot_model_->getFeasibleControlRollPitchDistsJacobian(), fc_rp_indices,
                        link_joint_num_, config_.fc_rp_min_threshold, current_vector, builder);
    if (config_.check_fc_t)
    {
      appendFcConstraints(robot_model_->getFeasibleControlTDists(),
                          robot_model_->getFeasibleControlTDistsJacobian(), fc_t_indices,
                          link_joint_num_, config_.fc_t_min_threshold, current_vector, builder);
    }
    appendThrustConstraints(robot_model_, link_joint_num_, config_, current_vector, builder);
    appendOverlapConstraint(robot_model_, link_joint_num_, config_, current_vector, builder);
    if (builder.row != constraint_count)
    {
      return false;
    }

    Eigen::VectorXd qp_lower = (current_vector.array() - config_.qp_joint_step_limit).matrix().cwiseMax(lower);
    Eigen::VectorXd qp_upper = (current_vector.array() + config_.qp_joint_step_limit).matrix().cwiseMin(upper);
    if (!hasNonEmptyIntersection(qp_lower, qp_upper, config_.feasibility_tolerance))
    {
      break;
    }

    Eigen::MatrixXd hessian =
        (1.0 + config_.qp_regularization) * Eigen::MatrixXd::Identity(link_joint_num_, link_joint_num_);
    const Eigen::VectorXd gradient = -(desired + config_.qp_regularization * current_vector);
    RowMajorMatrixXd hessian_row_major = hessian;
    RowMajorMatrixXd constraints_row_major = builder.matrix;
    qpOASES::SQProblem solver(link_joint_num_, constraint_count);
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;
    options.enableEqualities = qpOASES::BT_TRUE;
    solver.setOptions(options);
    int working_set_recalculations = kQpMaxWorkingSetRecalculations;
    const qpOASES::returnValue status =
        solver.init(hessian_row_major.data(), gradient.data(), constraints_row_major.data(), qp_lower.data(),
                    qp_upper.data(), builder.lower.data(), builder.upper.data(), working_set_recalculations);
    if (status != qpOASES::SUCCESSFUL_RETURN)
    {
      break;
    }
    Eigen::VectorXd solution = current_vector;
    solver.getPrimalSolution(solution.data());
    if (!solution.allFinite())
    {
      break;
    }
    current = clampJointPositions(solution);
    if ((current - current_vector).norm() <= config_.qp_convergence_tolerance)
    {
      StabilityMetrics final_metrics;
      if (evaluate(current, final_metrics) && final_metrics.safe)
      {
        stable_joint_positions = current;
        return true;
      }
      break;
    }
  }

  stable_joint_positions = best;
  StabilityMetrics best_metrics;
  return has_best && evaluate(best, best_metrics) && best_metrics.safe;
}

bool StabilityEvaluator::projectNearestSafe(const Eigen::VectorXd& desired_joint_positions,
                                            const std::vector<Eigen::VectorXd>& references,
                                            Eigen::VectorXd& stable_joint_positions)
{
  StabilityMetrics desired_metrics;
  if (evaluate(desired_joint_positions, desired_metrics) && desired_metrics.safe)
  {
    stable_joint_positions = desired_joint_positions;
    return true;
  }

  bool found = false;
  double best_distance = std::numeric_limits<double>::infinity();
  for (const Eigen::VectorXd& reference : references)
  {
    StabilityMetrics reference_metrics;
    const bool reference_safe = evaluate(clampJointPositions(reference), reference_metrics) && reference_metrics.safe;
    Eigen::VectorXd projected;
    if (!projectToSafe(desired_joint_positions, reference, projected, !reference_safe))
    {
      continue;
    }
    const double distance = (projected - desired_joint_positions).norm();
    if (!found || distance < best_distance)
    {
      stable_joint_positions = projected;
      best_distance = distance;
      found = true;
    }
  }
  return found;
}

bool StabilityEvaluator::withinJointLimits(const Eigen::VectorXd& joint_positions) const
{
  if (joint_positions.size() != link_joint_num_ || !joint_positions.allFinite())
  {
    return false;
  }
  const std::vector<double>& lower = robot_model_->getLinkJointLowerLimits();
  const std::vector<double>& upper = robot_model_->getLinkJointUpperLimits();
  if (lower.size() < static_cast<size_t>(link_joint_num_) || upper.size() < static_cast<size_t>(link_joint_num_))
  {
    return false;
  }
  for (int index = 0; index < link_joint_num_; ++index)
  {
    if (joint_positions(index) < lower[static_cast<size_t>(index)] - config_.feasibility_tolerance ||
        joint_positions(index) > upper[static_cast<size_t>(index)] + config_.feasibility_tolerance)
    {
      return false;
    }
  }
  return true;
}

Eigen::VectorXd StabilityEvaluator::clampJointPositions(const Eigen::VectorXd& joint_positions) const
{
  Eigen::VectorXd result = Eigen::VectorXd::Zero(link_joint_num_);
  const std::vector<double>& lower = robot_model_->getLinkJointLowerLimits();
  const std::vector<double>& upper = robot_model_->getLinkJointUpperLimits();
  for (int index = 0; index < std::min<int>(joint_positions.size(), link_joint_num_); ++index)
  {
    const double finite_value = std::isfinite(joint_positions(index)) ? joint_positions(index) : 0.0;
    if (index < static_cast<int>(lower.size()) && index < static_cast<int>(upper.size()))
    {
      result(index) = std::max(lower[static_cast<size_t>(index)],
                               std::min(finite_value, upper[static_cast<size_t>(index)]));
    }
    else
    {
      result(index) = finite_value;
    }
  }
  return result;
}

KDL::JntArray StabilityEvaluator::buildFullJointPositions(const Eigen::VectorXd& joint_positions) const
{
  KDL::JntArray result = robot_model_->getJointPositions();
  if (result.rows() != robot_model_->getTree().getNrOfJoints())
  {
    result.resize(robot_model_->getTree().getNrOfJoints());
  }
  for (int index = 0; index < std::min<int>(joint_positions.size(), link_joint_indices_.size()); ++index)
  {
    result(link_joint_indices_[static_cast<size_t>(index)]) = joint_positions(index);
  }
  return result;
}

void StabilityEvaluator::updateRobotModel(const Eigen::VectorXd& joint_positions)
{
  const KDL::JntArray full_joints = buildFullJointPositions(clampJointPositions(joint_positions));
  const KDL::Frame root_to_baselink =
      robot_model_->forwardKinematics<KDL::Frame>(robot_model_->getBaselinkName(), full_joints);
  robot_model_->setCogDesireOrientation(root_link_rotation_ * root_to_baselink.M);
  robot_model_->updateRobotModel(full_joints);
}

int StabilityEvaluator::jointCount() const
{
  return link_joint_num_;
}

const StabilityConfig& StabilityEvaluator::config() const
{
  return config_;
}

const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& StabilityEvaluator::robotModel() const
{
  return robot_model_;
}

}  // namespace multilink_copilot
