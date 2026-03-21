// copilot_stability_qp.cpp
// Yaw-only stability QP assembly and solve loop

#include <multilink_copilot/copilot.h>

#include <qpOASES.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

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

Eigen::RowVectorXd extractSelectedJacobianRow(const Eigen::MatrixXd& jacobian,
                                              int row,
                                              const std::vector<int>& yaw_joint_local_indices)
{
  Eigen::RowVectorXd selected = Eigen::RowVectorXd::Zero(yaw_joint_local_indices.size());
  if (row < 0 || row >= jacobian.rows())
  {
    return selected;
  }

  for (int i = 0; i < static_cast<int>(yaw_joint_local_indices.size()); ++i)
  {
    const int full_column = 6 + yaw_joint_local_indices.at(i);
    if (full_column < jacobian.cols())
    {
      selected(i) = jacobian(row, full_column);
    }
  }

  return selected;
}

bool hasNonEmptyIntersection(const Eigen::VectorXd& lower_bounds,
                             const Eigen::VectorXd& upper_bounds,
                             double tolerance)
{
  if (lower_bounds.size() != upper_bounds.size())
  {
    return false;
  }

  for (int i = 0; i < lower_bounds.size(); ++i)
  {
    if (lower_bounds(i) > upper_bounds(i) + tolerance)
    {
      return false;
    }
  }

  return true;
}

struct QpConstraintBuilder
{
  QpConstraintBuilder(int constraint_count, int variable_count)
    : A(Eigen::MatrixXd::Zero(constraint_count, variable_count))
    , lA(Eigen::VectorXd::Constant(constraint_count, -qpOASES::INFTY))
    , uA(Eigen::VectorXd::Constant(constraint_count, qpOASES::INFTY))
    , row(0)
  {
  }

  void appendLowerBound(const Eigen::RowVectorXd& jacobian_row, double lower_bound)
  {
    A.row(row) = jacobian_row;
    lA(row) = lower_bound;
    ++row;
  }

  void appendUpperBound(const Eigen::RowVectorXd& jacobian_row, double upper_bound)
  {
    A.row(row) = jacobian_row;
    uA(row) = upper_bound;
    ++row;
  }

  Eigen::MatrixXd A;
  Eigen::VectorXd lA;
  Eigen::VectorXd uA;
  int row;
};

Eigen::VectorXd buildYawLimitVector(const std::vector<double>& joint_limits,
                                    const std::vector<int>& yaw_joint_local_indices,
                                    double default_value)
{
  Eigen::VectorXd limit_vector = Eigen::VectorXd::Constant(yaw_joint_local_indices.size(), default_value);
  for (int i = 0; i < static_cast<int>(yaw_joint_local_indices.size()); ++i)
  {
    const int local_index = yaw_joint_local_indices.at(i);
    if (local_index < static_cast<int>(joint_limits.size()))
    {
      limit_vector(i) = joint_limits.at(local_index);
    }
  }

  return limit_vector;
}

void appendFcRpConstraints(const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& robot_model,
                           const std::vector<int>& active_indices,
                           const std::vector<int>& yaw_joint_local_indices,
                           double fc_rp_min_threshold,
                           const Eigen::VectorXd& current_yaw,
                           QpConstraintBuilder& builder)
{
  const Eigen::VectorXd& fc_rp_dists = robot_model->getFeasibleControlRollPitchDists();
  const Eigen::MatrixXd& fc_rp_jacobian = robot_model->getFeasibleControlRollPitchDistsJacobian();

  for (const int active_index : active_indices)
  {
    const Eigen::RowVectorXd jacobian_row =
        extractSelectedJacobianRow(fc_rp_jacobian, active_index, yaw_joint_local_indices);
    builder.appendLowerBound(jacobian_row,
                             fc_rp_min_threshold - fc_rp_dists(active_index) + jacobian_row.dot(current_yaw));
  }
}

void appendFcTConstraints(const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& robot_model,
                          const std::vector<int>& active_indices,
                          const std::vector<int>& yaw_joint_local_indices,
                          double fc_t_min_threshold,
                          const Eigen::VectorXd& current_yaw,
                          QpConstraintBuilder& builder)
{
  const Eigen::VectorXd& fc_t_dists = robot_model->getFeasibleControlTDists();
  const Eigen::MatrixXd& fc_t_jacobian = robot_model->getFeasibleControlTDistsJacobian();

  for (const int active_index : active_indices)
  {
    const Eigen::RowVectorXd jacobian_row =
        extractSelectedJacobianRow(fc_t_jacobian, active_index, yaw_joint_local_indices);
    builder.appendLowerBound(jacobian_row,
                             fc_t_min_threshold - fc_t_dists(active_index) + jacobian_row.dot(current_yaw));
  }
}

void appendStaticThrustConstraints(const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& robot_model,
                                   const std::vector<int>& yaw_joint_local_indices,
                                   double static_thrust_min,
                                   double static_thrust_max,
                                   const Eigen::VectorXd& current_yaw,
                                   QpConstraintBuilder& builder)
{
  const Eigen::VectorXd& static_thrust = robot_model->getStaticThrust();
  const Eigen::MatrixXd& lambda_jacobian = robot_model->getLambdaJacobian();

  for (int i = 0; i < static_thrust.size(); ++i)
  {
    const Eigen::RowVectorXd jacobian_row =
        extractSelectedJacobianRow(lambda_jacobian, i, yaw_joint_local_indices);
    builder.appendLowerBound(jacobian_row,
                             static_thrust_min - static_thrust(i) + jacobian_row.dot(current_yaw));
    builder.appendUpperBound(jacobian_row,
                             static_thrust_max - static_thrust(i) + jacobian_row.dot(current_yaw));
  }
}

void appendOverlapConstraint(const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& robot_model,
                             const std::vector<int>& yaw_joint_local_indices,
                             double minimum_overlap_clearance,
                             const Eigen::VectorXd& current_yaw,
                             QpConstraintBuilder& builder)
{
  const double overlap_clearance =
      robot_model->getClosestRotorDist() - 2.0 * robot_model->getEdfRadius();
  const Eigen::RowVectorXd overlap_jacobian =
      extractSelectedJacobianRow(robot_model->getRotorOverlapJacobian(), 0, yaw_joint_local_indices);
  builder.appendLowerBound(overlap_jacobian,
                           minimum_overlap_clearance - overlap_clearance + overlap_jacobian.dot(current_yaw));
}
}  // namespace

bool CopilotPlanner::solveStableYawQp(const Eigen::VectorXd& desired_joint_positions,
                                      const Eigen::VectorXd& reference_joint_positions,
                                      Eigen::VectorXd& stable_joint_positions)
{
  const Eigen::VectorXd desired_full = buildYawOnlyJointPositions(desired_joint_positions);
  const Eigen::VectorXd desired_yaw = extractYawJointPositions(desired_full);

  if (yaw_joint_local_indices_.empty())
  {
    stable_joint_positions = desired_full;
    return checkStability(stable_joint_positions, false);
  }

  Eigen::VectorXd current_joint_positions = buildYawOnlyJointPositions(reference_joint_positions);
  StabilityMetrics current_metrics;
  if (!evaluateStability(current_joint_positions, current_metrics) || !current_metrics.safe)
  {
    return false;
  }

  Eigen::VectorXd best_joint_positions = current_joint_positions;
  double best_yaw_error = (extractYawJointPositions(best_joint_positions) - desired_yaw).norm();

  const Eigen::VectorXd yaw_lower_bounds =
      buildYawLimitVector(dragon_robot_model_->getLinkJointLowerLimits(), yaw_joint_local_indices_, -M_PI);
  const Eigen::VectorXd yaw_upper_bounds =
      buildYawLimitVector(dragon_robot_model_->getLinkJointUpperLimits(), yaw_joint_local_indices_, M_PI);

  for (int iter = 0; iter < stability_qp_max_iterations_; ++iter)
  {
    StabilityMetrics iter_metrics;
    if (!evaluateStability(current_joint_positions, iter_metrics))
    {
      break;
    }

    const Eigen::VectorXd current_yaw = extractYawJointPositions(current_joint_positions);
    const double current_yaw_error = (current_yaw - desired_yaw).norm();
    if (iter_metrics.safe && current_yaw_error < best_yaw_error)
    {
      best_joint_positions = current_joint_positions;
      best_yaw_error = current_yaw_error;
    }

    if (iter_metrics.safe && current_yaw_error <= stability_qp_convergence_tol_)
    {
      stable_joint_positions = current_joint_positions;
      return true;
    }

    const auto active_fc_rp_indices =
        selectSmallestIndices(dragon_robot_model_->getFeasibleControlRollPitchDists(), link_num_);
    const auto active_fc_t_indices =
        stability_check_fc_t_
            ? selectSmallestIndices(dragon_robot_model_->getFeasibleControlTDists(), link_num_)
            : std::vector<int>();
    const int constraint_count = static_cast<int>(active_fc_rp_indices.size()) +
                                 static_cast<int>(active_fc_t_indices.size()) +
                                 dragon_robot_model_->getStaticThrust().size() * 2 + 1;

    QpConstraintBuilder builder(constraint_count, yaw_joint_local_indices_.size());
    appendFcRpConstraints(dragon_robot_model_, active_fc_rp_indices, yaw_joint_local_indices_,
                          stability_fc_rp_min_thre_, current_yaw, builder);
    if (stability_check_fc_t_)
    {
      appendFcTConstraints(dragon_robot_model_, active_fc_t_indices, yaw_joint_local_indices_,
                           stability_fc_t_min_thre_, current_yaw, builder);
    }
    appendStaticThrustConstraints(dragon_robot_model_, yaw_joint_local_indices_, stability_static_thrust_min_,
                                  stability_static_thrust_max_, current_yaw, builder);
    appendOverlapConstraint(dragon_robot_model_, yaw_joint_local_indices_, stability_overlap_min_clearance_,
                            current_yaw, builder);
    if (builder.row != constraint_count)
    {
      ROS_ERROR_STREAM("[CopilotPlanner] Stability QP constraint assembly mismatch: expected "
                       << constraint_count << " rows but built " << builder.row);
      return false;
    }

    Eigen::VectorXd qp_lower_bounds = current_yaw.array() - stability_qp_joint_step_limit_;
    Eigen::VectorXd qp_upper_bounds = current_yaw.array() + stability_qp_joint_step_limit_;
    qp_lower_bounds = qp_lower_bounds.cwiseMax(yaw_lower_bounds);
    qp_upper_bounds = qp_upper_bounds.cwiseMin(yaw_upper_bounds);

    if (!hasNonEmptyIntersection(qp_lower_bounds, qp_upper_bounds, stability_qp_feasibility_tol_))
    {
      break;
    }

    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(yaw_joint_local_indices_.size(), yaw_joint_local_indices_.size());
    H *= (1.0 + stability_qp_regularization_);
    const Eigen::VectorXd g = -(desired_yaw + stability_qp_regularization_ * current_yaw);

    RowMajorMatrixXd H_row_major = H;
    RowMajorMatrixXd A_row_major = builder.A;

    qpOASES::SQProblem solver(yaw_joint_local_indices_.size(), constraint_count);
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;
    options.enableEqualities = qpOASES::BT_TRUE;
    solver.setOptions(options);

    int n_wsr = kQpMaxWorkingSetRecalculations;
    const qpOASES::returnValue status =
        solver.init(H_row_major.data(), g.data(), A_row_major.data(), qp_lower_bounds.data(), qp_upper_bounds.data(),
                    builder.lA.data(), builder.uA.data(), n_wsr);
    if (status != qpOASES::SUCCESSFUL_RETURN)
    {
      ROS_WARN_THROTTLE(1.0, "[CopilotPlanner] Stability QP failed to find a feasible solution");
      break;
    }

    Eigen::VectorXd solved_yaw = current_yaw;
    solver.getPrimalSolution(solved_yaw.data());
    if (!solved_yaw.allFinite())
    {
      break;
    }

    current_joint_positions = composeYawOnlyJointPositions(solved_yaw);
    const double update_norm = (solved_yaw - current_yaw).norm();
    if (update_norm <= stability_qp_convergence_tol_)
    {
      StabilityMetrics next_metrics;
      if (evaluateStability(current_joint_positions, next_metrics) && next_metrics.safe)
      {
        stable_joint_positions = current_joint_positions;
        return true;
      }
      break;
    }
  }

  stable_joint_positions = best_joint_positions;
  return checkStability(stable_joint_positions, false);
}

}  // namespace multilink_copilot
