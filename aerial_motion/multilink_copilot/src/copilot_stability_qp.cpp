// copilot_stability_qp.cpp
// Joint-space stability QP assembly and solve loop

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

Eigen::RowVectorXd extractLinkJointJacobianRow(const Eigen::MatrixXd& jacobian, int row, int link_joint_num)
{
  Eigen::RowVectorXd selected = Eigen::RowVectorXd::Zero(link_joint_num);
  if (row < 0 || row >= jacobian.rows())
  {
    return selected;
  }

  const int available_columns = std::max(0, std::min(link_joint_num, static_cast<int>(jacobian.cols()) - 6));
  if (available_columns > 0)
  {
    selected.head(available_columns) = jacobian.block(row, 6, 1, available_columns);
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

Eigen::VectorXd buildJointLimitVector(const std::vector<double>& joint_limits, int joint_count, double default_value)
{
  Eigen::VectorXd limit_vector = Eigen::VectorXd::Constant(joint_count, default_value);
  const int available_count = std::min<int>(joint_count, joint_limits.size());
  for (int i = 0; i < available_count; ++i)
  {
    limit_vector(i) = joint_limits.at(i);
  }

  return limit_vector;
}

void appendFcRpConstraints(const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& robot_model,
                           const std::vector<int>& active_indices,
                           int link_joint_num,
                           double fc_rp_min_threshold,
                           const Eigen::VectorXd& current_joint_positions,
                           QpConstraintBuilder& builder)
{
  const Eigen::VectorXd& fc_rp_dists = robot_model->getFeasibleControlRollPitchDists();
  const Eigen::MatrixXd& fc_rp_jacobian = robot_model->getFeasibleControlRollPitchDistsJacobian();

  for (const int active_index : active_indices)
  {
    const Eigen::RowVectorXd jacobian_row =
        extractLinkJointJacobianRow(fc_rp_jacobian, active_index, link_joint_num);
    builder.appendLowerBound(
        jacobian_row, fc_rp_min_threshold - fc_rp_dists(active_index) + jacobian_row.dot(current_joint_positions));
  }
}

void appendFcTConstraints(const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& robot_model,
                          const std::vector<int>& active_indices,
                          int link_joint_num,
                          double fc_t_min_threshold,
                          const Eigen::VectorXd& current_joint_positions,
                          QpConstraintBuilder& builder)
{
  const Eigen::VectorXd& fc_t_dists = robot_model->getFeasibleControlTDists();
  const Eigen::MatrixXd& fc_t_jacobian = robot_model->getFeasibleControlTDistsJacobian();

  for (const int active_index : active_indices)
  {
    const Eigen::RowVectorXd jacobian_row =
        extractLinkJointJacobianRow(fc_t_jacobian, active_index, link_joint_num);
    builder.appendLowerBound(
        jacobian_row, fc_t_min_threshold - fc_t_dists(active_index) + jacobian_row.dot(current_joint_positions));
  }
}

void appendStaticThrustConstraints(const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& robot_model,
                                   int link_joint_num,
                                   double static_thrust_min,
                                   double static_thrust_max,
                                   const Eigen::VectorXd& current_joint_positions,
                                   QpConstraintBuilder& builder)
{
  const Eigen::VectorXd& static_thrust = robot_model->getStaticThrust();
  const Eigen::MatrixXd& lambda_jacobian = robot_model->getLambdaJacobian();

  for (int i = 0; i < static_thrust.size(); ++i)
  {
    const Eigen::RowVectorXd jacobian_row =
        extractLinkJointJacobianRow(lambda_jacobian, i, link_joint_num);
    builder.appendLowerBound(
        jacobian_row, static_thrust_min - static_thrust(i) + jacobian_row.dot(current_joint_positions));
    builder.appendUpperBound(
        jacobian_row, static_thrust_max - static_thrust(i) + jacobian_row.dot(current_joint_positions));
  }
}

void appendOverlapConstraint(const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& robot_model,
                             int link_joint_num,
                             double minimum_overlap_clearance,
                             const Eigen::VectorXd& current_joint_positions,
                             QpConstraintBuilder& builder)
{
  const double overlap_clearance =
      robot_model->getClosestRotorDist() - 2.0 * robot_model->getEdfRadius();
  const Eigen::RowVectorXd overlap_jacobian =
      extractLinkJointJacobianRow(robot_model->getRotorOverlapJacobian(), 0, link_joint_num);
  builder.appendLowerBound(
      overlap_jacobian, minimum_overlap_clearance - overlap_clearance + overlap_jacobian.dot(current_joint_positions));
}
}  // namespace

bool CopilotPlanner::solveStableJointQp(const Eigen::VectorXd& desired_joint_positions,
                                        const Eigen::VectorXd& reference_joint_positions,
                                        Eigen::VectorXd& stable_joint_positions)
{
  const Eigen::VectorXd desired_full = clampLinkJointPositions(desired_joint_positions);

  if (link_joint_num_ <= 0)
  {
    stable_joint_positions = desired_full;
    return checkStability(stable_joint_positions, false);
  }

  Eigen::VectorXd current_joint_positions = clampLinkJointPositions(reference_joint_positions);
  StabilityMetrics current_metrics;
  if (!evaluateStability(current_joint_positions, current_metrics) || !current_metrics.safe)
  {
    return false;
  }

  Eigen::VectorXd best_joint_positions = current_joint_positions;
  double best_joint_error = (best_joint_positions - desired_full).norm();

  const Eigen::VectorXd joint_lower_bounds =
      buildJointLimitVector(dragon_robot_model_->getLinkJointLowerLimits(), link_joint_num_, -M_PI);
  const Eigen::VectorXd joint_upper_bounds =
      buildJointLimitVector(dragon_robot_model_->getLinkJointUpperLimits(), link_joint_num_, M_PI);

  for (int iter = 0; iter < stability_qp_max_iterations_; ++iter)
  {
    StabilityMetrics iter_metrics;
    if (!evaluateStability(current_joint_positions, iter_metrics))
    {
      break;
    }

    const Eigen::VectorXd current_joint_vector = current_joint_positions;
    const double current_joint_error = (current_joint_vector - desired_full).norm();
    if (iter_metrics.safe && current_joint_error < best_joint_error)
    {
      best_joint_positions = current_joint_positions;
      best_joint_error = current_joint_error;
    }

    if (iter_metrics.safe && current_joint_error <= stability_qp_convergence_tol_)
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

    QpConstraintBuilder builder(constraint_count, link_joint_num_);
    appendFcRpConstraints(dragon_robot_model_, active_fc_rp_indices, link_joint_num_, stability_fc_rp_min_thre_,
                          current_joint_vector, builder);
    if (stability_check_fc_t_)
    {
      appendFcTConstraints(dragon_robot_model_, active_fc_t_indices, link_joint_num_, stability_fc_t_min_thre_,
                           current_joint_vector, builder);
    }
    appendStaticThrustConstraints(dragon_robot_model_, link_joint_num_, stability_static_thrust_min_,
                                  stability_static_thrust_max_, current_joint_vector, builder);
    appendOverlapConstraint(dragon_robot_model_, link_joint_num_, stability_overlap_min_clearance_,
                            current_joint_vector, builder);
    if (builder.row != constraint_count)
    {
      ROS_ERROR_STREAM("[CopilotPlanner] Stability QP constraint assembly mismatch: expected "
                       << constraint_count << " rows but built " << builder.row);
      return false;
    }

    Eigen::VectorXd qp_lower_bounds = current_joint_vector.array() - stability_qp_joint_step_limit_;
    Eigen::VectorXd qp_upper_bounds = current_joint_vector.array() + stability_qp_joint_step_limit_;
    qp_lower_bounds = qp_lower_bounds.cwiseMax(joint_lower_bounds);
    qp_upper_bounds = qp_upper_bounds.cwiseMin(joint_upper_bounds);

    if (!hasNonEmptyIntersection(qp_lower_bounds, qp_upper_bounds, stability_qp_feasibility_tol_))
    {
      break;
    }

    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(link_joint_num_, link_joint_num_);
    H *= (1.0 + stability_qp_regularization_);
    const Eigen::VectorXd g = -(desired_full + stability_qp_regularization_ * current_joint_vector);

    RowMajorMatrixXd H_row_major = H;
    RowMajorMatrixXd A_row_major = builder.A;

    qpOASES::SQProblem solver(link_joint_num_, constraint_count);
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

    Eigen::VectorXd solved_joint_positions = current_joint_vector;
    solver.getPrimalSolution(solved_joint_positions.data());
    if (!solved_joint_positions.allFinite())
    {
      break;
    }

    current_joint_positions = clampLinkJointPositions(solved_joint_positions);
    const double update_norm = (current_joint_positions - current_joint_vector).norm();
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
