/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/common/math/mpc_osqp.h"

namespace apollo {
namespace common {
namespace math {

void MpcOsqp::castMPCToQPHessian() {
  hessian_.resize(state_dim_ * (horizon_ + 1) + control_dim_ * horizon_,
                  state_dim_ * (horizon_ + 1) + control_dim_ * horizon_);

  // populate hessian matrix
  for (int i = 0; i < state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
       i++) {
    // state range
    if (i < state_dim_ * (horizon_ + 1)) {
      int posQ = i % state_dim_;
      float value = matrix_q_(posQ, posQ);
      if (value != 0) hessian_.insert(i, i) = value;
    } else {
      int posR = i % control_dim_;
      float value = matrix_r_(posR, posR);
      if (value != 0) hessian_.insert(i, i) = value;
    }
  }
}

// error reference is always zero
void MpcOsqp::castMPCToQPGradient() {
  // populate the gradient vector
  gradient_ = Eigen::VectorXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
}

void MpcOsqp::castMPCToQPConstraintMatrix() {
  matrix_constraint_.resize(
      state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) +
          control_dim_ * horizon_,
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_);

  // populate linear constraint matrix
  for (int i = 0; i < state_dim_ * (horizon_ + 1); i++) {
    matrix_constraint_.insert(i, i) = -1;
  }

  for (int i = 0; i < horizon_; i++)
    for (int j = 0; j < state_dim_; j++)
      for (int k = 0; k < state_dim_; k++) {
        float value = matrix_a_(j, k);
        if (value != 0) {
          matrix_constraint_.insert(state_dim_ * (i + 1) + j,
                                    state_dim_ * i + k) = value;
        }
      }

  for (int i = 0; i < horizon_; i++)
    for (int j = 0; j < state_dim_; j++)
      for (int k = 0; k < control_dim_; k++) {
        float value = matrix_b_(j, k);
        if (value != 0) {
          matrix_constraint_.insert(
              state_dim_ * (i + 1) + j,
              control_dim_ * i + k + state_dim_ * (horizon_ + 1)) = value;
        }
      }

  for (int i = 0; i < state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
       i++) {
    matrix_constraint_.insert(i + (horizon_ + 1) * state_dim_, i) = 1;
  }
}

void MpcOsqp::castMPCToQPConstraintVectors() {
  // evaluate the lower and the upper inequality vectors
  Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  for (int i = 0; i < horizon_ + 1; i++) {
    lowerInequality.block(state_dim_ * i, 0, state_dim_, 1) = xMin;
    upperInequality.block(state_dim_ * i, 0, state_dim_, 1) = xMax;
  }
  for (int i = 0; i < horizon_; i++) {
    lowerInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0,
                          control_dim_, 1) = matrix_lower_;
    upperInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0,
                          control_dim_, 1) = matrix_upper_;
  }

  // evaluate the lower and the upper equality vectors
  Eigen::VectorXd lowerEquality =
      Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1), 1);
  Eigen::VectorXd upperEquality;
  lowerEquality.block(0, 0, state_dim_, 1) = matrix_initial_state_;
  upperEquality = lowerEquality;
  lowerEquality = lowerEquality;

  // merge inequality and equality vectors
  lowerBound_ = Eigen::MatrixXd::Zero(
      2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  lowerBound_ << lowerEquality, lowerInequality;

  upperBound_ = Eigen::MatrixXd::Zero(
      2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  upperBound_ << upperEquality, upperInequality;
}

bool MpcOsqp::MpcOsqpSolver() {
  // cast the MPC problem as QP problem
  castMPCToQPHessian();
  castMPCToQPGradient();
  castMPCToQPConstraintMatrix();
  castMPCToQPConstraintVectors();
  // instantiate the solver
  OsqpEigen::Solver solver;
  // settings
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);
  solver.settings()->setMaxIteraction(max_iteration_);
  // set the initial data of the QP solver
  solver.data()->setNumberOfVariables(num_param_);
  solver.data()->setNumberOfConstraints(num_constraint_);
  if (!solver.data()->setHessianMatrix(hessian_)) return false;
  if (!solver.data()->setGradient(gradient_)) return false;
  if (!solver.data()->setLinearConstraintsMatrix(matrix_constraint_))
    return false;
  if (!solver.data()->setLowerBound(lowerBound_)) return false;
  if (!solver.data()->setUpperBound(upperBound_)) return false;
  // instantiate the solver
  if (!solver.initSolver()) return false;

  // controller input and QPSolution vector
  Eigen::Vector4d control_cmd;
  Eigen::VectorXd QPSolution;
  // solve the QP problem
  if (!solver.solve()) return false;
  // get the controller input
  QPSolution = solver.getSolution();
  control_cmd =
      QPSolution.block(state_dim_ * (horizon_ + 1), 0, control_dim_, 1);
  return true;
}
}  // namespace math
}  // namespace common
}  // namespace apollo
