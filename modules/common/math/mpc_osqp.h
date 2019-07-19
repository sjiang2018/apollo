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

#pragma once

#include <algorithm>
#include <memory>

#include "Eigen/Core"
#include "OsqpEigen/OsqpEigen.h"
#include "cyber/common/log.h"
#include "osqp/include/osqp.h"

namespace apollo {
namespace common {
namespace math {
class MpcOsqp {
 public:
  //   MpcOsqp(const Eigen::MatrixXd &matrix_a, const Eigen::MatrixXd &matrix_b,
  //           const Eigen::MatrixXd &matrix_c, const Eigen::MatrixXd &matrix_q,
  //           const Eigen::MatrixXd &matrix_r, const Eigen::MatrixXd
  //           &matrix_lower, const Eigen::MatrixXd &matrix_upper, const
  //           Eigen::MatrixXd &matrix_initial_state, const
  //           std::vector<Eigen::MatrixXd> &reference, const double eps, const
  //           int max_iter, std::vector<Eigen::MatrixXd> *control,
  //           std::vector<Eigen::MatrixXd> *control_gain,
  //           std::vector<Eigen::MatrixXd> *addition_gain);
  bool MpcOsqpSolver();

 private:
  void castMPCToQPHessian();
  void castMPCToQPGradient();
  void castMPCToQPConstraintMatrix();

 private:
  Eigen::MatrixXd matrix_a_;
  Eigen::MatrixXd matrix_b_;
  Eigen::MatrixXd matrix_q_;
  Eigen::MatrixXd matrix_r_;
  // const Eigen::MatrixXd
  //           &matrix_lower, const Eigen::MatrixXd &matrix_upper, const
  //           Eigen::MatrixXd &matrix_initial_state, const
  //           std::vector<Eigen::MatrixXd> &reference,
  int max_iteration_ = 1000;
  int num_constraint_;
  int num_param_;
  int state_dim_;
  int horizon_;
  int control_dim_;
  // allocate QP problem matrices and vectors
  Eigen::SparseMatrix<double> hessian_;
  Eigen::VectorXd gradient_;
  Eigen::SparseMatrix<double> matrix_constraint_;
  Eigen::VectorXd lowerBound_;
  Eigen::VectorXd upperBound_;
};
}  // namespace math
}  // namespace common
}  // namespace apollo
