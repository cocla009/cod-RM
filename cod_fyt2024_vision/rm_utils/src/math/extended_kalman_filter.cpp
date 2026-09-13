// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "rm_utils/math/extended_kalman_filter.hpp"

#include <algorithm>
#include <cmath>

namespace fyt {
ExtendedKalmanFilter::ExtendedKalmanFilter(const VecVecFunc &f,
                                           const VecVecFunc &h,
                                           const VecMatFunc &j_f,
                                           const VecMatFunc &j_h,
                                           const VoidMatFunc &u_q,
                                           const VecMatFunc &u_r,
                                           const Eigen::MatrixXd &P0,
                                           const VecVecSubtractFunc &subtract_measurement)
: f(f)
, h(h)
, jacobian_f(j_f)
, jacobian_h(j_h)
, update_Q(u_q)
, update_R(u_r)
, subtract_measurement(subtract_measurement)
, P_post(P0)
, P0_(P0)
, n(P0.rows())
, I(Eigen::MatrixXd::Identity(n, n))
, x_pri(n)
, x_post(n) {}

void ExtendedKalmanFilter::setState(const Eigen::VectorXd &x0) noexcept { x_post = x0; }

void ExtendedKalmanFilter::resetState(const Eigen::VectorXd &x0) noexcept {
  x_post = x0;
  x_pri = x0;
  P_post = P0_;
  P_pri = P0_;
  K = Eigen::MatrixXd::Zero(P0_.rows(), P0_.cols());
  resetInnovationHistory();
}

Eigen::MatrixXd ExtendedKalmanFilter::predict() noexcept {
  F = jacobian_f(x_post), Q = update_Q();

  x_pri = f(x_post);
  P_pri = F * P_post * F.transpose() + Q;

  // handle the case when there will be no measurement before the next predict
  x_post = x_pri;
  P_post = P_pri;

  return x_pri;
}

Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd &z) noexcept {
  H = jacobian_h(x_pri), R = update_R(z);

  const Eigen::MatrixXd S = H * P_pri * H.transpose() + R;
  const Eigen::VectorXd residual = subtract_measurement(z, h(x_pri));
  const Eigen::LDLT<Eigen::MatrixXd> decomposition(S);
  if (decomposition.info() != Eigen::Success || !S.allFinite()) {
    nis_failures_.push_back(true);
    if (nis_failures_.size() > kNisWindowSize) {
      nis_failures_.pop_front();
    }
    x_post = x_pri;
    P_post = P_pri;
    return x_post;
  }

  const Eigen::VectorXd innovation_solution = decomposition.solve(residual);
  const Eigen::MatrixXd gain_solution = decomposition.solve(H * P_pri);
  K = gain_solution.transpose();
  x_post = x_pri + K * residual;

  // Joseph form preserves covariance symmetry and positive semidefiniteness
  // better than the simplified (I - KH)P expression.
  const Eigen::MatrixXd identity_minus_gain = I - K * H;
  P_post = identity_minus_gain * P_pri * identity_minus_gain.transpose() + K * R * K.transpose();
  P_post = (P_post + P_post.transpose()) * 0.5;

  last_nis_ = residual.dot(innovation_solution);
  nis_failures_.push_back(!std::isfinite(last_nis_) || last_nis_ > kNisThreshold);
  if (nis_failures_.size() > kNisWindowSize) {
    nis_failures_.pop_front();
  }

  return x_post;
}

bool ExtendedKalmanFilter::isDiverged() const noexcept {
  if (nis_failures_.size() < kNisWindowSize) {
    return false;
  }
  const auto failures = static_cast<std::size_t>(
    std::count(nis_failures_.begin(), nis_failures_.end(), true));
  return failures >= static_cast<std::size_t>(0.4 * kNisWindowSize);
}

void ExtendedKalmanFilter::resetInnovationHistory() noexcept {
  nis_failures_.clear();
  last_nis_ = 0.0;
}

}  // namespace fyt
