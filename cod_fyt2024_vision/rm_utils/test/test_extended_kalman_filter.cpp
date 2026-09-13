#include <gtest/gtest.h>

#include <cmath>

#include "rm_utils/math/extended_kalman_filter.hpp"

namespace fyt {
namespace {

TEST(ExtendedKalmanFilter, SupportsWrappedMeasurementResidual) {
  const ExtendedKalmanFilter::VecVecFunc f = [](const Eigen::VectorXd &x) { return x; };
  const ExtendedKalmanFilter::VecVecFunc h = [](const Eigen::VectorXd &x) { return x; };
  const ExtendedKalmanFilter::VecMatFunc jacobian = [](const Eigen::VectorXd &) {
    return Eigen::MatrixXd::Identity(1, 1);
  };
  const ExtendedKalmanFilter::VoidMatFunc q = [] { return Eigen::MatrixXd::Zero(1, 1); };
  const ExtendedKalmanFilter::VecMatFunc r = [](const Eigen::VectorXd &) {
    return Eigen::MatrixXd::Constant(1, 1, 1e-3);
  };
  const ExtendedKalmanFilter::VecVecSubtractFunc subtract =
    [](const Eigen::VectorXd &measurement, const Eigen::VectorXd &prediction) {
      Eigen::VectorXd residual = measurement - prediction;
      residual[0] = std::remainder(residual[0], 2.0 * M_PI);
      return residual;
    };

  Eigen::VectorXd initial_state(1);
  initial_state[0] = 3.13;
  Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(1, 1);
  ExtendedKalmanFilter filter(
    f, h, jacobian, jacobian, q, r, initial_covariance, subtract);
  filter.resetState(initial_state);

  Eigen::VectorXd measurement(1);
  measurement(0) = -3.13;
  const auto updated = filter.update(measurement);

  EXPECT_NEAR(updated(0), 3.13, 0.1);
  EXPECT_TRUE(std::isfinite(updated(0)));
}

TEST(ExtendedKalmanFilter, WrapsAzimuthAndArmorYawResidualsIndependently) {
  const ExtendedKalmanFilter::VecVecFunc f = [](const Eigen::VectorXd &x) { return x; };
  const ExtendedKalmanFilter::VecVecFunc h = [](const Eigen::VectorXd &x) { return x; };
  const ExtendedKalmanFilter::VecMatFunc jacobian = [](const Eigen::VectorXd &) {
    return Eigen::MatrixXd::Identity(4, 4);
  };
  const ExtendedKalmanFilter::VoidMatFunc q = [] { return Eigen::MatrixXd::Zero(4, 4); };
  const ExtendedKalmanFilter::VecMatFunc r = [](const Eigen::VectorXd &) {
    return Eigen::MatrixXd::Identity(4, 4) * 1e-3;
  };
  const ExtendedKalmanFilter::VecVecSubtractFunc subtract =
    [](const Eigen::VectorXd &measurement, const Eigen::VectorXd &prediction) {
      Eigen::VectorXd residual = measurement - prediction;
      residual[0] = std::remainder(residual[0], 2.0 * M_PI);
      residual[3] = std::remainder(residual[3], 2.0 * M_PI);
      return residual;
    };

  Eigen::VectorXd initial_state(4);
  initial_state << 3.13, 0.1, 5.0, 3.13;
  ExtendedKalmanFilter filter(
    f, h, jacobian, jacobian, q, r, Eigen::MatrixXd::Identity(4, 4), subtract);
  filter.resetState(initial_state);

  Eigen::VectorXd measurement(4);
  measurement << -3.13, 0.1, 5.0, -3.13;
  const auto updated = filter.update(measurement);

  EXPECT_NEAR(updated(0), 3.13, 0.1);
  EXPECT_NEAR(updated(1), 0.1, 0.1);
  EXPECT_NEAR(updated(2), 5.0, 0.1);
  EXPECT_NEAR(updated(3), 3.13, 0.1);
}

}  // namespace
}  // namespace fyt
