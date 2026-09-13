#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "armor_solver/target_ekf_model.hpp"

namespace fyt::auto_aim {
namespace {

Eigen::VectorXd makeState() {
  Eigen::VectorXd state = Eigen::VectorXd::Zero(TargetEkfModel::kStateSize);
  state << 1.0, 0.2, 2.0, -0.1, 0.5, 0.03, 0.4, 1.2, 0.2, 0.05, 0.1;
  return state;
}

double finiteDifference(const Eigen::VectorXd &state,
                        int armor_index,
                        int armor_count,
                        int output_index,
                        int state_index,
                        double epsilon = 1e-6) {
  auto plus = state;
  auto minus = state;
  plus(state_index) += epsilon;
  minus(state_index) -= epsilon;

  Eigen::Vector3d plus_position;
  Eigen::Vector3d minus_position;
  if (!TargetEkfModel::armorPosition(plus, armor_index, armor_count, plus_position) ||
      !TargetEkfModel::armorPosition(minus, armor_index, armor_count, minus_position)) {
    ADD_FAILURE() << "Finite-difference sample is invalid";
    return std::numeric_limits<double>::quiet_NaN();
  }
  return (plus_position(output_index) - minus_position(output_index)) / (2.0 * epsilon);
}

double sphericalFiniteDifference(const Eigen::VectorXd &state,
                                int armor_index,
                                int armor_count,
                                int output_index,
                                int state_index,
                                double epsilon = 1e-6) {
  auto plus = state;
  auto minus = state;
  plus(state_index) += epsilon;
  minus(state_index) -= epsilon;

  Eigen::Vector4d plus_observation;
  Eigen::Vector4d minus_observation;
  if (!TargetEkfModel::sphericalObservation(
        plus, armor_index, armor_count, plus_observation) ||
      !TargetEkfModel::sphericalObservation(
        minus, armor_index, armor_count, minus_observation)) {
    ADD_FAILURE() << "Spherical finite-difference sample is invalid";
    return std::numeric_limits<double>::quiet_NaN();
  }

  double difference = plus_observation(output_index) - minus_observation(output_index);
  if (output_index == 0 || output_index == 3) {
    difference = TargetEkfModel::normalizeAngle(difference);
  }
  return difference / (2.0 * epsilon);
}

TEST(TargetEkfModel, PropagatesStateAndNormalizesYaw) {
  auto state = makeState();
  state(YAW) = 3.0;

  Eigen::VectorXd predicted;
  ASSERT_TRUE(TargetEkfModel::propagate(state, 0.5, predicted));
  EXPECT_NEAR(predicted(XC), 1.1, 1e-12);
  EXPECT_NEAR(predicted(YC), 1.95, 1e-12);
  EXPECT_NEAR(predicted(Z), 0.515, 1e-12);
  EXPECT_NEAR(predicted(YAW), -2.683185307179586, 1e-12);
}

TEST(TargetEkfModel, ComputesFourArmorGeometry) {
  Eigen::VectorXd state = Eigen::VectorXd::Zero(TargetEkfModel::kStateSize);
  state(XC) = 1.0;
  state(YC) = 2.0;
  state(Z) = 0.5;
  state(R1) = 0.2;
  state(DELTA_R) = 0.05;
  state(DZ) = 0.1;

  const std::vector<Eigen::Vector3d> expected{
    {0.8, 2.0, 0.5}, {1.0, 1.75, 0.6}, {1.2, 2.0, 0.5}, {1.0, 2.25, 0.6}};
  for (int i = 0; i < 4; ++i) {
    Eigen::Vector3d actual;
    ASSERT_TRUE(TargetEkfModel::armorPosition(state, i, 4, actual));
    EXPECT_NEAR((actual - expected[i]).norm(), 0.0, 1e-12);
  }
}

TEST(TargetEkfModel, ComputesTwoAndThreeArmorGeometryWithoutFourArmorOffsets) {
  auto state = makeState();
  state(XC) = 1.0;
  state(YC) = 2.0;
  state(Z) = 0.5;
  state(YAW) = 0.0;
  state(R1) = 0.2;
  state(DELTA_R) = 0.07;
  state(DZ) = 0.15;

  for (const int armor_count : {2, 3}) {
    for (int armor_index = 0; armor_index < armor_count; ++armor_index) {
      Eigen::Vector3d position;
      ASSERT_TRUE(TargetEkfModel::armorPosition(state, armor_index, armor_count, position));
      const double angle = armor_index * 2.0 * M_PI / armor_count;
      EXPECT_NEAR(position.x(), state(XC) - state(R1) * std::cos(angle), 1e-12);
      EXPECT_NEAR(position.y(), state(YC) - state(R1) * std::sin(angle), 1e-12);
      EXPECT_NEAR(position.z(), state(Z), 1e-12);
    }
  }
}

TEST(TargetEkfModel, RejectsInvalidArmorIndexAndSingularObservation) {
  auto state = makeState();
  Eigen::Vector3d position;
  EXPECT_FALSE(TargetEkfModel::armorPosition(state, 4, 4, position));

  state(XC) = 0.0;
  state(YC) = 0.0;
  state(Z) = 1.0;
  state(R1) = 0.0;
  Eigen::Vector4d observation;
  EXPECT_FALSE(TargetEkfModel::sphericalObservation(state, 0, 4, observation));
}

TEST(TargetEkfModel, SphericalObservationMatchesExpectedValues) {
  Eigen::VectorXd state = Eigen::VectorXd::Zero(TargetEkfModel::kStateSize);
  state(XC) = 1.0;
  state(YC) = 0.0;
  state(Z) = 0.0;
  state(YAW) = 0.0;
  state(R1) = 0.0;

  Eigen::Vector4d observation;
  ASSERT_TRUE(TargetEkfModel::sphericalObservation(state, 0, 4, observation));
  EXPECT_NEAR(observation[0], 0.0, 1e-12);
  EXPECT_NEAR(observation[1], 0.0, 1e-12);
  EXPECT_NEAR(observation[2], 1.0, 1e-12);
  EXPECT_NEAR(observation[3], 0.0, 1e-12);
}

TEST(TargetEkfModel, CartesianObservationMatchesArmorPosition) {
  const auto state = makeState();
  Eigen::Vector3d position;
  Eigen::Vector4d observation;
  ASSERT_TRUE(TargetEkfModel::armorPosition(state, 1, 4, position));
  ASSERT_TRUE(TargetEkfModel::cartesianObservation(state, 1, 4, observation));
  EXPECT_NEAR((observation.head<3>() - position).norm(), 0.0, 1e-12);
  EXPECT_NEAR(observation[3], TargetEkfModel::normalizeAngle(state(YAW) + M_PI / 2.0), 1e-12);
}

TEST(TargetEkfModel, CartesianObservationJacobianMatchesPositionJacobian) {
  const auto state = makeState();
  Eigen::MatrixXd position_jacobian;
  Eigen::MatrixXd observation_jacobian;
  ASSERT_TRUE(TargetEkfModel::armorPositionJacobian(state, 1, 4, position_jacobian));
  ASSERT_TRUE(
    TargetEkfModel::cartesianObservationJacobian(state, 1, 4, observation_jacobian));
  EXPECT_NEAR(
    (observation_jacobian.topRows(3) - position_jacobian).norm(), 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(observation_jacobian(3, YAW), 1.0);
}

TEST(TargetEkfModel, PositionJacobianMatchesFiniteDifference) {
  const auto state = makeState();
  Eigen::MatrixXd jacobian;
  ASSERT_TRUE(TargetEkfModel::armorPositionJacobian(state, 1, 4, jacobian));

  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < TargetEkfModel::kStateSize; ++column) {
      const double expected = finiteDifference(state, 1, 4, row, column);
      EXPECT_NEAR(jacobian(row, column), expected, 1e-6)
        << "row=" << row << " column=" << column;
    }
  }
}

TEST(TargetEkfModel, ObservationJacobianIsFiniteAndHasArmorYawDerivative) {
  const auto state = makeState();
  Eigen::MatrixXd jacobian;
  ASSERT_TRUE(TargetEkfModel::observationJacobian(state, 1, 4, jacobian));
  ASSERT_EQ(jacobian.rows(), 4);
  ASSERT_EQ(jacobian.cols(), TargetEkfModel::kStateSize);
  EXPECT_TRUE(jacobian.allFinite());
  EXPECT_DOUBLE_EQ(jacobian(3, YAW), 1.0);
}

TEST(TargetEkfModel, ObservationJacobianMatchesFiniteDifference) {
  const auto state = makeState();
  Eigen::MatrixXd jacobian;
  ASSERT_TRUE(TargetEkfModel::observationJacobian(state, 1, 4, jacobian));

  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < TargetEkfModel::kStateSize; ++column) {
      const double expected = sphericalFiniteDifference(state, 1, 4, row, column);
      EXPECT_NEAR(jacobian(row, column), expected, 1e-6)
        << "row=" << row << " column=" << column;
    }
  }
}

}  // namespace
}  // namespace fyt::auto_aim
