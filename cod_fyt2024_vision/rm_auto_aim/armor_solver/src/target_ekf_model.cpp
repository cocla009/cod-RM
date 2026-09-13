// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "armor_solver/target_ekf_model.hpp"

#include <cmath>

namespace fyt::auto_aim {
namespace {
constexpr double kTwoPi = 2.0 * M_PI;
}

bool TargetEkfModel::validState(const Eigen::VectorXd &state) noexcept {
  return state.size() == kStateSize && state.allFinite();
}

bool TargetEkfModel::validArmorIndex(int armor_index, int armor_count) noexcept {
  return armor_count >= 2 && armor_count <= 8 && armor_index >= 0 && armor_index < armor_count;
}

bool TargetEkfModel::isLongArmor(int armor_index, int armor_count) noexcept {
  return armor_count == 4 && (armor_index % 2 == 1);
}

double TargetEkfModel::normalizeAngle(double angle) noexcept {
  return std::remainder(angle, kTwoPi);
}

bool TargetEkfModel::propagate(const Eigen::VectorXd &state,
                               double dt,
                               Eigen::VectorXd &predicted) noexcept {
  if (!validState(state) || !std::isfinite(dt)) {
    return false;
  }

  predicted = state;
  predicted(XC) += state(VXC) * dt;
  predicted(YC) += state(VYC) * dt;
  predicted(Z) += state(VZ) * dt;
  predicted(YAW) = normalizeAngle(state(YAW) + state(VYAW) * dt);
  return predicted.allFinite();
}

Eigen::MatrixXd TargetEkfModel::transitionJacobian(double dt) noexcept {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(kStateSize, kStateSize);
  if (!std::isfinite(dt)) {
    return Eigen::MatrixXd();
  }

  jacobian(XC, VXC) = dt;
  jacobian(YC, VYC) = dt;
  jacobian(Z, VZ) = dt;
  jacobian(YAW, VYAW) = dt;
  return jacobian;
}

bool TargetEkfModel::armorPosition(const Eigen::VectorXd &state,
                                   int armor_index,
                                   int armor_count,
                                   Eigen::Vector3d &position) noexcept {
  position.setZero();
  if (!validState(state) || !validArmorIndex(armor_index, armor_count)) {
    return false;
  }

  const double angle = state(YAW) + armor_index * kTwoPi / armor_count;
  const bool long_armor = isLongArmor(armor_index, armor_count);
  const double radius = state(R1) + (long_armor ? state(DELTA_R) : 0.0);
  const double height = long_armor ? state(DZ) : 0.0;

  position << state(XC) - radius * std::cos(angle),
    state(YC) - radius * std::sin(angle), state(Z) + height;
  return position.allFinite();
}

bool TargetEkfModel::armorPositionJacobian(const Eigen::VectorXd &state,
                                           int armor_index,
                                           int armor_count,
                                           Eigen::MatrixXd &jacobian) noexcept {
  jacobian = Eigen::MatrixXd::Zero(3, kStateSize);
  if (!validState(state) || !validArmorIndex(armor_index, armor_count)) {
    return false;
  }

  const double angle = state(YAW) + armor_index * kTwoPi / armor_count;
  const bool long_armor = isLongArmor(armor_index, armor_count);
  const double radius = state(R1) + (long_armor ? state(DELTA_R) : 0.0);
  const double sin_angle = std::sin(angle);
  const double cos_angle = std::cos(angle);

  jacobian(0, XC) = 1.0;
  jacobian(0, YAW) = radius * sin_angle;
  jacobian(0, R1) = -cos_angle;
  if (long_armor) {
    jacobian(0, DELTA_R) = -cos_angle;
  }

  jacobian(1, YC) = 1.0;
  jacobian(1, YAW) = -radius * cos_angle;
  jacobian(1, R1) = -sin_angle;
  if (long_armor) {
    jacobian(1, DELTA_R) = -sin_angle;
  }

  jacobian(2, Z) = 1.0;
  if (long_armor) {
    jacobian(2, DZ) = 1.0;
  }

  return jacobian.allFinite();
}

bool TargetEkfModel::cartesianObservation(const Eigen::VectorXd &state,
                                          int armor_index,
                                          int armor_count,
                                          Eigen::Vector4d &observation) noexcept {
  observation.setZero();
  Eigen::Vector3d position;
  if (!armorPosition(state, armor_index, armor_count, position)) {
    return false;
  }

  observation << position,
    normalizeAngle(state(YAW) + armor_index * kTwoPi / armor_count);
  return observation.allFinite();
}

bool TargetEkfModel::cartesianObservationJacobian(const Eigen::VectorXd &state,
                                                  int armor_index,
                                                  int armor_count,
                                                  Eigen::MatrixXd &jacobian) noexcept {
  jacobian = Eigen::MatrixXd::Zero(4, kStateSize);
  Eigen::MatrixXd position_jacobian;
  if (!armorPositionJacobian(state, armor_index, armor_count, position_jacobian)) {
    return false;
  }

  jacobian.topRows(3) = position_jacobian;
  jacobian(3, YAW) = 1.0;
  return jacobian.allFinite();
}

bool TargetEkfModel::sphericalObservation(const Eigen::VectorXd &state,
                                          int armor_index,
                                          int armor_count,
                                          Eigen::Vector4d &observation) noexcept {
  observation.setZero();
  Eigen::Vector3d position;
  if (!armorPosition(state, armor_index, armor_count, position)) {
    return false;
  }

  Eigen::Vector3d spherical;
  if (!cartesianToSpherical(position, spherical)) {
    return false;
  }

  observation << spherical,
    normalizeAngle(state(YAW) + armor_index * kTwoPi / armor_count);
  return observation.allFinite();
}

bool TargetEkfModel::cartesianToSpherical(const Eigen::Vector3d &position,
                                          Eigen::Vector3d &spherical) noexcept {
  spherical.setZero();
  if (!position.allFinite()) {
    return false;
  }

  const double horizontal_distance = std::hypot(position.x(), position.y());
  const double distance = position.norm();
  if (horizontal_distance < kMinHorizontalDistance || distance < kMinDistance) {
    return false;
  }

  spherical << std::atan2(position.y(), position.x()),
    std::atan2(position.z(), horizontal_distance), distance;
  return spherical.allFinite();
}

bool TargetEkfModel::sphericalPositionJacobian(const Eigen::Vector3d &position,
                                              Eigen::MatrixXd &jacobian) noexcept {
  jacobian = Eigen::MatrixXd::Zero(3, 3);
  if (!position.allFinite()) {
    return false;
  }

  const double x = position.x();
  const double y = position.y();
  const double z = position.z();
  const double horizontal_distance = std::hypot(x, y);
  const double distance = position.norm();
  if (horizontal_distance < kMinHorizontalDistance || distance < kMinDistance) {
    return false;
  }

  const double horizontal_squared = x * x + y * y;
  const double distance_squared = distance * distance;

  // azimuth = atan2(y, x)
  jacobian(0, 0) = -y / horizontal_squared;
  jacobian(0, 1) = x / horizontal_squared;

  // elevation = atan2(z, hypot(x, y))
  jacobian(1, 0) = -z * x / (horizontal_distance * distance_squared);
  jacobian(1, 1) = -z * y / (horizontal_distance * distance_squared);
  jacobian(1, 2) = horizontal_distance / distance_squared;

  // distance = hypot(x, y, z)
  jacobian(2, 0) = x / distance;
  jacobian(2, 1) = y / distance;
  jacobian(2, 2) = z / distance;
  return jacobian.allFinite();
}

bool TargetEkfModel::observationJacobian(const Eigen::VectorXd &state,
                                         int armor_index,
                                         int armor_count,
                                         Eigen::MatrixXd &jacobian) noexcept {
  jacobian = Eigen::MatrixXd::Zero(4, kStateSize);
  Eigen::Vector3d position;
  Eigen::MatrixXd position_jacobian;
  Eigen::MatrixXd spherical_jacobian;
  if (!armorPosition(state, armor_index, armor_count, position) ||
      !armorPositionJacobian(state, armor_index, armor_count, position_jacobian) ||
      !sphericalPositionJacobian(position, spherical_jacobian)) {
    return false;
  }

  jacobian.topRows(3) = spherical_jacobian * position_jacobian;
  jacobian(3, YAW) = 1.0;
  return jacobian.allFinite();
}

}  // namespace fyt::auto_aim
