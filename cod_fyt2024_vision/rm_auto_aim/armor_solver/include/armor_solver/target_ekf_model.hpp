// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ARMOR_SOLVER_TARGET_EKF_MODEL_HPP_
#define ARMOR_SOLVER_TARGET_EKF_MODEL_HPP_

#include <Eigen/Core>

namespace fyt::auto_aim {

// Target state used by the multi-armor EKF.
//
// The state keeps the target center and motion in Cartesian coordinates while
// retaining the two geometric parameters needed by a four-armor target.
enum TargetStateIndex {
  XC = 0,
  VXC,
  YC,
  VYC,
  Z,
  VZ,
  YAW,
  VYAW,
  R1,
  DELTA_R,
  DZ,
  TARGET_STATE_SIZE
};

class TargetEkfModel {
public:
  static constexpr int kStateSize = TARGET_STATE_SIZE;
  static constexpr double kMinHorizontalDistance = 1e-6;
  static constexpr double kMinDistance = 1e-6;

  // Propagate the constant-velocity/constant-angular-velocity state.
  static bool propagate(const Eigen::VectorXd &state,
                        double dt,
                        Eigen::VectorXd &predicted) noexcept;

  static Eigen::MatrixXd transitionJacobian(double dt) noexcept;

  // Calculate one armor position in the target/world Cartesian frame.
  static bool armorPosition(const Eigen::VectorXd &state,
                            int armor_index,
                            int armor_count,
                            Eigen::Vector3d &position) noexcept;

  // Jacobian of armorPosition() with respect to the target state.
  static bool armorPositionJacobian(const Eigen::VectorXd &state,
                                    int armor_index,
                                    int armor_count,
                                    Eigen::MatrixXd &jacobian) noexcept;

  // Cartesian observation: armor xyz position and armor yaw.
  static bool cartesianObservation(const Eigen::VectorXd &state,
                                   int armor_index,
                                   int armor_count,
                                   Eigen::Vector4d &observation) noexcept;

  static bool cartesianObservationJacobian(const Eigen::VectorXd &state,
                                           int armor_index,
                                           int armor_count,
                                           Eigen::MatrixXd &jacobian) noexcept;

  // Spherical observation: azimuth, elevation, 3-D distance, armor yaw.
  static bool sphericalObservation(const Eigen::VectorXd &state,
                                   int armor_index,
                                   int armor_count,
                                   Eigen::Vector4d &observation) noexcept;

  // Convert Cartesian position [x, y, z] to [azimuth, elevation, distance].
  static bool cartesianToSpherical(const Eigen::Vector3d &position,
                                   Eigen::Vector3d &spherical) noexcept;

  // Jacobian of spherical coordinates [azimuth, elevation, distance] with
  // respect to Cartesian coordinates [x, y, z].
  static bool sphericalPositionJacobian(const Eigen::Vector3d &position,
                                        Eigen::MatrixXd &jacobian) noexcept;

  // Jacobian of sphericalObservation() with respect to the target state.
  static bool observationJacobian(const Eigen::VectorXd &state,
                                  int armor_index,
                                  int armor_count,
                                  Eigen::MatrixXd &jacobian) noexcept;

  static double normalizeAngle(double angle) noexcept;

private:
  static bool validState(const Eigen::VectorXd &state) noexcept;
  static bool validArmorIndex(int armor_index, int armor_count) noexcept;
  static bool isLongArmor(int armor_index, int armor_count) noexcept;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_SOLVER_TARGET_EKF_MODEL_HPP_
