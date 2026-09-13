// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include <gtest/gtest.h>

#include "armor_solver/aim_correction.hpp"

namespace fyt::auto_aim {
namespace {

AimCorrectionParameters defaultParameters() {
  return {2.0, 1.0, 6.0, 10.0, 1.2};
}

TEST(AimCorrectionTest, OffsetIsAppliedAndYawIsNormalized) {
  auto parameters = defaultParameters();
  double yaw = M_PI - 0.01;
  double pitch = 0.1;

  applyAimCorrection(0.0, parameters, yaw, pitch);

  EXPECT_NEAR(yaw, -M_PI - 0.01 + 2.0 * M_PI / 180.0, 1e-9);
  EXPECT_NEAR(pitch, 0.1 + M_PI / 180.0, 1e-9);
}

TEST(AimCorrectionTest, HighYawCompensationHasBoundaries) {
  const auto parameters = defaultParameters();

  EXPECT_DOUBLE_EQ(calculateHighYawPitchOffsetDeg(6.0, parameters), 0.0);
  EXPECT_GT(calculateHighYawPitchOffsetDeg(8.0, parameters), 0.0);
  EXPECT_LT(calculateHighYawPitchOffsetDeg(8.0, parameters),
            calculateHighYawPitchOffsetDeg(10.0, parameters));
  EXPECT_DOUBLE_EQ(calculateHighYawPitchOffsetDeg(10.0, parameters), 1.2);
  EXPECT_DOUBLE_EQ(calculateHighYawPitchOffsetDeg(100.0, parameters), 1.2);
}

TEST(AimCorrectionTest, NegativeYawSpeedUsesItsMagnitude) {
  const auto parameters = defaultParameters();
  EXPECT_DOUBLE_EQ(calculateHighYawPitchOffsetDeg(-10.0, parameters), 1.2);
}

}  // namespace
}  // namespace fyt::auto_aim
