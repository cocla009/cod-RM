// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ARMOR_SOLVER_AIM_CORRECTION_HPP_
#define ARMOR_SOLVER_AIM_CORRECTION_HPP_

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim {

struct AimCorrectionParameters {
  double yaw_offset_deg;
  double pitch_offset_deg;
  double high_yaw_threshold;
  double high_yaw_reference;
  double max_high_yaw_pitch_offset_deg;
};

inline double calculateHighYawPitchOffsetDeg(
  const double target_v_yaw, const AimCorrectionParameters &parameters) noexcept {
  const double speed = std::abs(target_v_yaw);
  const double threshold = std::max(0.0, parameters.high_yaw_threshold);
  const double reference = std::max(threshold + 1e-6, parameters.high_yaw_reference);
  const double max_offset = std::max(0.0, parameters.max_high_yaw_pitch_offset_deg);
  if (speed <= threshold || max_offset <= 0.0) {
    return 0.0;
  }

  double normalized = 1.0;
  if (threshold > 1e-6 && reference > threshold + 1e-6) {
    normalized = std::log(speed / threshold) / std::log(reference / threshold);
  }
  normalized = std::clamp(normalized, 0.0, 1.0);
  return normalized * max_offset;
}

inline void applyAimCorrection(const double target_v_yaw,
                               const AimCorrectionParameters &parameters,
                               double &yaw,
                               double &pitch) noexcept {
  yaw = std::remainder(yaw + parameters.yaw_offset_deg * M_PI / 180.0, 2.0 * M_PI);
  pitch += parameters.pitch_offset_deg * M_PI / 180.0;
  pitch -= calculateHighYawPitchOffsetDeg(target_v_yaw, parameters) * M_PI / 180.0;
}

}  // namespace fyt::auto_aim

#endif  // ARMOR_SOLVER_AIM_CORRECTION_HPP_
