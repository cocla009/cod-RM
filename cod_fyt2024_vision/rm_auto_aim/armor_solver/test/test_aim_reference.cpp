#include "armor_solver/aim_reference.hpp"

#include <cassert>
#include <cmath>

using fyt::auto_aim::AimReferenceGenerator;
using fyt::auto_aim::ArmorPlannerConfig;
using fyt::auto_aim::ArmorPlannerInput;

int main() {
  ArmorPlannerInput input;
  input.center = Eigen::Vector3d(3.0, 0.1, 0.5);
  input.velocity = Eigen::Vector3d(0.1, 0.0, 0.0);
  input.yaw = 3.13;
  input.v_yaw = 0.5;
  input.radius_1 = 0.22;
  input.radius_2 = 0.26;
  input.dz = 0.1;
  input.armors_num = 4;

  ArmorPlannerConfig config;
  const auto result = AimReferenceGenerator::generate(
    input, config, 0.01, 20,
    [](const Eigen::Vector3d &position) { return position.norm() / 20.0; },
    [](const Eigen::Vector3d &position, double &pitch) {
      pitch = std::atan2(position.z(), position.head<2>().norm());
      return true;
    });
  assert(result.valid);
  assert(result.samples.size() == 20);
  for (const auto &sample : result.samples) {
    assert(std::isfinite(sample.yaw));
    assert(std::isfinite(sample.pitch));
    assert(std::isfinite(sample.yaw_velocity));
  }

  assert(std::abs(AimReferenceGenerator::unwrapNear(-3.13, 3.13) - 3.153185307179586) < 1e-6);

  // An omitted envelope must not disturb the historical output.
  assert(!result.exceeded_limits);
  assert(result.max_limit_excess == 0.0);

  const auto flight = [](const Eigen::Vector3d &position) { return position.norm() / 20.0; };
  const auto pitch_of = [](const Eigen::Vector3d &position, double &pitch) {
    pitch = std::atan2(position.z(), position.head<2>().norm());
    return true;
  };

  // A fast spinner drives the raw reference acceleration far past what the
  // gimbal can produce, so the generator must clamp it and say so.
  ArmorPlannerInput spin = input;
  spin.yaw = 0.0;
  spin.v_yaw = 6.0;
  spin.velocity = Eigen::Vector3d::Zero();

  fyt::auto_aim::AimReferenceLimits limits;
  limits.max_velocity = 8.0;
  limits.max_acceleration = 20.0;
  limits.max_jerk = 120.0;

  const double dt = 0.01;
  const auto raw = AimReferenceGenerator::generate(
    spin, config, dt, 20, flight, pitch_of, -1, 0.0);
  const auto clamped = AimReferenceGenerator::generate(
    spin, config, dt, 20, flight, pitch_of, -1, 0.0, limits);

  assert(raw.valid);
  assert(clamped.valid);
  assert(raw.samples.size() == clamped.samples.size());

  // The raw reference really does leave the envelope, otherwise this scenario
  // would not exercise the clamp.
  bool raw_exceeds = false;
  for (const auto &sample : raw.samples) {
    if (std::abs(sample.yaw_acceleration) > limits.max_acceleration) raw_exceeds = true;
  }
  assert(raw_exceeds);
  assert(!raw.exceeded_limits);

  assert(clamped.exceeded_limits);
  assert(clamped.max_limit_excess > 0.0);
  // A clamped reference cannot be a firing solution.
  assert(!clamped.safe);

  // Every clamped derivative sits inside the envelope, jerk included.
  for (std::size_t i = 0; i < clamped.samples.size(); ++i) {
    const auto &sample = clamped.samples[i];
    assert(std::abs(sample.yaw_velocity) <= limits.max_velocity + 1e-9);
    assert(std::abs(sample.pitch_velocity) <= limits.max_velocity + 1e-9);
    assert(std::abs(sample.yaw_acceleration) <= limits.max_acceleration + 1e-9);
    assert(std::abs(sample.pitch_acceleration) <= limits.max_acceleration + 1e-9);
    if (i > 0) {
      const double step = limits.max_jerk * dt + 1e-9;
      assert(std::abs(sample.yaw_acceleration -
                      clamped.samples[i - 1].yaw_acceleration) <= step);
      assert(std::abs(sample.pitch_acceleration -
                      clamped.samples[i - 1].pitch_acceleration) <= step);
    }
    assert(std::isfinite(sample.yaw));
    assert(std::isfinite(sample.pitch));
  }

  // Positions must agree with the clamped velocities they were rebuilt from.
  for (std::size_t i = 1; i < clamped.samples.size(); ++i) {
    const double expected = clamped.samples[i - 1].yaw +
      clamped.samples[i].yaw_velocity * dt;
    assert(std::abs(clamped.samples[i].yaw - expected) < 1e-9);
  }

  // A generous envelope must leave the reference alone.
  fyt::auto_aim::AimReferenceLimits loose;
  loose.max_velocity = 1e6;
  loose.max_acceleration = 1e6;
  loose.max_jerk = 1e9;
  const auto untouched = AimReferenceGenerator::generate(
    spin, config, dt, 20, flight, pitch_of, -1, 0.0, loose);
  assert(untouched.valid);
  assert(!untouched.exceeded_limits);
  assert(untouched.max_limit_excess == 0.0);
  return 0;
}
