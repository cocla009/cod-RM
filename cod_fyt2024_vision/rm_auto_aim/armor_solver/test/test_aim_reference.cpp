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
  return 0;
}
