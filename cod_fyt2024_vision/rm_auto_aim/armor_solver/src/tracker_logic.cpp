#include "armor_solver/tracker_logic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fyt::auto_aim::tracker_logic {
namespace {
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kTieEpsilon = 1e-12;
}

double normalizedYawResidual(const double measured, const double predicted) noexcept {
  if (!std::isfinite(measured) || !std::isfinite(predicted)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::abs(std::remainder(measured - predicted, kTwoPi));
}

bool betterMatch(const MatchCandidate &candidate,
                 const MatchCandidate &incumbent) noexcept {
  if (!std::isfinite(candidate.cost)) {
    return false;
  }
  if (!std::isfinite(incumbent.cost)) {
    return true;
  }
  if (std::abs(candidate.cost - incumbent.cost) > kTieEpsilon) {
    return candidate.cost < incumbent.cost;
  }
  if (std::abs(candidate.position_diff - incumbent.position_diff) > kTieEpsilon) {
    return candidate.position_diff < incumbent.position_diff;
  }
  if (std::abs(candidate.yaw_diff - incumbent.yaw_diff) > kTieEpsilon) {
    return candidate.yaw_diff < incumbent.yaw_diff;
  }
  if (candidate.armor_index != incumbent.armor_index) {
    return candidate.armor_index < incumbent.armor_index;
  }
  return candidate.observation_index < incumbent.observation_index;
}

int armorCountFor(const std::string &number, const std::string &type) noexcept {
  if (type == "large" && (number == "3" || number == "4" || number == "5")) {
    return 2;
  }
  if (number == "outpost") {
    return 3;
  }
  return 4;
}

bool validArmorIndex(const int armor_index, const int armor_count) noexcept {
  return armor_count >= 2 && armor_count <= 4 && armor_index >= 0 && armor_index < armor_count;
}

int remapArmorIndex(const double measured_yaw,
                    const double center_yaw,
                    const int armor_count) noexcept {
  if (!validArmorIndex(0, armor_count) || !std::isfinite(measured_yaw) ||
      !std::isfinite(center_yaw)) {
    return -1;
  }
  const double phase = std::remainder(measured_yaw - center_yaw, kTwoPi);
  int index = static_cast<int>(std::llround(phase * armor_count / kTwoPi));
  index %= armor_count;
  if (index < 0) {
    index += armor_count;
  }
  return index;
}

}  // namespace fyt::auto_aim::tracker_logic
