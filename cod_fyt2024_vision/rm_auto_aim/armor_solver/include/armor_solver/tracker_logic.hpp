#ifndef ARMOR_SOLVER_TRACKER_LOGIC_HPP_
#define ARMOR_SOLVER_TRACKER_LOGIC_HPP_

#include <cstddef>
#include <string>

namespace fyt::auto_aim::tracker_logic {

struct MatchCandidate {
  std::size_t observation_index = 0;
  int armor_index = 0;
  double cost = 0.0;
  double position_diff = 0.0;
  double yaw_diff = 0.0;
};

// Compare candidates by the normalized joint gate cost, then stable geometric
// tie breakers. This makes duplicate observations deterministic.
bool betterMatch(const MatchCandidate &candidate,
                 const MatchCandidate &incumbent) noexcept;

double normalizedYawResidual(double measured, double predicted) noexcept;

int armorCountFor(const std::string &number, const std::string &type) noexcept;

bool validArmorIndex(int armor_index, int armor_count) noexcept;

// Pick the index whose nominal yaw is closest to the measured armor yaw.
int remapArmorIndex(double measured_yaw, double center_yaw, int armor_count) noexcept;

}  // namespace fyt::auto_aim::tracker_logic

#endif  // ARMOR_SOLVER_TRACKER_LOGIC_HPP_
