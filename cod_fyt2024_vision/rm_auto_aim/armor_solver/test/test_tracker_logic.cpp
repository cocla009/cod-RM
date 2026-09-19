#include "armor_solver/tracker_logic.hpp"

#include <cassert>
#include <cmath>
#include <limits>

using fyt::auto_aim::tracker_logic::MatchCandidate;
using fyt::auto_aim::tracker_logic::armorCountFor;
using fyt::auto_aim::tracker_logic::betterMatch;
using fyt::auto_aim::tracker_logic::normalizedYawResidual;
using fyt::auto_aim::tracker_logic::remapArmorIndex;
using fyt::auto_aim::tracker_logic::validArmorIndex;

int main() {
  const double pi = 3.14159265358979323846;
  assert(normalizedYawResidual(-pi + 0.01, pi - 0.01) < 0.03);
  assert(std::isinf(normalizedYawResidual(std::numeric_limits<double>::quiet_NaN(), 0.0)));

  MatchCandidate best{4, 2, 1.0, 0.4, 0.2};
  MatchCandidate lower_position{1, 3, 1.0, 0.2, 0.3};
  assert(betterMatch(lower_position, best));
  MatchCandidate lower_index{2, 1, 1.0, 0.2, 0.3};
  assert(betterMatch(lower_index, lower_position));
  MatchCandidate invalid{0, 0, std::numeric_limits<double>::infinity(), 0.0, 0.0};
  assert(!betterMatch(invalid, best));

  assert(armorCountFor("3", "large") == 2);
  assert(armorCountFor("outpost", "small") == 3);
  assert(armorCountFor("1", "small") == 4);
  assert(validArmorIndex(0, 2) && validArmorIndex(3, 4));
  assert(!validArmorIndex(4, 4) && !validArmorIndex(-1, 2));

  assert(remapArmorIndex(pi - 0.01, -pi + 0.01, 4) == 0);
  assert(remapArmorIndex(0.5 * pi, 0.0, 4) == 1);
  assert(remapArmorIndex(0.0, 0.0, 3) == 0);
  assert(remapArmorIndex(0.0, 0.0, 5) == -1);
  return 0;
}
