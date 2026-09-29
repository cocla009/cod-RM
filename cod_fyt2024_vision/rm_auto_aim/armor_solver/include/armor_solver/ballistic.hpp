// Ported from sp_vision_25 tools/trajectory.{hpp,cpp}
// https://github.com/TongjiSuperPower/sp_vision_25 (MIT, see thirdparty/LICENSE-TinyMPC)
//
// Closed-form ballistic solution for a drag-free projectile. Returns the
// elevation angle and flight time together, which is what the planner needs at
// every prediction step.

#ifndef ARMOR_SOLVER_BALLISTIC_HPP_
#define ARMOR_SOLVER_BALLISTIC_HPP_

namespace fyt::auto_aim {

// Gravity used by the reference implementation. Matches the value the proven
// configuration was tuned against; do not change without re-tuning.
constexpr double kBALLISTIC_GRAVITY = 9.7833;

struct Ballistic {
  bool unsolvable = true;
  double fly_time = 0.0;
  // Elevation above the horizontal, positive when aiming upward. This is the
  // physical angle, independent of the world-frame pitch sign convention.
  double pitch = 0.0;

  // v0: muzzle speed [m/s]
  // d:  target horizontal distance [m]
  // h:  target height relative to the muzzle [m]
  //
  // Of the two solutions the flatter (shorter) one is kept.
  Ballistic(double v0, double d, double h);
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_SOLVER_BALLISTIC_HPP_
