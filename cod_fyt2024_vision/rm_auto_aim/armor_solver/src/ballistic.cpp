#include "armor_solver/ballistic.hpp"

#include <cmath>

namespace fyt::auto_aim {

Ballistic::Ballistic(double v0, double d, double h) {
  if (!(v0 > 0.0) || !(d > 0.0) || !std::isfinite(v0) || !std::isfinite(d) ||
      !std::isfinite(h)) {
    unsolvable = true;
    return;
  }

  // Solve  h = d*tan(theta) - g*d^2 / (2*v0^2*cos^2(theta))  for tan(theta).
  const double a = kBALLISTIC_GRAVITY * d * d / (2.0 * v0 * v0);
  const double b = -d;
  const double c = a + h;
  const double delta = b * b - 4.0 * a * c;

  if (delta < 0.0) {
    unsolvable = true;
    return;
  }

  const double tan_pitch_1 = (-b + std::sqrt(delta)) / (2.0 * a);
  const double tan_pitch_2 = (-b - std::sqrt(delta)) / (2.0 * a);
  const double pitch_1 = std::atan(tan_pitch_1);
  const double pitch_2 = std::atan(tan_pitch_2);
  const double cos_1 = std::cos(pitch_1);
  const double cos_2 = std::cos(pitch_2);

  // A near-vertical solution has an unusable flight time; treat it as no
  // solution rather than emitting a huge value.
  if (!(std::abs(cos_1) > 1e-9) || !(std::abs(cos_2) > 1e-9)) {
    unsolvable = true;
    return;
  }

  const double t_1 = d / (v0 * cos_1);
  const double t_2 = d / (v0 * cos_2);

  unsolvable = false;
  const bool flat_is_first = t_1 < t_2;
  pitch = flat_is_first ? pitch_1 : pitch_2;
  fly_time = flat_is_first ? t_1 : t_2;

  if (!std::isfinite(pitch) || !std::isfinite(fly_time) || fly_time <= 0.0) {
    unsolvable = true;
  }
}

}  // namespace fyt::auto_aim
