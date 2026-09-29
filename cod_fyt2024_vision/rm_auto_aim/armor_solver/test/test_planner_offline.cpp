// Offline validation for the gimbal planner. Needs no camera and no lower
// controller: a synthetic target is advanced exactly the way the tracker's EKF
// would advance it, and the planner is stepped at the vision rate.
//
// Adapted from sp_vision_25 tests/planner_test_offline.cpp, but with numeric
// assertions instead of a live plot.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "armor_solver/planner.hpp"
#include "rm_interfaces/msg/target.hpp"

namespace {

constexpr double kDt = 0.01;
constexpr double kCentreX = 3.0;
constexpr double kCentreY = 0.0;
constexpr double kCentreZ = 1.0;
constexpr double kRadius = 0.2;
constexpr double kDz = 0.1;
constexpr double kBulletSpeed = 23.0;
// Mirror the delay a real frame carries: the tracker's state is stamped when
// the armour image was captured.
constexpr double kTargetAge = 0.030;

int failures = 0;

void check(const bool condition, const char *what) {
  if (!condition) {
    std::printf("FAIL: %s\n", what);
    ++failures;
  } else {
    std::printf("ok  : %s\n", what);
  }
}

rm_interfaces::msg::Target makeTarget(const double yaw, const double v_yaw) {
  rm_interfaces::msg::Target target;
  target.tracking = true;
  target.armors_num = 4;
  target.position.x = kCentreX;
  target.position.y = kCentreY;
  target.position.z = kCentreZ;
  target.velocity.x = 0.0;
  target.velocity.y = 0.0;
  target.velocity.z = 0.0;
  target.yaw = yaw;
  target.v_yaw = v_yaw;
  target.radius_1 = kRadius;
  target.radius_2 = kRadius;
  target.dz = kDz;
  return target;
}

// Azimuth of the nearest armour, computed independently of the planner so a
// geometry mistake in the planner shows up as a mismatch.
double expectedNearestAzimuth(const double yaw) {
  double best_distance = std::numeric_limits<double>::infinity();
  double best_azimuth = 0.0;
  for (int i = 0; i < 4; ++i) {
    const double angle = yaw + i * M_PI / 2.0;
    const double x = kCentreX - kRadius * std::cos(angle);
    const double y = kCentreY - kRadius * std::sin(angle);
    const double distance = std::hypot(x, y);
    if (distance < best_distance) {
      best_distance = distance;
      best_azimuth = std::atan2(y, x);
    }
  }
  return best_azimuth;
}

struct RunStats {
  int steps = 0;
  int control_ok = 0;
  int fire_steps = 0;
  double max_yaw_acc = 0.0;
  double max_pitch_acc = 0.0;
  double max_yaw_error = 0.0;
  double max_step_ms = 0.0;
  double total_ms = 0.0;
  bool finite = true;
};

RunStats run(fyt::auto_aim::Planner &planner, const double v_yaw, const int steps) {
  RunStats stats;
  rm_interfaces::msg::Target target = makeTarget(0.0, v_yaw);

  for (int i = 0; i < steps; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    const auto plan = planner.plan(target, kTargetAge, kBulletSpeed);
    const auto t1 = std::chrono::steady_clock::now();
    const double step_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    stats.max_step_ms = std::max(stats.max_step_ms, step_ms);
    stats.total_ms += step_ms;
    stats.steps = i + 1;

    if (!std::isfinite(plan.yaw) || !std::isfinite(plan.pitch) ||
        !std::isfinite(plan.yaw_vel) || !std::isfinite(plan.pitch_vel) ||
        !std::isfinite(plan.yaw_acc) || !std::isfinite(plan.pitch_acc)) {
      stats.finite = false;
      break;
    }
    if (!plan.control) break;
    ++stats.control_ok;
    stats.max_yaw_acc = std::max(stats.max_yaw_acc, std::abs(plan.yaw_acc));
    stats.max_pitch_acc = std::max(stats.max_pitch_acc, std::abs(plan.pitch_acc));
    if (plan.fire) ++stats.fire_steps;

    // The plan aims at the armour that will be nearest at the impact instant,
    // which trails the current nearest armour by the flight time. Allow that
    // lead plus the solver's transient.
    const double lead = v_yaw * (kTargetAge + 0.15);
    const double expected = expectedNearestAzimuth(target.yaw + lead);
    stats.max_yaw_error =
      std::max(stats.max_yaw_error, std::abs(std::remainder(plan.yaw - expected, 2 * M_PI)));

    // Advance the target the way the EKF would.
    target.yaw += v_yaw * kDt;
    target.yaw = std::remainder(target.yaw, 2 * M_PI);
  }
  return stats;
}

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_planner_offline");
  fyt::auto_aim::Planner planner(*node);

  // ── Static target ────────────────────────────────────────────────────────
  {
    const auto stats = run(planner, 0.0, 200);
    std::printf("\n[static] steps=%d control_ok=%d fire=%d max|yaw_acc|=%.2f "
                "max_yaw_error=%.4f max_step=%.2fms\n",
                stats.steps, stats.control_ok, stats.fire_steps, stats.max_yaw_acc,
                stats.max_yaw_error, stats.max_step_ms);
    check(stats.finite, "static: outputs are finite");
    check(stats.control_ok == stats.steps, "static: planner stays valid every step");
    check(stats.max_yaw_acc <= 50.0 + 1e-6, "static: yaw acceleration stays within bound");
    check(stats.max_pitch_acc <= 100.0 + 1e-6, "static: pitch acceleration stays within bound");
    check(stats.max_yaw_error < 0.02, "static: aims at the nearest armour");
    check(stats.fire_steps > stats.steps / 2, "static: fires in steady state");
  }

  // ── Rotating target ──────────────────────────────────────────────────────
  for (const double v_yaw : {1.0, 3.0, 5.0}) {
    const auto stats = run(planner, v_yaw, 400);
    std::printf("\n[v_yaw=%.1f] steps=%d control_ok=%d fire=%d max|yaw_acc|=%.2f "
                "max_yaw_error=%.4f max_step=%.2fms\n",
                v_yaw, stats.steps, stats.control_ok, stats.fire_steps, stats.max_yaw_acc,
                stats.max_yaw_error, stats.max_step_ms);
    check(stats.finite, "rotating: outputs are finite");
    check(stats.control_ok == stats.steps, "rotating: planner stays valid every step");
    check(stats.max_yaw_acc <= 50.0 + 1e-6, "rotating: yaw acceleration stays within bound");
    check(stats.max_yaw_error < 0.15, "rotating: tracks the predicted armour");
    check(stats.max_step_ms < 3.0, "rotating: per-call cost within budget");
  }

  rclcpp::shutdown();
  std::printf("\n%s (%d failure(s))\n", failures == 0 ? "PASS" : "FAIL", failures);
  return failures == 0 ? 0 : 1;
}
