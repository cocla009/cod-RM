#include "armor_solver/planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// tinympc/types.hpp does `using namespace Eigen;`, so it must never reach a
// public header. It is confined to this translation unit.
#include "tinympc/tiny_api.hpp"

#include "armor_solver/aim_correction.hpp"
#include "armor_solver/ballistic.hpp"
#include "armor_solver/target_ekf_model.hpp"

namespace fyt::auto_aim {
namespace {

constexpr double kDt = 0.01;
constexpr int kHalfHorizon = 50;
constexpr int kHorizon = kHalfHorizon * 2;
// The fire decision is taken this many steps past the impact index, covering
// the trigger-to-shot delay.
constexpr int kFireOffset = 2;

constexpr double kMinBulletSpeed = 10.0;
constexpr double kMaxBulletSpeed = 25.0;
constexpr double kFallbackBulletSpeed = 22.0;

using Reference = Eigen::Matrix<double, 4, kHorizon>;

double wrapAngle(const double angle) noexcept {
  return std::remainder(angle, 2.0 * M_PI);
}

struct AimResult {
  bool valid = false;
  double yaw = 0.0;
  double pitch = 0.0;
  double distance = 0.0;
};

}  // namespace

struct Planner::Impl {
  TinySolver *yaw_solver = nullptr;
  TinySolver *pitch_solver = nullptr;

  double yaw_offset_deg = 0.0;
  double pitch_offset_deg = 0.0;
  double high_yaw_threshold = 0.0;
  double high_yaw_reference = 0.0;
  double max_high_yaw_pitch_offset_deg = 0.0;

  double fire_thresh = 0.003;
  double decision_speed = 8.0;
  double high_speed_delay_time = 0.030;
  double low_speed_delay_time = 0.015;

  ~Impl();
};

namespace {

// Build the solver for one axis: double integrator, acceleration input, bound
// constraints on the input only. Mirrors setup_yaw_solver / setup_pitch_solver.
TinySolver *makeSolver(const std::vector<double> &Q_diag, const double R, const double max_acc) {
  if (Q_diag.size() != 2) return nullptr;

  TinySolver *solver = nullptr;
  Eigen::MatrixXd A{{1.0, kDt}, {0.0, 1.0}};
  Eigen::MatrixXd B{{0.0}, {kDt}};
  Eigen::MatrixXd f = Eigen::MatrixXd::Zero(2, 1);

  Eigen::Matrix<double, 2, 1> q_vector;
  q_vector << Q_diag[0], Q_diag[1];
  Eigen::Matrix<double, 1, 1> r_vector;
  r_vector << R;

  if (tiny_setup(&solver, A, B, f, q_vector.asDiagonal(), r_vector.asDiagonal(), 1.0, 2, 1,
                 kHorizon, 0) != 0 ||
      solver == nullptr) {
    return nullptr;
  }

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, kHorizon, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, kHorizon, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, kHorizon - 1, -max_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, kHorizon - 1, max_acc);
  tiny_set_bound_constraints(solver, x_min, x_max, u_min, u_max);

  solver->settings->max_iter = 10;
  return solver;
}

void destroySolver(TinySolver *solver) {
  if (solver == nullptr) return;
  delete solver->solution;
  delete solver->cache;
  delete solver->settings;
  delete solver->work;
  delete solver;
}

}  // namespace

Planner::Impl::~Impl() {
  destroySolver(yaw_solver);
  destroySolver(pitch_solver);
}

Planner::Planner(rclcpp::Node &node) : impl_(std::make_unique<Impl>()) {
  impl_->yaw_offset_deg = node.declare_parameter("solver.yaw_offset", 0.0);
  impl_->pitch_offset_deg = node.declare_parameter("solver.pitch_offset", 0.0);
  impl_->high_yaw_threshold =
    node.declare_parameter("solver.high_yaw_pitch_compensation_threshold", 6.0);
  impl_->high_yaw_reference =
    node.declare_parameter("solver.high_yaw_pitch_compensation_reference", 10.0);
  impl_->max_high_yaw_pitch_offset_deg =
    node.declare_parameter("solver.max_high_yaw_pitch_offset", 1.2);

  impl_->fire_thresh = node.declare_parameter("solver.planner.fire_thresh", 0.003);
  impl_->decision_speed = node.declare_parameter("solver.planner.decision_speed", 8.0);
  impl_->high_speed_delay_time =
    node.declare_parameter("solver.planner.high_speed_delay_time", 0.030);
  impl_->low_speed_delay_time =
    node.declare_parameter("solver.planner.low_speed_delay_time", 0.015);

  const auto q_yaw = node.declare_parameter("solver.planner.Q_yaw", std::vector<double>{9e6, 0.0});
  const auto q_pitch =
    node.declare_parameter("solver.planner.Q_pitch", std::vector<double>{9e6, 0.0});
  const double r_yaw = node.declare_parameter("solver.planner.R_yaw", 1.0);
  const double r_pitch = node.declare_parameter("solver.planner.R_pitch", 1.0);
  const double max_yaw_acc = node.declare_parameter("solver.planner.max_yaw_acc", 50.0);
  const double max_pitch_acc = node.declare_parameter("solver.planner.max_pitch_acc", 100.0);

  impl_->yaw_solver = makeSolver(q_yaw, r_yaw, max_yaw_acc);
  impl_->pitch_solver = makeSolver(q_pitch, r_pitch, max_pitch_acc);
  if (impl_->yaw_solver == nullptr || impl_->pitch_solver == nullptr) {
    throw std::runtime_error("Failed to set up the TinyMPC solvers");
  }
}

Planner::~Planner() = default;

namespace {

// Rebuild the 11-state vector from the tracked target message.
// Message radius_2 is the absolute radius of the second armour pair, while the
// EKF state stores the difference from radius_1.
bool toStateVector(const rm_interfaces::msg::Target &target, Eigen::VectorXd &state) {
  if (target.armors_num < 2 || target.armors_num > 4) return false;
  if (!(target.radius_1 > 0.0)) return false;

  state = Eigen::VectorXd::Zero(TargetEkfModel::kStateSize);
  state(XC) = target.position.x;
  state(VXC) = target.velocity.x;
  state(YC) = target.position.y;
  state(VYC) = target.velocity.y;
  state(Z) = target.position.z;
  state(VZ) = target.velocity.z;
  state(YAW) = target.yaw;
  state(VYAW) = target.v_yaw;
  state(R1) = target.radius_1;
  state(DELTA_R) = target.radius_2 - target.radius_1;
  state(DZ) = target.dz;
  return state.allFinite();
}

AimResult aim(const Eigen::VectorXd &state,
              const int armors_num,
              const double bullet_speed,
              const AimCorrectionParameters &correction) {
  AimResult result;

  // Nearest armour by horizontal distance, matching the reference planner.
  double min_distance = std::numeric_limits<double>::infinity();
  Eigen::Vector3d best = Eigen::Vector3d::Zero();
  for (int i = 0; i < armors_num; ++i) {
    Eigen::Vector3d position;
    if (!TargetEkfModel::armorPosition(state, i, armors_num, position)) continue;
    const double distance = position.head<2>().norm();
    if (distance < min_distance) {
      min_distance = distance;
      best = position;
    }
  }
  if (!(min_distance > 0.0) || !best.allFinite()) return result;

  const Ballistic ballistic(bullet_speed, min_distance, best.z());
  if (ballistic.unsolvable) return result;

  double yaw = std::atan2(best.y(), best.x());
  // [deviation] The reference implementation negates the ballistic pitch
  // because its world frame has upward pitch negative. Ours is upward positive
  // (see calcYawAndPitch and the serial driver's pitch sign), and the rune
  // solver shares the same firmware convention, so no negation here.
  double pitch = ballistic.pitch;
  applyAimCorrection(state(VYAW), correction, yaw, pitch);

  if (!std::isfinite(yaw) || !std::isfinite(pitch)) return result;

  result.valid = true;
  result.yaw = yaw;
  result.pitch = pitch;
  result.distance = min_distance;
  return result;
}

}  // namespace

Plan Planner::plan(const rm_interfaces::msg::Target &target,
                   const double target_age,
                   const double bullet_speed) {
  Plan plan;
  Impl &impl = *impl_;

  Eigen::VectorXd state;
  if (!toStateVector(target, state)) return plan;

  double speed = bullet_speed;
  if (!std::isfinite(speed) || speed < kMinBulletSpeed || speed > kMaxBulletSpeed) {
    speed = kFallbackBulletSpeed;
  }

  const AimCorrectionParameters correction{impl.yaw_offset_deg,
                                           impl.pitch_offset_deg,
                                           impl.high_yaw_threshold,
                                           impl.high_yaw_reference,
                                           impl.max_high_yaw_pitch_offset_deg};

  // Advance the state from the message stamp to "now". The reference planner
  // works from an always-current state and applies only its own delay; ours has
  // to make up the age of the message first.
  const double pipeline_delay = std::abs(target.v_yaw) > impl.decision_speed
                                  ? impl.high_speed_delay_time
                                  : impl.low_speed_delay_time;
  const double age = (std::isfinite(target_age) ? std::max(0.0, target_age) : 0.0) +
                     pipeline_delay;
  Eigen::VectorXd now_state;
  if (!TargetEkfModel::propagate(state, age, now_state)) return plan;

  // Estimate flight time from the nearest armour, then advance the state to the
  // impact instant: the whole prediction window is anchored there.
  const AimResult now_aim = aim(now_state, target.armors_num, speed, correction);
  if (!now_aim.valid) return plan;
  const Ballistic lead(Ballistic(speed, now_aim.distance, now_state(Z)));
  const double fly_time = lead.fly_time;
  if (!std::isfinite(fly_time) || fly_time <= 0.0) return plan;

  Eigen::VectorXd impact_state;
  if (!TargetEkfModel::propagate(now_state, fly_time, impact_state)) return plan;

  const AimResult impact_aim = aim(impact_state, target.armors_num, speed, correction);
  if (!impact_aim.valid) return plan;
  const double yaw0 = impact_aim.yaw;

  // Reference window spanning [-0.5 s, +0.5 s] around the impact instant, with
  // index kHalfHorizon at the impact. Armour switches show up as jumps in this
  // reference; smoothing them is the solver's job, not ours.
  Eigen::VectorXd previous_state;
  if (!TargetEkfModel::propagate(impact_state, -kDt * (kHalfHorizon + 1), previous_state)) {
    return plan;
  }
  AimResult previous = aim(previous_state, target.armors_num, speed, correction);

  Eigen::VectorXd current_state;
  if (!TargetEkfModel::propagate(impact_state, -kDt * kHalfHorizon, current_state)) return plan;
  AimResult current = aim(current_state, target.armors_num, speed, correction);
  if (!current.valid || !previous.valid) return plan;

  Reference reference;
  for (int i = 0; i < kHorizon; ++i) {
    Eigen::VectorXd next_state;
    if (!TargetEkfModel::propagate(current_state, kDt, next_state)) return plan;
    const AimResult next = aim(next_state, target.armors_num, speed, correction);
    if (!next.valid) return plan;

    const double yaw_velocity = wrapAngle(next.yaw - previous.yaw) / (2.0 * kDt);
    const double pitch_velocity = (next.pitch - previous.pitch) / (2.0 * kDt);

    reference.col(i) << wrapAngle(current.yaw - yaw0), yaw_velocity, current.pitch, pitch_velocity;

    previous = current;
    current_state = next_state;
    current = next;
  }

  // Solve both axes. The initial state is the start of the reference, matching
  // the reference planner: this is feed-forward trajectory shaping, not a
  // closed loop on the measured gimbal angle.
  Eigen::Vector2d initial;
  initial << reference(0, 0), reference(1, 0);
  tiny_set_x0(impl.yaw_solver, initial);
  impl.yaw_solver->work->Xref = reference.block(0, 0, 2, kHorizon);
  tiny_solve(impl.yaw_solver);

  initial << reference(2, 0), reference(3, 0);
  tiny_set_x0(impl.pitch_solver, initial);
  impl.pitch_solver->work->Xref = reference.block(2, 0, 2, kHorizon);
  tiny_solve(impl.pitch_solver);

  // Read the projected solution, not the raw ADMM iterates.
  const auto &yaw_solution = impl.yaw_solver->solution;
  const auto &pitch_solution = impl.pitch_solver->solution;

  plan.target_yaw = wrapAngle(reference(0, kHalfHorizon) + yaw0);
  plan.target_pitch = reference(2, kHalfHorizon);

  plan.yaw = wrapAngle(yaw_solution->x(0, kHalfHorizon) + yaw0);
  plan.yaw_vel = yaw_solution->x(1, kHalfHorizon);
  plan.yaw_acc = yaw_solution->u(0, kHalfHorizon);

  plan.pitch = pitch_solution->x(0, kHalfHorizon);
  plan.pitch_vel = pitch_solution->x(1, kHalfHorizon);
  plan.pitch_acc = pitch_solution->u(0, kHalfHorizon);

  plan.distance = now_aim.distance;

  if (!std::isfinite(plan.yaw) || !std::isfinite(plan.pitch) ||
      !std::isfinite(plan.yaw_vel) || !std::isfinite(plan.pitch_vel) ||
      !std::isfinite(plan.yaw_acc) || !std::isfinite(plan.pitch_acc)) {
    return Plan{};
  }

  // Fire when the plan is still tracking the reference just past the impact
  // instant: a large gap means the trajectory is mid-switch and the shot would
  // land wide.
  const double yaw_gap =
    std::abs(wrapAngle(reference(0, kHalfHorizon + kFireOffset) -
                       yaw_solution->x(0, kHalfHorizon + kFireOffset)));
  const double pitch_gap =
    std::abs(reference(2, kHalfHorizon + kFireOffset) -
             pitch_solution->x(0, kHalfHorizon + kFireOffset));
  plan.fire = std::hypot(yaw_gap, pitch_gap) < impl.fire_thresh;

  plan.control = true;
  return plan;
}

}  // namespace fyt::auto_aim
