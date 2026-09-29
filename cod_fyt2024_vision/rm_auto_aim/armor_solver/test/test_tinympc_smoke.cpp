// Smoke test for the vendored TinyMPC library.
//
// Mirrors the problem shape used by the gimbal planner (see
// sp_vision_25 tasks/auto_aim/planner/planner.cpp setup_yaw_solver):
// double integrator, position/velocity state, acceleration input, horizon 100,
// acceleration-only bound constraints, max_iter 10.
//
// Behaviours pinned down here, because the planner depends on them:
//
//  1. solution->x / solution->u hold the *projected* (constraint-satisfying)
//     trajectory; work->x / work->u are the raw ADMM iterates. Read the
//     solution, never the workspace.
//  2. solution->solved is NOT a usable health signal at max_iter = 10: with
//     abs_pri_tol = 1e-3 the solver routinely exhausts its iteration budget
//     while still returning an accurate plan. Judge the solution by its
//     residuals/quality instead.
//  3. solve() does not reset the workspace, so consecutive solves are warm
//     started. Each case below therefore uses a fresh solver.
//  4. When the initial state and reference are both zero, all four ADMM
//     residuals are zero on the first iteration and the solver returns an
//     all-zero plan immediately. Harmless for a static target (the plan is
//     re-anchored to the current aim angle outside the solver), but a zero
//     result must not be read as a failure.

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "tinympc/tiny_api.hpp"

namespace {

constexpr double kDt = 0.01;
constexpr int kHorizon = 100;
constexpr int kHalfHorizon = 50;
constexpr double kMaxAcc = 50.0;  // rad/s^2, matches configs/standard4.yaml
constexpr int kMaxIter = 10;

int failures = 0;

void check(const bool condition, const char *what) {
  if (!condition) {
    std::printf("FAIL: %s\n", what);
    ++failures;
  } else {
    std::printf("ok  : %s\n", what);
  }
}

TinySolver *makeSolver() {
  TinySolver *solver = nullptr;
  Eigen::MatrixXd A{{1.0, kDt}, {0.0, 1.0}};
  Eigen::MatrixXd B{{0.0}, {kDt}};
  Eigen::MatrixXd f = Eigen::MatrixXd::Zero(2, 1);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(2, 2);
  Q(0, 0) = 9e6;  // matches Tongji: position weighted, velocity unweighted
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);

  if (tiny_setup(&solver, A, B, f, Q, R, 1.0, 2, 1, kHorizon, 0) != 0 || solver == nullptr) {
    return nullptr;
  }
  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, kHorizon, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, kHorizon, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, kHorizon - 1, -kMaxAcc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, kHorizon - 1, kMaxAcc);
  tiny_set_bound_constraints(solver, x_min, x_max, u_min, u_max);
  solver->settings->max_iter = kMaxIter;
  return solver;
}

void destroySolver(TinySolver *solver) {
  delete solver->solution;
  delete solver->cache;
  delete solver->settings;
  delete solver->work;
  delete solver;
}

// Starts displaced at -0.25 rad with zero velocity, accelerates at 2.5 rad/s^2
// up to 1 rad/s, then holds. Well inside the acceleration bound.
Eigen::Matrix<double, 2, kHorizon> makeTrackableReference() {
  Eigen::Matrix<double, 2, kHorizon> reference;
  const double ramp_time = 0.4;
  const double accel = 1.0 / ramp_time;
  for (int i = 0; i < kHorizon; ++i) {
    const double t = static_cast<double>(i) * kDt;
    if (t < ramp_time) {
      reference(1, i) = accel * t;
      reference(0, i) = -0.25 + 0.5 * accel * t * t;
    } else {
      reference(1, i) = 1.0;
      reference(0, i) = -0.25 + 0.5 * accel * ramp_time * ramp_time + 1.0 * (t - ramp_time);
    }
  }
  return reference;
}

}  // namespace

int main() {
  // ── Case 1: trackable reference ──────────────────────────────────────────
  {
    TinySolver *solver = makeSolver();
    check(solver != nullptr, "tiny_setup succeeds");
    if (solver == nullptr) {
      std::printf("\n%d check(s) failed\n", failures);
      return 1;
    }

    const auto reference = makeTrackableReference();
    Eigen::Vector2d x0;
    x0 << reference(0, 0), reference(1, 0);
    tiny_set_x0(solver, x0);
    solver->work->Xref = reference;
    tiny_solve(solver);

    double track_error = 0.0;
    double max_acc = 0.0;
    bool finite = true;
    for (int i = 0; i < kHorizon; ++i) {
      const double px = solver->solution->x(0, i);
      const double vx = solver->solution->x(1, i);
      if (!std::isfinite(px) || !std::isfinite(vx)) finite = false;
      track_error = std::max(track_error, std::abs(px - reference(0, i)));
    }
    for (int i = 0; i < kHorizon - 1; ++i) {
      max_acc = std::max(max_acc, std::abs(solver->solution->u(0, i)));
    }
    std::printf("     track_error = %.3e, max|u| = %.4f, iter = %d\n",
                track_error, max_acc, solver->solution->iter);
    check(finite, "solution is finite");
    check(track_error < 5e-3, "trackable reference is tracked");
    check(max_acc <= kMaxAcc + 1e-9, "input respects the acceleration bound");
    destroySolver(solver);
  }

  // ── Case 2: infeasible step, i.e. the armour-switch case ─────────────────
  {
    TinySolver *solver = makeSolver();
    const auto reference = makeTrackableReference();
    auto step_reference = reference;
    for (int i = kHalfHorizon; i < kHorizon; ++i) {
      step_reference(0, i) += 0.5;  // instant 0.5 rad jump at the output index
      step_reference(1, i) = 0.0;
    }
    Eigen::Vector2d x0;
    x0 << reference(0, 0), reference(1, 0);
    tiny_set_x0(solver, x0);
    solver->work->Xref = step_reference;
    tiny_solve(solver);

    double max_acc = 0.0;
    bool finite = true;
    for (int i = 0; i < kHorizon - 1; ++i) {
      const double u = solver->solution->u(0, i);
      max_acc = std::max(max_acc, std::abs(u));
      if (!std::isfinite(u)) finite = false;
    }
    std::printf("     step: max|u| = %.4f\n", max_acc);
    check(finite, "step reference: solution is finite");
    check(max_acc <= kMaxAcc + 1e-9, "step reference: input respects the bound");

    // Anticipation: ahead of the jump the plan must already differ from the
    // pre-jump reference, otherwise the switch would only start at the step.
    const int probe = kHalfHorizon - 10;
    const double planned_vel = solver->solution->x(1, probe);
    const double reference_vel = step_reference(1, probe);
    const double planned_pos = solver->solution->x(0, probe);
    const double reference_pos = step_reference(0, probe);
    std::printf("     pre-jump probe %d: plan(%.4f, %.4f) vs ref(%.4f, %.4f)\n",
                probe, planned_pos, planned_vel, reference_pos, reference_vel);
    check(std::abs(planned_vel - reference_vel) > 1e-3 ||
            std::abs(planned_pos - reference_pos) > 1e-3,
          "solver starts the transition before the jump");
    destroySolver(solver);
  }

  // ── Case 3: all-zero reference on a fresh solver (static target) ─────────
  {
    TinySolver *solver = makeSolver();
    Eigen::Matrix<double, 2, kHorizon> zero_ref = Eigen::Matrix<double, 2, kHorizon>::Zero();
    Eigen::Vector2d zero_x0 = Eigen::Vector2d::Zero();
    tiny_set_x0(solver, zero_x0);
    solver->work->Xref = zero_ref;
    tiny_solve(solver);

    double max_acc = 0.0;
    for (int i = 0; i < kHorizon - 1; ++i) {
      max_acc = std::max(max_acc, std::abs(solver->solution->u(0, i)));
    }
    std::printf("     zero reference: max|u| = %.4f, solved = %d, iter = %d\n",
                max_acc, solver->solution->solved, solver->solution->iter);
    check(max_acc == 0.0, "zero reference returns a zero plan (documented behaviour)");
    destroySolver(solver);
  }

  std::printf("\n%s (%d failure(s))\n", failures == 0 ? "PASS" : "FAIL", failures);
  return failures == 0 ? 0 : 1;
}
