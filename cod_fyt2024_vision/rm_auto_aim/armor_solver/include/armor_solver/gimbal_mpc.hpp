#ifndef ARMOR_SOLVER_GIMBAL_MPC_HPP_
#define ARMOR_SOLVER_GIMBAL_MPC_HPP_

#include <cstddef>
#include <vector>

namespace fyt::auto_aim {

struct GimbalAxisState {
  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
};

struct GimbalAxisReference {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> acceleration;
};

struct GimbalMpcConfig {
  double dt = 0.01;
  std::size_t horizon = 50;
  double position_weight = 40.0;
  double velocity_weight = 4.0;
  double acceleration_weight = 0.2;
  double jerk_weight = 0.05;
  double terminal_weight = 80.0;
  double max_velocity = 12.0;
  double max_acceleration = 50.0;
  double max_jerk = 5000.0;
  double step_size = 0.01;
  double convergence_tolerance = 1e-5;
  int max_iterations = 40;
};

struct GimbalAxisResult {
  bool valid = false;
  bool converged = false;
  std::size_t iterations = 0;
  double objective = 0.0;
  double max_constraint_violation = 0.0;
  GimbalAxisState command;
  std::vector<GimbalAxisState> predicted;
};

struct GimbalMpcInput {
  GimbalAxisState yaw;
  GimbalAxisState pitch;
  GimbalAxisReference yaw_reference;
  GimbalAxisReference pitch_reference;
};

struct GimbalMpcResult {
  bool valid = false;
  bool converged = false;
  GimbalAxisResult yaw;
  GimbalAxisResult pitch;
};

class GimbalMpc {
public:
  explicit GimbalMpc(GimbalMpcConfig config = {});

  void setConfig(const GimbalMpcConfig &config) noexcept;
  const GimbalMpcConfig &config() const noexcept { return config_; }
  void reset() noexcept;

  GimbalMpcResult solve(const GimbalMpcInput &input) noexcept;

private:
  GimbalMpcConfig config_;
  std::vector<double> yaw_controls_;
  std::vector<double> pitch_controls_;

  GimbalAxisResult solveAxis(const GimbalAxisState &initial,
                             const GimbalAxisReference &reference,
                             std::vector<double> &warm_controls) const noexcept;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_SOLVER_GIMBAL_MPC_HPP_
