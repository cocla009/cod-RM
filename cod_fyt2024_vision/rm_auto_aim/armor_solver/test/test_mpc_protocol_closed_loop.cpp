#include "armor_solver/gimbal_mpc.hpp"
#include "armor_solver/gimbal_simulator.hpp"
#include "rm_serial_driver/gimbal_command_receiver.hpp"
#include "rm_serial_driver/gimbal_packet_v2.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

namespace {

using fyt::auto_aim::GimbalAxisReference;
using fyt::auto_aim::GimbalMpc;
using fyt::auto_aim::GimbalMpcConfig;
using fyt::auto_aim::GimbalMpcInput;
using fyt::auto_aim::GimbalSimulator;
using fyt::serial_driver::FixedPacket;
using fyt::serial_driver::GimbalActuatorCommand;
using fyt::serial_driver::GimbalCommandReceiver;
using fyt::serial_driver::GimbalPacketCommand;
using fyt::serial_driver::GimbalPacketMode;
using fyt::serial_driver::encodeGimbalPacketV2;

GimbalAxisReference linearReference(const double start,
                                    const double velocity,
                                    const std::size_t horizon,
                                    const double dt) {
  GimbalAxisReference reference;
  reference.position.resize(horizon);
  reference.velocity.assign(horizon, velocity);
  reference.acceleration.assign(horizon, 0.0);
  for (std::size_t i = 0; i < horizon; ++i) {
    reference.position[i] = start + velocity * static_cast<double>(i + 1) * dt;
  }
  return reference;
}

void assertFinite(const double value) { assert(std::isfinite(value)); }

}  // namespace

int main() {
  GimbalMpcConfig config;
  config.dt = 0.01;
  config.horizon = 20;
  config.max_velocity = 8.0;
  config.max_acceleration = 20.0;
  config.max_jerk = 120.0;
  config.max_iterations = 12;
  config.step_size = 0.001;

  GimbalMpc solver(config);
  GimbalSimulator actuator(config);
  GimbalCommandReceiver receiver;
  FixedPacket<32> packet;
  GimbalActuatorCommand decoded;
  constexpr double target_velocity = 1.0;
  constexpr int frames = 160;
  double accumulated_error = 0.0;

  for (std::uint32_t sequence = 0; sequence < frames; ++sequence) {
    const double target_position = target_velocity * sequence * config.dt;
    GimbalMpcInput input;
    input.yaw = actuator.yaw().state();
    input.pitch = actuator.pitch().state();
    input.yaw_reference = linearReference(
      target_position, target_velocity, config.horizon, config.dt);
    input.pitch_reference = linearReference(0.0, 0.0, config.horizon, config.dt);

    const auto result = solver.solve(input);
    assert(result.valid);
    assert(result.converged);
    assert(result.yaw.max_constraint_violation < 1e-8);
    assert(result.pitch.max_constraint_violation < 1e-8);
    assertFinite(result.yaw.control);
    assertFinite(result.pitch.control);

    GimbalPacketCommand command;
    command.mode = GimbalPacketMode::kJerk;
    command.yaw_jerk = static_cast<float>(result.yaw.control);
    command.pitch_jerk = static_cast<float>(result.pitch.control);
    command.command_dt = static_cast<float>(config.dt);
    command.sequence = sequence;
    assert(encodeGimbalPacketV2(command, packet));
    assert(receiver.receiveVersioned(packet, decoded));
    assert(decoded.mode == GimbalPacketMode::kJerk);
    assert(decoded.sequence == sequence);
    assert(decoded.command_dt == static_cast<float>(config.dt));
    assertFinite(decoded.yaw_jerk);
    assertFinite(decoded.pitch_jerk);

    actuator.stepJerk(decoded.yaw_jerk, decoded.pitch_jerk);
    const double error = actuator.yaw().state().position - target_position;
    accumulated_error += std::abs(error);
    assertFinite(error);
    assert(std::abs(actuator.yaw().state().velocity) <= config.max_velocity + 1e-8);
    assert(std::abs(actuator.yaw().state().acceleration) <= config.max_acceleration + 1e-8);
    assert(std::abs(actuator.yaw().lastJerk()) <= config.max_jerk + 1e-6);
  }

  assert(accumulated_error < 20.0);
  std::cout << "frames=" << frames << " accumulated_position_error="
            << accumulated_error << " final_yaw=" << actuator.yaw().state().position
            << "\n";
  return 0;
}
