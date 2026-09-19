#include "rm_serial_driver/gimbal_packet_v2.hpp"

#include <cassert>
#include <cmath>
#include <limits>

int main() {
  using fyt::serial_driver::FixedPacket;
  using fyt::serial_driver::GimbalPacketCommand;
  using fyt::serial_driver::GimbalPacketMode;
  using fyt::serial_driver::decodeGimbalPacketV2;
  using fyt::serial_driver::encodeGimbalPacketV2;

  GimbalPacketCommand source;
  source.mode = GimbalPacketMode::kJerk;
  source.fire = true;
  source.pitch = -3.25F;
  source.yaw = 21.5F;
  source.yaw_jerk = 120.0F;
  source.pitch_jerk = -80.0F;
  source.command_dt = 0.01F;
  source.sequence = 0x12345678U;
  FixedPacket<32> packet;
  assert(encodeGimbalPacketV2(source, packet));
  GimbalPacketCommand decoded;
  assert(decodeGimbalPacketV2(packet, decoded));
  assert(decoded.mode == source.mode);
  assert(decoded.fire == source.fire);
  assert(decoded.pitch == source.pitch);
  assert(decoded.yaw == source.yaw);
  assert(decoded.yaw_jerk == source.yaw_jerk);
  assert(decoded.pitch_jerk == source.pitch_jerk);
  assert(decoded.command_dt == source.command_dt);
  assert(decoded.sequence == source.sequence);

  source.command_dt = 0.0F;
  assert(!encodeGimbalPacketV2(source, packet));
  source.mode = GimbalPacketMode::kPosition;
  assert(encodeGimbalPacketV2(source, packet));
  assert(decodeGimbalPacketV2(packet, decoded));
  packet.loadData<std::uint8_t>(0x00U, 1);
  assert(!decodeGimbalPacketV2(packet, decoded));
  assert(encodeGimbalPacketV2(source, packet));
  packet.loadData<std::uint8_t>(2U, 2);
  assert(!decodeGimbalPacketV2(packet, decoded));
  assert(encodeGimbalPacketV2(source, packet));
  packet.loadData<float>(std::numeric_limits<float>::quiet_NaN(), 8);
  assert(!decodeGimbalPacketV2(packet, decoded));
  return 0;
}
