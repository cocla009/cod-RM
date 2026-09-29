// CRC-16/MCRF4XX as used by sp_vision_25 (tools/crc.cpp).
//
// Parameters: polynomial 0x1021 (reflected 0x8408), init 0xFFFF, reflected
// input and output, no final xor. Check value for "123456789" is 0x6F91.
//
// Note this is NOT CRC-16/MODBUS, which uses polynomial 0x8005 (reflected
// 0xA001) and a different table. The two agree on nothing useful, so the
// variant is pinned by a test vector in test_gimbal_packet_v3.cpp.

#ifndef RM_SERIAL_DRIVER_CRC16_HPP_
#define RM_SERIAL_DRIVER_CRC16_HPP_

#include <cstddef>
#include <cstdint>

namespace fyt::serial_driver {

inline std::uint16_t crc16(const std::uint8_t *data, const std::size_t length) noexcept {
  if (data == nullptr) return 0;

  std::uint16_t crc = 0xFFFFU;
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) ? static_cast<std::uint16_t>((crc >> 1) ^ 0x8408U)
                       : static_cast<std::uint16_t>(crc >> 1);
    }
  }
  return crc;
}

}  // namespace fyt::serial_driver

#endif  // RM_SERIAL_DRIVER_CRC16_HPP_
