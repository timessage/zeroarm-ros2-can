#include "emm_v5_driver/protocol.hpp"

namespace emm_v5_driver {

std::vector<uint8_t> build_enable_payload(bool enable, bool sync, uint8_t checksum) {
  return {0xAB, static_cast<uint8_t>(enable ? 1 : 0), static_cast<uint8_t>(sync ? 1 : 0), checksum};
}

std::vector<uint8_t> build_pos_fd_payload(
    uint8_t dir,
    uint16_t vel_0_1rpm,
    uint8_t acc_level,
    uint32_t pulses,
    bool abs_mode,
    bool sync,
    uint8_t checksum) {
  std::vector<uint8_t> p;
  p.reserve(11);
  p.push_back(static_cast<uint8_t>(dir & 0x01));
  p.push_back(static_cast<uint8_t>((vel_0_1rpm >> 8) & 0xFF));
  p.push_back(static_cast<uint8_t>(vel_0_1rpm & 0xFF));
  p.push_back(acc_level);
  p.push_back(static_cast<uint8_t>((pulses >> 24) & 0xFF));
  p.push_back(static_cast<uint8_t>((pulses >> 16) & 0xFF));
  p.push_back(static_cast<uint8_t>((pulses >> 8) & 0xFF));
  p.push_back(static_cast<uint8_t>(pulses & 0xFF));
  p.push_back(static_cast<uint8_t>(abs_mode ? 1 : 0));
  p.push_back(static_cast<uint8_t>(sync ? 1 : 0));
  p.push_back(checksum);
  return p;
}

std::vector<uint8_t> build_sync_start_payload(uint8_t checksum) {
  return {0x66, checksum};
}

std::vector<uint8_t> build_set_single_turn_zero_payload(bool store, uint8_t checksum) {
  return {0x88, static_cast<uint8_t>(store ? 1 : 0), checksum};
}

std::vector<uint8_t> build_home_trigger_payload(uint8_t mode, bool sync, uint8_t checksum) {
  return {mode, static_cast<uint8_t>(sync ? 1 : 0), checksum};
}

std::vector<uint8_t> build_read_pos_payload(uint8_t checksum) {
  // Manual: cmd format = addr + 0x36 + checksum
  return {checksum};
}

std::vector<uint8_t> build_read_pulse_mt_payload(uint8_t checksum) {
  // Manual (Rev1.3): cmd format = addr + 0x32 + checksum
  // Address is embedded in extended CAN ID and func is placed into data[0] by CanTransport.
  // Therefore we only send the trailing checksum byte.
  return {checksum};
}

bool parse_read_pos_response(const uint8_t* data, uint8_t dlc, ReadPosResponse* out, uint8_t checksum) {
  if (data == nullptr || out == nullptr) {
    return false;
  }
  // Expected: [0]=0x36, [1]=sign, [2..5]=pos(be), [6]=checksum
  if (dlc < 7) {
    return false;
  }
  if (data[0] != kFuncReadPos) {
    return false;
  }
  if (data[6] != checksum) {
    return false;
  }

  const uint8_t sign = data[1];
  const uint32_t raw = (static_cast<uint32_t>(data[2]) << 24) |
                       (static_cast<uint32_t>(data[3]) << 16) |
                       (static_cast<uint32_t>(data[4]) << 8) |
                       (static_cast<uint32_t>(data[5]) << 0);

  out->negative = (sign == 0x01);
  out->pos_raw = raw;

  // Manual: 0..65535 represents one revolution (360 deg). Example: 0x00010000 -> -360 deg.
  const double deg = (static_cast<double>(raw) * 360.0) / 65536.0;
  out->motor_deg = out->negative ? -deg : deg;
  return true;
}

bool parse_read_pulse_mt_response(const uint8_t* data, uint8_t dlc, ReadPulseMtResponse* out, uint8_t checksum) {
  if (data == nullptr || out == nullptr) {
    return false;
  }
  // Manual (Rev1.3): [0]=0x32, [1]=sign(00 positive /01 negative), [2..5]=pulses(be), [6]=checksum
  // Note: Some dumps show dlc=8 with addr included when using classic IDs; in our transport the addr is
  //       encoded in the extended CAN ID, so the data payload starts at the function code.
  if (dlc < 7) {
    return false;
  }
  if (data[0] != kFuncReadPulseMultiTurn) {
    return false;
  }
  if (data[6] != checksum) {
    return false;
  }

  const uint8_t sign = data[1];
  const uint32_t pulses = (static_cast<uint32_t>(data[2]) << 24) |
                          (static_cast<uint32_t>(data[3]) << 16) |
                          (static_cast<uint32_t>(data[4]) << 8) |
                          (static_cast<uint32_t>(data[5]) << 0);

  out->func = data[0];
  out->sign = sign;
  out->negative = (sign == 0x01);
  out->pulses_raw = pulses;
  out->signed_pulses = out->negative ? -static_cast<int64_t>(pulses) : static_cast<int64_t>(pulses);
  out->checksum = data[6];
  return true;
}

}  // namespace emm_v5_driver
