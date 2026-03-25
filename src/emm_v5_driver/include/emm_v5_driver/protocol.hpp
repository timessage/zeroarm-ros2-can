#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace emm_v5_driver {

/**
 * @brief Small helpers for building Emm_V5.0 commands.
 *
 * The manual describes:
 * - CAN frame uses extended frame format.
 * - CAN ID is (addr << 8) | packet_index.
 * - If a command is longer than 8 bytes, split it into multiple frames.
 *   Each frame's data[0] must be the function code (e.g. 0xFD), followed by up to 7 bytes payload.
 *
 * This library builds the *payload* of a command (without the leading function code byte), and the CAN
 * transport will handle splitting and CAN ID formatting.
 */

// Function codes
constexpr uint8_t kFuncEnable = 0xF3;
constexpr uint8_t kFuncPosMode = 0xFD;
constexpr uint8_t kFuncSync = 0xFF;
constexpr uint8_t kFuncHome = 0x9A;
constexpr uint8_t kFuncSetZero = 0x93;
constexpr uint8_t kFuncReadPos = 0x36;  // read motor real position
// Read multi-turn input pulse counter (0x32).
// Manual (Rev1.3): request payload = [0x32, 0x6B]; response = [addr,0x32,sign,p3,p2,p1,p0,0x6B].
constexpr uint8_t kFuncReadPulseMultiTurn = 0x32;

// Common checksum/end byte used throughout the manual's examples.
constexpr uint8_t kChecksumDefault = 0x6B;

/** Enable/disable driver: payload = [0xAB, enable(0/1), sync(0/1), checksum] */
std::vector<uint8_t> build_enable_payload(bool enable, bool sync = false, uint8_t checksum = kChecksumDefault);

/**
 * Position mode (0xFD) payload:
 * [dir, vel_hi, vel_lo, acc, pulses(4 bytes, big endian), abs(0/1), sync(0/1), checksum]
 * - dir: 0=CW, 1=CCW (per manual)
 * - vel: 0.1RPM units (driver uses velocity/10 as real RPM)
 * - pulses: step pulses (depends on microsteps & reduction)
 * - abs: 0=relative, 1=absolute
 */
std::vector<uint8_t> build_pos_fd_payload(
    uint8_t dir,
    uint16_t vel_0_1rpm,
    uint8_t acc_level,
    uint32_t pulses,
    bool abs_mode,
    bool sync,
    uint8_t checksum = kChecksumDefault);

/** Sync start: payload = [0x66, checksum] (function code = 0xFF) */
std::vector<uint8_t> build_sync_start_payload(uint8_t checksum = kChecksumDefault);

/** Set single-turn zero: payload = [0x88, store(0/1), checksum] (function code = 0x93) */
std::vector<uint8_t> build_set_single_turn_zero_payload(bool store, uint8_t checksum = kChecksumDefault);

/** Home trigger: payload = [mode, sync(0/1), checksum] (function code = 0x9A) */
std::vector<uint8_t> build_home_trigger_payload(uint8_t mode, bool sync, uint8_t checksum = kChecksumDefault);

/** Read motor real position (0x36): payload = [checksum] */
std::vector<uint8_t> build_read_pos_payload(uint8_t checksum = kChecksumDefault);

/** Parsed response of read motor real position (0x36). */
struct ReadPosResponse {
  bool negative{false};
  uint32_t pos_raw{0};   // 0..65535 represents 0..360 degrees; can extend beyond for multi-turn
  double motor_deg{0.0}; // motor shaft position in degrees, with sign
};

/** Parse a single CAN frame payload of 0x36 response (data[0]=0x36). */
bool parse_read_pos_response(const uint8_t* data, uint8_t dlc, ReadPosResponse* out, uint8_t checksum = kChecksumDefault);

/** Read input pulse count (multi-turn) (0x32): request payload = [checksum]. */
std::vector<uint8_t> build_read_pulse_mt_payload(uint8_t checksum = kChecksumDefault);

struct ReadPulseMtResponse
{
  // NOTE:
  //  - 本项目把“电机地址”编码在 CAN ID 里（例如 0x100/0x200/...），
  //    因此 data payload 一般从功能码开始：0x32 ...
  //  - 手册(Rev1.3) 的 0x32 回包字段：
  //      [0]=0x32,
  //      [1]=sign (00 正 / 01 负),
  //      [2..5]=pulses (BE, 32-bit),
  //      [6]=0x6B
  //  - “pulses”是累计脉冲数（多圈）。有符号值需结合 sign。

  uint8_t func{0};          // should be 0x32
  uint8_t sign{0};          // raw sign byte: 0x00 positive, 0x01 negative
  bool negative{false};     // sign==0x01
  uint32_t pulses_raw{0};   // magnitude, big-endian
  int64_t signed_pulses{0}; // pulses with sign applied
  uint8_t checksum{0};      // should be 0x6B
};

/** Parse read input pulse count (multi-turn) response (0x32). */
bool parse_read_pulse_mt_response(const uint8_t* data, uint8_t dlc, ReadPulseMtResponse* out, uint8_t checksum = kChecksumDefault);

}  // namespace emm_v5_driver
