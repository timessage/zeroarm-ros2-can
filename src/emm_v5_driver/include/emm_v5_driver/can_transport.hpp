#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <atomic>
#include <chrono>

namespace emm_v5_driver {

/**
 * @brief Raw SocketCAN transport for Emm_V5.0.
 *
 * Responsibilities:
 * - open SocketCAN interface (can0/vcan0)
 * - send an Emm command by splitting payload into multiple 8-byte CAN frames
 * - format CAN extended-ID as (addr<<8) | packet_index
 *
 * This class intentionally does not interpret responses. A higher layer can register a callback to observe frames.
 */
class CanTransport {
 public:
  struct RxFrame {
    uint32_t can_id;         // 29-bit ID (without EFF flag)
    uint8_t dlc;
    uint8_t data[8];
    std::chrono::steady_clock::time_point stamp{};
  };

  using RxCallback = std::function<void(const RxFrame&)>;

  CanTransport();
  ~CanTransport();

  CanTransport(const CanTransport&) = delete;
  CanTransport& operator=(const CanTransport&) = delete;

  /** Open and start RX thread. Throws std::runtime_error on failure. */
  void open(const std::string& iface);

  /** Stop RX thread and close socket. Safe to call multiple times. */
  void close();

  /**
   * @brief Send a command.
   * @param addr driver address (1..n), or 0 for broadcast.
   * @param func function code (e.g. 0xFD)
   * @param payload bytes after function code.
   */
  bool send_cmd(uint8_t addr, uint8_t func, const std::vector<uint8_t>& payload);

  /** Get the latest received frame for (addr, func=data[0]). Returns false if none available. */
  bool get_last(uint8_t addr, uint8_t func, RxFrame* out) const;

  /** Clear internal RX cache. */
  void clear_rx_cache();

  void set_rx_callback(RxCallback cb);

  bool is_open() const;

 private:
  void rx_loop();

  int sock_fd_{-1};
  std::thread rx_thread_;
  std::mutex cb_mtx_;
  std::mutex tx_mtx_;
  RxCallback cb_;
  std::atomic<bool> running_{false};

  // A small cache for short reply frames (e.g. 0x36 read-position). Key = (addr<<8)|func.
  mutable std::mutex rx_cache_mtx_;
  std::unordered_map<uint16_t, RxFrame> rx_cache_;
};

}  // namespace emm_v5_driver
