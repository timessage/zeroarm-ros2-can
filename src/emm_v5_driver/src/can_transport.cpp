#include "emm_v5_driver/can_transport.hpp"

#include <atomic>
#include <cerrno>
#include <cstring>
#include <stdexcept>

// Linux SocketCAN
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace emm_v5_driver {

CanTransport::CanTransport() = default;

CanTransport::~CanTransport() {
  close();
}

void CanTransport::open(const std::string& iface) {
  close();

  sock_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_fd_ < 0) {
    throw std::runtime_error("socket(PF_CAN) failed: " + std::string(std::strerror(errno)));
  }

  struct ifreq ifr {
  };
  std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
  if (ioctl(sock_fd_, SIOCGIFINDEX, &ifr) < 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
    throw std::runtime_error("ioctl(SIOCGIFINDEX) failed for " + iface + ": " + std::string(std::strerror(errno)));
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
    throw std::runtime_error("bind(AF_CAN) failed for " + iface + ": " + std::string(std::strerror(errno)));
  }

  timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = 100000;  // 100 ms, so close() can stop the RX thread promptly.
  if (::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
    throw std::runtime_error("setsockopt(SO_RCVTIMEO) failed for " + iface + ": " + std::string(std::strerror(errno)));
  }

  running_.store(true);
  rx_thread_ = std::thread(&CanTransport::rx_loop, this);
}

void CanTransport::close() {
  running_.store(false);

  const int fd = sock_fd_;
  sock_fd_ = -1;
  if (fd >= 0) {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
  }

  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }

  clear_rx_cache();
}

bool CanTransport::is_open() const {
  return sock_fd_ >= 0;
}

void CanTransport::set_rx_callback(RxCallback cb) {
  std::lock_guard<std::mutex> lk(cb_mtx_);
  cb_ = std::move(cb);
}

bool CanTransport::get_last(uint8_t addr, uint8_t func, RxFrame* out) const {
  if (out == nullptr) {
    return false;
  }
  const uint16_t key = (static_cast<uint16_t>(addr) << 8) | static_cast<uint16_t>(func);
  std::lock_guard<std::mutex> lk(rx_cache_mtx_);
  auto it = rx_cache_.find(key);
  if (it == rx_cache_.end()) {
    return false;
  }
  *out = it->second;
  return true;
}

void CanTransport::clear_rx_cache() {
  std::lock_guard<std::mutex> lk(rx_cache_mtx_);
  rx_cache_.clear();
}

bool CanTransport::send_cmd(uint8_t addr, uint8_t func, const std::vector<uint8_t>& payload) {
  const int fd = sock_fd_;
  if (fd < 0) {
    return false;
  }

  std::lock_guard<std::mutex> lk(tx_mtx_);

  // Split payload into chunks of 7 bytes. Each CAN frame data[0] is function code.
  uint8_t pack_idx = 0;
  size_t i = 0;
  while (i < payload.size()) {
    const size_t chunk_len = std::min<size_t>(7, payload.size() - i);

    can_frame frame{};
    const uint32_t can_id_29 = (static_cast<uint32_t>(addr) << 8) | pack_idx;
    frame.can_id = can_id_29 | CAN_EFF_FLAG;  // extended frame required by manual
    frame.can_dlc = static_cast<__u8>(chunk_len + 1);
    frame.data[0] = func;
    std::memcpy(&frame.data[1], payload.data() + i, chunk_len);

    const ssize_t n = ::write(fd, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame))) {
      return false;
    }

    i += chunk_len;
    pack_idx++;
  }

  return true;
}

void CanTransport::rx_loop() {
  while (running_.load()) {
    can_frame frame{};
    const ssize_t n = ::read(sock_fd_, &frame, sizeof(frame));
    if (n < 0) {
      if (!running_.load()) {
        break;
      }
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      continue;
    }
    if (n != static_cast<ssize_t>(sizeof(frame))) {
      continue;
    }

    // Only accept extended frames
    if ((frame.can_id & CAN_EFF_FLAG) == 0) {
      continue;
    }

    RxFrame rx{};
    rx.can_id = (frame.can_id & CAN_EFF_MASK);
    rx.dlc = frame.can_dlc;
    std::memcpy(rx.data, frame.data, 8);
    rx.stamp = std::chrono::steady_clock::now();

    // Cache the latest reply by (addr, func). Works well for short replies like 0x36.
    const uint8_t addr8 = static_cast<uint8_t>((rx.can_id >> 8) & 0xFF);
    const uint8_t func = rx.data[0];
    const uint16_t key = (static_cast<uint16_t>(addr8) << 8) | static_cast<uint16_t>(func);
    {
      std::lock_guard<std::mutex> lk(rx_cache_mtx_);
      rx_cache_[key] = rx;
    }

    RxCallback cb_copy;
    {
      std::lock_guard<std::mutex> lk(cb_mtx_);
      cb_copy = cb_;
    }
    if (cb_copy) {
      cb_copy(rx);
    }
  }
}
}  // namespace emm_v5_driver
