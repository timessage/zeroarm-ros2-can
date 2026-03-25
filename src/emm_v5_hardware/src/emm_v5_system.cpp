#include "emm_v5_hardware/emm_v5_system.hpp"

#include <cmath>
#include <stdexcept>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <unordered_map>
#include <mutex>
#include <chrono>

#include "emm_v5_driver/protocol.hpp"

#ifdef EMM_V5_HAVE_YAML_CPP
#include <yaml-cpp/yaml.h>
#endif

#include "pluginlib/class_list_macros.hpp"

namespace emm_v5_hardware {
namespace {

// ===== FD latch / dedupe runtime (kept in cpp so header need not change) =====
struct JointFdLatch {
  bool motion_active = false;
  bool pending_valid = false;

  double latched_target_deg = 0.0;   // command currently owned by hardware
  double pending_target_deg = 0.0;   // newest target received while hardware still moving

  double sent_start_deg = 0.0;       // feedback position when the latched command was sent
  double sent_target_deg = 0.0;      // expected target after current latched command
  double last_progress_deg = 0.0;

  std::chrono::steady_clock::time_point last_send_tp{};
  std::chrono::steady_clock::time_point last_progress_tp{};
};

static std::mutex g_latch_mtx;
static std::unordered_map<const void*, std::vector<JointFdLatch>> g_latches;

static std::vector<JointFdLatch>& get_latches(const void* key, const size_t n) {
  std::lock_guard<std::mutex> lk(g_latch_mtx);
  auto &v = g_latches[key];
  if (v.size() != n) v.assign(n, JointFdLatch{});
  return v;
}

static void erase_latches(const void* key) {
  std::lock_guard<std::mutex> lk(g_latch_mtx);
  g_latches.erase(key);
}

constexpr double kReachWindowDeg = 0.40;          // software "reached" window
constexpr double kRetargetThresholdDeg = 1.00;    // ignore tiny retargets while active
constexpr double kMinResendIntervalS = 0.30;      // do not re-issue same FD faster than this
constexpr double kStallWatchS = 0.35;             // if no progress for this long, allow resend
constexpr double kProgressEpsDeg = 0.10;          // what counts as progress

}  // namespace

EmmV5System::EmmV5System() = default;

EmmV5System::~EmmV5System() {
  erase_latches(this);
  stop_services();
  can_.close();
}

hardware_interface::CallbackReturn EmmV5System::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.count("can_iface")) {
    can_iface_ = info_.hardware_parameters.at("can_iface");
  }
  if (info_.hardware_parameters.count("use_sync_start")) {
    use_sync_start_ =
        (info_.hardware_parameters.at("use_sync_start") == "true" || info_.hardware_parameters.at("use_sync_start") == "1");
  }
  if (info_.hardware_parameters.count("open_loop")) {
    open_loop_ =
        (info_.hardware_parameters.at("open_loop") == "true" || info_.hardware_parameters.at("open_loop") == "1");
  }
  if (info_.hardware_parameters.count("config_yaml")) {
    config_yaml_ = info_.hardware_parameters.at("config_yaml");
  }
  if (info_.hardware_parameters.count("zero_offset_path")) {
    zero_offset_path_ = info_.hardware_parameters.at("zero_offset_path");
  } else {
    const char* home = std::getenv("HOME");
    std::string base = home ? std::string(home) : std::string("/tmp");
    zero_offset_path_ = base + "/.ros/emm_v5_zero_offsets.yaml";
  }

  bool ok = false;
  if (!config_yaml_.empty()) {
    ok = load_from_yaml_if_available();
  }
  if (!ok) {
    ok = load_from_hardware_info_params();
  }
  if (!ok || joints_.empty()) {
    RCLCPP_ERROR(logger_, "Failed to load joint configuration. Provide hardware param 'config_yaml' or per-joint params.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t n = joints_.size();
  hw_states_pos_.assign(n, 0.0);
  hw_states_vel_.assign(n, 0.0);
  hw_commands_pos_.assign(n, 0.0);
  last_pos_deg_.assign(n, 0.0);
  last_cmd_deg_.assign(n, 0.0);
  residual_deg_.assign(n, 0.0);

  fb_valid_.assign(n, false);
  last_pulse_mt_.assign(n, 0);
  last_fb_time_.assign(n, std::chrono::steady_clock::time_point{});

  ready_for_cmd_ = open_loop_;
  poll_index_ = 0;
  last_poll_tick_ = std::chrono::steady_clock::time_point{};
  poll_per_cycle_ = 6;

  zero_offset_deg_.assign(n, 0.0);
  (void)load_zero_offsets();

  auto &latches = get_latches(this, n);
  (void)latches;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EmmV5System::on_configure(const rclcpp_lifecycle::State&) {
  try {
    can_.open(can_iface_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "CAN open failed on iface '%s': %s", can_iface_.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  start_services();
  RCLCPP_INFO(logger_, "Configured CAN iface=%s, open_loop=%s, use_sync_start=%s, feedback_timeout=%.3fs",
              can_iface_.c_str(), open_loop_ ? "true" : "false", use_sync_start_ ? "true" : "false",
              feedback_timeout_s_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EmmV5System::on_activate(const rclcpp_lifecycle::State&) {
  std::fill(fb_valid_.begin(), fb_valid_.end(), false);
  ready_for_cmd_ = open_loop_;
  for (const auto& j : joints_) {
    const auto payload = emm_v5_driver::build_enable_payload(true, false);
    (void)can_.send_cmd(j.addr, emm_v5_driver::kFuncEnable, payload);
  }

  auto &latches = get_latches(this, joints_.size());
  const auto now = std::chrono::steady_clock::now();
  for (auto &lt : latches) {
    lt = JointFdLatch{};
    lt.last_send_tp = now;
    lt.last_progress_tp = now;
  }

  RCLCPP_INFO(logger_, "EmmV5System activated with FD latch/dedupe. open_loop=%s, use_sync_start=%s",
              open_loop_ ? "true" : "false", use_sync_start_ ? "true" : "false");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EmmV5System::on_deactivate(const rclcpp_lifecycle::State&) {
  for (const auto& j : joints_) {
    const auto payload = emm_v5_driver::build_enable_payload(false, false);
    (void)can_.send_cmd(j.addr, emm_v5_driver::kFuncEnable, payload);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EmmV5System::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(joints_.size() * 2);
  for (size_t i = 0; i < joints_.size(); ++i) {
    state_interfaces.emplace_back(joints_[i].name, hardware_interface::HW_IF_POSITION, &hw_states_pos_[i]);
    state_interfaces.emplace_back(joints_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_vel_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> EmmV5System::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i) {
    command_interfaces.emplace_back(joints_[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_pos_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type EmmV5System::read(const rclcpp::Time&, const rclcpp::Duration& period) {
  if (open_loop_) {
    hw_states_pos_ = hw_commands_pos_;
    std::fill(hw_states_vel_.begin(), hw_states_vel_.end(), 0.0);
    return hardware_interface::return_type::OK;
  }

  const double dt = std::max(1e-3, period.seconds());

  // 1) Decode cached replies
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto& j = joints_[i];

    if (!allow_addr0_broadcast_ && j.addr == 0) {
      continue;
    }

    emm_v5_driver::CanTransport::RxFrame rx{};
    if (can_.get_last(j.addr, emm_v5_driver::kFuncReadPulseMultiTurn, &rx)) {
      const bool is_new_sample = (!fb_valid_[i]) || (rx.stamp != last_fb_time_[i]);
      const double sample_age_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - rx.stamp).count();
      if (sample_age_s > feedback_timeout_s_) {
        continue;
      }

      emm_v5_driver::ReadPulseMtResponse resp{};
      if (emm_v5_driver::parse_read_pulse_mt_response(rx.data, rx.dlc, &resp)) {
        const int64_t pulses_signed = resp.signed_pulses;
        last_pulse_mt_[i] = pulses_signed;

        const double pulses_per_motor_rev =
            std::max(1.0, static_cast<double>(j.motor_steps_per_rev) * static_cast<double>(j.microsteps));
        const double motor_deg_internal = static_cast<double>(pulses_signed) * 360.0 / pulses_per_motor_rev;
        const double motor_deg_cw = j.pos_positive_is_cw ? motor_deg_internal : -motor_deg_internal;
        const double joint_deg_cw = motor_deg_cw / std::max(1e-9, j.reduction_ratio);
        double joint_deg_ros = joint_deg_cw * static_cast<double>(j.sign);

        {
          std::lock_guard<std::mutex> lk(mtx_);
          if (i < zero_offset_deg_.size()) {
            joint_deg_ros += zero_offset_deg_[i];
          }
        }

        if (j.cyclic) {
          joint_deg_ros = normalize_deg_0_360(joint_deg_ros);
        } else {
          joint_deg_ros = clamp(joint_deg_ros, j.min_deg, j.max_deg);
        }

        const double joint_rad = deg2rad(joint_deg_ros);

        if (is_new_sample && fb_valid_[i]) {
          const double dt_fb = std::max(1e-3, std::chrono::duration<double>(rx.stamp - last_fb_time_[i]).count());
          hw_states_vel_[i] = (joint_rad - hw_states_pos_[i]) / dt_fb;
        } else if (!fb_valid_[i]) {
          hw_states_vel_[i] = 0.0;
          fb_valid_[i] = true;
        }

        if (is_new_sample) {
          hw_states_pos_[i] = joint_rad;
          last_pos_deg_[i] = joint_deg_ros;
          last_fb_time_[i] = rx.stamp;
        }

        if (!ready_for_cmd_) {
          last_cmd_deg_[i] = joint_deg_ros;
          residual_deg_[i] = 0.0;
        }
      }
    }
  }

  // 2) Round-robin poll: send K polls per cycle
  if (!joints_.empty()) {
    const size_t K = std::min<size_t>(poll_per_cycle_, joints_.size());
    for (size_t k = 0; k < K; ++k) {
      poll_index_ = (poll_index_ + 1) % joints_.size();
      const auto& pj = joints_[poll_index_];
      if (allow_addr0_broadcast_ || pj.addr != 0) {
        (void)can_.send_cmd(pj.addr, emm_v5_driver::kFuncReadPulseMultiTurn,
                            emm_v5_driver::build_read_pulse_mt_payload());
      }
    }
  }

  // gate motion until all joints have at least one feedback sample
  if (!ready_for_cmd_) {
    bool all_ok = true;
    for (bool v : fb_valid_) all_ok = all_ok && v;
    if (all_ok) {
      hw_commands_pos_ = hw_states_pos_;
      ready_for_cmd_ = true;
      auto &latches = get_latches(this, joints_.size());
      for (size_t i = 0; i < joints_.size(); ++i) {
        latches[i].latched_target_deg = last_pos_deg_[i];
        latches[i].pending_target_deg = last_pos_deg_[i];
        latches[i].sent_start_deg = last_pos_deg_[i];
        latches[i].sent_target_deg = last_pos_deg_[i];
        latches[i].last_progress_deg = last_pos_deg_[i];
        latches[i].motion_active = false;
        latches[i].pending_valid = false;
      }
      RCLCPP_INFO(logger_, "Feedback synchronized for all joints. Motion commands enabled.");
    }
  }

  const auto now = std::chrono::steady_clock::now();
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (!fb_valid_[i]) {
      continue;
    }
    const double age_s = std::chrono::duration<double>(now - last_fb_time_[i]).count();
    if (age_s > feedback_timeout_s_) {
      RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 2000,
                           "Stale feedback on %s: age=%.3fs", joints_[i].name.c_str(), age_s);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EmmV5System::write(const rclcpp::Time&, const rclcpp::Duration& period) {
  const double dt = std::max(1e-3, period.seconds());

  if (!open_loop_ && !ready_for_cmd_) {
    return hardware_interface::return_type::OK;
  }

  auto &latches = get_latches(this, joints_.size());
  const auto now = std::chrono::steady_clock::now();

  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto& j = joints_[i];
    auto &lt = latches[i];

    if (!allow_addr0_broadcast_ && j.addr == 0) {
      continue;
    }

    double tgt_deg = rad2deg(hw_commands_pos_[i]);
    if (j.cyclic) {
      tgt_deg = normalize_deg_0_360(tgt_deg);
    } else {
      tgt_deg = clamp(tgt_deg, j.min_deg, j.max_deg);
    }

    const double cur_deg = last_pos_deg_[i];
    auto err_from = [&](double from_deg, double to_deg) {
      return j.cyclic ? shortest_diff_deg(from_deg, to_deg) : (to_deg - from_deg);
    };

    // ===== movement state evaluation =====
    if (lt.motion_active) {
      const double err_to_sent = err_from(cur_deg, lt.sent_target_deg);
      const double since_send_s = std::chrono::duration<double>(now - lt.last_send_tp).count();

      // track progress
      const double progress_now = std::abs(err_to_sent);
      if (std::abs(progress_now - std::abs(err_from(lt.last_progress_deg, lt.sent_target_deg))) > kProgressEpsDeg) {
        lt.last_progress_deg = cur_deg;
        lt.last_progress_tp = now;
      }

      const bool reached = std::abs(err_to_sent) <= kReachWindowDeg;
      if (reached) {
        lt.motion_active = false;
        lt.pending_valid = false;
        lt.latched_target_deg = lt.sent_target_deg;
        last_cmd_deg_[i] = lt.sent_target_deg;
      } else {
        const bool target_changed_a_lot = std::abs(err_from(lt.latched_target_deg, tgt_deg)) >= kRetargetThresholdDeg;
        if (target_changed_a_lot) {
          lt.pending_target_deg = tgt_deg;
          lt.pending_valid = true;
        }

        const double since_progress_s = std::chrono::duration<double>(now - lt.last_progress_tp).count();
        const bool stall_like = since_send_s >= kMinResendIntervalS && since_progress_s >= kStallWatchS;

        // dedupe: while command is active and making progress, do NOT keep re-sending the same FD
        if (!stall_like) {
          continue;
        }

        // if stalled, we are allowed to resend, but only toward the latest pending target if any
        if (lt.pending_valid) {
          tgt_deg = lt.pending_target_deg;
          lt.pending_valid = false;
        } else {
          tgt_deg = lt.sent_target_deg;
        }
      }
    }

    // ===== issue a fresh FD only when idle, or when stalled / retargeted =====
    double send_deg = err_from(cur_deg, tgt_deg);
    if (std::abs(send_deg) < cmd_deadband_deg_) {
      if (open_loop_) last_pos_deg_[i] = cur_deg;
      continue;
    }

    uint32_t pulses = deg_to_pulses(j, std::abs(send_deg));
    if (pulses < min_pulses_) {
      continue;
    }

    const uint8_t dir_bit = dir_bit_for_delta(j, send_deg);
    const double vel_deg_s = std::min(j.max_vel_deg_s, std::abs(send_deg) / dt);
    uint16_t vel_0_1rpm = vel_deg_s_to_0_1rpm(j, vel_deg_s);
    vel_0_1rpm = std::max(min_vel_0_1rpm_, vel_0_1rpm);

    const bool abs_mode = false;   // keep the current stable relative-mode behavior
    const bool sync_flag = use_sync_start_;

    const auto payload = emm_v5_driver::build_pos_fd_payload(
        dir_bit, vel_0_1rpm, j.acc_level, pulses, abs_mode, sync_flag);

    if (!can_.send_cmd(j.addr, emm_v5_driver::kFuncPosMode, payload)) {
      RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 2000,
                           "Failed to send latched position command to %s(addr=%u)", j.name.c_str(), j.addr);
      continue;
    }

    const double new_target = j.cyclic ? normalize_deg_0_360(cur_deg + send_deg)
                                       : clamp(cur_deg + send_deg, j.min_deg, j.max_deg);

    lt.motion_active = true;
    lt.pending_valid = false;
    lt.latched_target_deg = tgt_deg;
    lt.sent_start_deg = cur_deg;
    lt.sent_target_deg = new_target;
    lt.last_progress_deg = cur_deg;
    lt.last_send_tp = now;
    lt.last_progress_tp = now;

    last_cmd_deg_[i] = new_target;
    residual_deg_[i] = 0.0;

    if (open_loop_) {
      last_pos_deg_[i] = last_cmd_deg_[i];
    }
  }

  if (use_sync_start_) {
    const auto payload = emm_v5_driver::build_sync_start_payload();
    if (!can_.send_cmd(0 /*broadcast*/, emm_v5_driver::kFuncSync, payload)) {
      RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 2000,
                           "Failed to send sync-start broadcast.");
    }
  }

  return hardware_interface::return_type::OK;
}

// ===== config loading =====
bool EmmV5System::load_from_yaml_if_available() {
#ifndef EMM_V5_HAVE_YAML_CPP
  RCLCPP_WARN(logger_, "yaml-cpp not found at build time; cannot parse config_yaml. Falling back to per-joint ros2_control params.");
  return false;
#else
  try {
    YAML::Node root = YAML::LoadFile(config_yaml_);
    YAML::Node params = root["emm_v5_system"]["ros__parameters"];
    if (!params) params = root;

    if (params["can_iface"]) can_iface_ = params["can_iface"].as<std::string>();
    if (params["use_sync_start"]) use_sync_start_ = params["use_sync_start"].as<bool>();
    if (params["open_loop"]) open_loop_ = params["open_loop"].as<bool>();
    if (params["poll_one_joint_hz"]) poll_one_joint_hz_ = params["poll_one_joint_hz"].as<double>();
    if (params["feedback_timeout_s"]) feedback_timeout_s_ = params["feedback_timeout_s"].as<double>();
    if (params["cmd_deadband_deg"]) cmd_deadband_deg_ = params["cmd_deadband_deg"].as<double>();
    if (params["min_pulses"]) min_pulses_ = static_cast<uint32_t>(params["min_pulses"].as<int>());
    if (params["min_vel_0_1rpm"]) min_vel_0_1rpm_ = static_cast<uint16_t>(params["min_vel_0_1rpm"].as<int>());
    if (params["allow_addr0_broadcast"]) allow_addr0_broadcast_ = params["allow_addr0_broadcast"].as<bool>();
    if (params["poll_per_cycle"]) poll_per_cycle_ = static_cast<size_t>(std::max(1, params["poll_per_cycle"].as<int>()));

    YAML::Node joints = params["joints"];
    if (!joints || !joints.IsSequence()) {
      RCLCPP_ERROR(logger_, "config_yaml missing 'joints' list.");
      return false;
    }

    joints_.clear();
    for (const auto& jn : joints) {
      JointCfg j;
      j.name = jn["name"].as<std::string>();
      j.addr = static_cast<uint8_t>(jn["addr"].as<int>());
      if (jn["motor_steps_per_rev"]) j.motor_steps_per_rev = jn["motor_steps_per_rev"].as<int>();
      if (jn["microsteps"]) j.microsteps = jn["microsteps"].as<int>();
      if (jn["reduction_ratio"]) j.reduction_ratio = jn["reduction_ratio"].as<double>();
      if (jn["pos_positive_is_cw"]) j.pos_positive_is_cw = jn["pos_positive_is_cw"].as<bool>();
      if (jn["sign"]) j.sign = jn["sign"].as<int>();
      if (jn["min_deg"]) j.min_deg = jn["min_deg"].as<double>();
      if (jn["max_deg"]) j.max_deg = jn["max_deg"].as<double>();
      if (jn["cyclic"]) j.cyclic = jn["cyclic"].as<bool>();
      if (jn["max_vel_deg_s"]) j.max_vel_deg_s = jn["max_vel_deg_s"].as<double>();
      if (jn["acc_level"]) j.acc_level = static_cast<uint8_t>(jn["acc_level"].as<int>());
      joints_.push_back(j);
    }

    last_pos_deg_.assign(joints_.size(), 0.0);
    RCLCPP_INFO(logger_, "Loaded %zu joints from YAML: %s (poll_per_cycle=%zu)",
                joints_.size(), config_yaml_.c_str(), poll_per_cycle_);
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to parse config_yaml '%s': %s", config_yaml_.c_str(), e.what());
    return false;
  }
#endif
}

bool EmmV5System::load_from_hardware_info_params() {
  joints_.clear();
  bool bad_addr0 = false;

  if (info_.joints.empty()) return false;

  for (const auto& ji : info_.joints) {
    JointCfg j;
    j.name = ji.name;

    auto get_s = [&](const std::string& key, const std::string& def) -> std::string {
      auto it = ji.parameters.find(key);
      return it == ji.parameters.end() ? def : it->second;
    };

    j.addr = static_cast<uint8_t>(std::stoi(get_s("addr", "0")));
    j.motor_steps_per_rev = std::stoi(get_s("motor_steps_per_rev", "200"));
    j.microsteps = std::stoi(get_s("microsteps", "16"));
    j.reduction_ratio = std::stod(get_s("reduction_ratio", "1.0"));
    j.pos_positive_is_cw = (get_s("pos_positive_is_cw", "true") == "true" || get_s("pos_positive_is_cw", "1") == "1");
    j.sign = std::stoi(get_s("sign", "+1"));
    j.min_deg = std::stod(get_s("min_deg", "0.0"));
    j.max_deg = std::stod(get_s("max_deg", "360.0"));
    j.cyclic = (get_s("cyclic", "false") == "true" || get_s("cyclic", "1") == "1");
    j.max_vel_deg_s = std::stod(get_s("max_vel_deg_s", "20.0"));
    j.acc_level = static_cast<uint8_t>(std::stoi(get_s("acc_level", "10")));

    if (j.addr == 0) {
      bad_addr0 = true;
      RCLCPP_ERROR(logger_, "Joint '%s' missing 'addr' parameter (addr=0). This would broadcast on CAN.",
                   j.name.c_str());
    }

    joints_.push_back(j);
  }

  if (bad_addr0 && !allow_addr0_broadcast_) {
    return false;
  }

  last_pos_deg_.assign(joints_.size(), 0.0);
  RCLCPP_INFO(logger_, "Loaded %zu joints from ros2_control per-joint parameters.", joints_.size());
  return true;
}

// ===== helpers =====
double EmmV5System::rad2deg(double rad) { return rad * 180.0 / M_PI; }
double EmmV5System::deg2rad(double deg) { return deg * M_PI / 180.0; }

double EmmV5System::normalize_deg_0_360(double deg) {
  double d = std::fmod(deg, 360.0);
  if (d < 0) d += 360.0;
  return d;
}

double EmmV5System::shortest_diff_deg(double cur_deg, double tgt_deg) {
  double diff = tgt_deg - cur_deg;
  while (diff > 180.0) diff -= 360.0;
  while (diff <= -180.0) diff += 360.0;
  return diff;
}

double EmmV5System::clamp(double v, double lo, double hi) {
  return std::min(std::max(v, lo), hi);
}

uint32_t EmmV5System::deg_to_pulses(const JointCfg& j, double deg_abs) const {
  const double pulses_per_joint_rev =
      static_cast<double>(j.motor_steps_per_rev) * static_cast<double>(j.microsteps) * j.reduction_ratio;
  const double pulses = (deg_abs / 360.0) * pulses_per_joint_rev;
  const double clipped = std::min(std::max(pulses, 0.0), static_cast<double>(0xFFFFFFFFu));
  return static_cast<uint32_t>(std::llround(clipped));
}

uint16_t EmmV5System::vel_deg_s_to_0_1rpm(const JointCfg& j, double vel_deg_s) const {
  const double joint_rpm = vel_deg_s / 6.0;
  const double motor_rpm = joint_rpm * j.reduction_ratio;
  const double v_0_1rpm = motor_rpm * 10.0;
  const double clipped = std::min(std::max(v_0_1rpm, 0.0), static_cast<double>(0xFFFF));
  return static_cast<uint16_t>(std::llround(clipped));
}

uint8_t EmmV5System::dir_bit_for_delta(const JointCfg& j, double delta_deg_ros) const {
  const double delta_hw = static_cast<double>(j.sign) * delta_deg_ros;
  const bool cw = (delta_hw >= 0.0) ? j.pos_positive_is_cw : (!j.pos_positive_is_cw);
  return cw ? 0x00 : 0x01;
}

// ===== zero offset persistence =====
bool EmmV5System::save_zero_offsets() const {
  try {
    namespace fs = std::filesystem;
    fs::path p(zero_offset_path_);
    fs::create_directories(p.parent_path());

#ifdef EMM_V5_HAVE_YAML_CPP
    YAML::Node root;
    YAML::Node jo;
    for (size_t i = 0; i < joints_.size(); ++i) {
      jo[joints_[i].name] = zero_offset_deg_[i];
    }
    root["zero_offsets_deg"] = jo;

    std::ofstream fout(zero_offset_path_, std::ios::out | std::ios::trunc);
    fout << root;
    fout.close();
#else
    std::ofstream fout(zero_offset_path_, std::ios::out | std::ios::trunc);
    fout << "zero_offsets_deg:\n";
    for (size_t i = 0; i < joints_.size(); ++i) {
      fout << "  " << joints_[i].name << ": " << zero_offset_deg_[i] << "\n";
    }
    fout.close();
#endif
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to save zero offsets: %s", e.what());
    return false;
  }
}

bool EmmV5System::load_zero_offsets() {
  try {
    namespace fs = std::filesystem;
    if (!fs::exists(zero_offset_path_)) {
      RCLCPP_INFO(logger_, "Zero offset file not found, start with zeros: %s", zero_offset_path_.c_str());
      return false;
    }

#ifdef EMM_V5_HAVE_YAML_CPP
    YAML::Node root = YAML::LoadFile(zero_offset_path_);
    YAML::Node jo = root["zero_offsets_deg"];
    if (!jo || !jo.IsMap()) {
      RCLCPP_WARN(logger_, "Invalid zero offset yaml (missing zero_offsets_deg map).");
      return false;
    }
    for (size_t i = 0; i < joints_.size(); ++i) {
      const auto& name = joints_[i].name;
      if (jo[name]) {
        zero_offset_deg_[i] = jo[name].as<double>();
      }
    }
#else
    std::ifstream fin(zero_offset_path_);
    std::string line;
    auto trim = [](std::string s) {
      size_t b = s.find_first_not_of(" \t");
      size_t e = s.find_last_not_of(" \t\r\n");
      if (b == std::string::npos) return std::string();
      return s.substr(b, e - b + 1);
    };
    while (std::getline(fin, line)) {
      auto pos = line.find(':');
      if (pos == std::string::npos) continue;
      std::string key = trim(line.substr(0, pos));
      std::string val = trim(line.substr(pos + 1));
      if (key == "zero_offsets_deg") continue;
      for (size_t i = 0; i < joints_.size(); ++i) {
        if (key == joints_[i].name) {
          zero_offset_deg_[i] = std::stod(val);
        }
      }
    }
#endif

    RCLCPP_INFO(logger_, "Loaded zero offsets from: %s", zero_offset_path_.c_str());
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to load zero offsets: %s", e.what());
    return false;
  }
}

// ===== services =====
void EmmV5System::start_services() {
  if (srv_node_) return;

  srv_ctx_ = std::make_shared<rclcpp::Context>();
  srv_ctx_->init(0, nullptr);

  rclcpp::NodeOptions opts;
  opts.context(srv_ctx_);
  srv_node_ = std::make_shared<rclcpp::Node>("emm_v5_hw_srv", opts);

  rclcpp::ExecutorOptions eopts;
  eopts.context = srv_ctx_;
  srv_exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(eopts);
  srv_exec_->add_node(srv_node_);

  srv_set_zero_joint1_ = srv_node_->create_service<std_srvs::srv::Trigger>(
      "/emm_v5/set_zero",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        std::lock_guard<std::mutex> lk(mtx_);

        if (!open_loop_) {
          bool all_ok = true;
          for (bool v : fb_valid_) all_ok = all_ok && v;
          if (!all_ok) {
            resp->success = false;
            resp->message = "Feedback not valid for all joints yet. Wait and retry.";
            return;
          }
        }

        size_t idx = joints_.size();
        for (size_t i = 0; i < joints_.size(); ++i) {
          if (joints_[i].name == "joint1") { idx = i; break; }
        }
        if (idx == joints_.size()) {
          resp->success = false;
          resp->message = "joint1 not found.";
          return;
        }

        const double cur_deg = rad2deg(hw_states_pos_[idx]);
        zero_offset_deg_[idx] = -cur_deg;

        last_cmd_deg_[idx] = 0.0;
        residual_deg_[idx] = 0.0;
        last_pos_deg_[idx] = 0.0;
        hw_commands_pos_[idx] = 0.0;

        auto &latches = get_latches(this, joints_.size());
        if (idx < latches.size()) {
          latches[idx] = JointFdLatch{};
        }

        const bool ok = save_zero_offsets();
        resp->success = ok;
        resp->message = ok ? "joint1 zero offset saved." : "joint1 zero set, but failed to save file.";
      });

  srv_clear_zero_ = srv_node_->create_service<std_srvs::srv::Trigger>(
      "/emm_v5/clear_zero",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        std::lock_guard<std::mutex> lk(mtx_);
        for (auto &v : zero_offset_deg_) v = 0.0;
        const bool ok = save_zero_offsets();
        resp->success = ok;
        resp->message = ok ? "All zero offsets cleared and saved." : "Cleared offsets, but failed to save file.";
      });

  srv_thread_ = std::thread([this]() {
    while (srv_ctx_ && srv_ctx_->is_valid() && rclcpp::ok(srv_ctx_)) {
      srv_exec_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  RCLCPP_INFO(logger_, "Services started: /emm_v5/set_zero (joint1), /emm_v5/clear_zero. Save: %s",
              zero_offset_path_.c_str());
}

void EmmV5System::stop_services() {
  if (!srv_ctx_) return;

  try { srv_ctx_->shutdown("emm_v5_hw_srv shutdown"); } catch (...) {}

  if (srv_thread_.joinable()) srv_thread_.join();

  if (srv_exec_ && srv_node_) {
    try { srv_exec_->remove_node(srv_node_); } catch (...) {}
  }

  srv_set_zero_joint1_.reset();
  srv_clear_zero_.reset();
  srv_exec_.reset();
  srv_node_.reset();
  srv_ctx_.reset();
}

}  // namespace emm_v5_hardware

PLUGINLIB_EXPORT_CLASS(emm_v5_hardware::EmmV5System, hardware_interface::SystemInterface)
