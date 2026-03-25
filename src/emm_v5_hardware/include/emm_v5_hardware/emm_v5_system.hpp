#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <chrono>
#include <mutex>
#include <thread>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "emm_v5_driver/can_transport.hpp"

namespace emm_v5_hardware {

class EmmV5System final : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EmmV5System)

  EmmV5System();
  ~EmmV5System() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  struct JointCfg {
    std::string name;
    uint8_t addr{0};

    int motor_steps_per_rev{200};
    int microsteps{16};
    double reduction_ratio{1.0};

    bool pos_positive_is_cw{true};
    int sign{+1};

    double min_deg{0.0};
    double max_deg{360.0};
    bool cyclic{false};

    double max_vel_deg_s{20.0};
    uint8_t acc_level{10};
  };

  // ---- parameter parsing ----
  bool load_from_yaml_if_available();
  bool load_from_hardware_info_params();

  // ---- helpers ----
  static double rad2deg(double rad);
  static double deg2rad(double deg);
  static double normalize_deg_0_360(double deg);
  static double shortest_diff_deg(double cur_deg, double tgt_deg);
  static double clamp(double v, double lo, double hi);

  uint32_t deg_to_pulses(const JointCfg& j, double deg_abs) const;
  uint16_t vel_deg_s_to_0_1rpm(const JointCfg& j, double vel_deg_s) const;
  uint8_t dir_bit_for_delta(const JointCfg& j, double delta_deg_ros) const;

  // ---- zeroing / persistence + services ----
  bool load_zero_offsets();
  bool save_zero_offsets() const;
  void start_services();
  void stop_services();

 private:
  rclcpp::Logger logger_{rclcpp::get_logger("emm_v5_hardware")};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  std::string can_iface_{"can0"};
  bool use_sync_start_{true};
  bool open_loop_{true};

  std::string config_yaml_;

  std::vector<JointCfg> joints_;
  std::vector<double> hw_states_pos_;
  std::vector<double> hw_states_vel_;
  std::vector<double> hw_commands_pos_;

  emm_v5_driver::CanTransport can_;

  std::vector<double> last_pos_deg_;
  std::vector<double> last_cmd_deg_;
  std::vector<double> residual_deg_;

  double cmd_deadband_deg_{0.05};
  uint32_t min_pulses_{5};
  uint16_t min_vel_0_1rpm_{10};
  double poll_one_joint_hz_{20.0};
  double feedback_timeout_s_{0.25};
  bool allow_addr0_broadcast_{false};

  size_t poll_index_{0};
  std::chrono::steady_clock::time_point last_poll_tick_{std::chrono::steady_clock::time_point{}};
  size_t poll_per_cycle_{6};

  std::vector<bool> fb_valid_;
  std::vector<int64_t> last_pulse_mt_;
  std::vector<std::chrono::steady_clock::time_point> last_fb_time_;

  bool ready_for_cmd_{false};

  // --- software zero offsets (ROS-positive degrees), applied in read() ---
  std::vector<double> zero_offset_deg_;
  std::string zero_offset_path_;

  // --- service infra (private context) ---
  std::shared_ptr<rclcpp::Context> srv_ctx_;
  rclcpp::Node::SharedPtr srv_node_;
  std::shared_ptr<rclcpp::Executor> srv_exec_;
  std::thread srv_thread_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_set_zero_joint1_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_zero_;
  mutable std::mutex mtx_;
};

}  // namespace emm_v5_hardware
