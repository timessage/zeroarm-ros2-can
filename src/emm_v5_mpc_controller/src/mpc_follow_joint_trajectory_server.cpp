#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::chrono_literals;

namespace {

double clamp_val(const double v, const double lo, const double hi) {
  return std::max(lo, std::min(hi, v));
}

double deg2rad(const double deg) {
  return deg * M_PI / 180.0;
}

double to_seconds(const builtin_interfaces::msg::Duration &d) {
  return static_cast<double>(d.sec) + 1e-9 * static_cast<double>(d.nanosec);
}

struct Sample {
  std::vector<double> q;
  std::vector<double> v;
};

struct ActiveGoal {
  trajectory_msgs::msg::JointTrajectory traj;
  rclcpp::Time start_time;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> handle;
  bool active{false};
  std::vector<double> xq_cmd;
  std::vector<double> xv_cmd;
  bool final_hold{false};
  rclcpp::Time final_hold_since{0, 0, RCL_ROS_TIME};
};

}  // namespace

class MpcFollowJointTrajectoryServer : public rclcpp::Node {
public:
  using FollowJT = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJT>;

  MpcFollowJointTrajectoryServer() : Node("mpc_controller") {
    declare_parameter<std::vector<std::string>>("joints", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    declare_parameter<std::string>("action_name", "/mpc_controller/follow_joint_trajectory");
    declare_parameter<std::string>("command_topic", "/arm_controller/joint_trajectory");
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
    declare_parameter<double>("control_dt", 0.02);
    declare_parameter<int>("horizon_steps", 10);
    declare_parameter<double>("amax_rad_s2", 6.0);
    declare_parameter<double>("cmd_segment_deg", 10.0);
    declare_parameter<double>("pos_tolerance_deg", 0.8);
    declare_parameter<double>("vel_tolerance_rad_s", 0.12);
    declare_parameter<double>("goal_hold_s", 0.25);
    declare_parameter<double>("goal_timeout_padding_s", 1.0);
    declare_parameter<double>("q_pos", 14.0);
    declare_parameter<double>("q_vel", 1.0);
    declare_parameter<double>("r_acc", 0.08);
    declare_parameter<double>("reference_lookahead_s", 0.25);
    declare_parameter<std::vector<double>>("candidate_scales", {-1.0, -0.5, 0.0, 0.5, 1.0});

    joints_ = get_parameter("joints").as_string_array();
    action_name_ = get_parameter("action_name").as_string();
    command_topic_ = get_parameter("command_topic").as_string();
    joint_state_topic_ = get_parameter("joint_state_topic").as_string();
    control_dt_ = get_parameter("control_dt").as_double();
    horizon_steps_ = get_parameter("horizon_steps").as_int();
    amax_rad_s2_ = get_parameter("amax_rad_s2").as_double();
    cmd_segment_rad_ = deg2rad(get_parameter("cmd_segment_deg").as_double());
    pos_tolerance_rad_ = deg2rad(get_parameter("pos_tolerance_deg").as_double());
    vel_tolerance_rad_s_ = get_parameter("vel_tolerance_rad_s").as_double();
    goal_hold_s_ = get_parameter("goal_hold_s").as_double();
    goal_timeout_padding_s_ = get_parameter("goal_timeout_padding_s").as_double();
    q_pos_ = get_parameter("q_pos").as_double();
    q_vel_ = get_parameter("q_vel").as_double();
    r_acc_ = get_parameter("r_acc").as_double();
    reference_lookahead_s_ = get_parameter("reference_lookahead_s").as_double();
    candidate_scales_ = get_parameter("candidate_scales").as_double_array();

    cmd_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(command_topic_, 10);
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic_, 50,
        std::bind(&MpcFollowJointTrajectoryServer::on_joint_state, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<FollowJT>(
        this,
        action_name_,
        std::bind(&MpcFollowJointTrajectoryServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MpcFollowJointTrajectoryServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&MpcFollowJointTrajectoryServer::handle_accepted, this, std::placeholders::_1));

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(control_dt_)),
        std::bind(&MpcFollowJointTrajectoryServer::tick, this));

    RCLCPP_INFO(get_logger(), "MPC FollowJointTrajectory bridge started on %s -> %s",
                action_name_.c_str(), command_topic_.c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJT::Goal> goal) {
    if (goal->trajectory.joint_names.empty() || goal->trajectory.points.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting empty trajectory goal.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJT>) {
    std::lock_guard<std::mutex> lock(goal_mtx_);
    if (active_goal_.active) {
      const auto q_now = current_positions_ordered(joints_);
      std::vector<double> v_zero(joints_.size(), 0.0);
      publish_single_point(q_now, v_zero);
    }
    active_goal_.active = false;
    active_goal_.final_hold = false;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle) {
    std::lock_guard<std::mutex> lock(goal_mtx_);
    active_goal_.traj = goal_handle->get_goal()->trajectory;
    active_goal_.start_time = now();
    active_goal_.handle = goal_handle;
    active_goal_.active = true;
    active_goal_.xq_cmd = current_positions_ordered(joints_);
    active_goal_.xv_cmd.assign(joints_.size(), 0.0);
    active_goal_.final_hold = false;
    active_goal_.final_hold_since = now();
    RCLCPP_INFO(get_logger(), "Accepted FollowJointTrajectory goal with %zu points.",
                active_goal_.traj.points.size());
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mtx_);
    joint_pos_.clear();
    joint_vel_.clear();
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const auto &name = msg->name[i];
      if (i < msg->position.size()) {
        joint_pos_[name] = msg->position[i];
      }
      if (i < msg->velocity.size()) {
        joint_vel_[name] = msg->velocity[i];
      }
    }
  }

  std::vector<double> current_positions_ordered(const std::vector<std::string> &names) const {
    std::lock_guard<std::mutex> lock(state_mtx_);
    std::vector<double> out(names.size(), 0.0);
    for (size_t i = 0; i < names.size(); ++i) {
      auto it = joint_pos_.find(names[i]);
      if (it != joint_pos_.end()) out[i] = it->second;
    }
    return out;
  }

  std::vector<double> current_velocities_ordered(const std::vector<std::string> &names) const {
    std::lock_guard<std::mutex> lock(state_mtx_);
    std::vector<double> out(names.size(), 0.0);
    for (size_t i = 0; i < names.size(); ++i) {
      auto it = joint_vel_.find(names[i]);
      if (it != joint_vel_.end()) out[i] = it->second;
    }
    return out;
  }

  std::vector<double> final_goal_positions(const trajectory_msgs::msg::JointTrajectory &traj) const {
    std::vector<double> q_goal(joints_.size(), 0.0);
    if (traj.points.empty()) return q_goal;
    const auto &last = traj.points.back();
    for (size_t i = 0; i < joints_.size(); ++i) {
      auto it = std::find(traj.joint_names.begin(), traj.joint_names.end(), joints_[i]);
      if (it != traj.joint_names.end()) {
        const size_t idx = static_cast<size_t>(std::distance(traj.joint_names.begin(), it));
        if (idx < last.positions.size()) q_goal[i] = last.positions[idx];
      }
    }
    return q_goal;
  }

  bool within_goal_window(const std::vector<double> &q_goal,
                          const std::vector<double> &q_now,
                          const std::vector<double> &v_now,
                          const bool strict_velocity) const {
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (std::abs(q_goal[i] - q_now[i]) > pos_tolerance_rad_) return false;
      if (strict_velocity && std::abs(v_now[i]) > vel_tolerance_rad_s_) return false;
    }
    return true;
  }

  Sample sample_reference(const trajectory_msgs::msg::JointTrajectory &traj, const double t_sec) const {
    Sample out;
    const size_t n = joints_.size();
    out.q.assign(n, 0.0);
    out.v.assign(n, 0.0);

    std::vector<int> idx_map(n, -1);
    for (size_t i = 0; i < joints_.size(); ++i) {
      auto it = std::find(traj.joint_names.begin(), traj.joint_names.end(), joints_[i]);
      if (it != traj.joint_names.end()) idx_map[i] = static_cast<int>(std::distance(traj.joint_names.begin(), it));
    }

    if (traj.points.size() == 1) {
      for (size_t j = 0; j < n; ++j) {
        const int idx = idx_map[j];
        if (idx >= 0 && static_cast<size_t>(idx) < traj.points[0].positions.size()) out.q[j] = traj.points[0].positions[idx];
        if (idx >= 0 && static_cast<size_t>(idx) < traj.points[0].velocities.size()) out.v[j] = traj.points[0].velocities[idx];
      }
      return out;
    }

    size_t upper = 0;
    while (upper < traj.points.size() && to_seconds(traj.points[upper].time_from_start) < t_sec) {
      ++upper;
    }

    if (upper == 0) {
      upper = 1;
    } else if (upper >= traj.points.size()) {
      upper = traj.points.size() - 1;
    }
    const size_t lower = upper - 1;
    const double t0 = to_seconds(traj.points[lower].time_from_start);
    const double t1 = to_seconds(traj.points[upper].time_from_start);
    const double alpha = (t1 > t0) ? clamp_val((t_sec - t0) / (t1 - t0), 0.0, 1.0) : 1.0;

    for (size_t j = 0; j < n; ++j) {
      const int idx = idx_map[j];
      if (idx < 0) continue;
      const auto &p0 = traj.points[lower];
      const auto &p1 = traj.points[upper];
      if (static_cast<size_t>(idx) < p0.positions.size() && static_cast<size_t>(idx) < p1.positions.size()) {
        out.q[j] = (1.0 - alpha) * p0.positions[idx] + alpha * p1.positions[idx];
      }
      const double v0 = static_cast<size_t>(idx) < p0.velocities.size() ? p0.velocities[idx] : 0.0;
      const double v1 = static_cast<size_t>(idx) < p1.velocities.size() ? p1.velocities[idx] : 0.0;
      out.v[j] = (1.0 - alpha) * v0 + alpha * v1;
    }
    return out;
  }

  double solve_joint_mpc(const double q_now, const double v_now, const double t_now,
                         const trajectory_msgs::msg::JointTrajectory &traj, const size_t joint_idx) const {
    double best_acc = 0.0;
    double best_cost = std::numeric_limits<double>::infinity();

    for (const double scale : candidate_scales_) {
      const double acc = clamp_val(scale * amax_rad_s2_, -amax_rad_s2_, amax_rad_s2_);
      double q = q_now;
      double v = v_now;
      double cost = 0.0;
      for (int k = 1; k <= horizon_steps_; ++k) {
        const double tk = t_now + reference_lookahead_s_ + k * control_dt_;
        const auto ref = sample_reference(traj, tk);
        const double q_ref = ref.q[joint_idx];
        const double v_ref = ref.v[joint_idx];
        q = q + control_dt_ * v + 0.5 * control_dt_ * control_dt_ * acc;
        v = v + control_dt_ * acc;
        const double e = q_ref - q;
        const double ev = v_ref - v;
        cost += q_pos_ * e * e + q_vel_ * ev * ev + r_acc_ * acc * acc;
      }
      if (cost < best_cost) {
        best_cost = cost;
        best_acc = acc;
      }
    }
    return best_acc;
  }

  void publish_single_point(const std::vector<double> &q_cmd, const std::vector<double> &v_cmd) {
    trajectory_msgs::msg::JointTrajectory jt;
    jt.header.stamp = now();
    jt.joint_names = joints_;
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = q_cmd;
    pt.velocities = v_cmd;
    pt.time_from_start.sec = 0;
    pt.time_from_start.nanosec = static_cast<uint32_t>(control_dt_ * 1e9);
    jt.points.push_back(pt);
    cmd_pub_->publish(jt);
  }

  void succeed_active_goal(const std::string &msg) {
    auto result = std::make_shared<FollowJT::Result>();
    result->error_code = FollowJT::Result::SUCCESSFUL;
    result->error_string = msg;
    std::lock_guard<std::mutex> lock(goal_mtx_);
    if (active_goal_.handle) active_goal_.handle->succeed(result);
    active_goal_.active = false;
    active_goal_.final_hold = false;
  }

  void tick() {
    ActiveGoal goal;
    {
      std::lock_guard<std::mutex> lock(goal_mtx_);
      if (!active_goal_.active) return;
      goal = active_goal_;
    }

    const auto q_now = current_positions_ordered(joints_);
    const auto v_now = current_velocities_ordered(joints_);
    const double t_now = (now() - goal.start_time).seconds();
    const double traj_end_t = to_seconds(goal.traj.points.back().time_from_start);
    const auto q_goal = final_goal_positions(goal.traj);

    bool enter_final_hold = false;
    {
      std::lock_guard<std::mutex> lock(goal_mtx_);
      if (!active_goal_.final_hold && within_goal_window(q_goal, q_now, v_now, false)) {
        active_goal_.final_hold = true;
        active_goal_.final_hold_since = now();
        enter_final_hold = true;
      }
      goal = active_goal_;
    }

    if (goal.final_hold || enter_final_hold) {
      std::vector<double> v_zero(joints_.size(), 0.0);
      publish_single_point(q_goal, v_zero);

      trajectory_msgs::msg::JointTrajectoryPoint desired;
      desired.positions = q_goal;
      desired.velocities = v_zero;
      trajectory_msgs::msg::JointTrajectoryPoint actual;
      actual.positions = q_now;
      actual.velocities = v_now;
      trajectory_msgs::msg::JointTrajectoryPoint error;
      error.positions.resize(joints_.size(), 0.0);
      error.velocities.resize(joints_.size(), 0.0);
      for (size_t i = 0; i < joints_.size(); ++i) {
        error.positions[i] = q_goal[i] - q_now[i];
        error.velocities[i] = -v_now[i];
      }

      {
        std::lock_guard<std::mutex> lock(goal_mtx_);
        if (active_goal_.active && active_goal_.handle) {
          auto fb = std::make_shared<FollowJT::Feedback>();
          fb->joint_names = joints_;
          fb->desired = desired;
          fb->actual = actual;
          fb->error = error;
          active_goal_.handle->publish_feedback(fb);
        }
      }

      const double hold_time = (now() - goal.final_hold_since).seconds();
      const bool strict_done = within_goal_window(q_goal, q_now, v_now, true);
      const bool held_long_enough = hold_time >= goal_hold_s_;
      const bool timeout_done = (t_now >= traj_end_t + goal_timeout_padding_s_) && within_goal_window(q_goal, q_now, v_now, false);

      if ((strict_done && held_long_enough) || timeout_done) {
        succeed_active_goal("trajectory reached");
        RCLCPP_INFO(get_logger(), "MPC goal reached and held.");
      }
      return;
    }

    std::vector<double> q_cmd(joints_.size(), 0.0);
    std::vector<double> v_cmd(joints_.size(), 0.0);
    std::vector<double> q_prev = goal.xq_cmd.empty() ? q_now : goal.xq_cmd;
    std::vector<double> v_prev = goal.xv_cmd.empty() ? v_now : goal.xv_cmd;

    for (size_t i = 0; i < joints_.size(); ++i) {
      const double acc = solve_joint_mpc(q_now[i], v_now[i], t_now, goal.traj, i);
      double v_next = v_prev[i] + control_dt_ * acc;
      double q_next = q_prev[i] + control_dt_ * v_prev[i] + 0.5 * control_dt_ * control_dt_ * acc;

      const auto ref = sample_reference(goal.traj, t_now + reference_lookahead_s_ + control_dt_);
      const double q_ref = ref.q[i];
      const double dq = q_next - q_now[i];
      const double limited_dq = clamp_val(dq, -cmd_segment_rad_, cmd_segment_rad_);
      q_next = q_now[i] + limited_dq;
      v_next = clamp_val((q_next - q_now[i]) / control_dt_, -4.0, 4.0);

      const double err_goal = q_goal[i] - q_now[i];
      if (std::abs(err_goal) < cmd_segment_rad_) {
        q_next = q_goal[i];
        v_next = 0.0;
      } else if (std::abs(q_ref - q_next) < deg2rad(0.2)) {
        q_next = q_ref;
      }

      q_cmd[i] = q_next;
      v_cmd[i] = v_next;
    }

    publish_single_point(q_cmd, v_cmd);

    {
      std::lock_guard<std::mutex> lock(goal_mtx_);
      if (active_goal_.active) {
        active_goal_.xq_cmd = q_cmd;
        active_goal_.xv_cmd = v_cmd;
        if (active_goal_.handle) {
          auto fb = std::make_shared<FollowJT::Feedback>();
          fb->joint_names = joints_;
          trajectory_msgs::msg::JointTrajectoryPoint desired;
          desired.positions = q_cmd;
          desired.velocities = v_cmd;
          fb->desired = desired;
          trajectory_msgs::msg::JointTrajectoryPoint actual;
          actual.positions = q_now;
          actual.velocities = v_now;
          fb->actual = actual;
          trajectory_msgs::msg::JointTrajectoryPoint error;
          error.positions.resize(joints_.size(), 0.0);
          error.velocities.resize(joints_.size(), 0.0);
          for (size_t i = 0; i < joints_.size(); ++i) {
            error.positions[i] = q_cmd[i] - q_now[i];
            error.velocities[i] = v_cmd[i] - v_now[i];
          }
          fb->error = error;
          active_goal_.handle->publish_feedback(fb);
        }
      }
    }
  }

  std::vector<std::string> joints_;
  std::string action_name_;
  std::string command_topic_;
  std::string joint_state_topic_;
  double control_dt_{0.02};
  int horizon_steps_{10};
  double amax_rad_s2_{6.0};
  double cmd_segment_rad_{deg2rad(10.0)};
  double pos_tolerance_rad_{deg2rad(0.8)};
  double vel_tolerance_rad_s_{0.12};
  double goal_hold_s_{0.25};
  double goal_timeout_padding_s_{1.0};
  double q_pos_{14.0};
  double q_vel_{1.0};
  double r_acc_{0.08};
  double reference_lookahead_s_{0.25};
  std::vector<double> candidate_scales_;

  mutable std::mutex state_mtx_;
  std::unordered_map<std::string, double> joint_pos_;
  std::unordered_map<std::string, double> joint_vel_;

  std::mutex goal_mtx_;
  ActiveGoal active_goal_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp_action::Server<FollowJT>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcFollowJointTrajectoryServer>());
  rclcpp::shutdown();
  return 0;
}
