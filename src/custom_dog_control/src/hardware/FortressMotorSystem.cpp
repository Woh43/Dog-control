#include <array>
#include <cmath>
#include <deque>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <gz_ros2_control/gz_system_interface.hpp>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityReset.hh>
#include <pluginlib/class_list_macros.hpp>

#include "custom_dog_control/control/JointCommandUtils.hpp"
#include "custom_dog_control/control/SignedMotorPdBudget.hpp"

namespace custom_dog_control {
namespace {
bool ReadValues(const hardware_interface::HardwareInfo& info,
                const std::string& name, std::vector<double>& values) {
  const auto parameter = info.hardware_parameters.find(name);
  if (parameter == info.hardware_parameters.end()) return true;
  values.clear();
  std::istringstream stream(parameter->second);
  std::string token;
  while (stream >> token) {
    try {
      std::size_t consumed = 0;
      const double value = std::stod(token, &consumed);
      if (consumed != token.size() || !std::isfinite(value) || value < 0.0) return false;
      values.push_back(value);
    } catch (const std::exception&) {
      return false;
    }
  }
  return stream.eof() && !values.empty();
}

bool ReadFriction(const hardware_interface::HardwareInfo& info,
                  const std::string& name,
                  std::array<double, kJointCount>& output) {
  std::vector<double> values{0.0};
  if (!ReadValues(info, name, values) ||
      (values.size() != 1 && values.size() != kJointsPerLeg &&
       values.size() != kJointCount)) return false;
  for (std::size_t i = 0; i < kJointCount; ++i) output[i] = values[i % values.size()];
  return true;
}
}  // namespace

// Models the existing five-field actuator protocol. The outer NMPC/WBC loop
// runs at controller_manager's rate; the held motor-side PD command is evaluated
// using physical joint feedback at every physics step. Optional extra command
// delay and dissipative joint friction are stress-test parameters, not a
// calibration of the real drive's bandwidth, transmission or torque accuracy.
class FortressMotorSystem final : public gz_ros2_control::GazeboSimSystemInterface {
 public:
  bool initSim(rclcpp::Node::SharedPtr& node,
               std::map<std::string, sim::Entity>& joints,
               const hardware_interface::HardwareInfo& info,
               sim::EntityComponentManager& ecm, int& /*update_rate*/) override {
    nh_ = node;
    ecm_ = &ecm;
    const auto signed_budget = info.hardware_parameters.find("motor_pd_signed_budget");
    if (signed_budget != info.hardware_parameters.end()) {
      const auto& value = signed_budget->second;
      if (value != "true" && value != "false" && value != "1" && value != "0") return false;
      signed_budget_enabled_ = value == "true" || value == "1";
    }
    // gz_ros2_control fills update_rate only after initSim returns and the
    // controller_manager exists. It must not be read during this callback.
    std::vector<double> delay{0.0}, velocity_scale{0.05}, budget_horizon{0.0}, nominal_time{0.0};
    if (!ReadValues(info, "simulation_command_delay_s", delay) || delay.size() != 1 ||
        delay[0] > 0.05 ||
        !ReadValues(info, "simulation_friction_velocity_scale", velocity_scale) ||
        velocity_scale.size() != 1 || velocity_scale[0] <= 0.0 ||
        !ReadFriction(info, "simulation_coulomb_friction", coulomb_friction_) ||
        !ReadFriction(info, "simulation_viscous_friction", viscous_friction_) ||
        !ReadValues(info, "motor_pd_budget_horizon_s", budget_horizon) ||
        budget_horizon.size() != 1 || budget_horizon[0] > 0.1 ||
        !ReadValues(info, "motor_pd_nominal_execution_time_s", nominal_time) ||
        nominal_time.size() != 1 || nominal_time[0] > budget_horizon[0] ||
        !ReadFriction(info, "motor_pd_assumed_acceleration_rad_s2",
            diagnostic_window_model_.assumed_acceleration_bound_rad_s2)) {
      RCLCPP_ERROR(node->get_logger(),
          "Invalid motor simulation parameters: delay must be 0..0.05 s; friction "
          "must contain 1, 3 or 12 finite nonnegative values; velocity scale must be positive");
      return false;
    }
    command_delay_nanoseconds_ = static_cast<std::int64_t>(std::llround(delay[0] * 1e9));
    diagnostic_window_model_.horizon_seconds = budget_horizon[0];
    diagnostic_window_model_.nominal_time_seconds = nominal_time[0];
    friction_velocity_scale_ = velocity_scale[0];
    if (info.joints.size() != kJointCount) return false;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto& joint = info.joints[i];
      const auto entity = joints.find(joint.name);
      if (joint.name != kJointNames[i] || entity == joints.end()) return false;
      entities_[i] = entity->second;
      const auto* axis = ecm.Component<sim::components::JointAxis>(entities_[i]);
      if (!axis || !std::isfinite(axis->Data().Effort()) || axis->Data().Effort() <= 0.0) {
        RCLCPP_ERROR(node->get_logger(), "Missing effort limit: %s", joint.name.c_str());
        return false;
      }
      limits_[i] = axis->Data().Effort();
      double initial = 0.0;
      for (const auto& interface : joint.state_interfaces) {
        if (interface.name == "position" && !interface.initial_value.empty()) {
          initial = std::stod(interface.initial_value);
        }
      }
      if (!std::isfinite(initial)) return false;
      state_.position[i] = initial;
      state_.valid[i] = 1.0;
      if (!ecm.Component<sim::components::JointPosition>(entities_[i])) {
        ecm.CreateComponent(entities_[i], sim::components::JointPosition());
      }
      if (!ecm.Component<sim::components::JointVelocity>(entities_[i])) {
        ecm.CreateComponent(entities_[i], sim::components::JointVelocity());
      }
      ecm.CreateComponent(entities_[i], sim::components::JointPositionReset({initial}));
      ecm.CreateComponent(entities_[i], sim::components::JointVelocityReset({0.0}));
    }
    RCLCPP_INFO(node->get_logger(),
        "Motor-PD physics: one outer-cycle delay + %.3f ms extra + physics-step phase; "
        "local PD and configured dissipative joint friction every physics step", delay[0] * 1e3);
    return true;
  }

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override {
    return hardware_interface::SystemInterface::on_init(info);
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const std::string name(kJointNames[i]);
      interfaces.emplace_back(name, "position", &state_.position[i]);
      interfaces.emplace_back(name, "velocity", &state_.velocity[i]);
      // Requested simulated torque, not a physical torque sensor.
      interfaces.emplace_back(name, "effort", &state_.effort[i]);
    }
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const std::string name(kJointNames[i]);
      interfaces.emplace_back(name, "position", &command_.position[i]);
      interfaces.emplace_back(name, "velocity", &command_.velocity[i]);
      interfaces.emplace_back(name, "effort", &command_.effort[i]);
      interfaces.emplace_back(name, "kp", &command_.kp[i]);
      interfaces.emplace_back(name, "kd", &command_.kd[i]);
    }
    // 0 = disabled, 1 = immediate simulation pose/safe output,
    // 2 = active command through the one-cycle pipe plus optional extra delay.
    interfaces.emplace_back("custom_dog", "simulation_motor_mode", &command_mode_);
    return interfaces;
  }

  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>&,
      const std::vector<std::string>& stop) override {
    if (!stop.empty()) Reset();
    return hardware_interface::return_type::OK;
  }

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State&) override {
    Reset();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration&) override {
    // Hardware sends the previous write's packet at the next read. The PD
    // itself stays in the motor and does not wait for another host sample.
    ObserveTime(time);
    if (pending_valid_) {
      // Normal 500 Hz operation needs at most 25 packets for the allowed
      // 50 ms delay. Bound memory even if reads repeat at a frozen timestamp.
      if (delayed_.size() >= kMaxDelayedCommands) {
        Reset();
        RCLCPP_ERROR(nh_->get_logger(), "Motor simulation delay queue overflow; commands cleared");
      } else {
        delayed_.push_back({time.nanoseconds() + command_delay_nanoseconds_,
                            pending_created_nanoseconds_, pending_, pending_budget_});
      }
    }
    AdvanceCommands(time);
    ++read_epoch_;
    last_read_seconds_ = time.seconds();
    Sample(state_);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration&) override {
    ObserveTime(time);
    if ((command_mode_ != 1.0 && command_mode_ != 2.0) ||
        time.seconds() - last_read_seconds_ > 0.02) {
      ClearCommands();
      staged_mode_ = command_mode_;
    } else if (staged_epoch_ != read_epoch_ || command_mode_ != staged_mode_) {
      if (staged_mode_ == 2.0 && command_mode_ != 2.0) ReportStats(time.seconds());
      staged_epoch_ = read_epoch_;
      staged_mode_ = command_mode_;
      HybridJointCommand bounded;
      // In signed-window mode the controller has already covered delivery and
      // hold time. Recheck the instantaneous signed total without undoing the
      // QP's legitimate FF/PD cancellation with a second legacy L1 scaling.
      const bool valid = signed_budget_enabled_
          ? BoundedSignedMotorPdCommand(command_, state_, limits_,
                                        MotorPdWindowModel{}, bounded)
          : BoundedMotorPdCommand(command_, state_, limits_, bounded);
      if (!valid) {
        ClearCommands();
      } else if (command_mode_ == 1.0) {
        // Passive/FAULT output bypasses and clears every delayed motion packet.
        ClearCommands();
        active_ = bounded;
        active_created_nanoseconds_ = time.nanoseconds();
        active_timestamp_valid_ = true;
      } else {
        pending_ = bounded;
        pending_budget_ = {};
        if (signed_budget_enabled_) {
          // Read-only prediction telemetry for the packet that will actually
          // enter the queue; this does not clamp or alter physical execution.
          ComputeSignedMotorPdBudget(bounded, state_, limits_,
                                     diagnostic_window_model_, pending_budget_);
        }
        pending_created_nanoseconds_ = time.nanoseconds();
        pending_valid_ = true;
      }
    }
    AdvanceCommands(time);
    JointSample physical;
    Sample(physical);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (physical.valid[i] < 0.5) {
        ClearCommands();
        break;
      }
    }
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const double effort = EquivalentEffort(active_, physical, i);
      // Do not hide inter-sample PD peaks with a second software clamp that
      // the real motor protocol does not provide. Report requested torque;
      // do not assume an engine-side effort clamp has been verified.
      state_.effort[i] = physical.valid[i] > 0.5 && std::isfinite(effort) ? effort : 0.0;
      if (command_mode_ == 2.0 && active_timestamp_valid_ && physical.valid[i] > 0.5) {
        ++physics_joint_samples_;
        const double ratio = std::abs(state_.effort[i]) / limits_[i];
        over_nominal_samples_ += ratio > 1.0 + 1e-9;
        over_guard_samples_ += ratio > 1.10;
        if (signed_budget_enabled_ && active_budget_.all_feedforward_intervals_feasible) {
          ++signed_budget_samples_;
          const double pd_error = state_.effort[i] - active_.effort[i] - active_budget_.pd_nominal_nm[i];
          pd_prediction_error_abs_sum_ += std::abs(pd_error);
          pd_prediction_error_abs_max_ = std::max(pd_prediction_error_abs_max_, std::abs(pd_error));
          envelope_violation_samples_ +=
              state_.effort[i] < active_budget_.total_min_nm[i] - 1e-7 ||
              state_.effort[i] > active_budget_.total_max_nm[i] + 1e-7;
          stale_window_samples_ += (time.nanoseconds() - active_created_nanoseconds_) * 1e-9 >
              diagnostic_window_model_.horizon_seconds + 1e-9;
        }
        if (ratio > peak_ratio_) {
          peak_ratio_ = ratio;
          peak_joint_ = i;
          peak_ff_ = active_.effort[i];
          peak_p_ = active_.kp[i] * (active_.position[i] - physical.position[i]);
          peak_d_ = active_.kd[i] * (active_.velocity[i] - physical.velocity[i]);
          peak_time_ = time.seconds();
          peak_age_ms_ = (time.nanoseconds() - active_created_nanoseconds_) * 1e-6;
        }
        const double dt = time.seconds() - last_physics_sample_seconds_;
        if (last_physics_sample_seconds_ >= 0.0 && dt > 0.0 && dt <= 0.01) {
          max_sampled_acceleration_ = std::max(max_sampled_acceleration_,
              std::abs(physical.velocity[i] - previous_physical_velocity_[i]) / dt);
        }
      }
      previous_physical_velocity_[i] = physical.velocity[i];
      // Joint-side passive load, not extra actuator compensation. It remains
      // dissipative even when the motor is disabled and vanishes at rest.
      const double velocity = physical.velocity[i];
      const double friction = physical.valid[i] > 0.5
          ? -coulomb_friction_[i] * std::tanh(velocity / friction_velocity_scale_) -
                viscous_friction_[i] * velocity
          : 0.0;
      // Record the first inter-sample excursion for each joint in an enabled
      // episode. This is diagnostic only: neither the requested actuator torque
      // nor the passive friction is clamped or modified here. Logging once per
      // joint avoids a 1 kHz stream if the threshold remains exceeded.
      if (!effort_excursion_logged_[i] &&
          std::abs(state_.effort[i]) > limits_[i] * 1.10) {
        effort_excursion_logged_[i] = true;
        const double proportional =
            active_.kp[i] * (active_.position[i] - physical.position[i]);
        const double derivative =
            active_.kd[i] * (active_.velocity[i] - physical.velocity[i]);
        const double command_age_ms = active_timestamp_valid_
            ? (time.nanoseconds() - active_created_nanoseconds_) * 1e-6 : -1.0;
        RCLCPP_WARN(nh_->get_logger(),
            "Motor requested effort threshold crossing: sim_time=%.9f index=%zu joint=%s "
            "effort=%.9f limit=%.9f threshold=%.9f ff=%.9f p=%.9f d=%.9f "
            "q=%.9f dq=%.9f q_des=%.9f dq_des=%.9f kp=%.9f kd=%.9f "
            "command_age_ms=%.6f friction=%.9f",
            time.seconds(), i, std::string(kJointNames[i]).c_str(), state_.effort[i],
            limits_[i], limits_[i] * 1.10, active_.effort[i], proportional, derivative,
            physical.position[i], physical.velocity[i], active_.position[i],
            active_.velocity[i], active_.kp[i], active_.kd[i], command_age_ms, friction);
      }
      const double net = state_.effort[i] + friction;
      const double force_command = std::isfinite(net) ? net : 0.0;
      auto* force = ecm_->Component<sim::components::JointForceCmd>(entities_[i]);
      if (force) force->Data() = {force_command};
      else ecm_->CreateComponent(entities_[i], sim::components::JointForceCmd({force_command}));
    }
    last_physics_sample_seconds_ = time.seconds();
    if (command_mode_ == 2.0 && time.seconds() - last_stats_seconds_ >= 1.0) {
      ReportStats(time.seconds());
    }
    return hardware_interface::return_type::OK;
  }

 private:
  void ReportStats(double time) {
    last_stats_seconds_ = time;
    RCLCPP_INFO(nh_->get_logger(),
        "Motor physics cumulative stats: sim_time=%.9f joint_samples=%llu "
        "over_nominal=%llu over_guard=%llu peak_ratio=%.9f peak_time=%.9f "
        "peak_index=%zu peak_ff=%.9f peak_p=%.9f peak_d=%.9f peak_age_ms=%.6f "
        "max_sampled_acceleration=%.9f signed_budget=%d envelope_violations=%llu "
        "stale_window_samples=%llu signed_budget_samples=%llu pd_prediction_error_abs_sum=%.9f "
        "pd_prediction_error_abs_max=%.9f",
        time, static_cast<unsigned long long>(physics_joint_samples_),
        static_cast<unsigned long long>(over_nominal_samples_),
        static_cast<unsigned long long>(over_guard_samples_), peak_ratio_, peak_time_,
        peak_joint_, peak_ff_, peak_p_, peak_d_, peak_age_ms_, max_sampled_acceleration_,
        signed_budget_enabled_ ? 1 : 0,
        static_cast<unsigned long long>(envelope_violation_samples_),
        static_cast<unsigned long long>(stale_window_samples_),
        static_cast<unsigned long long>(signed_budget_samples_),
        pd_prediction_error_abs_sum_, pd_prediction_error_abs_max_);
  }
  static constexpr std::size_t kMaxDelayedCommands = 128;
  struct DelayedCommand {
    std::int64_t due_nanoseconds;
    std::int64_t created_nanoseconds;
    HybridJointCommand command;
    SignedMotorPdBudget budget;
  };

  void AdvanceCommands(const rclcpp::Time& time) {
    while (!delayed_.empty() && delayed_.front().due_nanoseconds <= time.nanoseconds()) {
      active_ = delayed_.front().command;
      active_budget_ = delayed_.front().budget;
      active_created_nanoseconds_ = delayed_.front().created_nanoseconds;
      active_timestamp_valid_ = true;
      delayed_.pop_front();
    }
  }

  void ObserveTime(const rclcpp::Time& time) {
    // Compare both reads and physics writes: a read after a reset must not
    // overwrite the old timestamp before the rollback can be detected.
    if (time_seen_ && time.nanoseconds() < last_time_nanoseconds_) Reset();
    last_time_nanoseconds_ = time.nanoseconds();
    time_seen_ = true;
  }

  void ClearCommands() {
    pending_ = {};
    active_ = {};
    pending_budget_ = active_budget_ = {};
    pending_valid_ = false;
    pending_created_nanoseconds_ = 0;
    active_created_nanoseconds_ = 0;
    active_timestamp_valid_ = false;
    effort_excursion_logged_.fill(false);
    delayed_.clear();
  }

  void Sample(JointSample& sample) {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto* q = ecm_->Component<sim::components::JointPosition>(entities_[i]);
      const auto* dq = ecm_->Component<sim::components::JointVelocity>(entities_[i]);
      const bool valid = q && dq && !q->Data().empty() && !dq->Data().empty() &&
          std::isfinite(q->Data()[0]) && std::isfinite(dq->Data()[0]);
      sample.valid[i] = valid ? 1.0 : 0.0;
      if (valid) {
        sample.position[i] = q->Data()[0];
        sample.velocity[i] = dq->Data()[0];
      }
    }
  }
  void Reset() {
    command_ = {};
    ClearCommands();
    command_mode_ = 0.0;
    staged_mode_ = 0.0;
  }
  sim::EntityComponentManager* ecm_ = nullptr;
  std::array<sim::Entity, kJointCount> entities_{};
  std::array<double, kJointCount> limits_{};
  std::array<double, kJointCount> coulomb_friction_{}, viscous_friction_{};
  std::array<bool, kJointCount> effort_excursion_logged_{};
  JointSample state_;
  HybridJointCommand command_, pending_, active_;
  double command_mode_ = 0.0;
  double staged_mode_ = 0.0;
  double friction_velocity_scale_ = 0.05;
  double last_read_seconds_ = 0.0;
  std::deque<DelayedCommand> delayed_;
  std::int64_t command_delay_nanoseconds_ = 0, last_time_nanoseconds_ = 0;
  std::int64_t pending_created_nanoseconds_ = 0, active_created_nanoseconds_ = 0;
  bool pending_valid_ = false, time_seen_ = false, active_timestamp_valid_ = false;
  bool signed_budget_enabled_ = false;
  MotorPdWindowModel diagnostic_window_model_;
  SignedMotorPdBudget pending_budget_, active_budget_;
  std::uint64_t envelope_violation_samples_ = 0, stale_window_samples_ = 0;
  std::uint64_t signed_budget_samples_ = 0;
  double pd_prediction_error_abs_sum_ = 0.0, pd_prediction_error_abs_max_ = 0.0;
  std::uint64_t physics_joint_samples_ = 0, over_nominal_samples_ = 0, over_guard_samples_ = 0;
  std::size_t peak_joint_ = 0;
  double peak_ratio_ = 0.0, peak_time_ = 0.0, peak_ff_ = 0.0, peak_p_ = 0.0, peak_d_ = 0.0;
  double peak_age_ms_ = 0.0, max_sampled_acceleration_ = 0.0;
  double last_physics_sample_seconds_ = -1.0, last_stats_seconds_ = 0.0;
  std::array<double, kJointCount> previous_physical_velocity_{};
  std::uint64_t read_epoch_ = 0, staged_epoch_ = 0;
};
}  // namespace custom_dog_control

PLUGINLIB_EXPORT_CLASS(custom_dog_control::FortressMotorSystem,
                      gz_ros2_control::GazeboSimSystemInterface)
