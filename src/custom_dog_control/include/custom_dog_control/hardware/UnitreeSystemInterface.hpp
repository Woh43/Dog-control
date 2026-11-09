#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>

#include "custom_dog_control/control/ControlTypes.hpp"
#include "custom_dog_control/hardware/ActuatorTypes.hpp"
#include "custom_dog_control/hardware/IOSDK.hpp"

namespace custom_dog_control {

class UnitreeSystemInterface final : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UnitreeSystemInterface)

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  bool ParseHardwareParameters();
  void SetDampingCommand();
  bool FillLowCommand();

  DriveParameters drive_parameters_;
  std::unique_ptr<IOSDK> sdk_;
  LowLevelCommand low_command_;
  LowLevelState low_state_;

  JointSample state_;
  HybridJointCommand command_;
  double calibration_request_ = 0.0;
  double emergency_stop_request_ = 0.0;
  double motion_enabled_request_ = 0.0;
  // Applied host authorization, not motor-reported drive mode.
  double motion_enabled_ = 0.0;
  static constexpr std::size_t kCommandInterfaceCount = kJointCount * 5 + 3;
  std::array<std::string, kCommandInterfaceCount> command_interface_names_{};
  std::array<bool, kCommandInterfaceCount> claimed_command_interfaces_{};
  double calibrated_ = 0.0;
  double communication_ok_ = 0.0;
  double physical_estop_ = 0.0;
  double startup_alignment_ = 0.0;
  double io_period_ms_ = 0.0;
  double io_timeout_count_ = 0.0;
  double safe_damping_ = 1.0;
  std::array<double, kJointCount> effort_limits_{};
  std::chrono::steady_clock::time_point last_command_written_{};
  double command_timeout_s_ = 0.02;
  bool command_written_ = false;
  bool require_physical_estop_confirmation_ = true;
  bool physical_estop_verified_ = false;
  int consecutive_failures_ = 0;
};

}  // namespace custom_dog_control
