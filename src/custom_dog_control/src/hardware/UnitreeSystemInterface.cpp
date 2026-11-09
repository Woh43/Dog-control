#include "custom_dog_control/hardware/UnitreeSystemInterface.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <set>
#include <stdexcept>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>

#include "custom_dog_control/control/JointCommandUtils.hpp"

namespace custom_dog_control {
namespace {

bool ParseBool(const std::string& value) {
  if (value == "true" || value == "True" || value == "1") return true;
  if (value == "false" || value == "False" || value == "0") return false;
  throw std::invalid_argument("Expected a true/false hardware parameter");
}

template <typename T>
T ParameterOr(
    const hardware_interface::HardwareInfo& info,
    const std::string& name,
    const T& default_value);

template <>
double ParameterOr(
    const hardware_interface::HardwareInfo& info,
    const std::string& name,
    const double& default_value) {
  const auto iterator = info.hardware_parameters.find(name);
  return iterator == info.hardware_parameters.end()
             ? default_value
             : std::stod(iterator->second);
}

template <>
std::string ParameterOr(
    const hardware_interface::HardwareInfo& info,
    const std::string& name,
    const std::string& default_value) {
  const auto iterator = info.hardware_parameters.find(name);
  return iterator == info.hardware_parameters.end()
             ? default_value
             : iterator->second;
}

}  // namespace

hardware_interface::CallbackReturn UnitreeSystemInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.joints.size() != kJointCount) {
    RCLCPP_ERROR(rclcpp::get_logger("UnitreeSystemInterface"), "Expected 12 joints");
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (info_.joints[i].name != kJointNames[i]) {
      RCLCPP_ERROR(
          rclcpp::get_logger("UnitreeSystemInterface"),
          "Joint order mismatch at index %zu: expected %s, got %s",
          i, kJointNames[i].data(), info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    state_.valid[i] = 0.0;
  }
  const std::array<std::string, 5> joint_commands = {
      "position", "velocity", "effort", "kp", "kd"};
  for (std::size_t i = 0; i < kJointCount; ++i) {
    for (std::size_t type = 0; type < joint_commands.size(); ++type) {
      command_interface_names_[i * joint_commands.size() + type] =
          std::string(kJointNames[i]) + "/" + joint_commands[type];
    }
  }
  command_interface_names_[kJointCount * 5] = "custom_dog/calibrate";
  command_interface_names_[kJointCount * 5 + 1] = "custom_dog/emergency_stop";
  command_interface_names_[kJointCount * 5 + 2] = "custom_dog/motion_enabled";
  return ParseHardwareParameters()
             ? hardware_interface::CallbackReturn::SUCCESS
             : hardware_interface::CallbackReturn::ERROR;
}

bool UnitreeSystemInterface::ParseHardwareParameters() {
  try {
    drive_parameters_.serial_ports = {
        ParameterOr<std::string>(info_, "fr_port", "/dev/ttyS3"),
        ParameterOr<std::string>(info_, "fl_port", "/dev/ttyS4"),
        ParameterOr<std::string>(info_, "rr_port", "/dev/ttyS7"),
        ParameterOr<std::string>(info_, "rl_port", "/dev/ttyS8")};
    drive_parameters_.base_gear_ratio =
        ParameterOr<double>(info_, "base_gear_ratio", 6.33);
    drive_parameters_.calf_total_gear_ratio =
        ParameterOr<double>(info_, "calf_gear_ratio", 12.66);
    drive_parameters_.use_parallel_leg_io = ParseBool(
        ParameterOr<std::string>(info_, "parallel_leg_io", "true"));
    drive_parameters_.startup_alignment_enabled = ParseBool(
        ParameterOr<std::string>(info_, "startup_alignment_enabled", "false"));
    startup_alignment_ = drive_parameters_.startup_alignment_enabled ? 1.0 : 0.0;
    require_physical_estop_confirmation_ = ParseBool(
        ParameterOr<std::string>(info_, "require_physical_estop_confirmation", "true"));
    physical_estop_verified_ = ParseBool(
        ParameterOr<std::string>(info_, "physical_estop_verified", "false"));
    // These parameters govern an optional manual startup confirmation only.
    // The desktop may omit that interaction without claiming verification.
    // Neither flag is a real E-stop input; no physical E-stop level is read.
    physical_estop_ = 0.0;
    safe_damping_ = ParameterOr<double>(info_, "safe_damping", 1.0);
    drive_parameters_.calibration_pose = {
        ParameterOr<double>(
            info_, "calibration_hip", drive_parameters_.calibration_pose[0]),
        ParameterOr<double>(
            info_, "calibration_thigh", drive_parameters_.calibration_pose[1]),
        ParameterOr<double>(
            info_, "calibration_calf", drive_parameters_.calibration_pose[2])};

    std::stringstream directions(
        ParameterOr<std::string>(
            info_, "motor_directions",
            "1 -1 1 1 1 -1 -1 -1 1 -1 1 -1"));
    for (auto& direction : drive_parameters_.motor_directions) {
      if (!(directions >> direction) ||
          (direction != 1.0 && direction != -1.0)) {
        throw std::invalid_argument("motor_directions must contain twelve +/-1 values");
      }
    }
    std::string extra;
    if (directions >> extra) {
      throw std::invalid_argument("motor_directions has more than twelve values");
    }
    if (!std::isfinite(drive_parameters_.base_gear_ratio) ||
        drive_parameters_.base_gear_ratio <= 0.0 ||
        !std::isfinite(drive_parameters_.calf_total_gear_ratio) ||
        drive_parameters_.calf_total_gear_ratio <= 0.0 ||
        !std::isfinite(safe_damping_) || safe_damping_ < 0.0 || safe_damping_ > 10.0) {
      throw std::invalid_argument("Invalid gear ratio or safe damping");
    }
    std::set<std::string> ports;
    for (const auto& port : drive_parameters_.serial_ports) {
      if (port.empty() || !ports.insert(port).second) {
        throw std::invalid_argument("Each leg needs a distinct nonempty serial port");
      }
    }
    command_timeout_s_ = ParameterOr<double>(info_, "command_timeout_s", 0.02);
    if (!std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0.0 ||
        command_timeout_s_ > 0.1) {
      throw std::invalid_argument("command_timeout_s must be in (0, 0.1]");
    }
    urdf::Model model;
    if (!model.initString(info_.original_xml)) {
      throw std::invalid_argument("Cannot read joint effort limits from robot_description");
    }
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto joint = model.getJoint(std::string(kJointNames[i]));
      if (!joint || !joint->limits || !std::isfinite(joint->limits->effort) ||
          joint->limits->effort <= 0.0) {
        throw std::invalid_argument("Missing/invalid joint effort limit");
      }
      effort_limits_[i] = joint->limits->effort;
      const auto pose = drive_parameters_.calibration_pose[i % kJointsPerLeg];
      if (!std::isfinite(pose) || pose < joint->limits->lower || pose > joint->limits->upper) {
        throw std::invalid_argument("Calibration pose outside URDF joint limits");
      }
    }
    return true;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(
        rclcpp::get_logger("UnitreeSystemInterface"),
        "Invalid hardware parameters: %s", exception.what());
    return false;
  }
}

std::vector<hardware_interface::StateInterface>
UnitreeSystemInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kJointCount * 5 + 7);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const std::string name(kJointNames[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &state_.position[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &state_.velocity[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &state_.effort[i]);
    interfaces.emplace_back(name, "temperature", &state_.temperature[i]);
    interfaces.emplace_back(name, "valid", &state_.valid[i]);
  }
  interfaces.emplace_back("custom_dog", "calibrated", &calibrated_);
  interfaces.emplace_back("custom_dog", "communication_ok", &communication_ok_);
  interfaces.emplace_back("custom_dog", "physical_estop", &physical_estop_);
  interfaces.emplace_back("custom_dog", "io_period_ms", &io_period_ms_);
  interfaces.emplace_back("custom_dog", "io_timeout_count", &io_timeout_count_);
  interfaces.emplace_back("custom_dog", "motion_enabled", &motion_enabled_);
  interfaces.emplace_back("custom_dog", "startup_alignment", &startup_alignment_);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
UnitreeSystemInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kCommandInterfaceCount);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const std::string name(kJointNames[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &command_.position[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &command_.velocity[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &command_.effort[i]);
    interfaces.emplace_back(name, "kp", &command_.kp[i]);
    interfaces.emplace_back(name, "kd", &command_.kd[i]);
  }
  interfaces.emplace_back("custom_dog", "calibrate", &calibration_request_);
  interfaces.emplace_back("custom_dog", "emergency_stop", &emergency_stop_request_);
  interfaces.emplace_back("custom_dog", "motion_enabled", &motion_enabled_request_);
  return interfaces;
}

hardware_interface::CallbackReturn UnitreeSystemInterface::on_configure(
    const rclcpp_lifecycle::State&) {
  try {
    sdk_ = std::make_unique<IOSDK>(drive_parameters_);
    startup_alignment_ = static_cast<double>(sdk_->StartupAlignmentStage());
    SetDampingCommand();
    low_command_ = {};
    motion_enabled_request_ = 0.0;
    motion_enabled_ = 0.0;
    claimed_command_interfaces_.fill(false);
    command_written_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(
        rclcpp::get_logger("UnitreeSystemInterface"),
        "Failed to configure actuator SDK: %s", exception.what());
    sdk_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn UnitreeSystemInterface::on_activate(
    const rclcpp_lifecycle::State&) {
  if (require_physical_estop_confirmation_ && !physical_estop_verified_) {
    RCLCPP_ERROR(
        rclcpp::get_logger("UnitreeSystemInterface"),
        "Activation blocked: physical_estop_verified is false");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!sdk_) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    low_command_ = {};
    motion_enabled_request_ = 0.0;
    motion_enabled_ = 0.0;
    emergency_stop_request_ = 0.0;
    command_written_ = false;
    sdk_->ResetCalibration();
    calibrated_ = 0.0;
    sdk_->Activate();
    startup_alignment_ = static_cast<double>(sdk_->StartupAlignmentStage());
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(
        rclcpp::get_logger("UnitreeSystemInterface"),
        "Failed to activate actuator SDK: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn UnitreeSystemInterface::on_deactivate(
    const rclcpp_lifecycle::State&) {
  emergency_stop_request_ = 1.0;
  motion_enabled_request_ = 0.0;
  motion_enabled_ = 0.0;
  claimed_command_interfaces_.fill(false);
  command_written_ = false;
  low_command_ = {};
  if (sdk_) {
    sdk_->SendReceive(low_command_, low_state_, false);
    sdk_->Deactivate();
    startup_alignment_ = static_cast<double>(sdk_->StartupAlignmentStage());
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type UnitreeSystemInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  bool changed = false;
  bool released = false;
  for (std::size_t i = 0; i < command_interface_names_.size(); ++i) {
    const auto& name = command_interface_names_[i];
    if (std::find(stop_interfaces.begin(), stop_interfaces.end(), name) !=
        stop_interfaces.end()) {
      claimed_command_interfaces_[i] = false;
      changed = true;
      released = true;
    }
    if (std::find(start_interfaces.begin(), start_interfaces.end(), name) !=
        start_interfaces.end()) {
      claimed_command_interfaces_[i] = true;
      changed = true;
    }
  }
  // Claiming an interface never grants torque. A newly active controller must
  // explicitly write the motion switch after the claim; releasing any required
  // interface also discards the previously packed frame immediately.
  if (released && sdk_) {
    sdk_->CancelStartupAlignment();
    startup_alignment_ = static_cast<double>(sdk_->StartupAlignmentStage());
  }
  if (changed) {
    motion_enabled_request_ = 0.0;
    motion_enabled_ = 0.0;
    low_command_ = {};
    command_written_ = false;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnitreeSystemInterface::read(
    const rclcpp::Time&, const rclcpp::Duration&) {
  // 本实现的串口收发集中在 read：发送上一轮 write 打包的命令并获取反馈。
  // 因此 read -> update -> write 链路中的新控制量在下一次 read 才真正发出。
  if (!sdk_) {
    return hardware_interface::return_type::ERROR;
  }
  const auto start = std::chrono::steady_clock::now();
  // After a scheduling pause, never replay an old torque frame before acquiring
  // new feedback. This host watchdog cannot stop a powered motor while the host
  // itself is stalled; an independent drive/physical watchdog is still required.
  const bool command_fresh = command_written_ &&
      std::chrono::duration<double>(start - last_command_written_).count() <= command_timeout_s_;
  if (!command_fresh) {
    low_command_ = {};
    motion_enabled_ = 0.0;
  }
  if (calibration_request_ < -0.5) {
    sdk_->CancelStartupAlignment();
    sdk_->ResetCalibration();
    low_command_ = {};
    motion_enabled_ = 0.0;
  }
  if (emergency_stop_request_ != 0.0 || physical_estop_ > 0.5) {
    sdk_->CancelStartupAlignment();
  }
  const bool alignment_allowed = command_fresh && motion_enabled_request_ == 0.0 &&
      emergency_stop_request_ == 0.0 && physical_estop_ < 0.5 && calibration_request_ >= 0.0 &&
      std::all_of(claimed_command_interfaces_.begin(), claimed_command_interfaces_.end(),
                  [](bool claimed) { return claimed; });
  sdk_->SendReceive(low_command_, low_state_, calibration_request_ > 0.5, alignment_allowed);
  startup_alignment_ = static_cast<double>(sdk_->StartupAlignmentStage());
  io_period_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();

  for (std::size_t i = 0; i < kJointCount; ++i) {
    state_.position[i] = low_state_.motors[i].position;
    state_.velocity[i] = low_state_.motors[i].velocity;
    state_.effort[i] = low_state_.motors[i].effort;
    state_.temperature[i] = low_state_.motors[i].temperature;
    state_.valid[i] = low_state_.motors[i].fault == 0 &&
        std::isfinite(state_.position[i]) && std::isfinite(state_.velocity[i]) &&
        std::isfinite(state_.effort[i]) ? 1.0 : 0.0;
  }
  calibrated_ = sdk_->IsCalibrated() ? 1.0 : 0.0;
  communication_ok_ = sdk_->CommunicationOk() ? 1.0 : 0.0;
  // 累计的是电机通信失败次数；同一控制周期多电机失败会累计多次。
  io_timeout_count_ = static_cast<double>(sdk_->TotalFailures());
  consecutive_failures_ =
      sdk_->CommunicationOk() ? 0 : consecutive_failures_ + 1;
  if (!sdk_->CommunicationOk()) {
    low_command_ = {};
    motion_enabled_ = 0.0;
  }
  if (consecutive_failures_ >= 3) {
    sdk_->Deactivate();
    startup_alignment_ = static_cast<double>(sdk_->StartupAlignmentStage());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

void UnitreeSystemInterface::SetDampingCommand() {
  for (std::size_t i = 0; i < kJointCount; ++i) {
    command_.position[i] = state_.position[i];
    command_.velocity[i] = 0.0;
    command_.effort[i] = 0.0;
    command_.kp[i] = 0.0;
    command_.kd[i] = safe_damping_;
  }
}

bool UnitreeSystemInterface::FillLowCommand() {
  low_command_ = {};
  motion_enabled_ = 0.0;
  // Raw encoder angles before calibration are not URDF joint coordinates.
  // Uncalibrated, calibrating, reset and invalid-feedback cycles are stop-only.
  if (motion_enabled_request_ != 1.0 || emergency_stop_request_ != 0.0 ||
      !std::all_of(claimed_command_interfaces_.begin(),
                   claimed_command_interfaces_.end(), [](bool value) { return value; }) ||
      calibrated_ < 0.5 || communication_ok_ < 0.5 ||
      calibration_request_ > 0.5 || calibration_request_ < -0.5) {
    return true;
  }
  // Check against the float targets that IOSDK will actually receive, so target
  // conversion cannot enlarge the position/velocity error after this check.
  HybridJointCommand targets = command_;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    targets.position[i] = static_cast<float>(targets.position[i]);
    targets.velocity[i] = static_cast<float>(targets.velocity[i]);
  }
  HybridJointCommand bounded;
  if (!BoundedMotorPdCommand(targets, state_, effort_limits_, bounded)) {
    return false;
  }
  const auto rounded_toward_zero = [](double value) {
    float result = static_cast<float>(value);
    if (std::abs(static_cast<double>(result)) > std::abs(value)) {
      result = std::nextafter(result, 0.0F);
    }
    return result;
  };
  LowLevelCommand next_command;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    auto& motor = next_command.motors[i];
    motor.mode = static_cast<std::uint32_t>(ActuatorControlMode::COMPOUND);
    motor.position = static_cast<float>(bounded.position[i]);
    motor.velocity = static_cast<float>(bounded.velocity[i]);
    motor.effort = rounded_toward_zero(bounded.effort[i]);
    motor.kp = rounded_toward_zero(bounded.kp[i]);
    motor.kd = rounded_toward_zero(bounded.kd[i]);
    if (!std::isfinite(motor.effort) || !std::isfinite(motor.kp) ||
        !std::isfinite(motor.kd)) {
      return false;
    }
  }
  // Preserve the legacy five-component motor-side PD protocol. The effort
  // boundary above applies at this feedback sample; motor PD is not continuously
  // saturated by the host once feedback changes or SDK quantization is applied.
  low_command_ = next_command;
  motion_enabled_ = 1.0;
  return true;
}

hardware_interface::return_type UnitreeSystemInterface::write(
    const rclcpp::Time&, const rclcpp::Duration&) {
  // Switch values are discrete protocol values; rounding or thresholding an
  // invalid/NaN request must never turn on a drive.
  const bool valid_calibration = calibration_request_ == -1.0 ||
      calibration_request_ == 0.0 || calibration_request_ == 1.0;
  const bool valid_emergency_stop = emergency_stop_request_ == 0.0 ||
      emergency_stop_request_ == 1.0;
  const bool valid_motion_enabled = motion_enabled_request_ == 0.0 ||
      motion_enabled_request_ == 1.0;
  if (!valid_calibration || !valid_emergency_stop || !valid_motion_enabled) {
    low_command_ = {};
    motion_enabled_ = 0.0;
    motion_enabled_request_ = 0.0;
    command_written_ = false;
    if (sdk_) {
      sdk_->Deactivate();
      startup_alignment_ = static_cast<double>(sdk_->StartupAlignmentStage());
    }
    return hardware_interface::return_type::ERROR;
  }
  if (!FillLowCommand()) {
    // Reject the entire frame, stop all motors and force lifecycle recovery.
    // Do not leave the last valid nonzero frame latched after a write error.
    motion_enabled_request_ = 0.0;
    command_written_ = false;
    if (sdk_) {
      sdk_->Deactivate();
      startup_alignment_ = static_cast<double>(sdk_->StartupAlignmentStage());
    }
    return hardware_interface::return_type::ERROR;
  }
  last_command_written_ = std::chrono::steady_clock::now();
  command_written_ = true;
  return hardware_interface::return_type::OK;
}

}  // namespace custom_dog_control

PLUGINLIB_EXPORT_CLASS(
    custom_dog_control::UnitreeSystemInterface,
    hardware_interface::SystemInterface)
