#include "custom_dog_control/safety/SafetyMonitor.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace custom_dog_control {

SafetyMonitor::SafetyMonitor(SafetyLimits limits) : limits_(std::move(limits)) {
  // Positive infinity intentionally disables a limit; NaN, negative infinity,
  // zero and negative limits are configuration errors, never silent disables.
  const auto valid_limit = [](double value) { return value > 0.0 && !std::isnan(value); };
  for (double limit : limits_.velocity_limit) {
    if (!valid_limit(limit)) {
      throw std::invalid_argument("joint velocity limits must be positive or +infinity");
    }
  }
  if (!valid_limit(limits_.max_horizontal_speed_m_s) ||
      !std::isfinite(limits_.overspeed_duration_s) || limits_.overspeed_duration_s < 0.0) {
    throw std::invalid_argument("invalid horizontal speed limit or overspeed duration");
  }
}

bool SafetyMonitor::Trip(const std::string& reason) {
  faulted_ = true;
  reason_ = reason;
  return false;
}

bool SafetyMonitor::Evaluate(const SafetyInput& input) {
  // 保留首次故障及其原因，后续周期的派生异常不覆盖排障线索。
  if (faulted_) {
    return false;
  }
  if (!std::isfinite(input.now_seconds)) {
    return Trip("non-finite safety time");
  }
  if (!std::isfinite(input.horizontal_speed_m_s) || input.horizontal_speed_m_s < 0.0) {
    return Trip("invalid horizontal speed");
  }
  if (input.physical_estop) {
    return Trip("physical emergency stop");
  }
  if (input.software_estop) {
    return Trip("software emergency stop");
  }
  if (!input.communication_ok ||
      input.consecutive_io_failures >= limits_.max_consecutive_io_failures) {
    return Trip("motor communication failure");
  }
  if (!input.imu.valid || input.now_seconds - input.imu.stamp_seconds > limits_.imu_timeout_s) {
    return Trip("IMU timeout");
  }
  if (std::abs(input.roll) > limits_.max_roll_pitch_rad ||
      std::abs(input.pitch) > limits_.max_roll_pitch_rad) {
    return Trip("base attitude limit");
  }
  if (input.dynamic_mode &&
      (!input.solver_valid || input.policy_age_s > limits_.max_policy_age_s)) {
    return Trip("NMPC policy stale or invalid");
  }
  bool joint_overspeed = false;
  std::size_t overspeed_joint = 0;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!std::isfinite(input.joints.position[i]) ||
        !std::isfinite(input.joints.velocity[i]) ||
        !std::isfinite(input.joints.effort[i])) {
      return Trip("non-finite joint state");
    }
    if (input.joints.valid[i] < 0.5) {
      return Trip("invalid motor state");
    }
    if (std::abs(input.joints.velocity[i]) > limits_.velocity_limit[i]) {
      joint_overspeed = true;
      overspeed_joint = i;
    }
    if (input.joints.temperature[i] > limits_.max_temperature_c) {
      return Trip("motor over-temperature");
    }
    if (input.check_joint_limits &&
        (input.joints.position[i] <
             limits_.lower_position[i] - limits_.joint_position_tolerance_rad ||
         input.joints.position[i] >
             limits_.upper_position[i] + limits_.joint_position_tolerance_rad)) {
      std::ostringstream reason;
      reason << "joint position limit: index=" << i
             << " position=" << input.joints.position[i]
             << " lower=" << limits_.lower_position[i]
             << " upper=" << limits_.upper_position[i];
      return Trip(reason.str());
    }
    // 对实测力矩留出 10% 瞬态裕量；这是故障判据，命令限幅在输出链路完成。
    if (std::abs(input.joints.effort[i]) > limits_.effort_limit[i] * 1.10) {
      return Trip("joint effort limit");
    }
  }
  // Dwell uses the caller's ROS clock. A paused simulation contributes no
  // elapsed time; a clock reset must not inherit an interval from the future.
  if (last_speed_check_seconds_ && input.now_seconds < *last_speed_check_seconds_) {
    joint_overspeed_since_.reset();
    base_overspeed_since_.reset();
  }
  last_speed_check_seconds_ = input.now_seconds;
  const auto sustained = [&](bool exceeds, std::optional<double>& since) {
    if (!exceeds) {
      since.reset();
      return false;
    }
    if (!since) since = input.now_seconds;
    return input.now_seconds - *since >= limits_.overspeed_duration_s;
  };
  if (sustained(joint_overspeed, joint_overspeed_since_)) {
    std::ostringstream reason;
    reason << "joint velocity limit: index=" << overspeed_joint
           << " velocity=" << input.joints.velocity[overspeed_joint]
           << " limit=" << limits_.velocity_limit[overspeed_joint];
    return Trip(reason.str());
  }
  if (sustained(input.horizontal_speed_m_s > limits_.max_horizontal_speed_m_s,
                base_overspeed_since_)) {
    return Trip("base horizontal speed limit");
  }
  return true;
}

void SafetyMonitor::Reset() {
  faulted_ = false;
  reason_.clear();
  joint_overspeed_since_.reset();
  base_overspeed_since_.reset();
  last_speed_check_seconds_.reset();
}

}  // namespace custom_dog_control
