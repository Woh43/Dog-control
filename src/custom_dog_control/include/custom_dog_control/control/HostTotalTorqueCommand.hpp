#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "custom_dog_control/control/JointCommandUtils.hpp"

namespace custom_dog_control {

struct HostTotalTorqueBudget {
  std::array<double, kJointCount> pd_nm{};
  std::array<double, kJointCount> raw_total_nm{};
  std::array<double, kJointCount> sent_total_nm{};
  std::array<double, kJointCount> scale{};
  bool active = false;
};

// Explicit alternative actuator command structure: evaluate the same-sample
// hybrid command on the host, bound its total, then send a fixed effort with
// zero motor Kp/Kd. Delayed execution cannot grow this packet's requested
// effort as physical q/dq changes. This is not an assumed continuous saturation
// inside the drive, nor a guarantee about measured physical torque/quantization.
// Keep the pre-bound raw value in diagnostics; internal FF may exceed L because
// that residual is never itself sent to the motor in this mode.
// Call after command handoff/fallback. Invalid data disables the whole frame.
inline bool BoundedHostTotalTorqueCommand(
    const HybridJointCommand& command, const JointSample& measured,
    const std::array<double, kJointCount>& effort_limits,
    HybridJointCommand& output, HostTotalTorqueBudget* result = nullptr,
    double max_kp = 200.0, double max_kd = 10.0) {
  output = {};
  if (result) *result = {};
  if (!std::isfinite(max_kp) || max_kp < 0.0 ||
      !std::isfinite(max_kd) || max_kd < 0.0) return false;
  HostTotalTorqueBudget budget;
  HybridJointCommand next = command;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double limit = effort_limits[i];
    if (!std::isfinite(limit) || limit <= 0.0 ||
        !std::isfinite(measured.valid[i]) || measured.valid[i] < 0.5 ||
        !std::isfinite(measured.position[i]) ||
        !std::isfinite(measured.velocity[i]) ||
        !std::isfinite(command.position[i]) ||
        !std::isfinite(command.velocity[i]) ||
        !std::isfinite(command.effort[i]) ||
        !std::isfinite(command.kp[i]) || command.kp[i] < 0.0 ||
        command.kp[i] > max_kp ||
        !std::isfinite(command.kd[i]) || command.kd[i] < 0.0 ||
        command.kd[i] > max_kd) return false;
    budget.pd_nm[i] =
        command.kp[i] * (command.position[i] - measured.position[i]) +
        command.kd[i] * (command.velocity[i] - measured.velocity[i]);
    budget.raw_total_nm[i] = command.effort[i] + budget.pd_nm[i];
    if (!std::isfinite(budget.pd_nm[i]) ||
        !std::isfinite(budget.raw_total_nm[i])) return false;
    budget.sent_total_nm[i] =
        std::clamp(budget.raw_total_nm[i], -limit, limit);
    budget.scale[i] = std::abs(budget.raw_total_nm[i]) > limit
        ? limit / std::abs(budget.raw_total_nm[i]) : 1.0;
    next.effort[i] = budget.sent_total_nm[i];
    next.kp[i] = 0.0;
    next.kd[i] = 0.0;
  }
  budget.active = true;
  output = next;
  if (result) *result = budget;
  return true;
}

}  // namespace custom_dog_control
