#pragma once

#include <array>
#include <cmath>

// A positive horizon is an engineering reachable-set assumption, not a verified
// motor saturation or guarantee that contact impulses obey the configured A.
#include "custom_dog_control/control/JointCommandUtils.hpp"

namespace custom_dog_control {

struct MotorPdWindowModel {
  // Must include feedback age + delivery delay + final hold/phase allowance.
  // Zero preserves the production sampled L1 arithmetic exactly.
  double horizon_seconds = 0.0;
  std::array<double, kJointCount> assumed_acceleration_bound_rad_s2{};
  // Signed-model EOM center: predict PD at D using constant measured velocity.
  // D is in [0,H]; it does not shorten the full-window bounds. The unsigned
  // legacy calculation validates this field but otherwise ignores its value.
  double nominal_time_seconds = 0.0;
};

struct MotorPdWindowBudget {
  std::array<double, kJointCount> feedforward_abs_nm{};
  std::array<double, kJointCount> position_now_abs_nm{};
  std::array<double, kJointCount> velocity_now_abs_nm{};
  std::array<double, kJointCount> position_change_bound_rad{};
  std::array<double, kJointCount> velocity_change_bound_rad_s{};
  std::array<double, kJointCount> position_growth_bound_nm{};
  std::array<double, kJointCount> velocity_growth_bound_nm{};
  // Before gain scaling. The QP may reserve this much for motor-side PD.
  std::array<double, kJointCount> pd_window_bound_nm{};
  // Before scaling: abs(FF) + PD window bound.
  std::array<double, kJointCount> total_bound_nm{};
  // Written only by the bounded-command helper, otherwise zero.
  std::array<double, kJointCount> scale{};
};

// The same calculation serves both pre-QP PD reservation (set FF=0) and the
// final hardware boundary. No assumption is made about cancellation of terms.
// Invalid data clears the entire result; partially filled arrays never escape.
inline bool ComputeMotorPdWindowBudget(
    const HybridJointCommand& command, const JointSample& measured,
    const MotorPdWindowModel& model, MotorPdWindowBudget& result,
    double max_kp = 200.0, double max_kd = 10.0) {
  result = {};
  if (!std::isfinite(model.horizon_seconds) || model.horizon_seconds < 0.0 ||
      !std::isfinite(model.nominal_time_seconds) || model.nominal_time_seconds < 0.0 ||
      model.nominal_time_seconds > model.horizon_seconds ||
      !std::isfinite(max_kp) || max_kp < 0.0 ||
      !std::isfinite(max_kd) || max_kd < 0.0) return false;
  MotorPdWindowBudget next;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double acceleration = model.assumed_acceleration_bound_rad_s2[i];
    if (!std::isfinite(acceleration) || acceleration < 0.0 ||
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
    next.feedforward_abs_nm[i] = std::abs(command.effort[i]);
    next.position_now_abs_nm[i] =
        command.kp[i] * std::abs(command.position[i] - measured.position[i]);
    next.velocity_now_abs_nm[i] =
        command.kd[i] * std::abs(command.velocity[i] - measured.velocity[i]);
    next.pd_window_bound_nm[i] =
        next.position_now_abs_nm[i] + next.velocity_now_abs_nm[i];
    // Match the old helper's summation order when H=0, including rounding.
    next.total_bound_nm[i] = next.feedforward_abs_nm[i] +
        next.position_now_abs_nm[i] + next.velocity_now_abs_nm[i];
    if (model.horizon_seconds > 0.0) {
      const double horizon = model.horizon_seconds;
      next.position_change_bound_rad[i] =
          std::abs(measured.velocity[i]) * horizon +
          0.5 * acceleration * horizon * horizon;
      next.velocity_change_bound_rad_s[i] = acceleration * horizon;
      next.position_growth_bound_nm[i] =
          command.kp[i] * next.position_change_bound_rad[i];
      next.velocity_growth_bound_nm[i] =
          command.kd[i] * next.velocity_change_bound_rad_s[i];
      next.pd_window_bound_nm[i] += next.position_growth_bound_nm[i] +
          next.velocity_growth_bound_nm[i];
      next.total_bound_nm[i] += next.position_growth_bound_nm[i] +
          next.velocity_growth_bound_nm[i];
    }
    if (!std::isfinite(next.feedforward_abs_nm[i]) ||
        !std::isfinite(next.position_now_abs_nm[i]) ||
        !std::isfinite(next.velocity_now_abs_nm[i]) ||
        !std::isfinite(next.position_change_bound_rad[i]) ||
        !std::isfinite(next.velocity_change_bound_rad_s[i]) ||
        !std::isfinite(next.position_growth_bound_nm[i]) ||
        !std::isfinite(next.velocity_growth_bound_nm[i]) ||
        !std::isfinite(next.pd_window_bound_nm[i]) ||
        !std::isfinite(next.total_bound_nm[i])) return false;
  }
  result = next;
  return true;
}

inline bool DelayAwareBoundedMotorPdCommand(
    const HybridJointCommand& command, const JointSample& measured,
    const std::array<double, kJointCount>& effort_limits,
    const MotorPdWindowModel& model, HybridJointCommand& output,
    MotorPdWindowBudget* result = nullptr,
    double max_kp = 200.0, double max_kd = 10.0) {
  output = {};
  if (result) *result = {};
  MotorPdWindowBudget budget;
  if (!ComputeMotorPdWindowBudget(command, measured, model, budget,
                                  max_kp, max_kd)) return false;
  // Validate all twelve limits before publishing any output or diagnostics.
  for (const double limit : effort_limits) {
    if (!std::isfinite(limit) || limit <= 0.0) return false;
  }
  HybridJointCommand next = command;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double scale = budget.total_bound_nm[i] > effort_limits[i]
        ? effort_limits[i] / budget.total_bound_nm[i] : 1.0;
    budget.scale[i] = scale;
    next.effort[i] *= scale;
    next.kp[i] *= scale;
    next.kd[i] *= scale;
  }
  output = next;
  if (result) *result = budget;
  return true;
}

}  // namespace custom_dog_control
