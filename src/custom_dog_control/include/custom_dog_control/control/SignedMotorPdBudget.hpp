#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "custom_dog_control/control/JointCommandWindowBudget.hpp"

namespace custom_dog_control {

struct SignedMotorPdBudget {
  // All quantities use joint-side SI units. These are command predictions,
  // not measured motor torque or a verified physical saturation model.
  // PD0 - Kp*dq0*D, for the model's nominal execution time D. This center is
  // a constant-velocity approximation; min/max still cover the entire [0,H].
  std::array<double, kJointCount> pd_nominal_nm{};
  std::array<double, kJointCount> pd_min_nm{};
  std::array<double, kJointCount> pd_max_nm{};
  // FF interval satisfying both total-window limits and the independent FF
  // field cap. A valid calculation may have an empty interval; callers must
  // check feasibility instead of silently changing gains before a WBC solve.
  std::array<double, kJointCount> feedforward_min_nm{};
  std::array<double, kJointCount> feedforward_max_nm{};
  std::array<bool, kJointCount> feedforward_interval_feasible{};
  bool all_feedforward_intervals_feasible = false;
  // Bounds for the supplied command, before any final boundary scaling.
  std::array<double, kJointCount> total_min_nm{};
  std::array<double, kJointCount> total_max_nm{};
  std::array<double, kJointCount> total_abs_bound_nm{};
  std::array<double, kJointCount> feedforward_abs_nm{};
  // Proposed common scale for FF, Kp and Kd. Computing it does not apply it.
  std::array<double, kJointCount> scale{};
};

// Fixed packet targets/gains, t in [0,H], and assumed |qdd(t)| <= A imply:
// PD(t) = PD0 - Kp*dq0*t +/- A*(0.5*Kp*t^2 + Kd*t).
// The lower envelope is concave and the upper envelope convex, so their
// extrema over the whole continuous window occur at an endpoint. No sampling
// or loss of signed FF/PD cancellation is needed to calculate these bounds.
// Nominal PD uses t=D, with zero assumed acceleration. Changing D alone changes
// neither the reachable interval nor FF feasibility or final boundary scaling.
// Using this center with current M/J is a zero-order dynamics approximation,
// not a complete prediction of the execution state or future contact mode.
//
// This is a conditional bound: H must cover feedback age, delivery and hold;
// contact impulses, stale packets or an uncalibrated A may invalidate it. It
// does not install continuous saturation inside a real motor. The caller
// decides when this mode is enabled; H=0 here means an instantaneous *signed*
// check, not the legacy absolute-contribution budget.
//
// Return false only for invalid input/arithmetic, atomically clearing result.
// Valid but infeasible FF intervals return true with feasibility flags false.
inline bool ComputeSignedMotorPdBudget(
    const HybridJointCommand& command, const JointSample& measured,
    const std::array<double, kJointCount>& effort_limits,
    const MotorPdWindowModel& model, SignedMotorPdBudget& result,
    double max_kp = 200.0, double max_kd = 10.0) {
  result = {};
  if (!std::isfinite(model.horizon_seconds) || model.horizon_seconds < 0.0 ||
      !std::isfinite(model.nominal_time_seconds) || model.nominal_time_seconds < 0.0 ||
      model.nominal_time_seconds > model.horizon_seconds ||
      !std::isfinite(max_kp) || max_kp < 0.0 ||
      !std::isfinite(max_kd) || max_kd < 0.0) return false;
  SignedMotorPdBudget next;
  next.all_feedforward_intervals_feasible = true;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double acceleration = model.assumed_acceleration_bound_rad_s2[i];
    const double limit = effort_limits[i];
    if (!std::isfinite(acceleration) || acceleration < 0.0 ||
        !std::isfinite(limit) || limit <= 0.0 ||
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
    const double pd0 =
        command.kp[i] * (command.position[i] - measured.position[i]) +
        command.kd[i] * (command.velocity[i] - measured.velocity[i]);
    double drift = 0.0;
    double uncertainty = 0.0;
    if (model.horizon_seconds > 0.0) {
      const double horizon = model.horizon_seconds;
      drift = -command.kp[i] * measured.velocity[i] * horizon;
      uncertainty = acceleration *
          (0.5 * command.kp[i] * horizon * horizon + command.kd[i] * horizon);
    }
    // Preserve the original instantaneous arithmetic exactly when D=0. In
    // particular, do not form a potentially overflowing Kp*dq0 product then
    // multiply it by zero for an otherwise valid instantaneous packet.
    next.pd_nominal_nm[i] = pd0;
    if (model.nominal_time_seconds > 0.0) {
      next.pd_nominal_nm[i] -=
          command.kp[i] * measured.velocity[i] * model.nominal_time_seconds;
    }
    next.pd_min_nm[i] = pd0 + std::min(0.0, drift - uncertainty);
    next.pd_max_nm[i] = pd0 + std::max(0.0, drift + uncertainty);
    next.feedforward_min_nm[i] = std::max(-limit, -limit - next.pd_min_nm[i]);
    next.feedforward_max_nm[i] = std::min(limit, limit - next.pd_max_nm[i]);
    next.feedforward_interval_feasible[i] =
        next.feedforward_min_nm[i] <= next.feedforward_max_nm[i];
    next.all_feedforward_intervals_feasible &= next.feedforward_interval_feasible[i];
    next.total_min_nm[i] = command.effort[i] + next.pd_min_nm[i];
    next.total_max_nm[i] = command.effort[i] + next.pd_max_nm[i];
    next.total_abs_bound_nm[i] =
        std::max(std::abs(next.total_min_nm[i]), std::abs(next.total_max_nm[i]));
    next.feedforward_abs_nm[i] = std::abs(command.effort[i]);
    const double demand =
        std::max(next.total_abs_bound_nm[i], next.feedforward_abs_nm[i]);
    next.scale[i] = demand > limit ? limit / demand : 1.0;
    if (!std::isfinite(pd0) || !std::isfinite(next.pd_nominal_nm[i]) ||
        !std::isfinite(drift) ||
        !std::isfinite(uncertainty) || !std::isfinite(next.pd_min_nm[i]) ||
        !std::isfinite(next.pd_max_nm[i]) ||
        !std::isfinite(next.feedforward_min_nm[i]) ||
        !std::isfinite(next.feedforward_max_nm[i]) ||
        !std::isfinite(next.total_min_nm[i]) ||
        !std::isfinite(next.total_max_nm[i]) ||
        !std::isfinite(next.total_abs_bound_nm[i]) ||
        !std::isfinite(next.scale[i])) return false;
  }
  result = next;
  return true;
}

// Final output boundary only: preserve q/dq and jointly scale FF/Kp/Kd if
// either the signed total-window bound or independent FF field cap exceeds L.
// The PD envelopes are positively homogeneous in Kp/Kd, so this same scale
// shrinks the entire requested-torque interval. A WBC solve using the same
// inputs and feasible FF interval should already have scale=1 apart from
// numerical tolerance; a later handoff or changed state may require fallback.
inline bool BoundedSignedMotorPdCommand(
    const HybridJointCommand& command, const JointSample& measured,
    const std::array<double, kJointCount>& effort_limits,
    const MotorPdWindowModel& model, HybridJointCommand& output,
    SignedMotorPdBudget* result = nullptr,
    double max_kp = 200.0, double max_kd = 10.0) {
  output = {};
  if (result) *result = {};
  SignedMotorPdBudget budget;
  if (!ComputeSignedMotorPdBudget(command, measured, effort_limits, model,
                                  budget, max_kp, max_kd)) return false;
  HybridJointCommand next = command;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    next.effort[i] *= budget.scale[i];
    next.kp[i] *= budget.scale[i];
    next.kd[i] *= budget.scale[i];
  }
  output = next;
  if (result) *result = budget;
  return true;
}

}  // namespace custom_dog_control
