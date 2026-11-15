#pragma once

#include <array>
#include <cmath>
#include <optional>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

// Slew only the desired joint-velocity input before WBC. The resulting input
// must also supply motor dq targets. Position targets are unchanged: this is
// input shaping, not a trajectory generator guaranteeing d(q_des)/dt=dq_des.
class JointVelocityReferenceLimiter {
 public:
  using JointArray = std::array<double, kJointCount>;
  // Same accepted step range as NmpcBackend::ComputeWbc. The caller must Reset
  // after a lifecycle gap even if its control period has already been clamped.
  static constexpr double kMaxStepSeconds = 0.1;

  void Reset() { initialized_ = false; }

  // Joint-side units: rad/s, rad/s^2, seconds. Zero acceleration limit disables
  // shaping for that joint, passes its raw value exactly and updates history.
  // First valid frame seeds history without inventing an acceleration from
  // zero. An invalid frame returns no output and never commits a partial
  // history; its next valid frame seeds anew across the invalid interval.
  std::optional<JointArray> Update(
      const JointArray& raw_velocity, const JointArray& max_acceleration,
      double dt) {
    if (!std::isfinite(dt) || dt <= 0.0 || dt > kMaxStepSeconds) {
      Reset();
      return std::nullopt;
    }
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (!std::isfinite(raw_velocity[i]) ||
          !std::isfinite(max_acceleration[i]) || max_acceleration[i] < 0.0) {
        Reset();
        return std::nullopt;
      }
    }

    JointArray next = raw_velocity;
    if (initialized_) {
      for (std::size_t i = 0; i < kJointCount; ++i) {
        if (max_acceleration[i] == 0.0) continue;
        const double max_step = max_acceleration[i] * dt;
        const double delta = raw_velocity[i] - previous_[i];
        if (delta > max_step) next[i] = previous_[i] + max_step;
        else if (delta < -max_step) next[i] = previous_[i] - max_step;
        if (!std::isfinite(next[i])) {
          Reset();
          return std::nullopt;
        }
      }
    }
    previous_ = next;
    initialized_ = true;
    return next;
  }

 private:
  JointArray previous_{};
  bool initialized_ = false;
};

}  // namespace custom_dog_control
