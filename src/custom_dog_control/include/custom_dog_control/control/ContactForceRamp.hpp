#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

// Bounds the QP's planned normal force after a new planned contact. These are
// model constraints, not measured foot forces or a touchdown detector.
class ContactForceRamp {
 public:
  void Reset() { initialized_ = false; age_.fill(0.0); previous_.fill(false); }

  std::array<double, kLegCount> Update(
      const std::array<bool, kLegCount>& contacts, double dt,
      double duration, double ramp_force) {
    if (!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(duration) ||
        duration < 0.0 || !std::isfinite(ramp_force) || ramp_force <= 0.0) {
      throw std::invalid_argument("invalid planned contact-force ramp");
    }
    std::array<double, kLegCount> bounds;
    for (std::size_t i = 0; i < kLegCount; ++i) {
      if (!initialized_) age_[i] = duration;  // Already-standing first WBC handoff.
      else if (!contacts[i] || !previous_[i]) age_[i] = 0.0;
      else age_[i] = std::min(duration, age_[i] + dt);
      bounds[i] = !contacts[i] ? 0.0 :
          (duration == 0.0 || age_[i] >= duration
               ? std::numeric_limits<double>::infinity()
               : ramp_force * age_[i] / duration);
    }
    previous_ = contacts;
    initialized_ = true;
    return bounds;
  }

 private:
  bool initialized_ = false;
  std::array<bool, kLegCount> previous_{};
  std::array<double, kLegCount> age_{};
};

}  // namespace custom_dog_control
