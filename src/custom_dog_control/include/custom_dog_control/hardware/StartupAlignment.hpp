#pragma once

#include <cmath>

#include "custom_dog_control/hardware/ActuatorTypes.hpp"

namespace custom_dog_control {

// Once per SDK instance. This is the legacy timed, open-loop assistance, not a
// mechanical-zero detector. Normal motion authorization remains independent.
class StartupAlignment {
 public:
  enum class Stage {
    DISABLED = 0, WAITING = 1, HIP_THIGH = 2, ALL_JOINTS = 3,
    RELEASE = 4, DONE = 5, ABORTED = 6,
  };

  explicit StartupAlignment(bool enabled = false)
      : stage_(enabled ? Stage::WAITING : Stage::DISABLED) {}

  Stage stage() const { return stage_; }
  bool running() const {
    return stage_ == Stage::HIP_THIGH || stage_ == Stage::ALL_JOINTS ||
           stage_ == Stage::RELEASE;
  }
  bool canCalibrate() const {
    return stage_ == Stage::DISABLED || stage_ == Stage::DONE;
  }
  double forceDeadlineSeconds() const { return started_ + 0.600; }
  bool appliesTorque() const {
    return stage_ == Stage::HIP_THIGH || stage_ == Stage::ALL_JOINTS;
  }

  void Cancel() {
    if (stage_ == Stage::WAITING || running()) stage_ = Stage::ABORTED;
    completion_pending_ = false;
  }

  void BeginCycle(double now, bool allowed, bool previous_feedback_fresh) {
    if (canCalibrate() || stage_ == Stage::ABORTED) return;
    if (!std::isfinite(now)) { Cancel(); return; }
    if (stage_ == Stage::WAITING) {
      // Initial read cycles precede controller claims and its first write.
      if (!allowed || !previous_feedback_fresh) return;
      started_ = now;
      stage_ = Stage::HIP_THIGH;
    }
    if (!allowed || !previous_feedback_fresh || now < started_) {
      Cancel();
      return;
    }
    const double elapsed = now - started_;
    // Absolute deadlines prevent a late cycle from restarting/lengthening a
    // force phase. A stalled host still needs an independent drive watchdog.
    stage_ = elapsed < 0.300 ? Stage::HIP_THIGH :
             elapsed < 0.600 ? Stage::ALL_JOINTS : Stage::RELEASE;
    completion_pending_ = elapsed >= 0.900;
  }

  void FinishCycle(bool complete_valid_feedback) {
    if (!running()) return;
    if (!complete_valid_feedback) Cancel();
    else if (completion_pending_) stage_ = Stage::DONE;
    completion_pending_ = false;
  }

  LowLevelCommand command() const {
    LowLevelCommand output;
    if (!appliesTorque()) return output;  // Release/abort use mode0, not FOC.
    for (std::size_t i = 0; i < kJointCount; ++i) {
      auto& motor = output.motors[i];
      motor.mode = static_cast<std::uint32_t>(ActuatorControlMode::COMPOUND);
      const auto leg = i / kJointsPerLeg;
      const auto joint = i % kJointsPerLeg;
      // Legacy rotor signs expressed in the canonical URDF coordinates. The
      // SDK's ordinary direction/gear conversion is applied exactly once.
      motor.effort = joint == 0 ? ((leg == 0 || leg == 2) ? 0.95F : -0.95F) :
                     joint == 1 ? -0.63F :
                     stage_ == Stage::ALL_JOINTS ? -1.52F : 0.0F;
    }
    return output;
  }

 private:
  Stage stage_;
  double started_ = 0.0;
  bool completion_pending_ = false;
};

}  // namespace custom_dog_control
