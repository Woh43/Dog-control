#pragma once

namespace custom_dog_control {

// A START held before the startup assist finishes must not capture a zero
// later without a new press. Observe release after readiness before accepting.
class StartupCalibrationInputGate {
 public:
  bool Allow(bool ready, bool requested, bool input_valid = true) {
    if (!ready) {
      waiting_for_release_ = true;
      return false;
    }
    // Missing Joy cannot prove release, but also cannot undo a release that
    // was already observed. Preserve the pending/armed state across dropouts.
    if (!input_valid) return false;
    if (!requested) waiting_for_release_ = false;
    return requested && !waiting_for_release_;
  }
  void Reset() { waiting_for_release_ = false; }
 private:
  bool waiting_for_release_ = false;
};

// A zero change starts a new estimation epoch. No raw encoder pose may seed
// the estimator, and repeated calibrated frames must not reset its filter.
class CalibrationGate {
 public:
  bool Update(bool ready) {
    const bool changed = ready != ready_;
    ready_ = ready;
    return changed;
  }
  bool ready() const { return ready_; }
  void Reset() { ready_ = false; }
 private:
  bool ready_ = false;
};

}  // namespace custom_dog_control
