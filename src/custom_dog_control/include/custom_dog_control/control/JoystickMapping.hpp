#pragma once

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

enum class JoystickModeRequest : std::uint8_t {
  NONE,
  PASSIVE,
  CALIBRATE,
  STANCE,
  TROT,
  ESTOP,
};

struct JoystickInput {
  VelocityCommand velocity;
  JoystickModeRequest requested_mode = JoystickModeRequest::NONE;
  double stamp_seconds = 0.0;
  bool active = false;
  // Keep the physical START state even when an LT combination takes priority.
  bool calibration_button_valid = false;
  bool calibration_pressed = false;
};

// Match the old JoystickMapper's float deadband without rescaling the stick.
inline double LegacyJoystickAxis(double value) {
  constexpr double deadband = static_cast<double>(0.08F);
  if (!std::isfinite(value) || std::abs(value) < deadband) return 0.0;
  return std::clamp(value, -1.0, 1.0);
}

// Decode a complete input snapshot only. Calibration readiness and mode
// admission remain in the control thread's state machine, never in callbacks.
// Container arguments accept sensor_msgs::Joy arrays or synthetic test inputs.
template <typename Axes, typename Buttons>
JoystickInput MapJoystickInput(
    const Axes& axes, const Buttons& buttons, const VelocityLimits& limits,
    bool legacy_layout, bool legacy_y_right, double stamp_seconds) {
  JoystickInput input;
  input.stamp_seconds = stamp_seconds;
  input.velocity.stamp_seconds = stamp_seconds;
  const auto axis = [&](std::size_t index) -> double {
    return index < axes.size() ? axes[index] : 0.0;
  };
  const auto button = [&](std::size_t index) {
    return index < buttons.size() && buttons[index] != 0;
  };

  if (legacy_layout) {
    // The legacy event2joy layout contains six axes and at least eight buttons.
    // Reject incomplete frames instead of interpreting a missing trigger.
    if (axes.size() < 6 || buttons.size() < 8) return input;
    input.calibration_button_valid = true;
    input.calibration_pressed = button(7);
    // Old controller: vx=-LY, vy=RX (right), yaw=LX in its x-forward/y-right
    // geometry. Reflect y once into REP-103, which also reverses planar yaw.
    input.velocity.vx = -LegacyJoystickAxis(axis(1)) * limits.vx;
    input.velocity.vy = -LegacyJoystickAxis(axis(3)) * limits.vy;
    input.velocity.yaw = -LegacyJoystickAxis(axis(0)) * limits.yaw;
    const bool trigger_pressed = axis(2) > 0.0;
    if (trigger_pressed && button(1)) {
      input.requested_mode = JoystickModeRequest::PASSIVE;
    } else if (trigger_pressed && button(0)) {
      input.requested_mode = JoystickModeRequest::STANCE;
    } else if (trigger_pressed && button(2)) {
      input.requested_mode = JoystickModeRequest::TROT;
    } else if (trigger_pressed && button(3)) {
      // The removed legacy STEP_TEST has no NMPC/WBC speed-control equivalent.
      // Keep this explicit combination as the current software stop request.
      input.requested_mode = JoystickModeRequest::ESTOP;
    } else if (button(7)) {
      input.requested_mode = JoystickModeRequest::CALIBRATE;
    }
  } else {
    // Existing simulation, keyboard and standard Joy layout stays unchanged.
    input.calibration_button_valid = buttons.size() > 7;
    input.calibration_pressed = button(7);
    input.velocity.vx = axis(1) * limits.vx;
    input.velocity.vy = LegacyJoyLateralToRep103(
        axis(0) * limits.vy, legacy_y_right);
    input.velocity.yaw = axis(3) * limits.yaw;
    if (button(3)) {
      input.requested_mode = JoystickModeRequest::ESTOP;
    } else if (button(1)) {
      input.requested_mode = JoystickModeRequest::PASSIVE;
    } else if (button(7)) {
      input.requested_mode = JoystickModeRequest::CALIBRATE;
    } else if (button(2)) {
      input.requested_mode = JoystickModeRequest::TROT;
    } else if (button(0)) {
      input.requested_mode = JoystickModeRequest::STANCE;
    }
  }
  input.velocity.active = true;
  const double magnitude = std::max({std::abs(input.velocity.vx),
                                    std::abs(input.velocity.vy),
                                    std::abs(input.velocity.yaw)});
  // A mode button with centered sticks does not override a fresh /cmd_vel.
  input.active = magnitude > 0.01;
  return input;
}

}  // namespace custom_dog_control
