#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace custom_dog_control {

// XY world-frame position references for a velocity-commanded robot. Preserve
// the previous anchor only perpendicular to the current linear command; along
// that command the reference starts at the latest observed position. The bound
// is in the estimator's coordinates, not independent ground-truth coordinates.
// Limit0 and a near-zero linear command preserve the former full reanchor.
inline std::optional<std::array<double, 2>> BoundedCrossTrackAnchor(
    const std::array<double, 2>& previous_xy,
    const std::array<double, 2>& observed_xy,
    const std::array<double, 2>& world_linear_velocity,
    double max_cross_error_m) {
  if (!std::isfinite(max_cross_error_m) || max_cross_error_m < 0.) {
    return std::nullopt;
  }
  for (std::size_t i = 0; i < 2; ++i) {
    if (!std::isfinite(previous_xy[i]) || !std::isfinite(observed_xy[i]) ||
        !std::isfinite(world_linear_velocity[i])) {
      return std::nullopt;
    }
  }
  // Exact bypass avoids arithmetic perturbations to the compatibility path.
  if (max_cross_error_m == 0.) return observed_xy;
  const double speed = std::hypot(
      world_linear_velocity[0], world_linear_velocity[1]);
  if (!std::isfinite(speed)) return std::nullopt;
  constexpr double kMinimumLinearSpeedMps = 1e-6;
  if (speed <= kMinimumLinearSpeedMps) return observed_xy;

  const std::array<double, 2> normal{
      -world_linear_velocity[1] / speed, world_linear_velocity[0] / speed};
  const double dx = previous_xy[0] - observed_xy[0];
  const double dy = previous_xy[1] - observed_xy[1];
  const double cross_error = normal[0] * dx + normal[1] * dy;
  if (!std::isfinite(cross_error)) return std::nullopt;
  const double bounded_error = std::clamp(
      cross_error, -max_cross_error_m, max_cross_error_m);
  const std::array<double, 2> next{
      observed_xy[0] + normal[0] * bounded_error,
      observed_xy[1] + normal[1] * bounded_error};
  if (!std::isfinite(next[0]) || !std::isfinite(next[1])) return std::nullopt;
  return next;
}

}  // namespace custom_dog_control
