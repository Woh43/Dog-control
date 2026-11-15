#pragma once

#include <array>
#include <cmath>
#include <optional>

#include <ocs2_core/misc/LinearInterpolation.h>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

struct NominalJointReference {
  // OCS2 model order. The backend owns the model-to-hardware permutation.
  std::array<double, kJointCount> position_model_rad{};
  std::array<double, kJointCount> velocity_model_rad_s{};
  std::size_t mode = kStanceMode;
};

// Pass one active mrt->getPolicy() snapshot, queried on the control thread
// without another updatePolicy() between state/input/mode sampling. The template
// avoids an ocs2_oc/controller dependency: PrimalSolution supplies these fields.
//
// Uses exactly the OCS2 interpolation rule, including its lower/pre-event side
// at duplicate timestamps and zero-order extrapolation. The caller still owns
// policy-age/horizon checks. This samples the optimizer's nominal q and dq, not
// controller(t, x_nominal), whose interpolated gain/bias can add a cross term.
// It is not a new continuous trajectory generator: interpolation discretization,
// policy replacement and subsequent safety projection can break q_dot == dq.
template <typename PrimalSolution>
std::optional<NominalJointReference> SampleNominalJointReference(
    const PrimalSolution& policy, double evaluation_time, std::size_t expected_mode) {
  const auto& times = policy.timeTrajectory_;
  const auto& states = policy.stateTrajectory_;
  const auto& inputs = policy.inputTrajectory_;
  const auto& schedule = policy.modeSchedule_;
  if (!std::isfinite(evaluation_time) || times.empty() ||
      states.size() != times.size() || inputs.size() != times.size() ||
      schedule.modeSequence.size() != schedule.eventTimes.size() + 1) {
    return std::nullopt;
  }
  for (std::size_t i = 0; i < times.size(); ++i) {
    if (!std::isfinite(times[i]) || (i > 0 && times[i] < times[i - 1])) return std::nullopt;
  }
  for (std::size_t i = 0; i < schedule.eventTimes.size(); ++i) {
    if (!std::isfinite(schedule.eventTimes[i]) ||
        (i > 0 && schedule.eventTimes[i] < schedule.eventTimes[i - 1])) return std::nullopt;
  }
  const auto mode = schedule.modeAtTime(evaluation_time);
  if (mode != expected_mode) return std::nullopt;

  const auto segment = ocs2::LinearInterpolation::timeSegment(evaluation_time, times);
  if (segment.first < 0 || static_cast<std::size_t>(segment.first) >= times.size()) return std::nullopt;
  const auto first = static_cast<std::size_t>(segment.first);
  const auto last = times.size() == 1 ? first : first + 1;
  if (last >= times.size()) return std::nullopt;
  // Validate both interpolation operands before Eigen arithmetic; OCS2's
  // differing-dimension snap fallback must not silently accept a broken policy.
  for (const auto index : {first, last}) {
    if (states[index].size() != static_cast<Eigen::Index>(kCentroidalStateDim) ||
        inputs[index].size() != static_cast<Eigen::Index>(kCentroidalInputDim) ||
        !states[index].allFinite() || !inputs[index].allFinite()) return std::nullopt;
  }
  const auto state = ocs2::LinearInterpolation::interpolate(segment, states);
  const auto input = ocs2::LinearInterpolation::interpolate(segment, inputs);
  if (!state.allFinite() || !input.allFinite()) return std::nullopt;
  NominalJointReference result;
  result.mode = mode;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    result.position_model_rad[i] = state(12 + i);
    result.velocity_model_rad_s[i] = input(12 + i);
  }
  return result;
}

}  // namespace custom_dog_control
