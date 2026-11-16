#pragma once

#include <cmath>
#include <ocs2_core/misc/LinearInterpolation.h>
#include "custom_dog_control/diagnostics/CycleTraceTypes.hpp"

namespace custom_dog_control::diagnostics {

// Trace-only scalar interpolation: matches MRT's segment/endpoint rule while
// avoiding an extra controller evaluation or allocating Eigen temporaries.
template <class Policy>
bool SampleCycleTraceNominal(
    const Policy& policy, double query, const std::array<int, kJointCount>& slots,
    std::array<double, kJointCount>& q, std::array<double, kJointCount>& dq,
    ControllerCycleRecord* trace = nullptr) noexcept {
  try {
    const auto& times = policy.timeTrajectory_;
    if (!std::isfinite(query) || times.empty() ||
        policy.stateTrajectory_.size() != times.size() ||
        policy.inputTrajectory_.size() != times.size()) return false;
    const auto segment = ocs2::LinearInterpolation::timeSegment(query, times);
    const auto left = static_cast<std::size_t>(segment.first);
    const auto right = times.size() == 1 ? left : left + 1;
    if (left >= times.size() || right >= times.size()) return false;
    for (const auto node : {left, right}) {
      if (policy.stateTrajectory_[node].size() != kCentroidalStateDim ||
          policy.inputTrajectory_[node].size() != kCentroidalInputDim) return false;
    }
    const double a = segment.second;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (slots[i] < 0 || slots[i] >= static_cast<int>(kJointCount)) return false;
      const auto slot = 12 + slots[i];
      const double ql = policy.stateTrajectory_[left](slot);
      const double qr = policy.stateTrajectory_[right](slot);
      const double vl = policy.inputTrajectory_[left](slot);
      const double vr = policy.inputTrajectory_[right](slot);
      if (!std::isfinite(ql) || !std::isfinite(qr) ||
          !std::isfinite(vl) || !std::isfinite(vr)) return false;
      q[i] = a * ql + (1.0 - a) * qr;
      dq[i] = a * vl + (1.0 - a) * vr;
      if (trace) {
        trace->node_left_position[i] = ql;
        trace->node_right_position[i] = qr;
        trace->node_left_velocity[i] = vl;
        trace->node_right_velocity[i] = vr;
      }
    }
    if (trace) {
      trace->query_time_s = query;
      trace->policy_first_time_s = times.front();
      trace->policy_last_time_s = times.back();
      trace->node_left_time_s = times[left];
      trace->node_right_time_s = times[right];
    }
    return true;
  } catch (...) { return false; }
}
}  // namespace custom_dog_control::diagnostics
