#pragma once

#include <cmath>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

enum class ImuFreshnessStatus {
  Fresh,
  InvalidSample,
  InvalidTimestamp,
  FutureTimestamp,
  Stale,
};

// Read the input snapshot first, then obtain check_time from its ROS clock.
// A sample received during hardware I/O can be newer than the control cycle's
// start timestamp without being in the future when the controller checks it.
// Keep the acquisition stamp unchanged: I/O and transport delay count as age.
inline ImuFreshnessStatus EvaluateImuFreshness(
    const ImuSample& sample, double check_time, double timeout) {
  if (!std::isfinite(sample.stamp_seconds) || !std::isfinite(check_time) ||
      !std::isfinite(timeout) || timeout <= 0.0) {
    return ImuFreshnessStatus::InvalidTimestamp;
  }
  if (!sample.valid || !FiniteImuSample(sample)) {
    return ImuFreshnessStatus::InvalidSample;
  }
  if (sample.stamp_seconds > check_time) {
    return ImuFreshnessStatus::FutureTimestamp;
  }
  if (check_time - sample.stamp_seconds > timeout) {
    return ImuFreshnessStatus::Stale;
  }
  return ImuFreshnessStatus::Fresh;
}

inline const char* ImuFreshnessReason(ImuFreshnessStatus status) {
  switch (status) {
    case ImuFreshnessStatus::Fresh:
      return "IMU sample fresh";
    case ImuFreshnessStatus::InvalidSample:
      return "Invalid IMU sample";
    case ImuFreshnessStatus::InvalidTimestamp:
      return "Invalid IMU timestamp or freshness timeout";
    case ImuFreshnessStatus::FutureTimestamp:
      return "IMU timestamp is in the future";
    case ImuFreshnessStatus::Stale:
      return "IMU timeout";
  }
  return "Unknown IMU freshness status";
}

}  // namespace custom_dog_control
