#include "custom_dog_control/control/ImuFreshness.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace custom_dog_control {
namespace {

ImuSample ValidSample(double stamp) {
  ImuSample sample;
  sample.valid = true;
  sample.stamp_seconds = stamp;
  sample.linear_acceleration = {0.0, 0.0, 9.81};
  return sample;
}

TEST(ImuFreshness, SampleReceivedDuringHardwareReadUsesCheckTime) {
  // Deterministic ordering: cycle begins, hardware read runs, a new IMU
  // arrives, then update copies that snapshot and reads the same ROS clock.
  // This reproduces the timing defect, not any particular hardware incident.
  const double cycle_start = 10.000;
  const auto snapshot = ValidSample(10.001);
  const double after_snapshot = 10.002;
  EXPECT_EQ(EvaluateImuFreshness(snapshot, cycle_start, 0.1),
            ImuFreshnessStatus::FutureTimestamp);
  EXPECT_EQ(EvaluateImuFreshness(snapshot, after_snapshot, 0.1),
            ImuFreshnessStatus::Fresh);
  EXPECT_DOUBLE_EQ(snapshot.stamp_seconds, 10.001);
}

TEST(ImuFreshness, TrulyFutureSamplesHaveNoTolerance) {
  constexpr double check_time = 10.002;
  EXPECT_EQ(EvaluateImuFreshness(ValidSample(10.003), check_time, 0.1),
            ImuFreshnessStatus::FutureTimestamp);
  EXPECT_EQ(EvaluateImuFreshness(
                ValidSample(std::nextafter(check_time,
                    std::numeric_limits<double>::infinity())), check_time, 0.1),
            ImuFreshnessStatus::FutureTimestamp);
}

TEST(ImuFreshness, HardwareReadTimeCountsTowardStaleness) {
  const auto snapshot = ValidSample(9.901);
  EXPECT_EQ(EvaluateImuFreshness(snapshot, 10.000, 0.1),
            ImuFreshnessStatus::Fresh);
  EXPECT_EQ(EvaluateImuFreshness(snapshot, 10.002, 0.1),
            ImuFreshnessStatus::Stale);
}

TEST(ImuFreshness, TimeoutBoundaryIsInclusive) {
  // Binary-exact values distinguish the boundary from rounding effects.
  const auto snapshot = ValidSample(9.875);
  EXPECT_EQ(EvaluateImuFreshness(snapshot, 10.0, 0.125),
            ImuFreshnessStatus::Fresh);
  EXPECT_EQ(EvaluateImuFreshness(snapshot,
                std::nextafter(10.0, std::numeric_limits<double>::infinity()),
                0.125),
            ImuFreshnessStatus::Stale);
  EXPECT_EQ(EvaluateImuFreshness(ValidSample(10.0), 10.0, 0.125),
            ImuFreshnessStatus::Fresh);
}

TEST(ImuFreshness, ClockRollbackIsNotHiddenByTheCycleTimestamp) {
  const auto snapshot = ValidSample(10.001);
  EXPECT_EQ(EvaluateImuFreshness(snapshot, 10.002, 0.1),
            ImuFreshnessStatus::Fresh);
  EXPECT_EQ(EvaluateImuFreshness(snapshot, 9.999, 0.1),
            ImuFreshnessStatus::FutureTimestamp);
}

TEST(ImuFreshness, RepeatedReadsDoNotRefreshAcquisitionTime) {
  const auto snapshot = ValidSample(10.0);
  EXPECT_EQ(EvaluateImuFreshness(snapshot, 10.05, 0.1),
            ImuFreshnessStatus::Fresh);
  EXPECT_EQ(EvaluateImuFreshness(snapshot, 10.101, 0.1),
            ImuFreshnessStatus::Stale);
  EXPECT_DOUBLE_EQ(snapshot.stamp_seconds, 10.0);
}

TEST(ImuFreshness, InvalidSamplesAreRejectedEvenWhenCurrent) {
  auto snapshot = ValidSample(10.0);
  snapshot.valid = false;
  EXPECT_EQ(EvaluateImuFreshness(snapshot, 10.0, 0.1),
            ImuFreshnessStatus::InvalidSample);
  snapshot = ValidSample(10.0);
  snapshot.orientation_wxyz.fill(0.0);
  EXPECT_EQ(EvaluateImuFreshness(snapshot, 10.0, 0.1),
            ImuFreshnessStatus::InvalidSample);
  snapshot = ValidSample(10.0);
  snapshot.angular_velocity[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(EvaluateImuFreshness(snapshot, 10.0, 0.1),
            ImuFreshnessStatus::InvalidSample);
  snapshot = ValidSample(10.0);
  snapshot.linear_acceleration[2] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(EvaluateImuFreshness(snapshot, 10.0, 0.1),
            ImuFreshnessStatus::InvalidSample);
}

TEST(ImuFreshness, InvalidTimestampsAndTimeoutsFailClosed) {
  const double nonfinite[] = {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()};
  for (double value : nonfinite) {
    EXPECT_EQ(EvaluateImuFreshness(ValidSample(value), 10.0, 0.1),
              ImuFreshnessStatus::InvalidTimestamp);
    EXPECT_EQ(EvaluateImuFreshness(ValidSample(10.0), value, 0.1),
              ImuFreshnessStatus::InvalidTimestamp);
    EXPECT_EQ(EvaluateImuFreshness(ValidSample(10.0), 10.0, value),
              ImuFreshnessStatus::InvalidTimestamp);
  }
  for (double timeout : {0.0, -0.1}) {
    EXPECT_EQ(EvaluateImuFreshness(ValidSample(10.0), 10.0, timeout),
              ImuFreshnessStatus::InvalidTimestamp);
  }
}

}  // namespace
}  // namespace custom_dog_control
