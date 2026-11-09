#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "custom_dog_control/nmpc/KinematicStateEstimator.hpp"
#include "custom_dog_control/safety/SafetyMonitor.hpp"

namespace custom_dog_control {
namespace {

SafetyLimits Limits() {
  SafetyLimits limits;
  limits.lower_position.fill(-2.0);
  limits.upper_position.fill(2.0);
  limits.effort_limit.fill(50.0);
  return limits;
}

SafetyInput ValidInput() {
  SafetyInput input;
  input.now_seconds = 10.0;
  input.imu.valid = true;
  input.imu.stamp_seconds = 10.0;
  input.joints.valid.fill(1.0);
  input.solver_valid = true;
  return input;
}

void AtTime(SafetyInput& input, double seconds) {
  input.now_seconds = seconds;
  input.imu.stamp_seconds = seconds;
}

TEST(SafetyMonitor, AcceptsNominalState) {
  SafetyMonitor monitor(Limits());
  EXPECT_TRUE(monitor.Evaluate(ValidInput()));
  EXPECT_FALSE(monitor.faulted());
}

TEST(SafetyMonitor, FaultIsLatchedUntilReset) {
  SafetyMonitor monitor(Limits());
  auto input = ValidInput();
  input.physical_estop = true;
  EXPECT_FALSE(monitor.Evaluate(input));
  EXPECT_EQ(monitor.reason(), "physical emergency stop");

  input.physical_estop = false;
  EXPECT_FALSE(monitor.Evaluate(input));
  monitor.Reset();
  EXPECT_TRUE(monitor.Evaluate(input));
}

TEST(SafetyMonitor, RejectsStaleDynamicPolicy) {
  SafetyMonitor monitor(Limits());
  auto input = ValidInput();
  input.dynamic_mode = true;
  input.policy_age_s = 0.11;
  EXPECT_FALSE(monitor.Evaluate(input));
  EXPECT_EQ(monitor.reason(), "NMPC policy stale or invalid");
}

TEST(SafetyMonitor, RejectsImuMotorAndLimitFailures) {
  {
    SafetyMonitor monitor(Limits());
    auto input = ValidInput();
    input.imu.stamp_seconds = 9.8;
    EXPECT_FALSE(monitor.Evaluate(input));
    EXPECT_EQ(monitor.reason(), "IMU timeout");
  }
  {
    SafetyMonitor monitor(Limits());
    auto input = ValidInput();
    input.joints.valid[4] = 0.0;
    EXPECT_FALSE(monitor.Evaluate(input));
    EXPECT_EQ(monitor.reason(), "invalid motor state");
  }
  {
    SafetyMonitor monitor(Limits());
    auto input = ValidInput();
    input.joints.position[7] = 2.1;
    EXPECT_FALSE(monitor.Evaluate(input));
    EXPECT_EQ(monitor.reason().find("joint position limit: index=7"), 0U);
  }
}

TEST(SafetyMonitor, JointLimitToleranceRejectsOnlyMaterialViolations) {
  const auto limits = Limits();
  {
    SafetyMonitor monitor(limits);
    auto input = ValidInput();
    input.joints.position[0] = 2.0 + 0.5 * limits.joint_position_tolerance_rad;
    EXPECT_TRUE(monitor.Evaluate(input));
  }
  {
    SafetyMonitor monitor(limits);
    auto input = ValidInput();
    input.joints.position[0] = 2.0 + 2.0 * limits.joint_position_tolerance_rad;
    EXPECT_FALSE(monitor.Evaluate(input));
    EXPECT_EQ(monitor.reason().find("joint position limit: index=0"), 0U);
  }
}

TEST(SafetyMonitor, DefaultInfiniteSpeedLimitsPreserveExistingEnvelope) {
  SafetyMonitor monitor(Limits());
  auto input = ValidInput();
  input.joints.velocity.fill(1000.0);
  input.horizontal_speed_m_s = 100.0;
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 20.0);
  EXPECT_TRUE(monitor.Evaluate(input));
}

TEST(SafetyMonitor, SustainedJointOverspeedTripsAndLatchesAfterDwell) {
  auto limits = Limits();
  limits.velocity_limit.fill(8.0);
  SafetyMonitor monitor(limits);
  auto input = ValidInput();
  input.joints.velocity[2] = -15.0;
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 10.099);
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 10.101);
  EXPECT_FALSE(monitor.Evaluate(input));
  EXPECT_EQ(monitor.reason().find("joint velocity limit: index=2"), 0U);
  input.joints.velocity[2] = 0.0;
  EXPECT_FALSE(monitor.Evaluate(input));
}

TEST(SafetyMonitor, BaseOverspeedHasIndependentDwellAndReason) {
  auto limits = Limits();
  limits.max_horizontal_speed_m_s = 0.6;
  SafetyMonitor monitor(limits);
  auto input = ValidInput();
  input.horizontal_speed_m_s = 0.9;
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 10.099);
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 10.101);
  EXPECT_FALSE(monitor.Evaluate(input));
  EXPECT_EQ(monitor.reason(), "base horizontal speed limit");
}

TEST(SafetyMonitor, IntermittentOverspeedDoesNotAccumulateAcrossRecovery) {
  auto limits = Limits();
  limits.velocity_limit.fill(8.0);
  SafetyMonitor monitor(limits);
  auto input = ValidInput();
  input.joints.velocity[0] = 9.0;
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 10.08);
  EXPECT_TRUE(monitor.Evaluate(input));
  input.joints.velocity[0] = 8.0;
  AtTime(input, 10.09);
  EXPECT_TRUE(monitor.Evaluate(input));
  input.joints.velocity[0] = 9.0;
  AtTime(input, 10.10);
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 10.19);
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 10.201);
  EXPECT_FALSE(monitor.Evaluate(input));
}

TEST(SafetyMonitor, DifferentSpeedFaultsDoNotShareTheirDwell) {
  auto limits = Limits();
  limits.velocity_limit.fill(8.0);
  limits.max_horizontal_speed_m_s = 0.6;
  SafetyMonitor monitor(limits);
  auto input = ValidInput();
  input.joints.velocity[0] = 9.0;
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 10.06);
  EXPECT_TRUE(monitor.Evaluate(input));
  input.joints.velocity[0] = 0.0;
  input.horizontal_speed_m_s = 0.9;
  AtTime(input, 10.07);
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 10.14);
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 10.171);
  EXPECT_FALSE(monitor.Evaluate(input));
  EXPECT_EQ(monitor.reason(), "base horizontal speed limit");
}

TEST(SafetyMonitor, PausedClockDoesNotAdvanceOverspeedDwell) {
  auto limits = Limits();
  limits.velocity_limit.fill(8.0);
  SafetyMonitor monitor(limits);
  auto input = ValidInput();
  input.joints.velocity[0] = 9.0;
  for (int i = 0; i < 100; ++i) EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 10.099);
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 10.101);
  EXPECT_FALSE(monitor.Evaluate(input));
}

TEST(SafetyMonitor, ClockRewindStartsNewOverspeedInterval) {
  auto limits = Limits();
  limits.max_horizontal_speed_m_s = 0.6;
  SafetyMonitor monitor(limits);
  auto input = ValidInput();
  input.horizontal_speed_m_s = 0.9;
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 10.08);
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 9.0);
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 9.099);
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 9.101);
  EXPECT_FALSE(monitor.Evaluate(input));
}

TEST(SafetyMonitor, ResetClearsBothOverspeedIntervals) {
  auto limits = Limits();
  limits.velocity_limit.fill(8.0);
  limits.max_horizontal_speed_m_s = 0.6;
  SafetyMonitor monitor(limits);
  auto input = ValidInput();
  input.joints.velocity[0] = 9.0;
  input.horizontal_speed_m_s = 0.9;
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 10.08);
  EXPECT_TRUE(monitor.Evaluate(input));
  monitor.Reset();
  AtTime(input, 20.0);
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 20.099);
  EXPECT_TRUE(monitor.Evaluate(input));
  AtTime(input, 20.101);
  EXPECT_FALSE(monitor.Evaluate(input));
}

TEST(SafetyMonitor, InvalidVelocitiesTripImmediatelyEvenWithInfiniteLimits) {
  for (double value : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity(),
                       -std::numeric_limits<double>::infinity()}) {
    {
      SafetyMonitor monitor(Limits());
      auto input = ValidInput();
      input.joints.velocity[5] = value;
      EXPECT_FALSE(monitor.Evaluate(input));
      EXPECT_EQ(monitor.reason(), "non-finite joint state");
    }
    {
      SafetyMonitor monitor(Limits());
      auto input = ValidInput();
      input.horizontal_speed_m_s = value;
      EXPECT_FALSE(monitor.Evaluate(input));
      EXPECT_EQ(monitor.reason(), "invalid horizontal speed");
    }
  }
}

TEST(SafetyMonitor, RejectsInvalidSpeedGuardParametersAndSafetyTime) {
  for (double value : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                       -std::numeric_limits<double>::infinity()}) {
    auto limits = Limits();
    limits.velocity_limit[3] = value;
    EXPECT_THROW(SafetyMonitor{limits}, std::invalid_argument);
    limits = Limits();
    limits.max_horizontal_speed_m_s = value;
    EXPECT_THROW(SafetyMonitor{limits}, std::invalid_argument);
  }
  for (double value : {-0.1, std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) {
    auto limits = Limits();
    limits.overspeed_duration_s = value;
    EXPECT_THROW(SafetyMonitor{limits}, std::invalid_argument);
  }
  SafetyMonitor monitor(Limits());
  auto input = ValidInput();
  input.now_seconds = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(monitor.Evaluate(input));
  EXPECT_EQ(monitor.reason(), "non-finite safety time");
}

TEST(SafetyMonitor, ZeroDwellMakesConfiguredOverspeedImmediate) {
  auto limits = Limits();
  limits.velocity_limit.fill(8.0);
  limits.overspeed_duration_s = 0.0;
  SafetyMonitor monitor(limits);
  auto input = ValidInput();
  input.joints.velocity[0] = 8.001;
  EXPECT_FALSE(monitor.Evaluate(input));
}

TEST(StateEstimator, NegativeYawDoesNotBecomeFlippedRollAndPitch) {
  constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
  const double yaw = -170.0 * kDegreesToRadians;
  const double pitch = 1.0 * kDegreesToRadians;
  const double roll = -2.0 * kDegreesToRadians;
  const Eigen::Matrix3d rotation =
      (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();

  const Eigen::Vector3d euler_zyx = EulerZyxFromRotation(rotation);
  EXPECT_NEAR(euler_zyx.x(), yaw, 1e-12);
  EXPECT_NEAR(euler_zyx.y(), pitch, 1e-12);
  EXPECT_NEAR(euler_zyx.z(), roll, 1e-12);
}

}  // namespace
}  // namespace custom_dog_control
