#include <gtest/gtest.h>
#include <limits>

#include "custom_dog_control/control/ControlTypes.hpp"
#include "custom_dog_control/control/CalibrationGate.hpp"
#include "custom_dog_control/control/JoystickMapping.hpp"
#include "custom_dog_control/control/TimingStatistics.hpp"
#include "custom_dog_control/hardware/ActuatorTypes.hpp"
#include "custom_dog_control/hardware/Calibration.hpp"

namespace custom_dog_control {
namespace {

TEST(ControlContract, InvalidImuAxisCannotPoisonEstimator) {
  ImuSample imu;
  EXPECT_TRUE(FiniteImuSample(imu));
  for (std::size_t axis = 0; axis < 3; ++axis) {
    auto invalid = imu;
    invalid.angular_velocity[axis] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(FiniteImuSample(invalid));
    invalid = imu;
    invalid.linear_acceleration[axis] = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(FiniteImuSample(invalid));
  }
  for (std::size_t axis = 0; axis < 4; ++axis) {
    auto invalid = imu;
    invalid.orientation_wxyz[axis] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(FiniteImuSample(invalid));
  }
  imu.orientation_wxyz.fill(0.0);
  EXPECT_FALSE(FiniteImuSample(imu));
}

TEST(ControlContract, CalibrationEpochRejectsRawPoseAndResetsExactlyOnEdges) {
  CalibrationGate gate;
  EXPECT_FALSE(gate.Update(false));
  EXPECT_FALSE(gate.ready());
  EXPECT_TRUE(gate.Update(true));
  EXPECT_TRUE(gate.ready());
  EXPECT_FALSE(gate.Update(true));
  EXPECT_TRUE(gate.Update(false));  // Fault or zero reset.
  EXPECT_FALSE(gate.ready());
  EXPECT_FALSE(gate.Update(false));
  EXPECT_TRUE(gate.Update(true));  // Recalibration creates a fresh epoch.
  gate.Reset();
  EXPECT_FALSE(gate.ready());
  EXPECT_TRUE(gate.Update(true));  // Reactivation also reinitializes.
}

TEST(ControlContract, StartupAssistRequiresReleaseThenFreshStartAfterCompletion) {
  StartupCalibrationInputGate gate;
  EXPECT_FALSE(gate.Allow(false, false));  // Startup waiting.
  EXPECT_FALSE(gate.Allow(false, true));   // START during applied torque.
  EXPECT_FALSE(gate.Allow(true, true));    // Still held when release completes.
  EXPECT_FALSE(gate.Allow(true, true));    // Never replay a queued calibration.
  EXPECT_FALSE(gate.Allow(true, false));   // Release START after completion.
  EXPECT_TRUE(gate.Allow(true, true));     // Fresh START is admitted.
  EXPECT_FALSE(gate.Allow(false, true));  // Aborted assist cannot calibrate.
  gate.Reset();
  EXPECT_TRUE(gate.Allow(true, true));     // Assist disabled keeps old flow.
}

TEST(ControlContract, HeldStartIsNotReleasedByLtPriorityOrMissingInput) {
  StartupCalibrationInputGate gate;
  VelocityLimits limits;
  std::array<float, 6> axes{};
  std::array<int, 8> buttons{};
  buttons[7] = 1;
  EXPECT_FALSE(gate.Allow(false, true));  // START during assist.
  axes[2] = 1.0;
  buttons[0] = 1;
  auto input = MapJoystickInput(axes, buttons, limits, true, false, 1.0);
  ASSERT_EQ(input.requested_mode, JoystickModeRequest::STANCE);
  ASSERT_TRUE(input.calibration_button_valid);
  EXPECT_TRUE(input.calibration_pressed);
  EXPECT_FALSE(gate.Allow(true, input.calibration_pressed));
  EXPECT_FALSE(gate.Allow(true, false, false));  // Missing input is not release.
  buttons[0] = 0;
  input = MapJoystickInput(axes, buttons, limits, true, false, 2.0);
  ASSERT_EQ(input.requested_mode, JoystickModeRequest::CALIBRATE);
  EXPECT_FALSE(gate.Allow(true, input.calibration_pressed));
  buttons[7] = 0;
  input = MapJoystickInput(axes, buttons, limits, true, false, 3.0);
  EXPECT_FALSE(gate.Allow(true, input.calibration_pressed));
  EXPECT_FALSE(gate.Allow(true, false, false));  // Do not undo observed release.
  buttons[7] = 1;
  input = MapJoystickInput(axes, buttons, limits, true, false, 4.0);
  EXPECT_TRUE(gate.Allow(true, input.calibration_pressed));
}

TEST(ControlContract, JointAndFootOrderIsFixed) {
  EXPECT_EQ(kJointNames.front(), "FR_hip_joint");
  EXPECT_EQ(kJointNames[3], "FL_hip_joint");
  EXPECT_EQ(kJointNames[6], "RR_hip_joint");
  EXPECT_EQ(kJointNames[9], "RL_hip_joint");
  EXPECT_EQ(kFootFrameNames[0], "FR_foot");
  EXPECT_EQ(kFootFrameNames[3], "RL_foot");
}

TEST(ControlContract, DiagonalTrotUsesFixedTwoHundredFiftyMillisecondCycle) {
  EXPECT_EQ(TrotModeAtTime(0.00), kTrotFrRlMode);
  EXPECT_EQ(TrotModeAtTime(0.124), kTrotFrRlMode);
  EXPECT_EQ(TrotModeAtTime(0.125), kTrotFlRrMode);
  EXPECT_EQ(TrotModeAtTime(0.249), kTrotFlRrMode);
  EXPECT_EQ(TrotModeAtTime(0.250), kTrotFrRlMode);

  EXPECT_EQ(ContactFlags(kTrotFrRlMode),
            (std::array<bool, 4>{true, false, false, true}));
  EXPECT_EQ(ContactFlags(kTrotFlRrMode),
            (std::array<bool, 4>{false, true, true, false}));
}

TEST(ControlContract, ConfiguredTrotPeriodPreservesDiagonalOrderAndWraparound) {
  EXPECT_EQ(TrotModeAtTime(0.249, 0.5), kTrotFrRlMode);
  EXPECT_EQ(TrotModeAtTime(0.250, 0.5), kTrotFlRrMode);
  EXPECT_EQ(TrotModeAtTime(0.499, 0.5), kTrotFlRrMode);
  EXPECT_EQ(TrotModeAtTime(0.500, 0.5), kTrotFrRlMode);
  EXPECT_EQ(TrotModeAtTime(-0.001, 0.5), kTrotFlRrMode);
  EXPECT_THROW(TrotModeAtTime(0.0, 0.0), std::invalid_argument);
  EXPECT_THROW(TrotModeAtTime(0.0, -0.5), std::invalid_argument);
  EXPECT_THROW(TrotModeAtTime(0.0, std::numeric_limits<double>::infinity()),
               std::invalid_argument);
}

TEST(ControlContract, TrotOverlapKeepsBothDiagonalOrdersAndFullStanceTransfers) {
  const auto schedule = MakeTrotContactSchedule(0.5, 0.05, kTrotFrRlMode);
  ASSERT_EQ(schedule.times.size(), 5U);
  const std::array<double, 5> expected{0.0, 0.2, 0.25, 0.45, 0.5};
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(schedule.times[i], expected[i], 1e-12);
  }
  EXPECT_EQ(schedule.modes, (std::vector<std::size_t>{9, 15, 6, 15}));
  EXPECT_EQ(MakeTrotContactSchedule(0.5, 0.05, kTrotFlRrMode).modes,
            (std::vector<std::size_t>{6, 15, 9, 15}));
  EXPECT_EQ(MakeTrotContactSchedule(0.25, 0.0, kTrotFrRlMode).times,
            (std::vector<double>{0.0, 0.125, 0.25}));
  EXPECT_THROW(MakeTrotContactSchedule(0.5, 0.25, 9), std::invalid_argument);
  EXPECT_THROW(MakeTrotContactSchedule(0.5, -0.01, 9), std::invalid_argument);
  EXPECT_THROW(MakeTrotContactSchedule(0.5, 0.05, 15), std::invalid_argument);
}

TEST(ControlContract, TrotCommandThresholdsHaveHysteresis) {
  VelocityCommand command;
  EXPECT_FALSE(ExceedsTrotEntryThreshold(command));
  EXPECT_TRUE(IsBelowTrotExitThreshold(command));

  command.vx = 0.03;
  EXPECT_FALSE(ExceedsTrotEntryThreshold(command));
  EXPECT_FALSE(IsBelowTrotExitThreshold(command));

  command.vx = 0.05;
  EXPECT_TRUE(ExceedsTrotEntryThreshold(command));
  EXPECT_FALSE(IsBelowTrotExitThreshold(command));
}

TEST(ControlContract, Rep103ConversionOccursOnce) {
  EXPECT_DOUBLE_EQ(LegacyJoyLateralToRep103(0.1, true), -0.1);
  EXPECT_DOUBLE_EQ(LegacyJoyLateralToRep103(0.1, false), 0.1);
}

TEST(ControlContract, LegacyStartRequestsCalibrationWithoutStandOrStickMotion) {
  const std::vector<float> axes(6, 0.0F);
  std::vector<int> buttons(10, 0);
  buttons[7] = 1;
  const auto input = MapJoystickInput(axes, buttons, VelocityLimits{}, true, true, 12.5);
  EXPECT_EQ(input.requested_mode, JoystickModeRequest::CALIBRATE);
  EXPECT_FALSE(input.active);
  EXPECT_DOUBLE_EQ(input.velocity.vx, 0.0);
  EXPECT_DOUBLE_EQ(input.velocity.vy, 0.0);
  EXPECT_DOUBLE_EQ(input.velocity.yaw, 0.0);
  EXPECT_DOUBLE_EQ(input.stamp_seconds, 12.5);
  EXPECT_DOUBLE_EQ(input.velocity.stamp_seconds, 12.5);
}

TEST(ControlContract, LegacyFaceButtonsRequireTheOldLeftTriggerCombination) {
  std::vector<float> axes(6, 0.0F);
  std::vector<int> buttons(10, 0);
  const std::array<JoystickModeRequest, 4> expected{
      JoystickModeRequest::STANCE, JoystickModeRequest::PASSIVE,
      JoystickModeRequest::TROT, JoystickModeRequest::ESTOP};
  for (std::size_t button = 0; button < expected.size(); ++button) {
    SCOPED_TRACE(button);
    buttons.assign(10, 0);
    buttons[button] = 1;
    for (const float released : {-1.0F, 0.0F}) {
      axes[2] = released;
      EXPECT_EQ(MapJoystickInput(axes, buttons, {}, true, true, 0.0).requested_mode,
                JoystickModeRequest::NONE);
    }
    axes[2] = 1.0F;
    const auto input = MapJoystickInput(axes, buttons, {}, true, true, 0.0);
    EXPECT_EQ(input.requested_mode, expected[button]);
    EXPECT_FALSE(input.active);  // A trigger is not a velocity command.
  }
  buttons.assign(10, 0);
  axes[2] = 1.0F;
  EXPECT_EQ(MapJoystickInput(axes, buttons, {}, true, true, 0.0).requested_mode,
            JoystickModeRequest::NONE);
}

TEST(ControlContract, LegacyConflictingButtonsKeepTheOldStopAndModePriority) {
  std::vector<float> axes(6, 0.0F);
  std::vector<int> buttons(10, 0);
  axes[2] = 1.0F;
  buttons[0] = buttons[1] = buttons[2] = buttons[3] = buttons[7] = 1;
  EXPECT_EQ(MapJoystickInput(axes, buttons, {}, true, true, 0.0).requested_mode,
            JoystickModeRequest::PASSIVE);
  buttons[1] = 0;
  EXPECT_EQ(MapJoystickInput(axes, buttons, {}, true, true, 0.0).requested_mode,
            JoystickModeRequest::STANCE);
  buttons[0] = 0;
  EXPECT_EQ(MapJoystickInput(axes, buttons, {}, true, true, 0.0).requested_mode,
            JoystickModeRequest::TROT);
  buttons[2] = 0;
  EXPECT_EQ(MapJoystickInput(axes, buttons, {}, true, true, 0.0).requested_mode,
            JoystickModeRequest::ESTOP);
  buttons[3] = 0;
  EXPECT_EQ(MapJoystickInput(axes, buttons, {}, true, true, 0.0).requested_mode,
            JoystickModeRequest::CALIBRATE);
}

TEST(ControlContract, LegacyStickAxesPreservePhysicalMeaningAndReflectYawOnce) {
  const VelocityLimits limits;
  const std::vector<int> buttons(10, 0);
  for (const bool legacy_y_right : {false, true}) {
    for (const float sign : {-1.0F, 1.0F}) {
      for (const std::size_t axis : {0U, 1U, 3U}) {
        SCOPED_TRACE(axis);
        SCOPED_TRACE(sign);
        std::vector<float> axes(6, 0.0F);
        axes[axis] = sign;
        const auto input = MapJoystickInput(
            axes, buttons, limits, true, legacy_y_right, 1.0);
        EXPECT_DOUBLE_EQ(input.velocity.vx, axis == 1 ? -sign * limits.vx : 0.0);
        EXPECT_DOUBLE_EQ(input.velocity.vy, axis == 3 ? -sign * limits.vy : 0.0);
        EXPECT_DOUBLE_EQ(input.velocity.yaw, axis == 0 ? -sign * limits.yaw : 0.0);
        EXPECT_TRUE(input.active);
        EXPECT_EQ(input.requested_mode, JoystickModeRequest::NONE);
      }
    }
  }
}

TEST(ControlContract, LegacyDeadbandAppliesToEveryStickAxisWithoutRescaling) {
  const VelocityLimits limits;
  const std::vector<int> buttons(10, 0);
  for (const float sign : {-1.0F, 1.0F}) {
    for (const std::size_t axis : {0U, 1U, 3U}) {
      SCOPED_TRACE(axis);
      SCOPED_TRACE(sign);
      std::vector<float> axes(6, 0.0F);
      axes[axis] = sign * 0.079F;
      auto input = MapJoystickInput(axes, buttons, limits, true, true, 0.0);
      EXPECT_DOUBLE_EQ(input.velocity.vx, 0.0);
      EXPECT_DOUBLE_EQ(input.velocity.vy, 0.0);
      EXPECT_DOUBLE_EQ(input.velocity.yaw, 0.0);
      EXPECT_FALSE(input.active);
      axes[axis] = sign * 0.08F;  // Old killZeroOffset keeps the exact boundary.
      input = MapJoystickInput(axes, buttons, limits, true, true, 0.0);
      EXPECT_DOUBLE_EQ(input.velocity.vx, axis == 1 ? -axes[axis] * limits.vx : 0.0);
      EXPECT_DOUBLE_EQ(input.velocity.vy, axis == 3 ? -axes[axis] * limits.vy : 0.0);
      EXPECT_DOUBLE_EQ(input.velocity.yaw, axis == 0 ? -axes[axis] * limits.yaw : 0.0);
    }
  }
  std::vector<float> axes(6, 0.0F);
  axes[1] = 1.0F;
  axes[3] = -0.079F;
  axes[0] = 0.079F;
  const auto input = MapJoystickInput(axes, buttons, limits, true, true, 0.0);
  EXPECT_DOUBLE_EQ(input.velocity.vx, -limits.vx);
  EXPECT_DOUBLE_EQ(input.velocity.vy, 0.0);
  EXPECT_DOUBLE_EQ(input.velocity.yaw, 0.0);
}

TEST(ControlContract, LegacyIncompleteJoyFramesCannotRequestModesOrVelocity) {
  std::vector<float> axes(6, 1.0F);
  std::vector<int> buttons(10, 1);
  const auto check_invalid = [](const auto& input) {
    EXPECT_EQ(input.requested_mode, JoystickModeRequest::NONE);
    EXPECT_FALSE(input.active);
    EXPECT_FALSE(input.velocity.active);
    EXPECT_DOUBLE_EQ(input.velocity.vx, 0.0);
    EXPECT_DOUBLE_EQ(input.velocity.vy, 0.0);
    EXPECT_DOUBLE_EQ(input.velocity.yaw, 0.0);
  };
  axes.resize(5);
  check_invalid(MapJoystickInput(axes, buttons, {}, true, true, 0.0));
  axes.resize(6, 1.0F);
  buttons.resize(7);
  check_invalid(MapJoystickInput(axes, buttons, {}, true, true, 0.0));
  check_invalid(MapJoystickInput(std::vector<float>{}, std::vector<int>{},
                                {}, true, true, 0.0));
}

TEST(ControlContract, ModernJoystickLayoutAndSingleButtonRequestsRemainUnchanged) {
  const VelocityLimits limits;
  std::vector<float> axes{0.5F, -1.0F, -1.0F, -0.5F, 0.0F, 0.0F};
  std::vector<int> buttons(10, 0);
  buttons[0] = 1;
  auto input = MapJoystickInput(axes, buttons, limits, false, true, 0.0);
  EXPECT_DOUBLE_EQ(input.velocity.vx, -limits.vx);
  EXPECT_DOUBLE_EQ(input.velocity.vy, -0.5 * limits.vy);
  EXPECT_DOUBLE_EQ(input.velocity.yaw, -0.5 * limits.yaw);
  EXPECT_EQ(input.requested_mode, JoystickModeRequest::STANCE);
  buttons[0] = 0;
  buttons[2] = 1;
  EXPECT_EQ(MapJoystickInput(axes, buttons, limits, false, true, 0.0).requested_mode,
            JoystickModeRequest::TROT);
  buttons[3] = buttons[1] = 1;
  EXPECT_EQ(MapJoystickInput(axes, buttons, limits, false, true, 0.0).requested_mode,
            JoystickModeRequest::ESTOP);
  axes.assign(6, 0.0F);
  buttons.assign(10, 0);
  axes[0] = 0.05F;
  input = MapJoystickInput(axes, buttons, limits, false, false, 0.0);
  EXPECT_NEAR(input.velocity.vy, 0.05 * limits.vy, 1e-9);
}

TEST(ControlContract, StartCalibrationAlignsDirectlyToUrdfPronePose) {
  EXPECT_NEAR(kProneCalibrationPose[0], 0.0, 1e-12);
  EXPECT_NEAR(kProneCalibrationPose[1], 71.8 * kDegreesToRadians, 1e-12);
  EXPECT_NEAR(kProneCalibrationPose[2], -161.8 * kDegreesToRadians, 1e-12);

  constexpr double kGearRatio = 6.33;
  constexpr double kDirection = -1.0;
  constexpr double kRawMotorPosition = -9.5;
  const double uncalibrated = MotorPositionToUncalibratedUrdf(
      kRawMotorPosition, kGearRatio, kDirection);
  const double offset = CalibrationOffset(uncalibrated, 1);
  EXPECT_NEAR(
      CalibratedUrdfPosition(uncalibrated, offset),
      kProneCalibrationPose[1], 1e-12);
  EXPECT_NEAR(
      UrdfPositionToMotor(
          kProneCalibrationPose[1], offset, kGearRatio, kDirection),
      kRawMotorPosition, 1e-12);
}

TEST(ControlContract, RealHardwareUsesSharedFoldedCalibrationPose) {
  const DriveParameters parameters;
  EXPECT_NEAR(parameters.calibration_pose[0], 0.0, 1e-12);
  EXPECT_NEAR(
      parameters.calibration_pose[1], 71.8 * kDegreesToRadians, 1e-12);
  EXPECT_NEAR(
      parameters.calibration_pose[2], -161.8 * kDegreesToRadians, 1e-12);
}

TEST(ControlContract, VelocityIsClampedAndSlewLimited) {
  VelocityLimits limits;
  VelocityCommand requested;
  requested.vx = 1.0;
  requested.vy = -1.0;
  requested.yaw = 1.0;
  const auto clamped = ClampVelocity(requested, limits);
  EXPECT_DOUBLE_EQ(clamped.vx, 0.30);
  EXPECT_DOUBLE_EQ(clamped.vy, -0.10);
  EXPECT_DOUBLE_EQ(clamped.yaw, 0.30);

  const auto first = SlewVelocity({}, clamped, limits, 0.1);
  EXPECT_NEAR(first.vx, 0.06, 1e-12);
  EXPECT_NEAR(first.vy, -0.06, 1e-12);
  EXPECT_NEAR(first.yaw, 0.08, 1e-12);
}

TEST(ControlContract, TimingWindowIsBounded) {
  TimingWindow<4> timing;
  timing.Add(1.0);
  timing.Add(2.0);
  timing.Add(3.0);
  timing.Add(4.0);
  timing.Add(5.0);
  const auto snapshot = timing.Snapshot();
  EXPECT_EQ(snapshot.samples, 4U);
  EXPECT_DOUBLE_EQ(snapshot.mean, 3.5);
  EXPECT_DOUBLE_EQ(snapshot.p99, 5.0);
  EXPECT_DOUBLE_EQ(snapshot.maximum, 5.0);
}

}  // namespace
}  // namespace custom_dog_control
