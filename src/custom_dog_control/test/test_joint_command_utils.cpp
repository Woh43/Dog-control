#include <gtest/gtest.h>
#include <limits>

#include "custom_dog_control/control/JointCommandUtils.hpp"
#include "custom_dog_control/hardware/ActuatorTypes.hpp"
#include "custom_dog_control/hardware/Calibration.hpp"

namespace custom_dog_control {
namespace {

TEST(JointCommand, FinalOutputClampsCombinedTorqueAndRejectsInvalidFeedback) {
  JointSample measured;
  HybridJointCommand command;
  command.position[0] = 1.;
  command.kp[0] = 50.;
  command.effort[0] = 5.;
  EXPECT_DOUBLE_EQ(BoundedEquivalentEffort(command, measured, 0, 10.), 10.);
  command.position[0] = -1.;
  EXPECT_DOUBLE_EQ(BoundedEquivalentEffort(command, measured, 0, 10.), -10.);
  command.kp[0] = 0.;
  measured.position[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(BoundedEquivalentEffort(command, measured, 0, 10.), 0.);
  measured.position[0] = 0.;
  measured.velocity[0] = std::numeric_limits<double>::infinity();
  EXPECT_DOUBLE_EQ(BoundedEquivalentEffort(command, measured, 0, 10.), 0.);
  measured.velocity[0] = 0.;
  EXPECT_DOUBLE_EQ(BoundedEquivalentEffort(command, measured, 0, -1.), 0.);
}

TEST(JointCommand, MotorPdScalesAllEffortContributionsAndPreservesTargets) {
  JointSample measured;
  measured.valid.fill(1.0);
  measured.position.fill(0.2);
  measured.velocity.fill(-0.5);
  HybridJointCommand command;
  command.position.fill(1.0);
  command.velocity.fill(0.5);
  command.kp.fill(55.0);
  command.kd.fill(2.5);
  command.effort.fill(5.0);
  std::array<double, kJointCount> limits;
  limits.fill(23.7);
  HybridJointCommand output;
  ASSERT_TRUE(BoundedMotorPdCommand(command, measured, limits, output));
  const double scale = 23.7 / (5.0 + 55.0 * 0.8 + 2.5 * 1.0);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    EXPECT_DOUBLE_EQ(output.position[i], command.position[i]);
    EXPECT_DOUBLE_EQ(output.velocity[i], command.velocity[i]);
    EXPECT_DOUBLE_EQ(output.effort[i], command.effort[i] * scale);
    EXPECT_DOUBLE_EQ(output.kp[i], command.kp[i] * scale);
    EXPECT_DOUBLE_EQ(output.kd[i], command.kd[i] * scale);
    EXPECT_NEAR(EquivalentEffort(output, measured, i), 23.7, 1e-12);
  }
  command.kp.fill(0.0);
  command.effort.fill(0.0);
  command.velocity.fill(0.0);
  measured.velocity.fill(100.0);
  ASSERT_TRUE(BoundedMotorPdCommand(command, measured, limits, output));
  for (std::size_t i = 0; i < kJointCount; ++i) {
    EXPECT_DOUBLE_EQ(output.effort[i], 0.0);
    EXPECT_GT(output.kd[i], 0.0);
    EXPECT_NEAR(EquivalentEffort(output, measured, i), -23.7, 1e-12);
  }
}

TEST(JointCommand, MotorPdDoesNotRelyOnCancellationOrClaimBetweenSampleLimit) {
  JointSample measured;
  measured.valid.fill(1.0);
  HybridJointCommand command;
  command.position.fill(1.0);
  command.kp.fill(50.0);
  command.effort.fill(-50.0);
  std::array<double, kJointCount> limits;
  limits.fill(10.0);
  HybridJointCommand output;
  ASSERT_TRUE(BoundedMotorPdCommand(command, measured, limits, output));
  EXPECT_DOUBLE_EQ(output.kp[0], 5.0);
  EXPECT_DOUBLE_EQ(output.effort[0], -5.0);
  EXPECT_DOUBLE_EQ(EquivalentEffort(output, measured, 0), 0.0);
  EXPECT_DOUBLE_EQ(std::abs(output.effort[0]) + output.kp[0], 10.0);
  // Motor-side PD still responds to feedback between host samples; the helper
  // does not install a continuous saturation inside the motor.
  measured.position[0] = -3.0;
  EXPECT_GT(EquivalentEffort(output, measured, 0), limits[0]);
}

TEST(JointCommand, MotorPdLeavesWithinBudgetCommandsUnchangedIncludingZeroBudget) {
  JointSample measured;
  measured.valid.fill(1.0);
  HybridJointCommand command;
  command.position.fill(0.2);
  command.velocity.fill(0.1);
  command.effort.fill(-1.0);
  command.kp.fill(10.0);
  command.kd.fill(0.8);
  std::array<double, kJointCount> limits;
  limits.fill(23.7);
  HybridJointCommand output;
  ASSERT_TRUE(BoundedMotorPdCommand(command, measured, limits, output));
  EXPECT_EQ(output.position, command.position);
  EXPECT_EQ(output.velocity, command.velocity);
  EXPECT_EQ(output.effort, command.effort);
  EXPECT_EQ(output.kp, command.kp);
  EXPECT_EQ(output.kd, command.kd);
  command = {};
  ASSERT_TRUE(BoundedMotorPdCommand(command, measured, limits, output));
  for (const auto effort : output.effort) EXPECT_DOUBLE_EQ(effort, 0.0);
}

TEST(JointCommand, OneInvalidJointDisablesTheEntireHardwareFrame) {
  JointSample measured;
  measured.valid.fill(1.0);
  HybridJointCommand command;
  command.effort.fill(5.0);
  std::array<double, kJointCount> limits;
  limits.fill(23.7);
  HybridJointCommand output;
  const auto expect_disabled = [&]() {
    output.effort.fill(99.0);
    EXPECT_FALSE(BoundedMotorPdCommand(command, measured, limits, output));
    for (std::size_t i = 0; i < kJointCount; ++i) {
      EXPECT_DOUBLE_EQ(output.position[i], 0.0);
      EXPECT_DOUBLE_EQ(output.velocity[i], 0.0);
      EXPECT_DOUBLE_EQ(output.effort[i], 0.0);
      EXPECT_DOUBLE_EQ(output.kp[i], 0.0);
      EXPECT_DOUBLE_EQ(output.kd[i], 0.0);
    }
  };
  measured.valid[11] = 0.0;
  expect_disabled();
  measured.valid[11] = 1.0;
  measured.position[11] = std::numeric_limits<double>::quiet_NaN();
  expect_disabled();
  measured.position[11] = 0.0;
  command.effort[11] = std::numeric_limits<double>::infinity();
  expect_disabled();
  command.effort[11] = 5.0;
  command.kp[11] = 201.0;
  expect_disabled();
  command.kp[11] = -1.0;
  expect_disabled();
  command.kp[11] = 0.0;
  command.kd[11] = 11.0;
  expect_disabled();
  command.kd[11] = 0.0;
  limits[11] = -1.0;
  expect_disabled();
  limits[11] = 23.7;
  command.position[11] = std::numeric_limits<double>::max();
  command.kp[11] = 200.0;
  expect_disabled();
  command.position[11] = 0.0;
  command.velocity[11] = std::numeric_limits<double>::infinity();
  expect_disabled();
}

TEST(JointCommand, LegacyDirectionMappingPreservesCalibratedCoordinatesAndPower) {
  const DriveParameters drive;
  const std::array<double, kJointCount> expected =
      {1, -1, 1, 1, 1, -1, -1, -1, 1, -1, 1, -1};
  EXPECT_EQ(drive.motor_directions, expected);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const auto joint = i % kJointsPerLeg;
    const double gear = joint == 2 ? drive.calf_total_gear_ratio : drive.base_gear_ratio;
    const double direction = drive.motor_directions[i];
    const double raw = 2.5 + i;
    const double offset = CalibrationOffset(
        MotorPositionToUncalibratedUrdf(raw, gear, direction), joint,
        drive.calibration_pose);
    const double q = drive.calibration_pose[joint] + 0.1;
    const double motor_q = UrdfPositionToMotor(q, offset, gear, direction);
    EXPECT_NEAR(CalibratedUrdfPosition(
        MotorPositionToUncalibratedUrdf(motor_q, gear, direction), offset), q, 1e-12);
    EXPECT_NEAR(motor_q - raw, 0.1 * gear * direction, 1e-12);
    const double joint_velocity = -0.3;
    const double joint_effort = 7.0;
    const double motor_velocity = joint_velocity * gear * direction;
    const double motor_effort = joint_effort / gear * direction;
    EXPECT_NEAR(motor_effort * motor_velocity, joint_effort * joint_velocity, 1e-12);
  }
}

TEST(JointCommand, PositionPdUsesHipThighCalfOrderOnEveryLeg) {
  std::array<double, kJointCount> pose{};
  pose.fill(0.4);
  const auto command = PositionPdCommand(pose, {10.0, 20.0, 30.0}, 2.0);
  JointSample measured;
  measured.position.fill(0.1);
  measured.velocity.fill(0.5);
  for (std::size_t leg = 0; leg < kLegCount; ++leg) {
    const auto i = leg * kJointsPerLeg;
    EXPECT_NEAR(EquivalentEffort(command, measured, i), 2.0, 1e-12);
    EXPECT_NEAR(EquivalentEffort(command, measured, i + 1), 5.0, 1e-12);
    EXPECT_NEAR(EquivalentEffort(command, measured, i + 2), 8.0, 1e-12);
  }
}

TEST(JointCommand, HandoffPreservesEndpointsAndContinuousNetTorque) {
  JointSample measured;
  HybridJointCommand stand;
  HybridJointCommand wbc;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    measured.position[i] = -0.2 + 0.03 * i;
    measured.velocity[i] = 0.1 * i;
    stand.position[i] = 0.3;
    stand.kp[i] = 80.0;
    stand.kd[i] = 3.0;
    wbc.position[i] = -0.1;
    wbc.velocity[i] = 0.4;
    wbc.effort[i] = 2.0 - i;
    wbc.kp[i] = 5.0;
    wbc.kd[i] = 0.5;
  }
  for (const double alpha : {0.0, 0.1, 0.5, 0.9, 1.0}) {
    const auto blended = BlendHybridCommands(stand, wbc, measured, alpha);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const double start = EquivalentEffort(stand, measured, i);
      const double end = EquivalentEffort(wbc, measured, i);
      EXPECT_NEAR(EquivalentEffort(blended, measured, i),
                  start + alpha * (end - start), 1e-12);
      if (alpha == 0.0 || alpha == 1.0) {
        const auto& endpoint = alpha == 0.0 ? stand : wbc;
        EXPECT_DOUBLE_EQ(blended.position[i], endpoint.position[i]);
        EXPECT_DOUBLE_EQ(blended.velocity[i], endpoint.velocity[i]);
        EXPECT_DOUBLE_EQ(blended.kp[i], endpoint.kp[i]);
        EXPECT_DOUBLE_EQ(blended.kd[i], endpoint.kd[i]);
        EXPECT_NEAR(blended.effort[i], endpoint.effort[i], 1e-12);
      }
    }
  }
}

}  // namespace
}  // namespace custom_dog_control
