#include <gtest/gtest.h>

#include <limits>

#include "custom_dog_control/control/HostTotalTorqueCommand.hpp"
#include "custom_dog_control/control/SimulationEffortDelay.hpp"

namespace custom_dog_control {
namespace {

class HostTotalTorqueCommandTest : public ::testing::Test {
 protected:
  void SetUp() override {
    measured.valid.fill(1.0);
    command.position.fill(0.2);
    command.effort.fill(20.0);
    command.kp.fill(50.0);
    command.kd.fill(1.0);
    limits.fill(23.7);
  }
  JointSample measured;
  HybridJointCommand command;
  std::array<double, kJointCount> limits;
};

TEST_F(HostTotalTorqueCommandTest, BoundsTotalAndExportsTheRawRequestWithZeroMotorGains) {
  HybridJointCommand output;
  HostTotalTorqueBudget budget;
  ASSERT_TRUE(BoundedHostTotalTorqueCommand(command, measured, limits, output, &budget));
  ASSERT_TRUE(budget.active);
  EXPECT_EQ(output.position, command.position);
  EXPECT_EQ(output.velocity, command.velocity);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    EXPECT_DOUBLE_EQ(budget.pd_nm[i], 10.0);
    EXPECT_DOUBLE_EQ(budget.raw_total_nm[i], 30.0);
    EXPECT_DOUBLE_EQ(budget.sent_total_nm[i], 23.7);
    EXPECT_DOUBLE_EQ(budget.scale[i], 23.7 / 30.0);
    EXPECT_DOUBLE_EQ(output.effort[i], 23.7);
    EXPECT_DOUBLE_EQ(output.kp[i], 0.0);
    EXPECT_DOUBLE_EQ(output.kd[i], 0.0);
  }
}

TEST_F(HostTotalTorqueCommandTest, InternalResidualFeedforwardDoesNotHaveItsOwnPhysicalFieldCap) {
  command.kp.fill(100.0);
  command.kd.fill(0.0);
  for (const double sign : {-1.0, 1.0}) {
    command.position.fill(-0.2 * sign);
    command.effort.fill(40.0 * sign);
    HybridJointCommand output;
    HostTotalTorqueBudget budget;
    ASSERT_TRUE(BoundedHostTotalTorqueCommand(command, measured, limits, output, &budget));
    for (std::size_t i = 0; i < kJointCount; ++i) {
      EXPECT_DOUBLE_EQ(budget.raw_total_nm[i], 20.0 * sign);
      EXPECT_DOUBLE_EQ(budget.sent_total_nm[i], 20.0 * sign);
      EXPECT_DOUBLE_EQ(budget.scale[i], 1.0);
      EXPECT_DOUBLE_EQ(output.effort[i], 20.0 * sign);
      EXPECT_DOUBLE_EQ(output.kp[i], 0.0);
      EXPECT_DOUBLE_EQ(output.kd[i], 0.0);
    }
  }
}

TEST_F(HostTotalTorqueCommandTest, DelayedHeldPacketCannotGrowWithChangedPhysicalFeedback) {
  HybridJointCommand packet;
  ASSERT_TRUE(BoundedHostTotalTorqueCommand(command, measured, limits, packet));
  SimulationEffortDelay delivery;
  delivery.Configure(10);
  SimulationEffortDelay::Effort effort = packet.effort;
  for (int cycle = 0; cycle < 10; ++cycle) {
    for (const double value : delivery.Process(effort, true)) EXPECT_DOUBLE_EQ(value, 0.0);
  }
  EXPECT_EQ(delivery.Process(effort, true), packet.effort);
  JointSample execution = measured;
  for (const double sign : {-1.0, 1.0}) {
    execution.position.fill(5.0 * sign);
    execution.velocity.fill(100.0 * sign);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      EXPECT_DOUBLE_EQ(EquivalentEffort(packet, execution, i), packet.effort[i]);
      EXPECT_LE(std::abs(EquivalentEffort(packet, execution, i)), limits[i]);
    }
  }
}

TEST_F(HostTotalTorqueCommandTest, CompositionAfterHandoffPreservesItsTotalBeforeTheExplicitCap) {
  const auto from = PositionPdCommand(
      std::array<double, kJointCount>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
      {60.0, 60.0, 60.0}, 2.0);
  command.position.fill(-0.2);
  command.effort.fill(40.0);  // To-side net 30, from-side net 60.
  const auto blended = BlendHybridCommands(from, command, measured, 0.5);
  HybridJointCommand output;
  HostTotalTorqueBudget budget;
  ASSERT_TRUE(BoundedHostTotalTorqueCommand(blended, measured, limits, output, &budget));
  for (std::size_t i = 0; i < kJointCount; ++i) {
    EXPECT_NEAR(budget.raw_total_nm[i], 45.0, 1e-12);
    EXPECT_DOUBLE_EQ(output.effort[i], 23.7);
    EXPECT_NEAR(budget.scale[i], 23.7 / 45.0, 1e-12);
    EXPECT_DOUBLE_EQ(output.kp[i], 0.0);
    EXPECT_DOUBLE_EQ(output.kd[i], 0.0);
  }
}

TEST_F(HostTotalTorqueCommandTest, InvalidLastJointOrArithmeticAtomicallyClearsOutputAndDiagnostics) {
  const auto expect_disabled = [&]() {
    HybridJointCommand output = command;
    HostTotalTorqueBudget budget;
    budget.active = true;
    budget.scale.fill(99.0);
    EXPECT_FALSE(BoundedHostTotalTorqueCommand(command, measured, limits, output, &budget));
    EXPECT_FALSE(budget.active);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      EXPECT_DOUBLE_EQ(output.position[i], 0.0);
      EXPECT_DOUBLE_EQ(output.velocity[i], 0.0);
      EXPECT_DOUBLE_EQ(output.effort[i], 0.0);
      EXPECT_DOUBLE_EQ(output.kp[i], 0.0);
      EXPECT_DOUBLE_EQ(output.kd[i], 0.0);
      EXPECT_DOUBLE_EQ(budget.pd_nm[i], 0.0);
      EXPECT_DOUBLE_EQ(budget.raw_total_nm[i], 0.0);
      EXPECT_DOUBLE_EQ(budget.sent_total_nm[i], 0.0);
      EXPECT_DOUBLE_EQ(budget.scale[i], 0.0);
    }
  };
  measured.valid[11] = 0.0;
  expect_disabled();
  measured.valid[11] = 1.0;
  command.effort[11] = std::numeric_limits<double>::quiet_NaN();
  expect_disabled();
  command.effort[11] = 20.0;
  limits[11] = std::numeric_limits<double>::infinity();
  expect_disabled();
  limits[11] = 23.7;
  command.kp[11] = 201.0;
  expect_disabled();
  command.kp[11] = 50.0;
  measured.velocity[11] = std::numeric_limits<double>::infinity();
  expect_disabled();
  measured.velocity[11] = 0.0;
  command.position[11] = std::numeric_limits<double>::max();
  expect_disabled();
}


TEST_F(HostTotalTorqueCommandTest, LatchedHandoffUsesOneStartEffortAtBothEndpointsAndInside) {
  auto stand = PositionPdCommand(
      std::array<double, kJointCount>{0.4, 0.4, 0.4, 0.4, 0.4, 0.4,
                                     0.4, 0.4, 0.4, 0.4, 0.4, 0.4},
      {60.0, 60.0, 90.0}, 2.0);
  HybridJointCommand fixed_start;
  ASSERT_TRUE(BoundedHostTotalTorqueCommand(stand, measured, limits, fixed_start));
  for (const double effort : fixed_start.effort) EXPECT_DOUBLE_EQ(effort, 23.7);
  // The stand endpoint was captured before these later physical states. A
  // freshly evaluated stand PD would grow, but the locked start remains fixed.
  for (const double q : {-0.2, 0.3}) {
    measured.position.fill(q);
    measured.velocity.fill(-2.0);
    command.position.fill(0.1);
    command.velocity.fill(0.5);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      command.effort[i] = -12.0 - command.kp[i] * (command.position[i] - measured.position[i]) -
          command.kd[i] * (command.velocity[i] - measured.velocity[i]);
    }
    for (const double alpha : {0.0, 0.25, 0.5, 0.75, 1.0}) {
      const auto blend = BlendHybridCommands(fixed_start, command, measured, alpha);
      HybridJointCommand output;
      HostTotalTorqueBudget budget;
      ASSERT_TRUE(BoundedHostTotalTorqueCommand(blend, measured, limits, output, &budget));
      for (std::size_t i = 0; i < kJointCount; ++i) {
        const double total = (1.0 - alpha) * fixed_start.effort[i] + alpha * -12.0;
        EXPECT_NEAR(budget.raw_total_nm[i], total, 1e-12);
        EXPECT_NEAR(output.effort[i], total, 1e-12);
        EXPECT_NEAR(budget.scale[i], 1.0, 1e-12);
        EXPECT_DOUBLE_EQ(output.kp[i], 0.0);
        EXPECT_DOUBLE_EQ(output.kd[i], 0.0);
      }
    }
  }
}

TEST_F(HostTotalTorqueCommandTest, HostFallbackHoldsLastSentTotalDespiteNewJointFeedback) {
  HybridJointCommand prior_output;
  HostTotalTorqueBudget prior_budget;
  ASSERT_TRUE(BoundedHostTotalTorqueCommand(command, measured, limits, prior_output, &prior_budget));
  HybridJointCommand fallback;
  fallback.effort = prior_budget.sent_total_nm;
  measured.position.fill(-5.0);
  measured.velocity.fill(-100.0);
  // Four failed WBC cycles use the same last fixed effort. Failure counting
  // and the fifth-cycle FAULT remain the state machine's existing contract.
  for (int cycle = 0; cycle < 4; ++cycle) {
    HybridJointCommand output;
    HostTotalTorqueBudget budget;
    ASSERT_TRUE(BoundedHostTotalTorqueCommand(fallback, measured, limits, output, &budget));
    EXPECT_EQ(output.effort, prior_budget.sent_total_nm);
    EXPECT_EQ(budget.raw_total_nm, prior_budget.sent_total_nm);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      EXPECT_DOUBLE_EQ(output.kp[i], 0.0);
      EXPECT_DOUBLE_EQ(output.kd[i], 0.0);
      EXPECT_DOUBLE_EQ(budget.scale[i], 1.0);
    }
    measured.position[0] -= 0.1;
    measured.velocity[0] -= 1.0;
  }
  HybridJointCommand disabled;
  HostTotalTorqueBudget budget;
  fallback = {};  // No previous active budget and no valid latch.
  ASSERT_TRUE(BoundedHostTotalTorqueCommand(fallback, measured, limits, disabled, &budget));
  for (const double effort : disabled.effort) EXPECT_DOUBLE_EQ(effort, 0.0);
  measured.valid[11] = 0.0;
  EXPECT_FALSE(BoundedHostTotalTorqueCommand(command, measured, limits, disabled));
}

}  // namespace
}  // namespace custom_dog_control
