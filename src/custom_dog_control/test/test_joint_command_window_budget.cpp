#include <gtest/gtest.h>

#include <limits>

#include "custom_dog_control/control/JointCommandWindowBudget.hpp"

namespace custom_dog_control {
namespace {

struct WindowBudgetFixture : public ::testing::Test {
  JointSample measured;
  HybridJointCommand command;
  std::array<double, kJointCount> limits;
  MotorPdWindowModel model;
  void SetUp() override {
    measured.valid.fill(1.0);
    measured.position.fill(0.2);
    measured.velocity.fill(-3.0);
    command.position.fill(0.25);
    command.velocity.fill(2.0);
    command.effort.fill(14.0);
    command.kp.fill(45.0);
    command.kd.fill(1.0);
    limits.fill(23.7);
    model.horizon_seconds = 0.024;
    model.assumed_acceleration_bound_rad_s2.fill(160.0);
  }
};

TEST_F(WindowBudgetFixture, ZeroHorizonMatchesLegacyArithmeticExactly) {
  model.horizon_seconds = 0.0;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    command.position[i] += i * 0.037;
    command.velocity[i] -= i * 0.081;
    command.effort[i] = i % 2 ? -14.0 + i * 0.13 : 14.0 - i * 0.13;
    command.kp[i] += i * 0.27;
    command.kd[i] += i * 0.017;
    limits[i] += i * 0.13;
  }
  HybridJointCommand legacy, candidate;
  MotorPdWindowBudget budget;
  ASSERT_TRUE(BoundedMotorPdCommand(command, measured, limits, legacy));
  ASSERT_TRUE(DelayAwareBoundedMotorPdCommand(
      command, measured, limits, model, candidate, &budget));
  EXPECT_EQ(candidate.position, legacy.position);
  EXPECT_EQ(candidate.velocity, legacy.velocity);
  EXPECT_EQ(candidate.effort, legacy.effort);
  EXPECT_EQ(candidate.kp, legacy.kp);
  EXPECT_EQ(candidate.kd, legacy.kd);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    EXPECT_DOUBLE_EQ(budget.position_growth_bound_nm[i], 0.0);
    EXPECT_DOUBLE_EQ(budget.velocity_growth_bound_nm[i], 0.0);
  }
}

// Reproduce a genuinely delayed physical state, not the implementation formula.
// Both acceleration extremes and a reversal are integrated at 0.1 ms resolution.
TEST_F(WindowBudgetFixture, HeldCommandRespectsBoundAlongDelayedPhysicalTrajectories) {
  HybridJointCommand bounded;
  MotorPdWindowBudget budget;
  ASSERT_TRUE(DelayAwareBoundedMotorPdCommand(
      command, measured, limits, model, bounded, &budget));
  ASSERT_LT(budget.scale[0], 1.0);
  EXPECT_EQ(bounded.position, command.position);
  EXPECT_EQ(bounded.velocity, command.velocity);
  for (const int trajectory : {-1, 0, 1}) {
    JointSample physical = measured;
    constexpr double step = 0.0001;
    constexpr int steps = 240;
    for (int sample = 0; sample <= steps; ++sample) {
      for (std::size_t i = 0; i < kJointCount; ++i) {
        EXPECT_LE(std::abs(EquivalentEffort(bounded, physical, i)),
                  limits[i] + 1e-9);
      }
      if (sample == steps) break;
      const double direction = trajectory != 0 ? trajectory :
          (sample < steps / 2 ? 1.0 : -1.0);
      for (std::size_t i = 0; i < kJointCount; ++i) {
        const double acceleration = direction *
            model.assumed_acceleration_bound_rad_s2[i];
        physical.position[i] += physical.velocity[i] * step +
                                0.5 * acceleration * step * step;
        physical.velocity[i] += acceleration * step;
      }
    }
  }
}

TEST_F(WindowBudgetFixture, PerJointAssumptionsAffectOnlyThatJointBudget) {
  model.assumed_acceleration_bound_rad_s2.fill(0.0);
  model.assumed_acceleration_bound_rad_s2[11] = 300.0;
  HybridJointCommand bounded;
  MotorPdWindowBudget budget;
  ASSERT_TRUE(DelayAwareBoundedMotorPdCommand(
      command, measured, limits, model, bounded, &budget));
  for (std::size_t i = 0; i < kJointCount - 1; ++i) {
    EXPECT_DOUBLE_EQ(budget.total_bound_nm[i], budget.total_bound_nm[0]);
    EXPECT_DOUBLE_EQ(budget.scale[i], budget.scale[0]);
    EXPECT_DOUBLE_EQ(budget.velocity_growth_bound_nm[i], 0.0);
  }
  EXPECT_GT(budget.total_bound_nm[11], budget.total_bound_nm[0]);
  EXPECT_LT(budget.scale[11], budget.scale[0]);
}

TEST_F(WindowBudgetFixture, PreQpPdReservationAvoidsFinalBoundaryScaling) {
  HybridJointCommand pd_only = command;
  pd_only.effort.fill(0.0);
  MotorPdWindowBudget pd;
  ASSERT_TRUE(ComputeMotorPdWindowBudget(pd_only, measured, model, pd));
  // Emulate a valid QP solution strictly inside the remaining FF interval.
  HybridJointCommand qp_command = command;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    ASSERT_LT(pd.pd_window_bound_nm[i], limits[i]);
    qp_command.effort[i] = (i % 2 ? -1.0 : 1.0) *
        0.999 * (limits[i] - pd.pd_window_bound_nm[i]);
  }
  HybridJointCommand bounded;
  MotorPdWindowBudget final;
  ASSERT_TRUE(DelayAwareBoundedMotorPdCommand(
      qp_command, measured, limits, model, bounded, &final));
  EXPECT_EQ(bounded.effort, qp_command.effort);
  EXPECT_EQ(bounded.kp, qp_command.kp);
  EXPECT_EQ(bounded.kd, qp_command.kd);
  for (const double scale : final.scale) EXPECT_DOUBLE_EQ(scale, 1.0);
}

TEST_F(WindowBudgetFixture, InvalidOrOverflowingWindowDisablesEntireFrame) {
  const auto expect_disabled = [&]() {
    HybridJointCommand bounded = command;
    MotorPdWindowBudget budget;
    budget.scale.fill(99.0);
    EXPECT_FALSE(DelayAwareBoundedMotorPdCommand(
        command, measured, limits, model, bounded, &budget));
    for (std::size_t i = 0; i < kJointCount; ++i) {
      EXPECT_DOUBLE_EQ(bounded.position[i], 0.0);
      EXPECT_DOUBLE_EQ(bounded.velocity[i], 0.0);
      EXPECT_DOUBLE_EQ(bounded.effort[i], 0.0);
      EXPECT_DOUBLE_EQ(bounded.kp[i], 0.0);
      EXPECT_DOUBLE_EQ(bounded.kd[i], 0.0);
      EXPECT_DOUBLE_EQ(budget.total_bound_nm[i], 0.0);
      EXPECT_DOUBLE_EQ(budget.scale[i], 0.0);
    }
  };
  model.horizon_seconds = -0.001;
  expect_disabled();
  model.horizon_seconds = std::numeric_limits<double>::quiet_NaN();
  expect_disabled();
  model.horizon_seconds = std::numeric_limits<double>::max();
  expect_disabled();
  model.horizon_seconds = 0.024;
  model.assumed_acceleration_bound_rad_s2[11] = -1.0;
  expect_disabled();
  model.assumed_acceleration_bound_rad_s2[11] =
      std::numeric_limits<double>::infinity();
  expect_disabled();
  model.assumed_acceleration_bound_rad_s2[11] = 160.0;
  limits[11] = 0.0;
  expect_disabled();
  limits[11] = 23.7;
  measured.valid[11] = 0.0;
  expect_disabled();
}

TEST_F(WindowBudgetFixture, WindowAssumptionIsNotContinuousMotorSaturation) {
  HybridJointCommand bounded;
  ASSERT_TRUE(DelayAwareBoundedMotorPdCommand(
      command, measured, limits, model, bounded));
  JointSample outside_assumption = measured;
  // A later state outside the claimed window/reachable set can still exceed L.
  outside_assumption.position[0] -= 2.0;
  EXPECT_GT(std::abs(EquivalentEffort(bounded, outside_assumption, 0)), limits[0]);
}

}  // namespace
}  // namespace custom_dog_control
