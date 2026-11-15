#include <gtest/gtest.h>

#include <limits>

#include "custom_dog_control/control/SignedMotorPdBudget.hpp"

namespace custom_dog_control {
namespace {

class SignedMotorPdBudgetTest : public ::testing::Test {
 protected:
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
    model.assumed_acceleration_bound_rad_s2.fill(100.0);
  }
  JointSample measured;
  HybridJointCommand command;
  std::array<double, kJointCount> limits;
  MotorPdWindowModel model;
};

TEST_F(SignedMotorPdBudgetTest, ContinuousWindowEnvelopesAndFeasibleFeedforwardInterval) {
  SignedMotorPdBudget budget;
  ASSERT_TRUE(ComputeSignedMotorPdBudget(command, measured, limits, model, budget));
  EXPECT_TRUE(budget.all_feedforward_intervals_feasible);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    EXPECT_NEAR(budget.pd_nominal_nm[i], 7.25, 1e-12);
    EXPECT_NEAR(budget.pd_min_nm[i], 6.794, 1e-12);
    EXPECT_NEAR(budget.pd_max_nm[i], 14.186, 1e-12);
    EXPECT_NEAR(budget.feedforward_min_nm[i], -23.7, 1e-12);
    EXPECT_NEAR(budget.feedforward_max_nm[i], 9.514, 1e-12);
    EXPECT_NEAR(budget.total_min_nm[i], 20.794, 1e-12);
    EXPECT_NEAR(budget.total_max_nm[i], 28.186, 1e-12);
    EXPECT_NEAR(budget.scale[i], 23.7 / 28.186, 1e-12);
    // A feasible interval does not imply this particular requested FF lies in it.
    EXPECT_GT(command.effort[i], budget.feedforward_max_nm[i]);
  }
}

TEST_F(SignedMotorPdBudgetTest, ExecutionTimeCenterMatchesConstantVelocityPhysicalPd) {
  model.assumed_acceleration_bound_rad_s2.fill(0.0);
  model.nominal_time_seconds = 0.021;
  SignedMotorPdBudget budget;
  ASSERT_TRUE(ComputeSignedMotorPdBudget(command, measured, limits, model, budget));
  JointSample execution_state = measured;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    execution_state.position[i] += measured.velocity[i] * model.nominal_time_seconds;
    // PD0=7.25; the 21 ms held-target drift adds 2.835 Nm. This checks
    // the physical future state independently of the budget's drift expression.
    EXPECT_NEAR(budget.pd_nominal_nm[i], 10.085, 1e-12);
    EXPECT_NEAR(budget.pd_nominal_nm[i],
                EquivalentEffort(command, execution_state, i) - command.effort[i], 1e-12);
    EXPECT_NEAR(budget.pd_min_nm[i], 7.25, 1e-12);
    EXPECT_NEAR(budget.pd_max_nm[i], 10.49, 1e-12);
  }
}

TEST_F(SignedMotorPdBudgetTest, MovingExecutionCenterPreservesFullWindowAndFinalOutput) {
  SignedMotorPdBudget current, future;
  HybridJointCommand current_output, future_output;
  ASSERT_TRUE(BoundedSignedMotorPdCommand(command, measured, limits, model,
                                          current_output, &current));
  model.nominal_time_seconds = 0.021;
  ASSERT_TRUE(BoundedSignedMotorPdCommand(command, measured, limits, model,
                                          future_output, &future));
  EXPECT_NE(current.pd_nominal_nm, future.pd_nominal_nm);
  EXPECT_EQ(current.pd_min_nm, future.pd_min_nm);
  EXPECT_EQ(current.pd_max_nm, future.pd_max_nm);
  EXPECT_EQ(current.feedforward_min_nm, future.feedforward_min_nm);
  EXPECT_EQ(current.feedforward_max_nm, future.feedforward_max_nm);
  EXPECT_EQ(current.feedforward_interval_feasible, future.feedforward_interval_feasible);
  EXPECT_EQ(current.all_feedforward_intervals_feasible, future.all_feedforward_intervals_feasible);
  EXPECT_EQ(current.total_min_nm, future.total_min_nm);
  EXPECT_EQ(current.total_max_nm, future.total_max_nm);
  EXPECT_EQ(current.total_abs_bound_nm, future.total_abs_bound_nm);
  EXPECT_EQ(current.feedforward_abs_nm, future.feedforward_abs_nm);
  EXPECT_EQ(current.scale, future.scale);
  EXPECT_EQ(current_output.position, future_output.position);
  EXPECT_EQ(current_output.velocity, future_output.velocity);
  EXPECT_EQ(current_output.effort, future_output.effort);
  EXPECT_EQ(current_output.kp, future_output.kp);
  EXPECT_EQ(current_output.kd, future_output.kd);
}

TEST_F(SignedMotorPdBudgetTest, NominalExecutionTimeDoesNotChangeUnsignedLegacyWindowBudget) {
  MotorPdWindowBudget current, future;
  HybridJointCommand current_output, future_output;
  ASSERT_TRUE(DelayAwareBoundedMotorPdCommand(command, measured, limits, model,
                                              current_output, &current));
  model.nominal_time_seconds = model.horizon_seconds;
  ASSERT_TRUE(DelayAwareBoundedMotorPdCommand(command, measured, limits, model,
                                              future_output, &future));
  EXPECT_EQ(current.pd_window_bound_nm, future.pd_window_bound_nm);
  EXPECT_EQ(current.total_bound_nm, future.total_bound_nm);
  EXPECT_EQ(current.scale, future.scale);
  EXPECT_EQ(current_output.position, future_output.position);
  EXPECT_EQ(current_output.velocity, future_output.velocity);
  EXPECT_EQ(current_output.effort, future_output.effort);
  EXPECT_EQ(current_output.kp, future_output.kp);
  EXPECT_EQ(current_output.kd, future_output.kd);
}

TEST_F(SignedMotorPdBudgetTest, InvalidExecutionTimeAtomicallyDisablesBothBudgetModes) {
  const auto expect_disabled = [&]() {
    HybridJointCommand signed_output = command, legacy_output = command;
    SignedMotorPdBudget signed_budget;
    MotorPdWindowBudget legacy_budget;
    signed_budget.scale.fill(99.0);
    legacy_budget.scale.fill(99.0);
    EXPECT_FALSE(BoundedSignedMotorPdCommand(command, measured, limits, model,
                                             signed_output, &signed_budget));
    EXPECT_FALSE(DelayAwareBoundedMotorPdCommand(command, measured, limits, model,
                                                 legacy_output, &legacy_budget));
    for (std::size_t i = 0; i < kJointCount; ++i) {
      EXPECT_DOUBLE_EQ(signed_output.position[i], 0.0);
      EXPECT_DOUBLE_EQ(signed_output.velocity[i], 0.0);
      EXPECT_DOUBLE_EQ(signed_output.effort[i], 0.0);
      EXPECT_DOUBLE_EQ(signed_output.kp[i], 0.0);
      EXPECT_DOUBLE_EQ(signed_output.kd[i], 0.0);
      EXPECT_DOUBLE_EQ(legacy_output.position[i], 0.0);
      EXPECT_DOUBLE_EQ(legacy_output.velocity[i], 0.0);
      EXPECT_DOUBLE_EQ(legacy_output.effort[i], 0.0);
      EXPECT_DOUBLE_EQ(legacy_output.kp[i], 0.0);
      EXPECT_DOUBLE_EQ(legacy_output.kd[i], 0.0);
      EXPECT_DOUBLE_EQ(signed_budget.pd_nominal_nm[i], 0.0);
      EXPECT_DOUBLE_EQ(signed_budget.scale[i], 0.0);
      EXPECT_DOUBLE_EQ(legacy_budget.total_bound_nm[i], 0.0);
      EXPECT_DOUBLE_EQ(legacy_budget.scale[i], 0.0);
    }
  };
  for (const double invalid : {-0.001, 0.024001,
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::infinity()}) {
    model.nominal_time_seconds = invalid;
    expect_disabled();
  }
  model.horizon_seconds = 0.0;
  model.nominal_time_seconds = 0.001;
  expect_disabled();
  model.nominal_time_seconds = 0.0;
  SignedMotorPdBudget instantaneous;
  ASSERT_TRUE(ComputeSignedMotorPdBudget(command, measured, limits, model, instantaneous));
  EXPECT_NEAR(instantaneous.pd_nominal_nm[0], 7.25, 1e-12);
}

TEST_F(SignedMotorPdBudgetTest, FeasibleQpFeedforwardIsNotRescaledAtFinalBoundary) {
  SignedMotorPdBudget reservation;
  ASSERT_TRUE(ComputeSignedMotorPdBudget(command, measured, limits, model, reservation));
  for (std::size_t i = 0; i < kJointCount; ++i) {
    command.effort[i] = 0.5 * (reservation.feedforward_min_nm[i] +
                               reservation.feedforward_max_nm[i]);
  }
  HybridJointCommand bounded;
  SignedMotorPdBudget final;
  ASSERT_TRUE(BoundedSignedMotorPdCommand(command, measured, limits, model,
                                          bounded, &final));
  EXPECT_EQ(bounded.position, command.position);
  EXPECT_EQ(bounded.velocity, command.velocity);
  EXPECT_EQ(bounded.effort, command.effort);
  EXPECT_EQ(bounded.kp, command.kp);
  EXPECT_EQ(bounded.kd, command.kd);
  for (double scale : final.scale) EXPECT_DOUBLE_EQ(scale, 1.0);
}

TEST_F(SignedMotorPdBudgetTest, InstantaneousSignedCancellationWorksInBothDirections) {
  model.horizon_seconds = 0.0;
  limits.fill(10.0);
  measured.position.fill(0.0);
  measured.velocity.fill(0.0);
  command.velocity.fill(0.0);
  command.kp.fill(12.0);
  command.kd.fill(0.0);
  for (double sign : {-1.0, 1.0}) {
    command.position.fill(sign);
    command.effort.fill(-8.0 * sign);
    HybridJointCommand bounded, legacy;
    SignedMotorPdBudget budget;
    ASSERT_TRUE(BoundedSignedMotorPdCommand(command, measured, limits, model,
                                            bounded, &budget));
    ASSERT_TRUE(BoundedMotorPdCommand(command, measured, limits, legacy));
    EXPECT_EQ(bounded.effort, command.effort);
    EXPECT_EQ(bounded.kp, command.kp);
    EXPECT_DOUBLE_EQ(budget.scale[0], 1.0);
    EXPECT_DOUBLE_EQ(EquivalentEffort(bounded, measured, 0), 4.0 * sign);
    EXPECT_LT(legacy.kp[0], bounded.kp[0]);
  }
}

TEST_F(SignedMotorPdBudgetTest, IndependentFeedforwardFieldCapStillAppliesWithZeroNetTorque) {
  model.horizon_seconds = 0.0;
  limits.fill(10.0);
  measured.position.fill(0.0);
  measured.velocity.fill(0.0);
  command.velocity.fill(0.0);
  command.kp.fill(20.0);
  command.kd.fill(0.0);
  for (double sign : {-1.0, 1.0}) {
    command.position.fill(sign);
    command.effort.fill(-20.0 * sign);
    HybridJointCommand bounded;
    SignedMotorPdBudget budget;
    ASSERT_TRUE(BoundedSignedMotorPdCommand(command, measured, limits, model,
                                            bounded, &budget));
    EXPECT_DOUBLE_EQ(budget.total_abs_bound_nm[0], 0.0);
    EXPECT_DOUBLE_EQ(budget.scale[0], 0.5);
    EXPECT_DOUBLE_EQ(bounded.effort[0], -10.0 * sign);
    EXPECT_DOUBLE_EQ(bounded.kp[0], 10.0);
    EXPECT_DOUBLE_EQ(EquivalentEffort(bounded, measured, 0), 0.0);
    EXPECT_EQ(bounded.position, command.position);
    EXPECT_EQ(bounded.velocity, command.velocity);
  }
}

TEST_F(SignedMotorPdBudgetTest, ValidButEmptyFeedforwardIntervalDoesNotSilentlyReduceGains) {
  model.assumed_acceleration_bound_rad_s2[11] = 1000.0;
  const auto original = command;
  SignedMotorPdBudget budget;
  ASSERT_TRUE(ComputeSignedMotorPdBudget(command, measured, limits, model, budget));
  EXPECT_FALSE(budget.all_feedforward_intervals_feasible);
  EXPECT_FALSE(budget.feedforward_interval_feasible[11]);
  EXPECT_GT(budget.feedforward_min_nm[11], budget.feedforward_max_nm[11]);
  for (std::size_t i = 0; i < kJointCount - 1; ++i) {
    EXPECT_TRUE(budget.feedforward_interval_feasible[i]);
  }
  EXPECT_EQ(command.kp, original.kp);
  EXPECT_EQ(command.kd, original.kd);
  EXPECT_EQ(command.effort, original.effort);
}

TEST_F(SignedMotorPdBudgetTest, BoundedAccelerationTrajectoriesRespectSignedWindowAfterScaling) {
  HybridJointCommand bounded;
  SignedMotorPdBudget budget;
  ASSERT_TRUE(BoundedSignedMotorPdCommand(command, measured, limits, model,
                                          bounded, &budget));
  EXPECT_LT(budget.scale[0], 1.0);
  EXPECT_EQ(bounded.position, command.position);
  EXPECT_EQ(bounded.velocity, command.velocity);
  for (int trajectory : {-1, 0, 1}) {
    JointSample physical = measured;
    constexpr double step = 0.0001;
    constexpr int steps = 240;
    for (int sample = 0; sample <= steps; ++sample) {
      for (std::size_t i = 0; i < kJointCount; ++i) {
        EXPECT_LE(std::abs(EquivalentEffort(bounded, physical, i)), limits[i] + 1e-9);
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

TEST_F(SignedMotorPdBudgetTest, ContactLikeAccelerationOutsideAssumptionCanBreakTheWindowBound) {
  HybridJointCommand bounded;
  ASSERT_TRUE(BoundedSignedMotorPdCommand(command, measured, limits, model, bounded));
  JointSample physical = measured;
  // A counterexample to an unconditional saturation claim, not a calibrated
  // contact model: acceleration exceeds the configured100 rad/s^2 assumption.
  const double actual_acceleration = 2000.0;
  const double elapsed = model.horizon_seconds;
  physical.position[0] += physical.velocity[0] * elapsed +
                         0.5 * actual_acceleration * elapsed * elapsed;
  physical.velocity[0] += actual_acceleration * elapsed;
  EXPECT_GT(std::abs(EquivalentEffort(bounded, physical, 0)), limits[0]);
}

TEST_F(SignedMotorPdBudgetTest, ExpiredPacketCanExceedLimitEvenWithAccelerationWithinAssumption) {
  HybridJointCommand bounded;
  ASSERT_TRUE(BoundedSignedMotorPdCommand(command, measured, limits, model, bounded));
  JointSample physical = measured;
  const double actual_acceleration = -100.0;
  const double elapsed = 2.0 * model.horizon_seconds;
  physical.position[0] += physical.velocity[0] * elapsed +
                         0.5 * actual_acceleration * elapsed * elapsed;
  physical.velocity[0] += actual_acceleration * elapsed;
  EXPECT_GT(std::abs(EquivalentEffort(bounded, physical, 0)), limits[0]);
}

TEST_F(SignedMotorPdBudgetTest, InvalidInputOrOverflowAtomicallyDisablesEntireFrame) {
  const auto expect_disabled = [&]() {
    HybridJointCommand bounded = command;
    SignedMotorPdBudget budget;
    budget.scale.fill(99.0);
    EXPECT_FALSE(BoundedSignedMotorPdCommand(command, measured, limits, model,
                                             bounded, &budget));
    EXPECT_FALSE(budget.all_feedforward_intervals_feasible);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      EXPECT_DOUBLE_EQ(bounded.position[i], 0.0);
      EXPECT_DOUBLE_EQ(bounded.velocity[i], 0.0);
      EXPECT_DOUBLE_EQ(bounded.effort[i], 0.0);
      EXPECT_DOUBLE_EQ(bounded.kp[i], 0.0);
      EXPECT_DOUBLE_EQ(bounded.kd[i], 0.0);
      EXPECT_DOUBLE_EQ(budget.pd_nominal_nm[i], 0.0);
      EXPECT_DOUBLE_EQ(budget.total_abs_bound_nm[i], 0.0);
      EXPECT_DOUBLE_EQ(budget.scale[i], 0.0);
      EXPECT_FALSE(budget.feedforward_interval_feasible[i]);
    }
  };
  model.horizon_seconds = std::numeric_limits<double>::quiet_NaN();
  expect_disabled();
  model.horizon_seconds = -0.001;
  expect_disabled();
  model.horizon_seconds = std::numeric_limits<double>::max();
  expect_disabled();
  model.horizon_seconds = 0.024;
  model.assumed_acceleration_bound_rad_s2[11] = -1.0;
  expect_disabled();
  model.assumed_acceleration_bound_rad_s2[11] = 100.0;
  measured.valid[11] = 0.0;
  expect_disabled();
  measured.valid[11] = 1.0;
  limits[11] = std::numeric_limits<double>::infinity();
  expect_disabled();
  limits[11] = 23.7;
  command.kp[11] = 201.0;
  expect_disabled();
  command.kp[11] = 45.0;
  command.position[11] = std::numeric_limits<double>::max();
  measured.position[11] = -std::numeric_limits<double>::max();
  expect_disabled();
}

}  // namespace
}  // namespace custom_dog_control
