#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <ocs2_core/control/LinearController.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>

#include "custom_dog_control/control/NominalJointReference.hpp"

namespace custom_dog_control {
namespace {

ocs2::PrimalSolution MakePolicy() {
  ocs2::PrimalSolution policy;
  policy.timeTrajectory_ = {0., 1.};
  policy.modeSchedule_ = ocs2::ModeSchedule({}, {kTrotFrRlMode});
  for (int node = 0; node < 2; ++node) {
    auto state = ocs2::vector_t::Zero(kCentroidalStateDim).eval();
    auto input = ocs2::vector_t::Zero(kCentroidalInputDim).eval();
    for (std::size_t i = 0; i < kJointCount; ++i) {
      state(12 + i) = 2. + 4. * node + .01 * i;
      input(12 + i) = 3. + 6. * node + .02 * i;
      input(i) = 1000. + 100. * node + i;  // Distinct contact-force data must not be returned as dq.
    }
    policy.stateTrajectory_.push_back(state);
    policy.inputTrajectory_.push_back(input);
  }
  return policy;
}

TEST(NominalJointReference, SamplesNominalPositionAndInputInModelOrderAtTheSameTime) {
  const auto policy = MakePolicy();
  const auto result = SampleNominalJointReference(policy, .25, kTrotFrRlMode);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->mode, kTrotFrRlMode);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    EXPECT_NEAR(result->position_model_rad[i], 3. + .01 * i, 1e-12);
    EXPECT_NEAR(result->velocity_model_rad_s[i], 4.5 + .02 * i, 1e-12);
  }
}

TEST(NominalJointReference, DirectInputSamplingAvoidsInterpolatedFeedbackGainBiasCrossTerm) {
  auto policy = MakePolicy();
  ocs2::matrix_array_t gains(2, ocs2::matrix_t::Zero(kCentroidalInputDim, kCentroidalStateDim));
  gains[0](12, 12) = 1.;
  gains[1](12, 12) = 3.;
  ocs2::vector_array_t bias = policy.inputTrajectory_;
  for (std::size_t i = 0; i < 2; ++i) bias[i] -= gains[i] * policy.stateTrajectory_[i];
  ocs2::LinearController controller(policy.timeTrajectory_, bias, gains);
  const auto nominal_state = ocs2::LinearInterpolation::interpolate(.5, policy.timeTrajectory_, policy.stateTrajectory_);
  const auto reevaluated_input = controller.computeInput(.5, nominal_state);
  const auto result = SampleNominalJointReference(policy, .5, kTrotFrRlMode);
  ASSERT_TRUE(result);
  EXPECT_DOUBLE_EQ(result->velocity_model_rad_s[0], 6.);
  EXPECT_DOUBLE_EQ(reevaluated_input(12), 4.);
}

TEST(NominalJointReference, DuplicateContactEventUsesTheOriginalMrtPreEventSideAndPostEventSide) {
  auto policy = MakePolicy();
  policy.timeTrajectory_ = {0., 1., 1., 2.};
  policy.modeSchedule_ = ocs2::ModeSchedule({1.}, {kTrotFrRlMode, kTrotFlRrMode});
  policy.postEventIndices_ = {2};
  policy.stateTrajectory_.clear();
  policy.inputTrajectory_.clear();
  for (const auto q : {0., 10., 100., 200.}) {
    auto state = ocs2::vector_t::Zero(kCentroidalStateDim).eval();
    auto input = ocs2::vector_t::Zero(kCentroidalInputDim).eval();
    state.tail(kJointCount).setConstant(q);
    input.tail(kJointCount).setConstant(q / 10. + 1.);
    policy.stateTrajectory_.push_back(state);
    policy.inputTrajectory_.push_back(input);
  }
  const auto event = SampleNominalJointReference(policy, 1., kTrotFrRlMode);
  ASSERT_TRUE(event);
  EXPECT_DOUBLE_EQ(event->position_model_rad[0], 10.);
  EXPECT_DOUBLE_EQ(event->velocity_model_rad_s[0], 2.);
  EXPECT_FALSE(SampleNominalJointReference(policy, 1., kTrotFlRrMode));
  const auto after = SampleNominalJointReference(policy, 1. + 1e-6, kTrotFlRrMode);
  ASSERT_TRUE(after);
  EXPECT_NEAR(after->position_model_rad[0], 100. + 100. * 1e-6, 1e-10);
  EXPECT_NEAR(after->velocity_model_rad_s[0], 11. + 10. * 1e-6, 1e-10);
}

TEST(NominalJointReference, SingleNodeAndOutOfHorizonFollowOriginalZeroOrderExtrapolation) {
  auto policy = MakePolicy();
  auto before = SampleNominalJointReference(policy, -1., kTrotFrRlMode);
  auto after = SampleNominalJointReference(policy, 2., kTrotFrRlMode);
  ASSERT_TRUE(before); ASSERT_TRUE(after);
  EXPECT_DOUBLE_EQ(before->position_model_rad[0], 2.);
  EXPECT_DOUBLE_EQ(after->position_model_rad[0], 6.);
  policy.timeTrajectory_.resize(1);
  policy.stateTrajectory_.resize(1);
  policy.inputTrajectory_.resize(1);
  const auto single = SampleNominalJointReference(policy, 20., kTrotFrRlMode);
  ASSERT_TRUE(single);
  EXPECT_DOUBLE_EQ(single->position_model_rad[0], 2.);
  EXPECT_DOUBLE_EQ(single->velocity_model_rad_s[0], 3.);
}

TEST(NominalJointReference, RejectsShapeErrorsRatherThanOcs2DimensionSnapFallback) {
  auto policy = MakePolicy();
  policy.inputTrajectory_.pop_back();
  EXPECT_FALSE(SampleNominalJointReference(policy, .25, kTrotFrRlMode));
  policy = MakePolicy();
  policy.stateTrajectory_[1].conservativeResize(23);
  EXPECT_FALSE(SampleNominalJointReference(policy, .25, kTrotFrRlMode));
  policy = MakePolicy();
  policy.inputTrajectory_[1].conservativeResize(25);
  EXPECT_FALSE(SampleNominalJointReference(policy, .25, kTrotFrRlMode));
  policy = MakePolicy();
  policy.modeSchedule_.modeSequence.clear();
  EXPECT_FALSE(SampleNominalJointReference(policy, .25, kTrotFrRlMode));
  EXPECT_FALSE(SampleNominalJointReference(ocs2::PrimalSolution{}, .25, kTrotFrRlMode));
}

TEST(NominalJointReference, RejectsNonfiniteDataAndInvalidTimeOrderingAtomically) {
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  const auto inf = std::numeric_limits<double>::infinity();
  auto policy = MakePolicy();
  EXPECT_FALSE(SampleNominalJointReference(policy, nan, kTrotFrRlMode));
  policy.stateTrajectory_[1](12) = nan;
  EXPECT_FALSE(SampleNominalJointReference(policy, .25, kTrotFrRlMode));
  policy = MakePolicy();
  policy.inputTrajectory_[0](12) = inf;
  EXPECT_FALSE(SampleNominalJointReference(policy, .25, kTrotFrRlMode));
  policy = MakePolicy();
  policy.timeTrajectory_ = {1., 0.};
  EXPECT_FALSE(SampleNominalJointReference(policy, .25, kTrotFrRlMode));
  policy = MakePolicy();
  policy.timeTrajectory_[1] = nan;
  EXPECT_FALSE(SampleNominalJointReference(policy, .25, kTrotFrRlMode));
  policy = MakePolicy();
  policy.modeSchedule_ = ocs2::ModeSchedule({.7, .3}, {9, 6, 9});
  EXPECT_FALSE(SampleNominalJointReference(policy, .25, kTrotFrRlMode));
}

TEST(NominalJointReference, PolicyReplacementIsStatelessAndDoesNotClaimToHideAJump) {
  auto policy = MakePolicy();
  const auto first = SampleNominalJointReference(policy, .5, kTrotFrRlMode);
  ASSERT_TRUE(first);
  for (auto& state : policy.stateTrajectory_) state.tail(kJointCount).array() += .3;
  for (auto& input : policy.inputTrajectory_) input.tail(kJointCount).array() -= 2.;
  const auto next = SampleNominalJointReference(policy, .5, kTrotFrRlMode);
  ASSERT_TRUE(next);
  EXPECT_NEAR(next->position_model_rad[0] - first->position_model_rad[0], .3, 1e-12);
  EXPECT_DOUBLE_EQ(next->velocity_model_rad_s[0] - first->velocity_model_rad_s[0], -2.);
}

}  // namespace
}  // namespace custom_dog_control
