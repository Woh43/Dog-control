#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "custom_dog_control/control/JointVelocityReferenceLimiter.hpp"

namespace custom_dog_control {
namespace {

using JointArray = JointVelocityReferenceLimiter::JointArray;

JointArray Filled(double value) {
  JointArray result;
  result.fill(value);
  return result;
}

TEST(JointVelocityReferenceLimiter, FirstNonzeroFrameSeedsExactly) {
  JointVelocityReferenceLimiter limiter;
  JointArray raw;
  for (std::size_t i = 0; i < kJointCount; ++i) raw[i] = -3.0 + .5 * i;
  const auto result = limiter.Update(raw, Filled(100.0), .002);
  ASSERT_TRUE(result);
  EXPECT_EQ(*result, raw);
}

TEST(JointVelocityReferenceLimiter, ContactReversalRespectsEachJointAcceleration) {
  JointVelocityReferenceLimiter limiter;
  JointArray limits;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    limits[i] = i % 3 == 0 ? 100.0 : (i % 3 == 1 ? 200.0 : 400.0);
  }
  const auto initial = Filled(-2.5);
  ASSERT_TRUE(limiter.Update(initial, limits, .002));
  const auto result = limiter.Update(Filled(1.0), limits, .002);
  ASSERT_TRUE(result);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    EXPECT_NEAR((*result)[i], initial[i] + limits[i] * .002, 1e-12);
    EXPECT_LT((*result)[i], 1.0);
  }
  const auto reverse = limiter.Update(Filled(-10.0), limits, .001);
  ASSERT_TRUE(reverse);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    EXPECT_NEAR((*reverse)[i], (*result)[i] - limits[i] * .001, 1e-12);
  }
}

TEST(JointVelocityReferenceLimiter, SmallChangesReachRawTargetWithoutOvershoot) {
  JointVelocityReferenceLimiter limiter;
  ASSERT_TRUE(limiter.Update(Filled(0.0), Filled(100.0), .002));
  const auto positive = limiter.Update(Filled(.1), Filled(100.0), .002);
  ASSERT_TRUE(positive);
  EXPECT_EQ(*positive, Filled(.1));
  const auto negative = limiter.Update(Filled(-.1), Filled(100.0), .004);
  ASSERT_TRUE(negative);
  EXPECT_EQ(*negative, Filled(-.1));
}

TEST(JointVelocityReferenceLimiter, DisabledJointsPassRawAndRefreshHistory) {
  JointVelocityReferenceLimiter limiter;
  ASSERT_TRUE(limiter.Update(Filled(0.0), Filled(100.0), .002));
  auto limits = Filled(100.0);
  limits[0] = 0.0;
  const auto mixed = limiter.Update(Filled(3.0), limits, .002);
  ASSERT_TRUE(mixed);
  EXPECT_DOUBLE_EQ((*mixed)[0], 3.0);
  EXPECT_DOUBLE_EQ((*mixed)[1], .2);
  const auto reenabled = limiter.Update(Filled(4.0), Filled(100.0), .002);
  ASSERT_TRUE(reenabled);
  EXPECT_NEAR((*reenabled)[0], 3.2, 1e-12);
  EXPECT_NEAR((*reenabled)[1], .4, 1e-12);
}

TEST(JointVelocityReferenceLimiter, ResetSeedsNewEpochInsteadOfBridgingOldVelocity) {
  JointVelocityReferenceLimiter limiter;
  ASSERT_TRUE(limiter.Update(Filled(5.0), Filled(100.0), .002));
  limiter.Reset();
  const auto result = limiter.Update(Filled(-5.0), Filled(100.0), .002);
  ASSERT_TRUE(result);
  EXPECT_EQ(*result, Filled(-5.0));
}

TEST(JointVelocityReferenceLimiter, InvalidPeriodsRejectFrameAndInvalidateHistory) {
  const std::array<double, 5> invalid_periods{
      0.0, -.002, .2, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()};
  for (double dt : invalid_periods) {
    JointVelocityReferenceLimiter limiter;
    ASSERT_TRUE(limiter.Update(Filled(1.0), Filled(100.0), .002));
    EXPECT_FALSE(limiter.Update(Filled(8.0), Filled(100.0), dt));
    const auto resumed = limiter.Update(Filled(-3.0), Filled(100.0), .002);
    ASSERT_TRUE(resumed);
    EXPECT_EQ(*resumed, Filled(-3.0));
  }
}

TEST(JointVelocityReferenceLimiter, InvalidLastJointRejectsEntireFrameAndReseeds) {
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity()}) {
    JointVelocityReferenceLimiter limiter;
    ASSERT_TRUE(limiter.Update(Filled(0.0), Filled(100.0), .002));
    auto raw = Filled(10.0);
    raw.back() = bad;
    EXPECT_FALSE(limiter.Update(raw, Filled(100.0), .002));
    const auto resumed = limiter.Update(Filled(-4.0), Filled(100.0), .002);
    ASSERT_TRUE(resumed);
    EXPECT_EQ(*resumed, Filled(-4.0));
  }
}

TEST(JointVelocityReferenceLimiter, InvalidLimitCannotDisableProtectionSilently) {
  for (double bad : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity()}) {
    JointVelocityReferenceLimiter limiter;
    ASSERT_TRUE(limiter.Update(Filled(0.0), Filled(100.0), .002));
    auto limits = Filled(100.0);
    limits.back() = bad;
    EXPECT_FALSE(limiter.Update(Filled(10.0), limits, .002));
    const auto resumed = limiter.Update(Filled(2.0), Filled(100.0), .002);
    ASSERT_TRUE(resumed);
    EXPECT_EQ(*resumed, Filled(2.0));
  }
}

}  // namespace
}  // namespace custom_dog_control
