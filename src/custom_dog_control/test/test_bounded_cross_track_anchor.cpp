#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "custom_dog_control/control/BoundedCrossTrackAnchor.hpp"

namespace custom_dog_control {
namespace {
using Xy = std::array<double, 2>;
Xy Rotate(const Xy& xy, double angle) {
  return {std::cos(angle) * xy[0] - std::sin(angle) * xy[1],
          std::sin(angle) * xy[0] + std::cos(angle) * xy[1]};
}
void ExpectNear(const Xy& actual, const Xy& expected) {
  EXPECT_NEAR(actual[0], expected[0], 1e-12);
  EXPECT_NEAR(actual[1], expected[1], 1e-12);
}

TEST(BoundedCrossTrackAnchor, ZeroLimitPreservesFullObservedReanchorExactly) {
  const Xy observed{4.125, -6.25};
  const auto result = BoundedCrossTrackAnchor({-100., 200.}, observed, {.4, -.3}, 0.);
  ASSERT_TRUE(result);
  EXPECT_EQ(*result, observed);
}

TEST(BoundedCrossTrackAnchor, ForwardAndBackwardRetainLateralAnchorOnly) {
  for (double vx : {-.5, .5}) {
    const auto result = BoundedCrossTrackAnchor({1., 2.}, {4., 2.03}, {vx, 0.}, .04);
    ASSERT_TRUE(result);
    ExpectNear(*result, {4., 2.});
  }
}

TEST(BoundedCrossTrackAnchor, LeftAndRightRetainLongitudinalAnchorOnly) {
  for (double vy : {-.3, .3}) {
    const auto result = BoundedCrossTrackAnchor({1., 2.}, {1.03, 4.}, {0., vy}, .04);
    ASSERT_TRUE(result);
    ExpectNear(*result, {1., 4.});
  }
}

TEST(BoundedCrossTrackAnchor, RotatingACommandAndWorldRotatesTheSameResult) {
  const Xy previous{.2, -.1}, observed{.5, -.08}, velocity{.3, 0.};
  const auto result = BoundedCrossTrackAnchor(previous, observed, velocity, .04);
  ASSERT_TRUE(result);
  for (double angle : {.73, -1.4, 2.3}) {
    const auto rotated = BoundedCrossTrackAnchor(
        Rotate(previous, angle), Rotate(observed, angle), Rotate(velocity, angle), .04);
    ASSERT_TRUE(rotated);
    ExpectNear(*rotated, Rotate(*result, angle));
  }
}

TEST(BoundedCrossTrackAnchor, ClipsCrossTrackErrorWithoutAddingAlongTrackError) {
  const Xy previous{0., 0.}, observed{.5, .2}, velocity{.4, 0.};
  const auto result = BoundedCrossTrackAnchor(previous, observed, velocity, .04);
  ASSERT_TRUE(result);
  ExpectNear(*result, {.5, .16});
  const auto diagonal = BoundedCrossTrackAnchor({-4., 5.}, {.8, -.3}, {.3, .4}, .04);
  ASSERT_TRUE(diagonal);
  const Xy error{(*diagonal)[0] - .8, (*diagonal)[1] + .3};
  EXPECT_NEAR(std::hypot(error[0], error[1]), .04, 1e-12);
  EXPECT_NEAR(error[0] * .3 + error[1] * .4, 0., 1e-12);
}

TEST(BoundedCrossTrackAnchor, RepeatedCommandsNeverIntegrateAnAlongTrackTarget) {
  Xy anchor{0., 0.};
  for (int i = 1; i <= 1000; ++i) {
    const Xy observed{.1 * i, .01};
    const auto result = BoundedCrossTrackAnchor(anchor, observed, {.5, 0.}, .04);
    ASSERT_TRUE(result);
    EXPECT_DOUBLE_EQ((*result)[0], observed[0]);
    EXPECT_DOUBLE_EQ((*result)[1], 0.);
    anchor = *result;
  }
}

TEST(BoundedCrossTrackAnchor, ZeroAndNearZeroCommandsUseTheFormerReanchor) {
  const Xy observed{.5, -.2};
  for (const Xy& velocity : {Xy{0., 0.}, Xy{1e-9, -1e-9}}) {
    const auto result = BoundedCrossTrackAnchor({0., 0.}, observed, velocity, .04);
    ASSERT_TRUE(result);
    EXPECT_EQ(*result, observed);
  }
}

TEST(BoundedCrossTrackAnchor, RejectsNonfiniteOrNegativeInputsBeforeBypass) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (double bad : {nan, inf, -inf}) {
    EXPECT_FALSE(BoundedCrossTrackAnchor({bad, 0.}, {0., 0.}, {.1, 0.}, .04));
    EXPECT_FALSE(BoundedCrossTrackAnchor({0., 0.}, {0., bad}, {.1, 0.}, .04));
    EXPECT_FALSE(BoundedCrossTrackAnchor({0., 0.}, {0., 0.}, {bad, 0.}, .04));
    EXPECT_FALSE(BoundedCrossTrackAnchor({0., 0.}, {0., 0.}, {.1, 0.}, bad));
    EXPECT_FALSE(BoundedCrossTrackAnchor({bad, 0.}, {0., 0.}, {.1, 0.}, 0.));
  }
  EXPECT_FALSE(BoundedCrossTrackAnchor({0., 0.}, {0., 0.}, {.1, 0.}, -.04));
}
}  // namespace
}  // namespace custom_dog_control
