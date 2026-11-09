#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "custom_dog_control/control/ContactForceRamp.hpp"

namespace custom_dog_control {
TEST(ContactForceRamp, ExistingStandIsNotUnloadedAtFirstWbcHandoff) {
  ContactForceRamp ramp;
  for (double bound : ramp.Update(ContactFlags(15), .004, .1, 150.)) EXPECT_TRUE(std::isinf(bound));
}
TEST(ContactForceRamp, NewContactLoadsGraduallyWhileOldSupportsRemainAvailable) {
  ContactForceRamp ramp;
  ramp.Update(ContactFlags(15), .004, .1, 150.);
  ramp.Update(ContactFlags(9), .004, .1, 150.);
  auto bounds = ramp.Update(ContactFlags(15), .004, .1, 150.);
  EXPECT_TRUE(std::isinf(bounds[0])); EXPECT_TRUE(std::isinf(bounds[3]));
  EXPECT_DOUBLE_EQ(bounds[1], 0.); EXPECT_DOUBLE_EQ(bounds[2], 0.);
  for (int i = 1; i <= 12; ++i) {
    bounds = ramp.Update(ContactFlags(15), .004, .1, 150.);
    EXPECT_NEAR(bounds[1], 6. * i, 1e-10);
    EXPECT_NEAR(bounds[2], 6. * i, 1e-10);
  }
  // Load transfer can finish on the new diagonal without resetting its age.
  bounds = ramp.Update(ContactFlags(6), .004, .1, 150.);
  EXPECT_NEAR(bounds[1], 78., 1e-10); EXPECT_DOUBLE_EQ(bounds[0], 0.);
  for (int i = 0; i < 20; ++i) bounds = ramp.Update(ContactFlags(6), .004, .1, 150.);
  EXPECT_TRUE(std::isinf(bounds[1]));
}
TEST(ContactForceRamp, LiftoffAndEpochResetCannotReuseContactAge) {
  ContactForceRamp ramp;
  ramp.Update(ContactFlags(15), .004, .1, 150.);
  ramp.Update(ContactFlags(9), .004, .1, 150.);
  ramp.Update(ContactFlags(15), .004, .1, 150.);
  ramp.Update(ContactFlags(15), .004, .1, 150.);
  ramp.Update(ContactFlags(9), .004, .1, 150.);
  EXPECT_DOUBLE_EQ(ramp.Update(ContactFlags(15), .004, .1, 150.)[1], 0.);
  ramp.Reset();
  EXPECT_TRUE(std::isinf(ramp.Update(ContactFlags(15), .004, .1, 150.)[1]));
}
TEST(ContactForceRamp, DisabledRampPreservesImmediateForceAvailability) {
  ContactForceRamp ramp;
  ramp.Update(ContactFlags(9), .004, 0., 150.);
  for (double bound : ramp.Update(ContactFlags(15), .004, 0., 150.)) EXPECT_TRUE(std::isinf(bound));
}
TEST(ContactForceRamp, InvalidParametersAreRejected) {
  ContactForceRamp ramp;
  EXPECT_THROW(ramp.Update(ContactFlags(15), 0., .1, 150.), std::invalid_argument);
  EXPECT_THROW(ramp.Update(ContactFlags(15), .004, -.1, 150.), std::invalid_argument);
  EXPECT_THROW(ramp.Update(ContactFlags(15), .004, .1, 0.), std::invalid_argument);
  EXPECT_THROW(ramp.Update(ContactFlags(15), .004, .1,
                          std::numeric_limits<double>::infinity()), std::invalid_argument);
}
}  // namespace custom_dog_control
