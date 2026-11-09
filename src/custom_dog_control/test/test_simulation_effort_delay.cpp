#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "custom_dog_control/control/JointCommandUtils.hpp"
#include "custom_dog_control/control/SimulationEffortDelay.hpp"

namespace custom_dog_control {
namespace {

SimulationEffortDelay::Effort CommandAt(int cycle) {
  SimulationEffortDelay::Effort effort;
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    effort[joint] = 100.0 * cycle + joint + 0.25;
  }
  return effort;
}

TEST(SimulationEffortDelay, DefaultZeroDelayPreservesEveryCurrentAxis) {
  SimulationEffortDelay delay;
  for (int cycle = 0; cycle < 100; ++cycle) {
    EXPECT_EQ(delay.Process(CommandAt(cycle), true), CommandAt(cycle));
  }
}

TEST(SimulationEffortDelay, OneCycleSendsPreviousComposedCommand) {
  SimulationEffortDelay delay;
  delay.Configure(1);
  EXPECT_EQ(delay.Process(CommandAt(0), true), SimulationEffortDelay::Effort{});
  for (int cycle = 1; cycle < 100; ++cycle) {
    EXPECT_EQ(delay.Process(CommandAt(cycle), true), CommandAt(cycle - 1));
  }
}

TEST(SimulationEffortDelay, AllSupportedDelaysHaveExactLatencyAcrossWraparound) {
  for (std::int64_t cycles = 2; cycles <= SimulationEffortDelay::kMaxCycles; ++cycles) {
    SimulationEffortDelay delay;
    delay.Configure(cycles);
    for (int cycle = 0; cycle < 150; ++cycle) {
      SCOPED_TRACE("delay=" + std::to_string(cycles) + ", cycle=" + std::to_string(cycle));
      const auto expected = cycle < cycles
          ? SimulationEffortDelay::Effort{} : CommandAt(cycle - cycles);
      EXPECT_EQ(delay.Process(CommandAt(cycle), true), expected);
    }
  }
}

TEST(SimulationEffortDelay, SafetyBypassesImmediatelyAndDiscardsQueuedMotion) {
  SimulationEffortDelay delay;
  delay.Configure(3);
  for (int cycle = 0; cycle < 8; ++cycle) delay.Process(CommandAt(cycle), true);
  const auto safe = CommandAt(-10);
  EXPECT_EQ(delay.Process(safe, false), safe);
  // Repeated safe writes also bypass; none may seed a later motion pipeline.
  EXPECT_EQ(delay.Process(CommandAt(-11), false), CommandAt(-11));
  for (int cycle = 0; cycle < 3; ++cycle) {
    EXPECT_EQ(delay.Process(CommandAt(100 + cycle), true), SimulationEffortDelay::Effort{});
  }
  EXPECT_EQ(delay.Process(CommandAt(103), true), CommandAt(100));
}

TEST(SimulationEffortDelay, LifecycleResetAndReconfigureDiscardAllPendingCommands) {
  SimulationEffortDelay delay;
  delay.Configure(1);
  delay.Process(CommandAt(0), true);
  delay.Reset();
  EXPECT_EQ(delay.Process(CommandAt(1), true), SimulationEffortDelay::Effort{});
  EXPECT_EQ(delay.Process(CommandAt(2), true), CommandAt(1));
  delay.Configure(2);
  EXPECT_EQ(delay.Process(CommandAt(3), true), SimulationEffortDelay::Effort{});
  EXPECT_EQ(delay.Process(CommandAt(4), true), SimulationEffortDelay::Effort{});
  EXPECT_EQ(delay.Process(CommandAt(5), true), CommandAt(3));
  delay.Configure(0);
  EXPECT_EQ(delay.Process(CommandAt(6), true), CommandAt(6));
}

TEST(SimulationEffortDelay, DelaysSaturatedPdAndFeedforwardFromTheOriginalSample) {
  SimulationEffortDelay delay;
  delay.Configure(1);
  JointSample measured;
  HybridJointCommand command;
  command.position.fill(1.0);
  command.kp.fill(55.0);
  command.kd.fill(2.5);
  command.effort.fill(5.0);
  SimulationEffortDelay::Effort effort;
  const auto compose = [&]() {
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      effort[joint] = BoundedEquivalentEffort(command, measured, joint, 23.7);
    }
    return effort;
  };
  EXPECT_EQ(delay.Process(compose(), true), SimulationEffortDelay::Effort{});
  measured.position.fill(2.0);
  measured.velocity.fill(0.5);
  const auto output = delay.Process(compose(), true);
  for (double value : output) EXPECT_DOUBLE_EQ(value, 23.7);
  const auto next = delay.Process(compose(), true);
  for (double value : next) EXPECT_DOUBLE_EQ(value, -23.7);
}

TEST(SimulationEffortDelay, RejectsInvalidIntegerRangeWithoutChangingPipeline) {
  SimulationEffortDelay delay;
  delay.Configure(1);
  delay.Process(CommandAt(0), true);
  EXPECT_THROW(delay.Configure(-1), std::invalid_argument);
  EXPECT_THROW(delay.Configure(11), std::invalid_argument);
  EXPECT_THROW(delay.Configure(std::numeric_limits<std::int64_t>::max()), std::invalid_argument);
  EXPECT_EQ(delay.Process(CommandAt(1), true), CommandAt(0));
}

}  // namespace
}  // namespace custom_dog_control
