#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

// Simulation-only delay of already composed/saturated efforts. No allocation
// occurs in Process/Reset. Active gait transitions keep the same pipeline;
// safety/lifecycle resets bypass it and discard all pending motion commands.
class SimulationEffortDelay {
 public:
  using Effort = std::array<double, kJointCount>;
  static constexpr std::int64_t kMaxCycles = 10;

  void Configure(std::int64_t cycles) {
    if (cycles < 0 || cycles > kMaxCycles) {
      throw std::invalid_argument(
          "simulation_command_delay_cycles must be an integer in [0, 10]");
    }
    delay_cycles_ = static_cast<std::size_t>(cycles);
    Reset();
  }

  void Reset() {
    head_ = 0;
    size_ = 0;
  }

  Effort Process(const Effort& effort, bool allow_delay) {
    if (!allow_delay) {
      Reset();
      return effort;
    }
    if (delay_cycles_ == 0) return effort;
    if (size_ < delay_cycles_) {
      history_[(head_ + size_) % delay_cycles_] = effort;
      ++size_;
      return {};
    }
    const Effort output = history_[head_];
    history_[head_] = effort;
    head_ = (head_ + 1) % delay_cycles_;
    return output;
  }

 private:
  std::array<Effort, kMaxCycles> history_{};
  std::size_t delay_cycles_ = 0;
  std::size_t head_ = 0;
  std::size_t size_ = 0;
};

}  // namespace custom_dog_control
