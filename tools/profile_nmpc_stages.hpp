#pragma once

// Test-only instrumentation for a generated copy of OCS2 Transcription.cpp.
// Each worker owns its counters. Read them only after the backend is stopped.
#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace dog_nmpc_profile {
enum Stage { Intermediate, Dynamics, Precomputation, Cost, StateEquality,
             StateInputEquality, StateInequality, StateInputInequality,
             Projection, Terminal, Event, Count };
inline const std::array<const char*, Count> names{
  "intermediate_inclusive", "dynamics_discretization", "precomputation", "cost",
  "state_equality", "state_input_equality", "state_inequality", "state_input_inequality",
  "projection", "terminal_inclusive", "event_inclusive"};
struct Counter {
  std::uint64_t calls = 0, wall_ns = 0, cpu_ns = 0;
  std::uint64_t wall_min_ns = std::numeric_limits<std::uint64_t>::max(), wall_max_ns = 0;
};
struct ThreadCounters { long tid; std::array<Counter, Count> stages; };
inline std::atomic<bool> enabled{false};
inline thread_local unsigned scope_depth = 0;
inline thread_local bool capture_this_call = false;
inline std::mutex registry_mutex;
inline std::vector<std::unique_ptr<ThreadCounters>> registry;
inline std::int64_t Now(clockid_t clock) {
  timespec stamp{};
  if (clock_gettime(clock, &stamp)) std::abort();
  return std::int64_t(stamp.tv_sec) * 1000000000LL + stamp.tv_nsec;
}
inline ThreadCounters& Counters() {
  thread_local ThreadCounters* counters = [] {
    std::lock_guard<std::mutex> lock(registry_mutex);
    registry.push_back(std::make_unique<ThreadCounters>());
    auto* value = registry.back().get();
    value->tid = syscall(SYS_gettid);
    return value;
  }();
  return *counters;
}
class Scope {
 public:
  explicit Scope(Stage stage) : stage_(stage) {
    // Pin the capture decision for a whole node. Begin may race with a warmup
    // node already in flight; its nested stages must remain excluded together.
    if (scope_depth++ == 0) capture_this_call = enabled.load(std::memory_order_relaxed);
    active_ = capture_this_call;
    if (active_) { counters_ = &Counters(); wall_ = Now(CLOCK_MONOTONIC); cpu_ = Now(CLOCK_THREAD_CPUTIME_ID); }
  }
  ~Scope() {
    if (active_) {
      const auto cpu = Now(CLOCK_THREAD_CPUTIME_ID) - cpu_;
      const auto wall = Now(CLOCK_MONOTONIC) - wall_;
      auto& c = counters_->stages[stage_];
      ++c.calls; c.cpu_ns += cpu; c.wall_ns += wall;
      c.wall_min_ns = std::min(c.wall_min_ns, std::uint64_t(wall));
      c.wall_max_ns = std::max(c.wall_max_ns, std::uint64_t(wall));
    }
    --scope_depth;
  }
 private:
  Stage stage_;
  bool active_ = false;
  ThreadCounters* counters_ = nullptr;
  std::int64_t wall_ = 0, cpu_ = 0;
};
}  // namespace dog_nmpc_profile

extern "C" void dog_nmpc_profile_begin() {
  dog_nmpc_profile::enabled.store(true, std::memory_order_release);
}
extern "C" void dog_nmpc_profile_end() {
  using namespace dog_nmpc_profile;
  enabled.store(false, std::memory_order_release);
  const char* output = std::getenv("DOG_NMPC_PROFILE_OUTPUT");
  if (!output) throw std::runtime_error("DOG_NMPC_PROFILE_OUTPUT missing");
  std::ofstream csv(output);
  if (!csv) throw std::runtime_error("cannot write stage counters");
  csv << "tid,stage,calls,wall_ns,cpu_ns,wall_min_ns,wall_max_ns\n";
  for (const auto& thread : registry)
    for (int i = 0; i < Count; ++i) {
      const auto& c = thread->stages[i];
      if (c.calls) csv << thread->tid << ',' << names[i] << ',' << c.calls << ',' << c.wall_ns << ','
                      << c.cpu_ns << ',' << c.wall_min_ns << ',' << c.wall_max_ns << '\n';
    }
}
