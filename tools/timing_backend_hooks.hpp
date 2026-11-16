#pragma once

// Linked only into the standalone timing probe, never the installed backend.
#include <legged_wbc/WeightedWbc.h>
#include <sched.h>
#include <sys/prctl.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>

namespace timing_probe {
struct BackendProbeOptions {
  bool wbc_reuse = false, wbc_compare = false;
  std::string nmpc_cpus;
};
inline BackendProbeOptions backend_options;
struct WbcProbeResult {
  legged::WeightedWbc::SolverMetrics measured, reference;
  bool compared = false, comparison_ok = true;
  double equality_residual = 0, inequality_violation = 0;
  double reference_equality_residual = 0, reference_inequality_violation = 0;
  double max_solution_difference = 0, max_torque_difference = 0, objective_relative_difference = 0;
};
inline thread_local WbcProbeResult last_wbc_probe;

inline cpu_set_t ParseCpuSet(const std::string& value) {
  cpu_set_t mask, allowed;
  CPU_ZERO(&mask);
  if (sched_getaffinity(0, sizeof(allowed), &allowed)) throw std::runtime_error("read CPU affinity failed");
  std::stringstream stream(value);
  std::string entry;
  while (std::getline(stream, entry, ',')) {
    const auto dash = entry.find('-');
    std::size_t used = 0;
    const int first = std::stoi(entry.substr(0, dash), &used);
    if (used != entry.substr(0, dash).size()) throw std::invalid_argument("invalid CPU list");
    int last = first;
    if (dash != std::string::npos) {
      const auto tail = entry.substr(dash + 1);
      last = std::stoi(tail, &used);
      if (used != tail.size()) throw std::invalid_argument("invalid CPU range");
    }
    if (first < 0 || last < first || last >= CPU_SETSIZE) throw std::invalid_argument("invalid CPU range");
    for (int cpu = first; cpu <= last; ++cpu) {
      if (!CPU_ISSET(cpu, &allowed)) throw std::invalid_argument("requested CPU outside inherited affinity");
      CPU_SET(cpu, &mask);
    }
  }
  if (!CPU_COUNT(&mask)) throw std::invalid_argument("empty CPU list");
  return mask;
}
inline void ApplyCpuSet(const cpu_set_t& mask) {
  if (sched_setaffinity(0, sizeof(mask), &mask))
    throw std::runtime_error(std::string("set CPU affinity: ") + std::strerror(errno));
}
// A prevalidated mask is necessary: a split control thread cannot subsequently
// validate NMPC CPUs against its own narrower mask.
inline cpu_set_t nmpc_cpu_mask;
inline bool split_nmpc_cpus = false;
class NmpcConstructionAffinity {
 public:
  NmpcConstructionAffinity() {
    if (split_nmpc_cpus) {
      if (sched_getaffinity(0, sizeof(previous_), &previous_)) throw std::runtime_error("read affinity failed");
      ApplyCpuSet(nmpc_cpu_mask);
      active_ = true;
    }
  }
  void Restore() {
    if (active_) {
      ApplyCpuSet(previous_);
      active_ = false;
    }
  }
  ~NmpcConstructionAffinity() noexcept {
    // Normal completion calls Restore() explicitly so an error is observable.
    // During unwinding, make a best-effort restoration without throwing again.
    if (active_) sched_setaffinity(0, sizeof(previous_), &previous_);
  }
  NmpcConstructionAffinity(const NmpcConstructionAffinity&) = delete;
  NmpcConstructionAffinity& operator=(const NmpcConstructionAffinity&) = delete;
 private:
  cpu_set_t previous_{};
  bool active_ = false;
};
}  // namespace timing_probe
