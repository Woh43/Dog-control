//
// Created by qiayuan on 22-12-23.
//

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <qpOASES.hpp>

#include "legged_wbc/WbcBase.h"

namespace legged {

class WeightedWbc : public WbcBase {
 public:
  using WbcBase::WbcBase;

  enum SolverResetReason : std::uint32_t {
    kExplicitReset = 1u << 0,
    kContactChanged = 1u << 1,
    kLayoutChanged = 1u << 2,
    kInputHistoryReset = 1u << 3,
    kSolveFailed = 1u << 4,
  };

  struct SolverMetrics {
    double model_ms{}, constraints_ms{}, tasks_ms{}, assembly_ms{};
    double solver_ms{}, validation_ms{}, total_ms{};
    // 0.5*x'H*x + g'x, using the original task before qpOASES regularisation.
    double objective{std::numeric_limits<double>::quiet_NaN()};
    int n_wsr{}, cold_n_wsr{}, hot_n_wsr{};
    int status{-1}, hot_status{-1}, cold_status{-1};
    std::uint32_t reset_reasons{};
    bool reuse_enabled{}, cold_start{}, hot_start{}, cold_retry{};
    bool hot_failed{}, validation_failed{}, failed{};
  };

  // Experimental opt-in for isolated cold/hot comparisons. Production callers
  // retain a fresh, cold-started QProblem unless they explicitly enable this.
  void setSolverReuseEnabled(bool enabled);
  bool solverReuseEnabled() const { return solverReuseEnabled_; }
  const SolverMetrics& lastSolverMetrics() const { return lastSolverMetrics_; }
  void resetSolverState();
  // WbcBase's method is non-virtual. Also observe its protected history flag
  // at update entry so calls made through WbcBase invalidate reuse as well.
  void resetInputHistory();

  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period) override;

  void loadTasksSetting(const std::string& taskFile, bool verbose) override;

 protected:
  virtual Task formulateConstraints();
  virtual Task formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);

 private:
  using RowMatrix = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  struct QpBuffers {
    RowMatrix h, a;
    vector_t g, lb_a, ub_a, solution;
  };

  void invalidateSolver(std::uint32_t reason);

  scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;
  scalar_t weightJointLimitSlack_{100.0};
  // qpOASES retains shallow references to H/A and may regularise H in-place.
  // Never overwrite the current solver's matrices while building its successor.
  std::array<QpBuffers, 2> qpBuffers_;
  std::unique_ptr<qpOASES::SQProblem> qpProblem_;
  bool solverReuseEnabled_{false};
  int activeBuffer_{-1};
  std::size_t solverMode_{}, solverNormalBoundMask_{};
  Eigen::Index solverEqualityRows_{-1}, solverInequalityRows_{-1};
  std::uint32_t pendingResetReasons_{};
  SolverMetrics lastSolverMetrics_;
};

}  // namespace legged
