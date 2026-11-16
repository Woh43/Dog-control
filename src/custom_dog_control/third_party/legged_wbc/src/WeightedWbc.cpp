//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/WeightedWbc.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <qpOASES.hpp>

namespace legged {

namespace {
using Clock = std::chrono::steady_clock;
double elapsedMs(Clock::time_point begin, Clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

qpOASES::Options solverOptions() {
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  options.enableEqualities = qpOASES::BT_TRUE;
  return options;
}
}  // namespace

void WeightedWbc::invalidateSolver(std::uint32_t reason) {
  qpProblem_.reset();
  activeBuffer_ = -1;
  pendingResetReasons_ |= reason;
}

void WeightedWbc::setSolverReuseEnabled(bool enabled) {
  if (solverReuseEnabled_ != enabled) {
    invalidateSolver(kExplicitReset);
    solverReuseEnabled_ = enabled;
  }
}

void WeightedWbc::resetSolverState() {
  invalidateSolver(kExplicitReset);
}

void WeightedWbc::resetInputHistory() {
  WbcBase::resetInputHistory();
  invalidateSolver(kInputHistoryReset);
}

vector_t WeightedWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                             scalar_t period) {
  const auto begin = Clock::now();
  lastSolverMetrics_ = {};
  auto& metrics = lastSolverMetrics_;
  metrics.reuse_enabled = solverReuseEnabled_;
  // Rejecting the previous command resets input differentiation in the backend.
  // A solver active set must not silently bridge that observation epoch either.
  if (qpProblem_ && !inputHistoryInitialized_) invalidateSolver(kInputHistoryReset);
  if (qpProblem_ && mode != solverMode_) invalidateSolver(kContactChanged);
  metrics.reset_reasons = pendingResetReasons_;
  pendingResetReasons_ = 0;

  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);
  auto stamp = Clock::now();
  metrics.model_ms = elapsedMs(begin, stamp);

  Task constraints = formulateConstraints();
  auto next = Clock::now();
  metrics.constraints_ms = elapsedMs(stamp, next);
  stamp = next;
  // Form stateful objectives exactly once: formulating base acceleration also
  // advances inputLast_, so a retry must reuse these same task matrices.
  Task weightedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
  next = Clock::now();
  metrics.tasks_ms = elapsedMs(stamp, next);
  stamp = next;

  const auto variables = static_cast<Eigen::Index>(getNumDecisionVars());
  const Eigen::Index equalities = constraints.b_.size();
  const Eigen::Index inequalities = constraints.f_.size();
  const Eigen::Index numConstraints = equalities + inequalities;
  std::size_t normalBoundMask = 0;
  for (std::size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (contactFlag_[i] && std::isfinite(contactNormalForceUpperBounds_(i))) {
      normalBoundMask |= std::size_t{1} << i;
    }
  }
  if (qpProblem_ &&
      (qpProblem_->getNV() != variables || qpProblem_->getNC() != numConstraints ||
       solverEqualityRows_ != equalities || solverInequalityRows_ != inequalities ||
       solverNormalBoundMask_ != normalBoundMask)) {
    invalidateSolver(kLayoutChanged);
    metrics.reset_reasons |= pendingResetReasons_;
    pendingResetReasons_ = 0;
  }

  // Matrix lifetimes extend through the next update. Buffers are resized only
  // while no solver references them, or while they are the inactive bank.
  const int bufferIndex = activeBuffer_ == 0 ? 1 : 0;
  auto& buffers = qpBuffers_[bufferIndex];
  auto& qpSol = buffers.solution;
  qpSol.setZero(variables);
  const auto finishFailure = [&]() -> vector_t {
    qpSol.setZero();  // Never return a failed or preceding solution as torque.
    lastSolverSucceeded_ = false;
    lastEqualityResidual_ = std::numeric_limits<scalar_t>::infinity();
    lastInequalityViolation_ = std::numeric_limits<scalar_t>::infinity();
    metrics.failed = true;
    invalidateSolver(kSolveFailed);
    metrics.reset_reasons |= pendingResetReasons_;
    pendingResetReasons_ = 0;
    metrics.total_ms = elapsedMs(begin, Clock::now());
    return qpSol;
  };
  // Check shapes before Eigen block assignments, including injected bad tasks
  // used by the isolated failure/recovery comparison.
  const bool shapesValid =
      constraints.a_.rows() == equalities && constraints.d_.rows() == inequalities &&
      (equalities == 0 || constraints.a_.cols() == variables) &&
      (inequalities == 0 || constraints.d_.cols() == variables) &&
      weightedTask.a_.cols() == variables &&
      weightedTask.a_.rows() == weightedTask.b_.size();
  if (!shapesValid || !constraints.a_.allFinite() || !constraints.b_.allFinite() ||
      !constraints.d_.allFinite() || !constraints.f_.allFinite() ||
      !weightedTask.a_.allFinite() || !weightedTask.b_.allFinite()) {
    metrics.validation_failed = true;
    metrics.assembly_ms = elapsedMs(stamp, Clock::now());
    return finishFailure();
  }
  buffers.a.resize(numConstraints, variables);
  buffers.lb_a.resize(numConstraints);
  buffers.ub_a.resize(numConstraints);
  if (equalities > 0) {
    buffers.a.topRows(equalities) = constraints.a_;
    buffers.lb_a.head(equalities) = constraints.b_;
    buffers.ub_a.head(equalities) = constraints.b_;
  }
  if (inequalities > 0) {
    buffers.a.bottomRows(inequalities) = constraints.d_;
    buffers.lb_a.tail(inequalities).setConstant(-qpOASES::INFTY);
    buffers.ub_a.tail(inequalities) = constraints.f_;
  }
  buffers.h.resize(variables, variables);
  buffers.g.resize(variables);
  const auto buildCost = [&]() {
    buffers.h.noalias() = weightedTask.a_.transpose() * weightedTask.a_;
    buffers.g.noalias() = -weightedTask.a_.transpose() * weightedTask.b_;
  };
  buildCost();
  next = Clock::now();
  metrics.assembly_ms = elapsedMs(stamp, next);

  const auto validate = [&](qpOASES::QProblem& problem,
                            qpOASES::returnValue status) {
    const auto validationBegin = Clock::now();
    lastSolverSucceeded_ = status == qpOASES::SUCCESSFUL_RETURN &&
        problem.getPrimalSolution(qpSol.data()) == qpOASES::SUCCESSFUL_RETURN &&
        qpSol.allFinite();
    if (lastSolverSucceeded_) {
      lastEqualityResidual_ = equalities > 0
          ? (constraints.a_ * qpSol - constraints.b_).lpNorm<Eigen::Infinity>() : 0.0;
      lastInequalityViolation_ = inequalities > 0
          ? (constraints.d_ * qpSol - constraints.f_).cwiseMax(0.0).maxCoeff() : 0.0;
      // Read the unregularised objective without relying on mutable H storage.
      const vector_t prediction = weightedTask.a_ * qpSol;
      metrics.objective = 0.5 * prediction.squaredNorm() - prediction.dot(weightedTask.b_);
    }
    const bool valid = lastSolverSucceeded_ &&
        std::isfinite(lastEqualityResidual_) && std::isfinite(lastInequalityViolation_) &&
        lastEqualityResidual_ <= 1e-3 && lastInequalityViolation_ <= 1e-4 &&
        std::isfinite(metrics.objective);
    metrics.validation_ms += elapsedMs(validationBegin, Clock::now());
    return valid;
  };

  // Retain the original cold-started QProblem by default. SQProblem is used
  // only for the explicit experimental path; H and A both change each cycle.
  if (!solverReuseEnabled_) {
    const auto solveBegin = Clock::now();
    qpOASES::QProblem problem(variables, numConstraints);
    problem.setOptions(solverOptions());
    int nWsr = 100;
    const auto status = problem.init(buffers.h.data(), buffers.g.data(), buffers.a.data(),
                                    nullptr, nullptr, buffers.lb_a.data(), buffers.ub_a.data(), nWsr);
    metrics.solver_ms += elapsedMs(solveBegin, Clock::now());
    metrics.cold_start = true;
    metrics.n_wsr = metrics.cold_n_wsr = nWsr;
    metrics.status = metrics.cold_status = static_cast<int>(status);
    const bool valid = validate(problem, status);
    metrics.validation_failed = lastSolverSucceeded_ && !valid;
    // Preserve the existing solver-success flag/residual reporting contract;
    // the backend applies its unchanged residual and per-joint torque checks.
    if (!lastSolverSucceeded_) return finishFailure();
  } else {
    bool valid = false;
    if (qpProblem_) {
      const auto solveBegin = Clock::now();
      int nWsr = 100;
      const auto status = qpProblem_->hotstart(
          buffers.h.data(), buffers.g.data(), buffers.a.data(), nullptr, nullptr,
          buffers.lb_a.data(), buffers.ub_a.data(), nWsr);
      metrics.solver_ms += elapsedMs(solveBegin, Clock::now());
      metrics.hot_start = true;
      metrics.n_wsr += nWsr;
      metrics.hot_n_wsr = nWsr;
      metrics.status = metrics.hot_status = static_cast<int>(status);
      valid = validate(*qpProblem_, status);
      if (!valid) {
        metrics.hot_failed = true;
        metrics.validation_failed = lastSolverSucceeded_;
        metrics.cold_retry = true;
        invalidateSolver(kSolveFailed);
        metrics.reset_reasons |= pendingResetReasons_;
        pendingResetReasons_ = 0;
        // SQProblem may have modified H during its failed auxiliary solve.
        // Reassemble this identical objective before the single cold retry.
        const auto assemblyBegin = Clock::now();
        buildCost();
        metrics.assembly_ms += elapsedMs(assemblyBegin, Clock::now());
      }
    }
    if (!valid) {
      const auto solveBegin = Clock::now();
      qpProblem_ = std::make_unique<qpOASES::SQProblem>(variables, numConstraints);
      qpProblem_->setOptions(solverOptions());
      int nWsr = 100;
      const auto status = qpProblem_->init(
          buffers.h.data(), buffers.g.data(), buffers.a.data(), nullptr, nullptr,
          buffers.lb_a.data(), buffers.ub_a.data(), nWsr);
      metrics.solver_ms += elapsedMs(solveBegin, Clock::now());
      metrics.cold_start = true;
      metrics.n_wsr += nWsr;
      metrics.cold_n_wsr = nWsr;
      metrics.status = metrics.cold_status = static_cast<int>(status);
      valid = validate(*qpProblem_, status);
      if (!valid) {
        metrics.validation_failed |= lastSolverSucceeded_;
        return finishFailure();
      }
    }
    activeBuffer_ = bufferIndex;
    solverMode_ = mode;
    solverNormalBoundMask_ = normalBoundMask;
    solverEqualityRows_ = equalities;
    solverInequalityRows_ = inequalities;
  }
  metrics.total_ms = elapsedMs(begin, Clock::now());
  return qpSol.head(getNumPhysicalDecisionVars());
}

Task WeightedWbc::formulateConstraints() {
  return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() +
         formulateJointLimitsTask() + formulateFrictionConeTask() +
         formulateNoContactMotionTask();
}

Task WeightedWbc::formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period) {
  return formulateSwingLegTask() * weightSwingLeg_ + formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
         formulateContactForceTask(inputDesired) * weightContactForce_ +
         formulateJointLimitSlackTask() * weightJointLimitSlack_;
}

void WeightedWbc::loadTasksSetting(const std::string& taskFile, bool verbose) {
  invalidateSolver(kExplicitReset);
  WbcBase::loadTasksSetting(taskFile, verbose);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "weight.";
  if (verbose) {
    std::cerr << "\n #### WBC weight:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
  loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
  loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
  loadData::loadPtreeValue(
      pt, weightJointLimitSlack_, prefix + "jointLimitSlack", verbose);
}

}  // namespace legged
