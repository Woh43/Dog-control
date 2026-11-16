#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <limits>

#include <legged_wbc/WeightedWbc.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include "custom_dog_control/model/RobotModel.hpp"

namespace custom_dog_control {
namespace {
class ReuseProbe final : public legged::WeightedWbc {
 public:
  using legged::WeightedWbc::WeightedWbc;
  enum class Injection { kNone, kImpossible, kNaN };
  Injection injection{Injection::kNone};
  ocs2::matrix_t last_h, last_a;

 protected:
  legged::Task formulateConstraints() override {
    auto task = WeightedWbc::formulateConstraints();
    if (injection == Injection::kImpossible) {
      // Keep both layout and dimensions: one variable cannot equal 0 and 1.
      // This exercises a real hot attempt and the bounded cold retry.
      task.a_.topRows(2).setZero();
      task.a_(0, 0) = task.a_(1, 0) = 1.0;
      task.b_(0) = 0.0;
      task.b_(1) = 1.0;
    }
    last_a.resize(task.a_.rows() + task.d_.rows(), task.a_.cols());
    last_a << task.a_, task.d_;
    return task;
  }

  legged::Task formulateWeightedTasks(const ocs2::vector_t& state,
                                      const ocs2::vector_t& input,
                                      ocs2::scalar_t period) override {
    auto task = WeightedWbc::formulateWeightedTasks(state, input, period);
    last_h = task.a_.transpose() * task.a_;
    if (injection == Injection::kNaN) {
      task.a_(0, 0) = std::numeric_limits<double>::quiet_NaN();
    }
    return task;
  }
};

class WbcSolverReuse : public ::testing::Test {
 protected:
  RobotModel robot{CUSTOM_DOG_CONTROL_CANONICAL_URDF, {}};
  ocs2::CentroidalModelPinocchioMapping mapping{robot.info};
  ocs2::PinocchioEndEffectorKinematics kinematics{
      robot.pin, mapping, {"FR_foot", "FL_foot", "RR_foot", "RL_foot"}};
  ReuseProbe cold{robot.pin, robot.info, kinematics};
  ReuseProbe reused{robot.pin, robot.info, kinematics};
  ocs2::vector_t state = ocs2::vector_t::Zero(robot.info.stateDim);
  ocs2::vector_t input = ocs2::vector_t::Zero(robot.info.inputDim);
  ocs2::vector_t measured = ocs2::vector_t::Zero(2 * robot.info.generalizedCoordinatesNum);
  double maxPhysicalDifference{}, maxTorqueDifference{}, maxObjectiveRelativeDifference{};

  void SetUp() override {
    cold.loadTasksSetting(CUSTOM_DOG_CONTROL_WBC_TASK_FILE, false);
    reused.loadTasksSetting(CUSTOM_DOG_CONTROL_WBC_TASK_FILE, false);
    reused.setSolverReuseEnabled(true);
    state(8) = measured(5) = .34;
    state.tail(12) << 0.0, .716130233, -1.454294645,
                      0.0, .715821774, -1.454461972,
                      0.0, .715631114, -1.454556830,
                      0.0, .713540237, -1.455606346;
    measured.segment(6, 12) = state.tail(12);
    SetForces(15);
  }

  void TearDown() override {
    // Other components of a semidefinite QP need not have a unique minimiser.
    // Record their difference without imposing artificial uniqueness.
    const auto record = [this](const char* name, double value) {
      std::ostringstream text;
      text << std::setprecision(17) << value;
      RecordProperty(name, text.str());
    };
    record("max_physical_solution_abs_difference", maxPhysicalDifference);
    record("max_torque_abs_difference_nm", maxTorqueDifference);
    record("max_objective_relative_difference", maxObjectiveRelativeDifference);
  }

  void SetForces(std::size_t mode) {
    const auto contact = ocs2::legged_robot::modeNumber2StanceLeg(mode);
    const auto count = std::count(contact.begin(), contact.end(), true);
    input.head(12).setZero();
    for (std::size_t i = 0; i < contact.size(); ++i) {
      if (contact[i]) input(3 * i + 2) = robot.info.robotMass * 9.81 / count;
    }
  }

  ::testing::AssertionResult Compare(std::size_t mode) {
    const auto reference = cold.update(state, input, measured, mode, .002);
    const auto candidate = reused.update(state, input, measured, mode, .002);
    const auto& a = cold.lastSolverMetrics();
    const auto& b = reused.lastSolverMetrics();
    if (!cold.lastSolverSucceeded() || !reused.lastSolverSucceeded() ||
        a.failed || b.failed || !reference.allFinite() || !candidate.allFinite() ||
        reference.size() != candidate.size() || reference.size() < 12) {
      return ::testing::AssertionFailure()
          << "cold/hot solve failed: cold status=" << a.status << " reused status=" << b.status
          << " hot_status=" << b.hot_status << " cold_retry=" << b.cold_retry
          << " residuals=" << cold.lastEqualityResidual() << ',' << cold.lastInequalityViolation()
          << " / " << reused.lastEqualityResidual() << ',' << reused.lastInequalityViolation();
    }
    if (cold.lastEqualityResidual() > 1e-3 || reused.lastEqualityResidual() > 1e-3 ||
        cold.lastInequalityViolation() > 1e-4 || reused.lastInequalityViolation() > 1e-4) {
      return ::testing::AssertionFailure() << "existing production residual thresholds exceeded";
    }
    const double physicalDifference = (reference - candidate).lpNorm<Eigen::Infinity>();
    const double torqueDifference = (reference.tail(12) - candidate.tail(12)).lpNorm<Eigen::Infinity>();
    const double objectiveRelativeDifference = std::abs(a.objective - b.objective) /
        std::max({1.0, std::abs(a.objective), std::abs(b.objective)});
    maxPhysicalDifference = std::max(maxPhysicalDifference, physicalDifference);
    maxTorqueDifference = std::max(maxTorqueDifference, torqueDifference);
    maxObjectiveRelativeDifference = std::max(maxObjectiveRelativeDifference, objectiveRelativeDifference);
    if (!std::isfinite(objectiveRelativeDifference) || objectiveRelativeDifference > 1e-5 ||
        torqueDifference > 1e-3) {
      return ::testing::AssertionFailure()
          << "same-QP difference: objective_relative=" << objectiveRelativeDifference
          << " (limit 1e-5), torque_abs_nm=" << torqueDifference
          << " (limit 1e-3), physical_abs=" << physicalDifference
          << " cold_objective=" << a.objective << " reused_objective=" << b.objective;
    }
    return ::testing::AssertionSuccess();
  }
};

TEST_F(WbcSolverReuse, DefaultIsColdAndIdenticalInputUsesHotStarts) {
  EXPECT_FALSE(cold.solverReuseEnabled());
  for (int i = 0; i < 40; ++i) {
    SCOPED_TRACE(i);
    ASSERT_TRUE(Compare(15));
    EXPECT_TRUE(cold.lastSolverMetrics().cold_start);
    EXPECT_FALSE(cold.lastSolverMetrics().hot_start);
    EXPECT_EQ(reused.lastSolverMetrics().hot_start, i > 0);
    EXPECT_EQ(reused.lastSolverMetrics().cold_start, i == 0);
    EXPECT_FALSE(reused.lastSolverMetrics().cold_retry);
  }
}

TEST_F(WbcSolverReuse, ChangingPoseVelocityAndDesiredInputUpdatesBothMatrices) {
  SetForces(9);
  const auto nominal = state.tail(12).eval();
  ocs2::matrix_t initialH, initialA;
  for (int i = 0; i < 80; ++i) {
    SCOPED_TRACE(i);
    const double phase = .07 * i;
    measured(0) = .003 * std::sin(phase);
    measured(1) = .002 * std::cos(phase);
    measured(2) = -.002 * std::sin(.8 * phase);
    measured(5) = .34 + .001 * std::sin(phase);
    state(9) = .001 * std::sin(.4 * phase);
    state(10) = -.001 * std::cos(.6 * phase);
    for (int j = 0; j < 12; ++j) {
      const double jointPhase = phase + .1 * j;
      measured(6 + j) = nominal(j) + .001 * std::sin(jointPhase);
      measured(robot.info.generalizedCoordinatesNum + 6 + j) = .003 * std::cos(jointPhase);
      state(12 + j) = nominal(j) + .0005 * std::cos(jointPhase);
      input(12 + j) = .001 * std::sin(jointPhase);
    }
    measured(robot.info.generalizedCoordinatesNum + 1) = .002 * std::cos(phase);
    measured(robot.info.generalizedCoordinatesNum + 3) = .003 * std::sin(phase);
    ASSERT_TRUE(Compare(9));
    EXPECT_EQ(reused.lastSolverMetrics().hot_start, i > 0);
    if (i == 0) { initialH = reused.last_h; initialA = reused.last_a; }
  }
  EXPECT_GT((reused.last_h - initialH).norm(), 1e-7);
  EXPECT_GT((reused.last_a - initialA).norm(), 1e-7);
}

TEST_F(WbcSolverReuse, ContactTransitionsResetEvenWhenContactCountIsUnchanged) {
  bool first = true;
  for (const std::size_t mode : {15u, 9u, 6u, 15u}) {
    SCOPED_TRACE(mode);
    SetForces(mode);
    ASSERT_TRUE(Compare(mode));
    EXPECT_TRUE(reused.lastSolverMetrics().cold_start);
    EXPECT_FALSE(reused.lastSolverMetrics().hot_start);
    if (!first) {
      EXPECT_NE(reused.lastSolverMetrics().reset_reasons & legged::WeightedWbc::kContactChanged, 0u);
    }
    ASSERT_TRUE(Compare(mode));
    EXPECT_TRUE(reused.lastSolverMetrics().hot_start);
    first = false;
  }
}

TEST_F(WbcSolverReuse, SameCountNormalBoundMaskChangeResetsConstraintLayout) {
  auto bounds = ocs2::vector_t::Constant(4, std::numeric_limits<double>::infinity()).eval();
  bounds(0) = 200.0;
  cold.setContactNormalForceUpperBounds(bounds);
  reused.setContactNormalForceUpperBounds(bounds);
  ASSERT_TRUE(Compare(15));
  ASSERT_TRUE(Compare(15));
  ASSERT_TRUE(reused.lastSolverMetrics().hot_start);
  bounds(0) = std::numeric_limits<double>::infinity();
  bounds(1) = 200.0;
  cold.setContactNormalForceUpperBounds(bounds);
  reused.setContactNormalForceUpperBounds(bounds);
  ASSERT_TRUE(Compare(15));
  EXPECT_TRUE(reused.lastSolverMetrics().cold_start);
  EXPECT_FALSE(reused.lastSolverMetrics().hot_start);
  EXPECT_NE(reused.lastSolverMetrics().reset_reasons & legged::WeightedWbc::kLayoutChanged, 0u);
  ASSERT_TRUE(Compare(15));
  EXPECT_TRUE(reused.lastSolverMetrics().hot_start);
}

TEST_F(WbcSolverReuse, InputHistoryResetInvalidatesReuseThroughDerivedAndBaseCalls) {
  ASSERT_TRUE(Compare(15));
  ASSERT_TRUE(Compare(15));
  input.tail(12).setConstant(.03);
  cold.resetInputHistory();
  reused.resetInputHistory();
  ASSERT_TRUE(Compare(15));
  EXPECT_TRUE(reused.lastSolverMetrics().cold_start);
  EXPECT_FALSE(reused.lastSolverMetrics().hot_start);
  EXPECT_NE(reused.lastSolverMetrics().reset_reasons & legged::WeightedWbc::kInputHistoryReset, 0u);
  ASSERT_TRUE(Compare(15));
  input.tail(12).setConstant(-.03);
  static_cast<legged::WbcBase&>(cold).resetInputHistory();
  static_cast<legged::WbcBase&>(reused).resetInputHistory();
  ASSERT_TRUE(Compare(15));
  EXPECT_TRUE(reused.lastSolverMetrics().cold_start);
  EXPECT_FALSE(reused.lastSolverMetrics().hot_start);
  EXPECT_NE(reused.lastSolverMetrics().reset_reasons & legged::WeightedWbc::kInputHistoryReset, 0u);
}

TEST_F(WbcSolverReuse, ImpossibleHotUpdateRetriesOnceAndNeverReturnsOldTorque) {
  ASSERT_TRUE(Compare(15));
  ASSERT_TRUE(Compare(15));
  reused.injection = ReuseProbe::Injection::kImpossible;
  const auto rejected = reused.update(state, input, measured, 15, .002);
  const auto metrics = reused.lastSolverMetrics();
  EXPECT_TRUE(metrics.hot_start);
  EXPECT_TRUE(metrics.hot_failed);
  EXPECT_TRUE(metrics.cold_retry);
  EXPECT_TRUE(metrics.cold_start);
  EXPECT_TRUE(metrics.failed);
  EXPECT_LE(metrics.hot_n_wsr, 100);
  EXPECT_LE(metrics.cold_n_wsr, 100);
  EXPECT_FALSE(reused.lastSolverSucceeded());
  EXPECT_TRUE(rejected.allFinite());
  EXPECT_DOUBLE_EQ(rejected.norm(), 0.0);
  reused.injection = ReuseProbe::Injection::kNone;
  ASSERT_TRUE(Compare(15));
  EXPECT_TRUE(reused.lastSolverMetrics().cold_start);
  EXPECT_FALSE(reused.lastSolverMetrics().hot_start);
}

TEST_F(WbcSolverReuse, NonfiniteTaskRejectsBeforeSolveAndRecoversCold) {
  ASSERT_TRUE(Compare(15));
  ASSERT_TRUE(Compare(15));
  reused.injection = ReuseProbe::Injection::kNaN;
  const auto rejected = reused.update(state, input, measured, 15, .002);
  EXPECT_TRUE(reused.lastSolverMetrics().failed);
  EXPECT_TRUE(reused.lastSolverMetrics().validation_failed);
  EXPECT_FALSE(reused.lastSolverMetrics().hot_start);
  EXPECT_FALSE(reused.lastSolverMetrics().cold_start);
  EXPECT_FALSE(reused.lastSolverSucceeded());
  EXPECT_TRUE(rejected.allFinite());
  EXPECT_DOUBLE_EQ(rejected.norm(), 0.0);
  reused.injection = ReuseProbe::Injection::kNone;
  ASSERT_TRUE(Compare(15));
  EXPECT_TRUE(reused.lastSolverMetrics().cold_start);
  EXPECT_FALSE(reused.lastSolverMetrics().hot_start);
}
}  // namespace
}  // namespace custom_dog_control
