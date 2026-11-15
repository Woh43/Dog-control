#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include <ocs2_core/PreComputation.h>

#include <legged_interface/SwingReferenceSettings.h>
#include <legged_interface/SwitchedModelReferenceManager.h>
#include <legged_interface/constraint/NormalVelocityConstraintCppAd.h>
#include <legged_interface/constraint/ZeroVelocityConstraintCppAd.h>

namespace ocs2 {
namespace legged_robot {
namespace {

class TemporaryInfo {
 public:
  explicit TemporaryInfo(const std::string& text) {
    char name[] = "/tmp/custom_dog_swing_reference_XXXXXX";
    const int fd = mkstemp(name);
    if (fd < 0) throw std::runtime_error("mkstemp failed");
    close(fd);
    path_ = name;
    std::ofstream(path_) << text;
  }
  ~TemporaryInfo() { std::remove(path_.c_str()); }
  const std::string& path() const { return path_; }
 private:
  std::string path_;
};

// A Cartesian identity plant exercises the actual EndEffectorLinearConstraint evaluator/Jacobians.
class CartesianFootKinematics final : public EndEffectorKinematics<scalar_t> {
 public:
  CartesianFootKinematics* clone() const override { return new CartesianFootKinematics(*this); }
  const std::vector<std::string>& getIds() const override { return ids_; }
  std::vector<vector3_t> getPosition(const vector_t& state) const override { return {state.head<3>()}; }
  std::vector<vector3_t> getVelocity(const vector_t&, const vector_t& input) const override { return {input.head<3>()}; }
  std::vector<vector3_t> getOrientationError(const vector_t&, const std::vector<quaternion_t>&) const override {
    return {vector3_t::Zero()};
  }
  std::vector<VectorFunctionLinearApproximation> getPositionLinearApproximation(const vector_t& state) const override {
    auto result = VectorFunctionLinearApproximation::Zero(3, state.size(), 0);
    result.f = state.head<3>();
    result.dfdx.leftCols<3>().setIdentity();
    return {result};
  }
  std::vector<VectorFunctionLinearApproximation> getVelocityLinearApproximation(const vector_t& state,
                                                                              const vector_t& input) const override {
    auto result = VectorFunctionLinearApproximation::Zero(3, state.size(), input.size());
    result.f = input.head<3>();
    result.dfdu.leftCols<3>().setIdentity();
    return {result};
  }
  std::vector<VectorFunctionLinearApproximation> getOrientationErrorLinearApproximation(
      const vector_t& state, const std::vector<quaternion_t>&) const override {
    return {VectorFunctionLinearApproximation::Zero(3, state.size(), 0)};
  }
 private:
  const std::vector<std::string> ids_{"foot"};
};

ModeSchedule TrotSchedule() { return ModeSchedule({.2, .4, .6, .8}, {15, 9, 6, 9, 15}); }

SwingTrajectoryPlanner::Config SwingConfig() {
  SwingTrajectoryPlanner::Config config;
  config.swingHeight = .10;
  config.swingTimeScale = .15;
  config.liftOffVelocity = .05;
  config.touchDownVelocity = -.10;
  return config;
}

std::shared_ptr<GaitSchedule> MakeGaitSchedule() {
  return std::make_shared<GaitSchedule>(TrotSchedule(), ModeSequenceTemplate({0., .2, .4}, {9, 6}), 0.);
}

class ReferenceManagerProbe final : public SwitchedModelReferenceManager {
 public:
  using SwitchedModelReferenceManager::SwitchedModelReferenceManager;
  using SwitchedModelReferenceManager::modifyReferences;
};

TEST(AbsoluteSwingReference, MissingSectionPreservesConfiguredPlaneZeroAndVelocityOnlyDefaults) {
  TemporaryInfo file("model_settings\n{\n positionErrorGain 13\n}\n");
  const auto settings = loadSwingReferenceSettings(file.path(), "swing_reference", false);
  EXPECT_DOUBLE_EQ(settings.flatGroundFootCenterHeight, 0.);
  EXPECT_DOUBLE_EQ(settings.positionErrorGain, 0.);
}

TEST(AbsoluteSwingReference, SeparateSwingSettingsDoNotReadOrChangeTheStanceGain) {
  TemporaryInfo file("model_settings\n{\n positionErrorGain 0\n}\nswing_reference\n{\n"
                     " flatGroundFootCenterHeight 0.026\n positionErrorGain 20\n}\n");
  const auto settings = loadSwingReferenceSettings(file.path(), "swing_reference", false);
  EXPECT_DOUBLE_EQ(settings.flatGroundFootCenterHeight, .026);
  EXPECT_DOUBLE_EQ(settings.positionErrorGain, 20.);
}

TEST(AbsoluteSwingReference, InvalidConfigurationIsRejectedInsteadOfDisablingRestorationSilently) {
  for (const std::string& value : {"-1", "nan", "inf", "invalid"}) {
    TemporaryInfo file("swing_reference\n{\n positionErrorGain " + value + "\n}\n");
    EXPECT_ANY_THROW(loadSwingReferenceSettings(file.path(), "swing_reference", false));
  }
  TemporaryInfo badHeight("swing_reference\n{\n flatGroundFootCenterHeight nan\n}\n");
  EXPECT_ANY_THROW(loadSwingReferenceSettings(badHeight.path(), "swing_reference", false));
}

TEST(AbsoluteSwingReference, SphereCenterPlaneMakesTenCentimeterClearancePeakAtPointOneTwoSixMeters) {
  SwingTrajectoryPlanner planner(SwingConfig(), 4);
  planner.update(TrotSchedule(), .026);
  // FR/RL swing in .4-.6, FL/RR swing in .2-.4.
  for (const auto leg : {0u, 3u}) EXPECT_NEAR(planner.getZpositionConstraint(leg, .5), .126, 1e-12);
  for (const auto leg : {1u, 2u}) EXPECT_NEAR(planner.getZpositionConstraint(leg, .3), .126, 1e-12);
  EXPECT_NEAR(planner.getZpositionConstraint(1, .2), .026, 1e-12);
  EXPECT_NEAR(planner.getZpositionConstraint(1, .4), .026, 1e-12);
  EXPECT_NEAR(planner.getZpositionConstraint(0, .3), .026, 1e-12);
  EXPECT_NEAR(planner.getZvelocityConstraint(1, .3), 0., 1e-12);
}

TEST(AbsoluteSwingReference, ReplanningFromDifferentObservedHeightsKeepsTheAbsolutePlaneAndApex) {
  auto planner = std::make_shared<SwingTrajectoryPlanner>(SwingConfig(), 4);
  ReferenceManagerProbe manager(MakeGaitSchedule(), planner, .026);
  TargetTrajectories targets;
  ModeSchedule schedule;
  for (const double observedHeight : {.34, .30, .38}) {
    vector_t observedState = vector_t::Zero(24);
    observedState(8) = observedHeight;
    manager.modifyReferences(.2, .6, observedState, targets, schedule);
    EXPECT_DOUBLE_EQ(manager.getFlatGroundFootCenterHeight(), .026);
    EXPECT_NEAR(planner->getZpositionConstraint(1, .3), .126, 1e-12);
  }
}

TEST(AbsoluteSwingReference, ShortSwingRetainsUpstreamHeightScalingInsteadOfPromisingAnUnreachableFixedApex) {
  SwingTrajectoryPlanner planner(SwingConfig(), 4);
  planner.update(ModeSchedule({.2, .3}, {15, 9, 15}), .026);
  EXPECT_NEAR(planner.getZpositionConstraint(1, .25), .026 + .10 * (.10 / .15), 1e-12);
}

TEST(AbsoluteSwingReference, ZeroGainExactlyRetainsVelocityOnlyConstraintAndIgnoresHeightError) {
  CartesianFootKinematics kinematics;
  EndEffectorLinearConstraint constraint(kinematics, 1, makeSwingNormalVelocityConstraintConfig(.126, -.1, 0.));
  vector_t state = (vector_t(3) << 100., -200., -.4).finished();
  vector_t input = (vector_t(3) << 3., 4., -.1).finished();
  EXPECT_DOUBLE_EQ(constraint.getValue(0., state, input, PreComputation())(0), 0.);
  const auto linear = constraint.getLinearApproximation(0., state, input, PreComputation());
  EXPECT_DOUBLE_EQ(linear.dfdx.norm(), 0.);
  EXPECT_DOUBLE_EQ(linear.dfdu(0, 2), 1.);
}

TEST(AbsoluteSwingReference, OneCentimeterHeightErrorWithGainTwentyAddsPointTwoMetersPerSecondUpward) {
  CartesianFootKinematics kinematics;
  EndEffectorLinearConstraint constraint(kinematics, 1, makeSwingNormalVelocityConstraintConfig(.126, .3, 20.));
  vector_t state = (vector_t(3) << 100., -200., .116).finished();
  vector_t input = (vector_t(3) << 5., -6., .3).finished();
  EXPECT_NEAR(constraint.getValue(0., state, input, PreComputation())(0), -.2, 1e-12);
  input(2) = .5;
  EXPECT_NEAR(constraint.getValue(0., state, input, PreComputation())(0), 0., 1e-12);
  state(2) = .136;
  input(2) = .1;
  EXPECT_NEAR(constraint.getValue(0., state, input, PreComputation())(0), 0., 1e-12);
  const auto linear = constraint.getLinearApproximation(0., state, input, PreComputation());
  EXPECT_DOUBLE_EQ(linear.dfdx(0, 2), 20.);
  EXPECT_DOUBLE_EQ(linear.dfdu(0, 2), 1.);
  EXPECT_DOUBLE_EQ(linear.dfdx.leftCols(2).norm(), 0.);
  EXPECT_DOUBLE_EQ(linear.dfdu.leftCols(2).norm(), 0.);
}

TEST(AbsoluteSwingReference, IdealRecoveryDecaysAbsoluteErrorAcrossRepeatedObservationReanchors) {
  CartesianFootKinematics kinematics;
  EndEffectorLinearConstraint constraint(kinematics, 1, makeSwingNormalVelocityConstraintConfig(.126, 0., 20.));
  vector_t state = (vector_t(3) << 0., 0., .026).finished();
  vector_t input = vector_t::Zero(3);
  for (int replan = 0; replan < 10; ++replan) {
    // Solve the actual linear constraint for vz at the freshly re-anchored observed position.
    const auto linear = constraint.getLinearApproximation(0., state, input, PreComputation());
    input(2) = -linear.f(0) / linear.dfdu(0, 2);
    EXPECT_GT(input(2), 0.);
    EXPECT_NEAR(constraint.getValue(0., state, input, PreComputation())(0), 0., 1e-12);
    state(2) += .02 * input(2);
    input.setZero();
  }
  EXPECT_NEAR(.126 - state(2), .10 * std::pow(.6, 10), 1e-12);
}

TEST(AbsoluteSwingReference, SwingRestorationIsInactiveForEveryPlannedSupportLeg) {
  auto planner = std::make_shared<SwingTrajectoryPlanner>(SwingConfig(), 4);
  SwitchedModelReferenceManager manager(MakeGaitSchedule(), planner, .026);
  manager.setModeSchedule(TrotSchedule());
  CartesianFootKinematics kinematics;
  EndEffectorLinearConstraint::Config supportConfig;
  supportConfig.b = vector_t::Zero(3);
  supportConfig.Av = matrix_t::Identity(3, 3);
  for (const double time : {.1, .3, .5, .7, .9}) {
    const auto contactFlags = manager.getContactFlags(time);
    for (size_t leg = 0; leg < 4; ++leg) {
      NormalVelocityConstraintCppAd normal(manager, kinematics, leg);
      ZeroVelocityConstraintCppAd support(manager, kinematics, leg, supportConfig);
      EXPECT_EQ(normal.isActive(time), !contactFlags[leg]);
      EXPECT_EQ(support.isActive(time), contactFlags[leg]);
      // With the unchanged stance gain of zero, a nonzero world foot-center height is not pulled toward z=0.
      const vector_t state = (vector_t(3) << .2, -.3, .026).finished();
      const vector_t input = vector_t::Zero(3);
      EXPECT_DOUBLE_EQ(support.getValue(time, state, input, PreComputation()).norm(), 0.);
    }
  }
}

TEST(AbsoluteSwingReference, DirectInvalidConstraintArgumentsAndNonfinitePlaneAreRejected) {
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(makeSwingNormalVelocityConstraintConfig(nan, 0., 20.), std::invalid_argument);
  EXPECT_THROW(makeSwingNormalVelocityConstraintConfig(.126, nan, 20.), std::invalid_argument);
  EXPECT_THROW(makeSwingNormalVelocityConstraintConfig(.126, 0., nan), std::invalid_argument);
  EXPECT_THROW(makeSwingNormalVelocityConstraintConfig(.126, 0., -1.), std::invalid_argument);
  EXPECT_THROW(SwitchedModelReferenceManager(MakeGaitSchedule(), std::make_shared<SwingTrajectoryPlanner>(SwingConfig(), 4), nan),
               std::invalid_argument);
}

}  // namespace
}  // namespace legged_robot
}  // namespace ocs2
