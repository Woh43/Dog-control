#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <legged_wbc/WeightedWbc.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include "custom_dog_control/control/ControlTypes.hpp"
#include "custom_dog_control/model/RobotModel.hpp"

namespace custom_dog_control {
namespace {
class WbcProbe : public legged::WeightedWbc {
 public:
  using legged::WeightedWbc::WeightedWbc;
  legged::Task Bounds(std::size_t mode) {
    contactFlag_ = ocs2::legged_robot::modeNumber2StanceLeg(mode);
    numContacts_ = std::count(contactFlag_.begin(), contactFlag_.end(), true);
    frictionCoeff_ = .5;
    return formulateFrictionConeTask();
  }
  std::size_t Columns() const { return getNumDecisionVars(); }
  legged::Task Dynamics() { return formulateFloatingBaseEomTask(); }
  legged::Task TorqueBounds() {
    torqueLimits_.resize(3);
    torqueLimits_ << 23.7, 23.7, 45.;
    return formulateTorqueLimitsTask();
  }
  void PrepareBaseTask(const ocs2::vector_t& state, const ocs2::vector_t& input,
                       const ocs2::vector_t& measured) {
    legged::WbcBase::update(state, input, measured, 15, .002);
  }
  legged::Task BaseTask(const ocs2::vector_t& state, const ocs2::vector_t& input,
                       const ocs2::vector_t& measured) {
    basePositionKp_ << 0., 0., 80.;
    basePositionKd_ << 0., 0., 15.;
    PrepareBaseTask(state, input, measured);
    return formulateBaseAccelTask(state, input, .002);
  }
};
class WbcContactBounds : public ::testing::Test {
 protected:
  RobotModel robot{CUSTOM_DOG_CONTROL_CANONICAL_URDF, {}};
  ocs2::CentroidalModelPinocchioMapping mapping{robot.info};
  ocs2::PinocchioEndEffectorKinematics kinematics{
      robot.pin, mapping, {"FR_foot", "FL_foot", "RR_foot", "RL_foot"}};
  WbcProbe wbc{robot.pin, robot.info, kinematics};
};
TEST_F(WbcContactBounds, DefaultDoesNotAlterFrictionCone) {
  const auto task = wbc.Bounds(15);
  EXPECT_EQ(task.d_.rows(), 20);
  auto x = ocs2::vector_t::Zero(wbc.Columns()).eval();
  for (int i = 0; i < 4; ++i) x(robot.info.generalizedCoordinatesNum + 3*i + 2) = 100.;
  EXPECT_LE((task.d_ * x - task.f_).maxCoeff(), 0.);
}
TEST_F(WbcContactBounds, UpperBoundsAreQpInequalitiesAndKeepUnilateralContact) {
  ocs2::vector_t caps(4);
  caps << std::numeric_limits<double>::infinity(), 10., 20., std::numeric_limits<double>::infinity();
  wbc.setContactNormalForceUpperBounds(caps);
  const auto task = wbc.Bounds(15);
  ASSERT_EQ(task.d_.rows(), 22);
  auto x = ocs2::vector_t::Zero(wbc.Columns()).eval();
  const auto fl = robot.info.generalizedCoordinatesNum + 5;
  x(fl) = 10.; EXPECT_LE((task.d_ * x - task.f_).maxCoeff(), 0.);
  x(fl) = 11.; EXPECT_NEAR((task.d_ * x - task.f_).maxCoeff(), 1., 1e-12);
  x(fl) = -1.; EXPECT_NEAR((task.d_ * x - task.f_).maxCoeff(), 1., 1e-12);
}
TEST_F(WbcContactBounds, SwingForcesRemainExactlyZero) {
  ocs2::vector_t caps(4);
  caps << std::numeric_limits<double>::infinity(), 0., 0., std::numeric_limits<double>::infinity();
  wbc.setContactNormalForceUpperBounds(caps);
  const auto task = wbc.Bounds(9);
  EXPECT_EQ(task.a_.rows(), 6); EXPECT_EQ(task.d_.rows(), 16);
  auto x = ocs2::vector_t::Zero(wbc.Columns()).eval();
  x(robot.info.generalizedCoordinatesNum + 5) = 1.;
  EXPECT_DOUBLE_EQ((task.a_ * x - task.b_).norm(), 1.);
}
TEST_F(WbcContactBounds, InvalidBoundsCannotSilentlyDisableProtection) {
  EXPECT_THROW(wbc.setContactNormalForceUpperBounds(ocs2::vector_t::Zero(3)), std::invalid_argument);
  auto caps = ocs2::vector_t::Zero(4).eval();
  caps(2) = -1.; EXPECT_THROW(wbc.setContactNormalForceUpperBounds(caps), std::invalid_argument);
  caps(2) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(wbc.setContactNormalForceUpperBounds(caps), std::invalid_argument);
}
TEST_F(WbcContactBounds, BaseHeightTelemetryIncludesFeedbackAndNeverReplaysPreviousTask) {
  EXPECT_TRUE(std::isnan(wbc.lastBaseVerticalAccelerationTarget()));
  auto state = ocs2::vector_t::Zero(robot.info.stateDim).eval();
  auto input = ocs2::vector_t::Zero(robot.info.inputDim).eval();
  auto measured = ocs2::vector_t::Zero(2 * robot.info.generalizedCoordinatesNum).eval();
  state(8) = measured(5) = .34;
  const auto nominal = wbc.BaseTask(state, input, measured);
  ASSERT_TRUE(nominal.b_.allFinite());
  EXPECT_DOUBLE_EQ(wbc.lastBaseVerticalAccelerationTarget(), nominal.b_(2));
  measured(5) -= .03;
  measured(robot.info.generalizedCoordinatesNum + 5) = -.2;
  const auto feedback = wbc.BaseTask(state, input, measured);
  EXPECT_NEAR(feedback.b_(2) - nominal.b_(2), 80. * .03 + 15. * .2, 1e-10);
  EXPECT_DOUBLE_EQ(wbc.lastBaseVerticalAccelerationTarget(), feedback.b_(2));
  wbc.PrepareBaseTask(state, input, measured);
  EXPECT_TRUE(std::isnan(wbc.lastBaseVerticalAccelerationTarget()));
}
TEST_F(WbcContactBounds, DynamicTorqueReservationIsAHardPerJointConstraint) {
  auto caps = ocs2::vector_t::Constant(12, 100.).eval();
  caps(1) = 4.;
  wbc.setJointTorqueUpperBounds(caps);
  const auto task = wbc.TorqueBounds();
  const auto column = robot.info.generalizedCoordinatesNum + 12 + 1;
  auto x = ocs2::vector_t::Zero(wbc.Columns()).eval();
  x(column) = 4.; EXPECT_LE((task.d_ * x - task.f_).maxCoeff(), 0.);
  x(column) = 4.1; EXPECT_NEAR((task.d_ * x - task.f_).maxCoeff(), .1, 1e-12);
  x(column) = -4.1; EXPECT_NEAR((task.d_ * x - task.f_).maxCoeff(), .1, 1e-12);
  EXPECT_DOUBLE_EQ(task.f_(0), 23.7);  // An override can never raise nominal limits.
  EXPECT_THROW(wbc.setJointTorqueUpperBounds(ocs2::vector_t::Zero(11)), std::invalid_argument);
  caps(1) = -1.; EXPECT_THROW(wbc.setJointTorqueUpperBounds(caps), std::invalid_argument);
}
TEST_F(WbcContactBounds, MotorPdIsIncludedInDynamicsAndSignedTotalTorqueBounds) {
  auto state = ocs2::vector_t::Zero(robot.info.stateDim).eval();
  auto input = ocs2::vector_t::Zero(robot.info.inputDim).eval();
  auto measured = ocs2::vector_t::Zero(2 * robot.info.generalizedCoordinatesNum).eval();
  state(8) = measured(5) = .34;
  wbc.PrepareBaseTask(state, input, measured);
  const auto baseline = wbc.Dynamics();
  auto pd = ocs2::vector_t::Zero(12).eval();
  auto lower = pd, upper = pd;
  pd(1) = -6.; lower(1) = -10.; upper(1) = -4.;
  wbc.setMotorPdTorqueModel(pd, lower, upper);
  const auto dynamics = wbc.Dynamics();
  EXPECT_LT((dynamics.a_ - baseline.a_).norm(), 1e-12);
  EXPECT_LT((dynamics.b_.head(6) - baseline.b_.head(6)).norm(), 1e-12);
  EXPECT_LT((dynamics.b_.tail(12) - baseline.b_.tail(12) - pd).norm(), 1e-12);
  const auto task = wbc.TorqueBounds();
  const auto column = robot.info.generalizedCoordinatesNum + 12 + 1;
  auto x = ocs2::vector_t::Zero(wbc.Columns()).eval();
  x(column) = -13.7; EXPECT_LE((task.d_ * x - task.f_).maxCoeff(), 1e-12);
  x(column) = -13.8; EXPECT_NEAR((task.d_ * x - task.f_).maxCoeff(), .1, 1e-12);
  x(column) = 23.8; EXPECT_NEAR((task.d_ * x - task.f_).maxCoeff(), .1, 1e-12);
  EXPECT_THROW(wbc.setMotorPdTorqueModel(upper, lower, pd), std::invalid_argument);
  EXPECT_THROW(wbc.setMotorPdTorqueModel(pd.head(11), lower, upper), std::invalid_argument);
  wbc.setMotorPdTorqueModel(ocs2::vector_t::Zero(12), ocs2::vector_t::Zero(12), ocs2::vector_t::Zero(12));
  EXPECT_LT((wbc.Dynamics().b_ - baseline.b_).norm(), 1e-12);
}
TEST_F(WbcContactBounds, CentroidalDecisionTaskMatchesLegacyForSameJointAcceleration) {
  auto state = ocs2::vector_t::Zero(robot.info.stateDim).eval();
  auto input = ocs2::vector_t::Zero(robot.info.inputDim).eval();
  auto measured = ocs2::vector_t::Zero(2 * robot.info.generalizedCoordinatesNum).eval();
  state(8) = measured(5) = .34;
  wbc.BaseTask(state, input, measured);  // Seed a real previous input.
  for (int i = 0; i < 12; ++i) input(12 + i) = .03 * (i + 1);
  const auto legacy = wbc.BaseTask(state, input, measured);
  wbc.setUseJointAccelerationDecision(true);
  const auto coupled = wbc.BaseTask(state, input, measured);
  const auto coupling = coupled.a_.block(0, 6, 6, 12).eval();
  const auto desiredAcceleration = (input.tail(12) / .002).eval();
  ASSERT_GT(coupling.norm(), 1e-5);
  EXPECT_LT((coupled.b_ - legacy.b_ - coupling * desiredAcceleration).norm(), 1e-8);
  auto x = ocs2::vector_t::Zero(wbc.Columns()).eval();
  x.segment(6, 12) = desiredAcceleration;
  EXPECT_LT(((coupled.a_ * x - coupled.b_) -
             (legacy.a_ * x - legacy.b_)).norm(), 1e-8);
  EXPECT_LT((wbc.lastBaseVerticalJointAccelerationCoupling() -
             coupling.row(2).transpose()).norm(), 1e-12);
  // Effective base-z target, rather than raw momentum-task RHS, has the
  // same residual as the complete QP row.
  const double target = coupled.b_(2) - coupling.row(2).dot(x.segment(6, 12));
  EXPECT_NEAR((coupled.a_ * x - coupled.b_)(2), x(2) - target, 1e-10);
  wbc.resetInputHistory();
  const auto afterReset = wbc.BaseTask(state, input, measured);
  EXPECT_LT((afterReset.b_ - coupled.b_).norm(), 1e-12);
}
TEST_F(WbcContactBounds, RestartSeedsVelocityInsteadOfDifferentiatingAcrossEpochs) {
  auto state = ocs2::vector_t::Zero(robot.info.stateDim).eval();
  auto input = ocs2::vector_t::Zero(robot.info.inputDim).eval();
  auto measured = ocs2::vector_t::Zero(2 * robot.info.generalizedCoordinatesNum).eval();
  state(8) = measured(5) = .34;
  input.tail(12).setConstant(.5);
  const auto first = wbc.BaseTask(state, input, measured);
  const auto next = wbc.BaseTask(state, input, measured);
  EXPECT_LT((first.b_ - next.b_).norm(), 1e-12);
  input.tail(12).setConstant(-.5);
  wbc.resetInputHistory();
  const auto restarted = wbc.BaseTask(state, input, measured);
  const auto seeded = wbc.BaseTask(state, input, measured);
  EXPECT_LT((restarted.b_ - seeded.b_).norm(), 1e-12);
}
TEST_F(WbcContactBounds, NominalHeightFeedbackIsBoundedAndDoesNotMoveDesiredState) {
  auto state = ocs2::vector_t::Zero(robot.info.stateDim).eval();
  auto input = ocs2::vector_t::Zero(robot.info.inputDim).eval();
  auto measured = ocs2::vector_t::Zero(2 * robot.info.generalizedCoordinatesNum).eval();
  state(8) = measured(5) = .32;
  const auto nominal = wbc.BaseTask(state, input, measured);
  wbc.setNominalHeightFeedback(.34, .5);
  const auto corrected = wbc.BaseTask(state, input, measured);
  EXPECT_NEAR(corrected.b_(2) - nominal.b_(2), .5, 1e-12);
  EXPECT_DOUBLE_EQ(wbc.lastNominalHeightCorrection(), .5);
  EXPECT_DOUBLE_EQ(state(8), .32);
  wbc.setNominalHeightFeedback(.34, 0.);
  const auto disabled = wbc.BaseTask(state, input, measured);
  EXPECT_NEAR(disabled.b_(2), nominal.b_(2), 1e-12);
  EXPECT_THROW(wbc.setNominalHeightFeedback(.34, -1.), std::invalid_argument);
}
}  // namespace
}  // namespace custom_dog_control
