#include <gtest/gtest.h>
#include <algorithm>
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
}  // namespace
}  // namespace custom_dog_control
