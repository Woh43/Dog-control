#include "custom_dog_control/control/ControlTypes.hpp"
#include "custom_dog_control/model/StaticSupport.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <pinocchio/multibody/model.hpp>
using namespace custom_dog_control;
TEST(StaticSupport, StandingCanBalanceAndNoContactsCannot) {
  RobotModel robot(CUSTOM_DOG_CONTROL_CANONICAL_URDF, {});
  const auto &m = robot.pin.getModel();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m.nq);
  q[2] = .3;
  for (size_t k = 0; k < 12; ++k) {
    const auto idx =
        m.joints[m.getJointId(std::string(kJointNames[k]))].idx_q();
    q[idx] = k % 3 == 0 ? 0 : (k % 3 == 1 ? .8 : -1.6);
  }
  const auto standing =
      CheckStaticSupport(robot, q, {true, true, true, true}, .5);
  ASSERT_TRUE(standing.feasible);
  EXPECT_NEAR(standing.forces(2) + standing.forces(5) + standing.forces(8) +
                  standing.forces(11),
              robot.info.robotMass * 9.81, 1e-5);
  EXPECT_FALSE(
      CheckStaticSupport(robot, q, {false, false, false, false}, .5).feasible);
  EXPECT_FALSE(
      CheckStaticSupport(robot, q, {true, false, false, false}, .5).feasible);
  q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
      CheckStaticSupport(robot, q, {true, true, true, true}, .5).feasible);
}
