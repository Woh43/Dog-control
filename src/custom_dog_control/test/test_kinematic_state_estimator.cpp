#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "custom_dog_control/model/RobotModel.hpp"
#include "custom_dog_control/nmpc/KinematicStateEstimator.hpp"

namespace custom_dog_control {
namespace {

constexpr double kPeriod = 0.004;
constexpr int kMinuteOfControlCycles = 60 * 250;

testing::AssertionResult FiniteEstimate(const EstimatedState& state) {
  if (!state.valid || !state.position.allFinite() ||
      !state.velocity_world.allFinite() || !state.euler_zyx.allFinite() ||
      !state.angular_velocity_world.allFinite() || !std::isfinite(state.stamp_seconds)) {
    return testing::AssertionFailure() << "invalid or non-finite estimated body state";
  }
  for (const auto& foot : state.foot_position_world) {
    if (!foot.allFinite()) return testing::AssertionFailure() << "non-finite foot state";
  }
  return testing::AssertionSuccess();
}

class KinematicStateEstimatorTest : public testing::Test {
 protected:
  RobotModel robot_{CUSTOM_DOG_CONTROL_CANONICAL_URDF, {}};

  static JointSample Pose(double thigh = 0.8, double calf = -1.6) {
    JointSample joints;
    joints.valid.fill(1.0);
    for (std::size_t index = 0; index < kJointCount; ++index) {
      joints.position[index] = index % 3 == 0 ? 0.0 : (index % 3 == 1 ? thigh : calf);
    }
    return joints;
  }

  static ImuSample RestingImu() {
    ImuSample imu;
    imu.valid = true;
    imu.linear_acceleration = {0.0, 0.0, 9.81};
    return imu;
  }

  double FlatGroundHeight(const JointSample& joints) {
    const auto& model = robot_.pin.getModel();
    auto& data = robot_.pin.getData();
    ocs2::vector_t q = ocs2::vector_t::Zero(robot_.info.generalizedCoordinatesNum);
    for (std::size_t index = 0; index < kJointCount; ++index) {
      const auto id = model.getJointId(std::string(kJointNames[index]));
      q(model.joints[id].idx_q()) = joints.position[index];
    }
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    double height = RobotModelConfig{}.foot_radius_m;
    for (const auto foot : kFootFrameNames) {
      height -= data.oMf[model.getFrameId(std::string(foot))].translation().z() / kLegCount;
    }
    return height;
  }

  void CheckMinuteOfStaticSupport(bool imu_at_100_hz) {
    KinematicStateEstimator estimator(robot_.pin, robot_.info);
    const auto joints = Pose();
    auto imu = RestingImu();
    const double expected_height = FlatGroundHeight(joints);
    ASSERT_GT(expected_height, 0.20);
    ASSERT_LT(expected_height, 0.40);
    estimator.Reset(0.29);
    int repeated_samples = 0;
    double previous_stamp = -1.0;
    for (int cycle = 0; cycle < kMinuteOfControlCycles; ++cycle) {
      // Integer milliseconds reproduce 100 Hz acquisition read by a 250 Hz
      // estimator: alternating two/three reads of the same IMU sample.
      imu.stamp_seconds = imu_at_100_hz ? ((cycle * 4) / 10) * 0.010 : cycle * kPeriod;
      if (imu.stamp_seconds == previous_stamp) ++repeated_samples;
      previous_stamp = imu.stamp_seconds;
      const auto state = estimator.Update(joints, imu, ContactFlags(kStanceMode), kPeriod);
      ASSERT_TRUE(FiniteEstimate(state)) << "cycle=" << cycle;
      ASSERT_NEAR(state.position.z(), expected_height, 0.005) << "cycle=" << cycle;
      ASSERT_LT(state.velocity_world.norm(), 0.01) << "cycle=" << cycle;
      ASSERT_LT(state.position.head<2>().norm(), 0.005) << "cycle=" << cycle;
      ASSERT_LT(state.euler_zyx.norm(), 1e-12);
      ASSERT_DOUBLE_EQ(state.stamp_seconds, imu.stamp_seconds);
    }
    if (imu_at_100_hz) EXPECT_GT(repeated_samples, kMinuteOfControlCycles / 2);
  }
};

TEST_F(KinematicStateEstimatorTest, StaticStandingRemainsFiniteForOneMinute) {
  CheckMinuteOfStaticSupport(false);
}

TEST_F(KinematicStateEstimatorTest, Repeated100HzImuAt250HzRemainsFiniteForOneMinute) {
  CheckMinuteOfStaticSupport(true);
}

TEST_F(KinematicStateEstimatorTest, ResetReinitializesHeightAndMatchesFreshEstimator) {
  KinematicStateEstimator reused(robot_.pin, robot_.info);
  reused.Reset(0.29);
  auto imu = RestingImu();
  const auto standing = Pose();
  for (int cycle = 0; cycle < 500; ++cycle) {
    ASSERT_TRUE(FiniteEstimate(reused.Update(standing, imu, ContactFlags(kStanceMode), kPeriod)));
  }
  const auto crouched = Pose(1.253146, -2.823943);
  ASSERT_GT(std::abs(FlatGroundHeight(standing) - FlatGroundHeight(crouched)), 0.05);
  for (const auto& pose : {crouched, standing}) {
    KinematicStateEstimator fresh(robot_.pin, robot_.info);
    reused.Reset(0.29);
    fresh.Reset(0.29);
    const double expected_height = FlatGroundHeight(pose);
    for (int cycle = 0; cycle < 500; ++cycle) {
      imu.stamp_seconds = cycle * kPeriod;
      const auto actual = reused.Update(pose, imu, ContactFlags(kStanceMode), kPeriod);
      const auto expected = fresh.Update(pose, imu, ContactFlags(kStanceMode), kPeriod);
      ASSERT_TRUE(FiniteEstimate(actual));
      ASSERT_TRUE(FiniteEstimate(expected));
      ASSERT_NEAR(actual.position.z(), expected_height, 0.005);
      ASSERT_LT((actual.position - expected.position).norm(), 1e-12);
      ASSERT_LT((actual.velocity_world - expected.velocity_world).norm(), 1e-12);
    }
  }
}

TEST_F(KinematicStateEstimatorTest, InvalidInputsDoNotPoisonFollowingValidEstimates) {
  KinematicStateEstimator estimator(robot_.pin, robot_.info), reference(robot_.pin, robot_.info);
  estimator.Reset(0.29);
  reference.Reset(0.29);
  const auto joints = Pose();
  const auto imu = RestingImu();
  for (int cycle = 0; cycle < 250; ++cycle) {
    ASSERT_TRUE(FiniteEstimate(estimator.Update(joints, imu, ContactFlags(kStanceMode), kPeriod)));
    ASSERT_TRUE(FiniteEstimate(reference.Update(joints, imu, ContactFlags(kStanceMode), kPeriod)));
  }
  struct InvalidInput { std::string name; JointSample joints; ImuSample imu; double dt; };
  std::vector<InvalidInput> cases;
  auto bad_imu = imu;
  bad_imu.valid = false;
  cases.push_back({"missing IMU", joints, bad_imu, kPeriod});
  bad_imu = imu;
  bad_imu.orientation_wxyz.fill(0.0);
  cases.push_back({"zero quaternion", joints, bad_imu, kPeriod});
  bad_imu = imu;
  bad_imu.linear_acceleration[0] = std::numeric_limits<double>::quiet_NaN();
  cases.push_back({"non-finite acceleration", joints, bad_imu, kPeriod});
  auto bad_joints = joints;
  bad_joints.valid[0] = 0.0;
  cases.push_back({"missing joint", bad_joints, imu, kPeriod});
  bad_joints = joints;
  bad_joints.position[4] = std::numeric_limits<double>::infinity();
  cases.push_back({"non-finite position", bad_joints, imu, kPeriod});
  bad_joints = joints;
  bad_joints.velocity[9] = std::numeric_limits<double>::quiet_NaN();
  cases.push_back({"non-finite velocity", bad_joints, imu, kPeriod});
  cases.push_back({"zero dt", joints, imu, 0.0});
  cases.push_back({"non-finite dt", joints, imu, std::numeric_limits<double>::quiet_NaN()});
  for (const auto& input : cases) {
    SCOPED_TRACE(input.name);
    EXPECT_FALSE(estimator.Update(input.joints, input.imu, ContactFlags(kStanceMode), input.dt).valid);
    const auto recovered = estimator.Update(joints, imu, ContactFlags(kStanceMode), kPeriod);
    const auto expected = reference.Update(joints, imu, ContactFlags(kStanceMode), kPeriod);
    ASSERT_TRUE(FiniteEstimate(recovered));
    ASSERT_LT((recovered.position - expected.position).norm(), 1e-12);
    ASSERT_LT((recovered.velocity_world - expected.velocity_world).norm(), 1e-12);
  }
}

}  // namespace
}  // namespace custom_dog_control
