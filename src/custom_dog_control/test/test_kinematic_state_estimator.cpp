#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/LU>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "custom_dog_control/model/RobotModel.hpp"
#include "custom_dog_control/nmpc/KinematicStateEstimator.hpp"

namespace custom_dog_control {
namespace {

constexpr double kPeriod = 0.004;

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

  // Generate a sustained rolling stance from the full robot contact Jacobian,
  // not from synthetic center velocities supplied to the estimator. World base
  // velocity is known, base attitude is fixed, and contact-point velocity is zero.
  ocs2::vector_t RollingStanceVelocity(const ocs2::vector_t& q,
                                     const Eigen::Vector3d& base_velocity,
                                     double radius) {
    const auto& model = robot_.pin.getModel();
    auto& data = robot_.pin.getData();
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    ocs2::vector_t v = ocs2::vector_t::Zero(model.nv);
    v.head<3>() = base_velocity;
    Eigen::Matrix3d radius_cross;
    radius_cross << 0.0, radius, 0.0, -radius, 0.0, 0.0, 0.0, 0.0, 0.0;
    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
      Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, model.nv);
      jacobian.setZero();
      pinocchio::getFrameJacobian(model, data,
          model.getFrameId(std::string(kFootFrameNames[leg])),
          pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
      const Eigen::Matrix<double, 3, Eigen::Dynamic> contact_jacobian =
          jacobian.topRows<3>() - radius_cross * jacobian.bottomRows<3>();
      Eigen::Matrix3d leg_jacobian;
      std::array<int, 3> slots;
      for (std::size_t joint = 0; joint < 3; ++joint) {
        const auto id = model.getJointId(std::string(kJointNames[3 * leg + joint]));
        slots[joint] = model.joints[id].idx_v();
        leg_jacobian.col(joint) = contact_jacobian.col(slots[joint]);
      }
      const Eigen::Vector3d leg_velocity = leg_jacobian.fullPivLu().solve(
          -contact_jacobian.leftCols<6>() * v.head<6>());
      for (std::size_t joint = 0; joint < 3; ++joint) v(slots[joint]) = leg_velocity(joint);
    }
    return v;
  }

  JointSample JointFeedback(const ocs2::vector_t& q, const ocs2::vector_t& v) {
    JointSample sample;
    sample.valid.fill(1.0);
    const auto& model = robot_.pin.getModel();
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      const auto id = model.getJointId(std::string(kJointNames[joint]));
      sample.position[joint] = q(model.joints[id].idx_q());
      sample.velocity[joint] = v(model.joints[id].idx_v());
    }
    return sample;
  }

  void CheckMinuteOfStaticSupport(bool imu_at_100_hz, int control_hz = 250) {
    KinematicStateEstimator estimator(robot_.pin, robot_.info);
    const auto joints = Pose();
    auto imu = RestingImu();
    const double expected_height = FlatGroundHeight(joints);
    ASSERT_GT(expected_height, 0.20);
    ASSERT_LT(expected_height, 0.40);
    estimator.Reset(0.29);
    int repeated_samples = 0;
    double previous_stamp = -1.0;
    const double period = 1.0 / control_hz;
    const int minute_of_control_cycles = 60 * control_hz;
    for (int cycle = 0; cycle < minute_of_control_cycles; ++cycle) {
      // Use integer sample indices: 250 Hz sees two/three reads per IMU
      // sample; the configured 500 Hz loop sees five reads per 100 Hz sample.
      imu.stamp_seconds = imu_at_100_hz ? ((cycle * 100) / control_hz) * 0.010 : cycle * period;
      if (imu.stamp_seconds == previous_stamp) ++repeated_samples;
      previous_stamp = imu.stamp_seconds;
      const auto state = estimator.Update(joints, imu, ContactFlags(kStanceMode), period);
      ASSERT_TRUE(FiniteEstimate(state)) << "cycle=" << cycle;
      ASSERT_NEAR(state.position.z(), expected_height, 0.005) << "cycle=" << cycle;
      ASSERT_LT(state.velocity_world.norm(), 0.01) << "cycle=" << cycle;
      ASSERT_LT(state.position.head<2>().norm(), 0.005) << "cycle=" << cycle;
      ASSERT_LT(state.euler_zyx.norm(), 1e-12);
      ASSERT_DOUBLE_EQ(state.stamp_seconds, imu.stamp_seconds);
    }
    if (imu_at_100_hz) EXPECT_GT(repeated_samples, minute_of_control_cycles / 2);
  }
};

TEST_F(KinematicStateEstimatorTest, StaticStandingRemainsFiniteForOneMinute) {
  CheckMinuteOfStaticSupport(false);
}

TEST_F(KinematicStateEstimatorTest, Repeated100HzImuAt250HzRemainsFiniteForOneMinute) {
  CheckMinuteOfStaticSupport(true);
}

TEST_F(KinematicStateEstimatorTest, Repeated100HzImuAt500HzRemainsFiniteForOneMinute) {
  CheckMinuteOfStaticSupport(true, 500);
}

TEST_F(KinematicStateEstimatorTest, Held100HzImuAt500HzPreservesRotationAndIntegratesElapsedTime) {
  // Isolate inertial prediction in flight by making the artificial foot
  // observations negligible. A tilted, yawing body sees constant world
  // acceleration; this checks gravity/frame conversion and the total elapsed
  // integration time, not only numerical finiteness of a static estimate.
  EstimatorNoise noise;
  noise.foot_sensor_position = 1e12;
  noise.foot_sensor_velocity = 1e12;
  noise.foot_height = 1e12;
  KinematicStateEstimator estimator(robot_.pin, robot_.info, noise);
  estimator.Reset(0.34);
  const auto joints = Pose();
  const std::array<bool, kLegCount> in_flight{};
  constexpr double period = 0.002;
  constexpr double yaw_rate = 0.4;
  const Eigen::Vector3d world_acceleration(0.5, -0.25, 0.0);
  auto imu = RestingImu();
  EstimatedState state;
  for (int cycle = 0; cycle < 1000; ++cycle) {
    const double sample_time = (cycle / 5) * 0.010;
    if (cycle % 5 == 0) {
      const Eigen::Quaterniond orientation =
          Eigen::AngleAxisd(yaw_rate * sample_time, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX());
      const Eigen::Vector3d body_omega = orientation.conjugate() *
          Eigen::Vector3d(0.0, 0.0, yaw_rate);
      const Eigen::Vector3d body_specific_force = orientation.conjugate() *
          (world_acceleration + Eigen::Vector3d(0.0, 0.0, 9.81));
      imu.orientation_wxyz = {orientation.w(), orientation.x(), orientation.y(), orientation.z()};
      imu.angular_velocity = {body_omega.x(), body_omega.y(), body_omega.z()};
      imu.linear_acceleration = {body_specific_force.x(), body_specific_force.y(), body_specific_force.z()};
      imu.stamp_seconds = 1.0 + sample_time;
    }
    state = estimator.Update(joints, imu, in_flight, period);
    ASSERT_TRUE(FiniteEstimate(state)) << "cycle=" << cycle;
    EXPECT_NEAR(state.euler_zyx.x(), yaw_rate * sample_time, 1e-12);
    EXPECT_NEAR(state.euler_zyx.y(), 0.3, 1e-12);
    EXPECT_NEAR(state.euler_zyx.z(), -0.2, 1e-12);
    EXPECT_LT((state.angular_velocity_world - Eigen::Vector3d(0.0, 0.0, yaw_rate)).norm(), 1e-12);
    EXPECT_DOUBLE_EQ(state.stamp_seconds, 1.0 + sample_time);
  }
  constexpr double elapsed = 1000 * period;
  EXPECT_LT((state.velocity_world - elapsed * world_acceleration).norm(), 1e-5);
  EXPECT_LT((state.position - (Eigen::Vector3d(0.0, 0.0, 0.34) +
                              0.5 * elapsed * elapsed * world_acceleration)).norm(), 1e-5);
}

TEST_F(KinematicStateEstimatorTest, OrientationPredictionUsesBodyGyroAndAgeWithoutRefreshingStamp) {
  KinematicStateEstimator estimator(robot_.pin, robot_.info);
  estimator.Reset(0.34);
  const auto joints = Pose();
  auto imu = RestingImu();
  const Eigen::Quaterniond sample_orientation =
      Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d body_omega(0.7, -0.4, 1.1);
  imu.orientation_wxyz = {sample_orientation.w(), sample_orientation.x(),
                          sample_orientation.y(), sample_orientation.z()};
  imu.angular_velocity = {body_omega.x(), body_omega.y(), body_omega.z()};
  imu.stamp_seconds = 10.0;
  const auto original_quaternion = imu.orientation_wxyz;
  for (const double sample_age : {0.0, 0.007, 0.011, 0.020}) {
    // Several reads of the same sample use their own age, not dt=2 ms or
    // the preceding extrapolated orientation. Noncommuting rotations expose
    // accidental world-axis (left-multiply) propagation.
    const auto state = estimator.Update(joints, imu, ContactFlags(kStanceMode), 0.002, sample_age);
    ASSERT_TRUE(FiniteEstimate(state));
    const Eigen::Quaterniond expected = sample_orientation * Eigen::Quaterniond(
        Eigen::AngleAxisd(body_omega.norm() * sample_age, body_omega.normalized()));
    const Eigen::Quaterniond actual =
        Eigen::AngleAxisd(state.euler_zyx.x(), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(state.euler_zyx.y(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(state.euler_zyx.z(), Eigen::Vector3d::UnitX());
    EXPECT_LT(actual.angularDistance(expected), 1e-12);
    EXPECT_LT((state.angular_velocity_world - expected * body_omega).norm(), 1e-12);
    EXPECT_DOUBLE_EQ(state.stamp_seconds, 10.0);
    EXPECT_EQ(imu.orientation_wxyz, original_quaternion);
    EXPECT_DOUBLE_EQ(imu.stamp_seconds, 10.0);
  }
}

TEST_F(KinematicStateEstimatorTest, OrientationPredictionDoesNotRotateOldSpecificForceWithNewPose) {
  EstimatorNoise noise;
  noise.foot_sensor_position = 1e12;
  noise.foot_sensor_velocity = 1e12;
  noise.foot_height = 1e12;
  KinematicStateEstimator estimator(robot_.pin, robot_.info, noise);
  estimator.Reset(0.34);
  auto imu = RestingImu();
  const Eigen::Quaterniond sample_orientation =
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d specific_force =
      sample_orientation.conjugate() * Eigen::Vector3d(0.0, 0.0, 9.81);
  imu.orientation_wxyz = {sample_orientation.w(), sample_orientation.x(),
                          sample_orientation.y(), sample_orientation.z()};
  imu.angular_velocity = {0.5, 1.1, -0.4};
  imu.linear_acceleration = {specific_force.x(), specific_force.y(), specific_force.z()};
  imu.stamp_seconds = 1.0;
  const std::array<bool, kLegCount> in_flight{};
  const auto state = estimator.Update(Pose(), imu, in_flight, 0.002, 0.020);
  ASSERT_TRUE(FiniteEstimate(state));
  EXPECT_LT(state.velocity_world.norm(), 1e-8);
  EXPECT_LT((state.position - Eigen::Vector3d(0.0, 0.0, 0.34)).norm(), 1e-8);
  EXPECT_DOUBLE_EQ(state.stamp_seconds, 1.0);
}

TEST_F(KinematicStateEstimatorTest, ZeroGyroPredictionPreservesPoseAndInvalidAgeDoesNotAdvanceFilter) {
  KinematicStateEstimator estimator(robot_.pin, robot_.info), reference(robot_.pin, robot_.info);
  estimator.Reset(0.34);
  reference.Reset(0.34);
  const auto joints = Pose();
  const auto imu = RestingImu();
  auto expected = reference.Update(joints, imu, ContactFlags(kStanceMode), 0.002);
  auto actual = estimator.Update(joints, imu, ContactFlags(kStanceMode), 0.002, 0.020);
  ASSERT_TRUE(FiniteEstimate(actual));
  EXPECT_LT((actual.euler_zyx - expected.euler_zyx).norm(), 1e-12);
  EXPECT_LT((actual.position - expected.position).norm(), 1e-12);
  for (const double invalid_age : {-0.001, 0.020001,
       std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    EXPECT_FALSE(estimator.Update(joints, imu, ContactFlags(kStanceMode), 0.002, invalid_age).valid);
    actual = estimator.Update(joints, imu, ContactFlags(kStanceMode), 0.002);
    expected = reference.Update(joints, imu, ContactFlags(kStanceMode), 0.002);
    ASSERT_TRUE(FiniteEstimate(actual));
    EXPECT_LT((actual.position - expected.position).norm(), 1e-12);
    EXPECT_LT((actual.velocity_world - expected.velocity_world).norm(), 1e-12);
  }
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

TEST_F(KinematicStateEstimatorTest, DefaultPlannedContactsExactlyMatchExplicitBinarySupport) {
  // The explicit support path retains the pre-schedule binary covariance
  // selection. Exercise liftoff, diagonal exchange and flight, not just rest.
  KinematicStateEstimator planned(robot_.pin, robot_.info),
      binary(robot_.pin, robot_.info);
  planned.Reset(0.34);
  binary.Reset(0.34);
  auto imu = RestingImu();
  for (int cycle = 0; cycle < 200; ++cycle) {
    const auto mode = cycle < 20 ? kStanceMode :
        (cycle < 80 ? std::size_t{6} : (cycle < 140 ? std::size_t{9} : std::size_t{0}));
    const auto contacts = ContactFlags(mode);
    auto joints = Pose(0.8 + 0.02 * std::sin(cycle * 0.02), -1.6);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (i % 3 == 1) joints.velocity[i] = 0.2 * std::cos(cycle * 0.02);
    }
    imu.stamp_seconds = cycle * 0.002;
    ContactSupport support;
    support.contact = contacts;
    support.height_valid = contacts;
    support.foot_center_height.fill(RobotModelConfig{}.foot_radius_m);
    const auto actual = planned.Update(joints, imu, contacts, 0.002);
    const auto expected = binary.Update(joints, imu, support, 0.002);
    ASSERT_TRUE(FiniteEstimate(actual));
    ASSERT_TRUE(FiniteEstimate(expected));
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
      EXPECT_EQ(actual.position(axis), expected.position(axis));
      EXPECT_EQ(actual.velocity_world(axis), expected.velocity_world(axis));
      for (std::size_t leg = 0; leg < kLegCount; ++leg) {
        EXPECT_EQ(actual.foot_position_world[leg](axis),
                  expected.foot_position_world[leg](axis));
      }
    }
    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
      EXPECT_DOUBLE_EQ(planned.plannedContactTrust()[leg], contacts[leg] ? 1.0 : 0.0);
    }
  }
}

TEST_F(KinematicStateEstimatorTest, PlannedTrustStartsStandingFullyTrustedAndResetClearsHistory) {
  EstimatorNoise noise;
  noise.planned_contact_delay_s = 0.020;
  noise.planned_contact_ramp_s = 0.010;
  KinematicStateEstimator estimator(robot_.pin, robot_.info, noise),
      binary(robot_.pin, robot_.info);
  const auto joints = Pose();
  const auto imu = RestingImu();
  estimator.Reset(0.34);
  binary.Reset(0.34);
  const auto first = estimator.Update(joints, imu, ContactFlags(kStanceMode), 0.002);
  const auto expected = binary.Update(joints, imu, ContactFlags(kStanceMode), 0.002);
  ASSERT_TRUE(FiniteEstimate(first));
  EXPECT_DOUBLE_EQ(first.position.z(), expected.position.z());
  for (double trust : estimator.plannedContactTrust()) EXPECT_DOUBLE_EQ(trust, 1.0);

  ASSERT_TRUE(FiniteEstimate(estimator.Update(joints, imu, ContactFlags(0), 0.002)));
  ASSERT_TRUE(FiniteEstimate(estimator.Update(joints, imu, ContactFlags(kStanceMode), 0.002)));
  for (double trust : estimator.plannedContactTrust()) EXPECT_DOUBLE_EQ(trust, 0.0);
  estimator.Reset(0.34);
  for (double trust : estimator.plannedContactTrust()) EXPECT_DOUBLE_EQ(trust, 0.0);
  ASSERT_TRUE(FiniteEstimate(estimator.Update(joints, imu, ContactFlags(kStanceMode), 0.002)));
  for (double trust : estimator.plannedContactTrust()) EXPECT_DOUBLE_EQ(trust, 1.0);
}

TEST_F(KinematicStateEstimatorTest, PlannedTouchdownWaitsThenRampsAndLiftoffRestartsSchedule) {
  EstimatorNoise noise;
  noise.planned_contact_delay_s = 0.020;
  noise.planned_contact_ramp_s = 0.010;
  KinematicStateEstimator estimator(robot_.pin, robot_.info, noise);
  estimator.Reset(0.34);
  const auto joints = Pose(1.25, -2.82);
  const auto imu = RestingImu();
  const auto airborne = estimator.Update(joints, imu, ContactFlags(0), 0.005);
  ASSERT_TRUE(FiniteEstimate(airborne));
  // Swing geometry must not initialize base height as a ground observation.
  EXPECT_NEAR(airborne.position.z(), 0.34, 1e-12);
  for (int cycle = 0; cycle <= 6; ++cycle) {
    ASSERT_TRUE(FiniteEstimate(estimator.Update(joints, imu, ContactFlags(9), 0.005)));
    const double expected = cycle <= 4 ? 0.0 : (cycle == 5 ? 0.5 : 1.0);
    const auto contacts = ContactFlags(9);
    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
      EXPECT_NEAR(estimator.plannedContactTrust()[leg], contacts[leg] ? expected : 0.0, 1e-12);
    }
  }
  ASSERT_TRUE(FiniteEstimate(estimator.Update(joints, imu, ContactFlags(0), 0.005)));
  for (double trust : estimator.plannedContactTrust()) EXPECT_DOUBLE_EQ(trust, 0.0);
  ASSERT_TRUE(FiniteEstimate(estimator.Update(joints, imu, ContactFlags(9), 0.050)));
  // Even a long transition interval is not counted as time already in stance.
  for (double trust : estimator.plannedContactTrust()) EXPECT_DOUBLE_EQ(trust, 0.0);
}

TEST_F(KinematicStateEstimatorTest, PlannedTrustSupportsDelayOnlyAndRampOnly) {
  for (bool delay_only : {true, false}) {
    EstimatorNoise noise;
    noise.planned_contact_delay_s = delay_only ? 0.010 : 0.0;
    noise.planned_contact_ramp_s = delay_only ? 0.0 : 0.010;
    KinematicStateEstimator estimator(robot_.pin, robot_.info, noise);
    estimator.Reset(0.34);
    ASSERT_TRUE(FiniteEstimate(estimator.Update(Pose(), RestingImu(), ContactFlags(0), 0.005)));
    for (int cycle = 0; cycle < 3; ++cycle) {
      ASSERT_TRUE(FiniteEstimate(estimator.Update(Pose(), RestingImu(), ContactFlags(kStanceMode), 0.005)));
      const double expected = cycle == 2 ? 1.0 : (cycle == 1 && !delay_only ? 0.5 : 0.0);
      for (double trust : estimator.plannedContactTrust()) EXPECT_NEAR(trust, expected, 1e-12);
    }
  }
}

TEST_F(KinematicStateEstimatorTest, PlannedDelayWeakensSpuriousMovingFootVelocityAtTouchdown) {
  EstimatorNoise noise;
  noise.planned_contact_delay_s = 0.020;
  noise.planned_contact_ramp_s = 0.010;
  KinematicStateEstimator delayed(robot_.pin, robot_.info, noise),
      binary(robot_.pin, robot_.info);
  delayed.Reset(0.34);
  binary.Reset(0.34);
  auto joints = Pose();
  const auto imu = RestingImu();
  for (int cycle = 0; cycle < 100; ++cycle) {
    ASSERT_TRUE(FiniteEstimate(delayed.Update(joints, imu, ContactFlags(kStanceMode), 0.002)));
    ASSERT_TRUE(FiniteEstimate(binary.Update(joints, imu, ContactFlags(kStanceMode), 0.002)));
  }
  ASSERT_TRUE(FiniteEstimate(delayed.Update(joints, imu, ContactFlags(0), 0.002)));
  ASSERT_TRUE(FiniteEstimate(binary.Update(joints, imu, ContactFlags(0), 0.002)));
  // The planned touchdown arrives while the foot still has swing velocity.
  // Stationary specific force and unchanged joint positions isolate this
  // deliberately false stationary-foot velocity measurement.
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (i % 3 == 2) joints.velocity[i] = 2.0;
  }
  const auto restrained = delayed.Update(joints, imu, ContactFlags(kStanceMode), 0.002);
  const auto trusted = binary.Update(joints, imu, ContactFlags(kStanceMode), 0.002);
  ASSERT_TRUE(FiniteEstimate(restrained));
  ASSERT_TRUE(FiniteEstimate(trusted));
  ASSERT_GT(trusted.velocity_world.norm(), 1e-5);
  EXPECT_LT(restrained.velocity_world.norm(), trusted.velocity_world.norm() * 0.25);
  joints.velocity.fill(0.0);
  EstimatedState recovered;
  for (int cycle = 0; cycle < 500; ++cycle) {
    recovered = delayed.Update(joints, imu, ContactFlags(kStanceMode), 0.002);
    ASSERT_TRUE(FiniteEstimate(recovered));
  }
  EXPECT_LT(recovered.velocity_world.norm(), 0.01);
  EXPECT_NEAR(recovered.position.z(), FlatGroundHeight(joints), 0.005);
  for (double trust : delayed.plannedContactTrust()) EXPECT_DOUBLE_EQ(trust, 1.0);
}

TEST_F(KinematicStateEstimatorTest, InvalidInputsDoNotAdvancePlannedTrustOrFilter) {
  EstimatorNoise noise;
  noise.planned_contact_delay_s = 0.020;
  noise.planned_contact_ramp_s = 0.010;
  KinematicStateEstimator estimator(robot_.pin, robot_.info, noise),
      reference(robot_.pin, robot_.info, noise);
  estimator.Reset(0.34);
  reference.Reset(0.34);
  const auto joints = Pose();
  const auto imu = RestingImu();
  for (auto* item : {&estimator, &reference}) {
    ASSERT_TRUE(FiniteEstimate(item->Update(joints, imu, ContactFlags(0), 0.002)));
    ASSERT_TRUE(FiniteEstimate(item->Update(joints, imu, ContactFlags(kStanceMode), 0.002)));
    ASSERT_TRUE(FiniteEstimate(item->Update(joints, imu, ContactFlags(kStanceMode), 0.020)));
  }
  const std::vector<double> invalid_steps{
      0.0, -0.002, 0.100001, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()};
  for (double dt : invalid_steps) {
    const auto before = estimator.plannedContactTrust();
    EXPECT_FALSE(estimator.Update(joints, imu, ContactFlags(0), dt).valid);
    EXPECT_EQ(estimator.plannedContactTrust(), before);
  }
  auto bad_imu = imu;
  bad_imu.valid = false;
  EXPECT_FALSE(estimator.Update(joints, bad_imu, ContactFlags(0), 0.010).valid);
  EXPECT_FALSE(estimator.Update(joints, imu, ContactFlags(0), 0.010, 0.021).valid);
  const auto recovered = estimator.Update(joints, imu, ContactFlags(kStanceMode), 0.005);
  const auto expected = reference.Update(joints, imu, ContactFlags(kStanceMode), 0.005);
  ASSERT_TRUE(FiniteEstimate(recovered));
  EXPECT_EQ(estimator.plannedContactTrust(), reference.plannedContactTrust());
  for (double trust : estimator.plannedContactTrust()) EXPECT_NEAR(trust, 0.5, 1e-12);
  EXPECT_LT((recovered.position - expected.position).norm(), 1e-12);
  EXPECT_LT((recovered.velocity_world - expected.velocity_world).norm(), 1e-12);
}

TEST_F(KinematicStateEstimatorTest, ExplicitContactSupportDoesNotUsePlannedDelay) {
  EstimatorNoise noise;
  noise.planned_contact_delay_s = 0.020;
  noise.planned_contact_ramp_s = 0.010;
  KinematicStateEstimator scheduled(robot_.pin, robot_.info, noise),
      binary(robot_.pin, robot_.info);
  scheduled.Reset(0.34);
  binary.Reset(0.34);
  for (std::size_t mode : {std::size_t{0}, kStanceMode, std::size_t{6}, std::size_t{9}}) {
    ContactSupport support;
    support.contact = ContactFlags(mode);
    support.height_valid = support.contact;
    support.foot_center_height.fill(RobotModelConfig{}.foot_radius_m);
    const auto actual = scheduled.Update(Pose(), RestingImu(), support, 0.002);
    const auto expected = binary.Update(Pose(), RestingImu(), support, 0.002);
    ASSERT_TRUE(FiniteEstimate(actual));
    EXPECT_LT((actual.position - expected.position).norm(), 1e-12);
    EXPECT_LT((actual.velocity_world - expected.velocity_world).norm(), 1e-12);
    for (double trust : scheduled.plannedContactTrust()) EXPECT_DOUBLE_EQ(trust, 0.0);
  }
}

TEST_F(KinematicStateEstimatorTest, InvalidPlannedTrustTimingIsRejected) {
  for (double invalid : {-0.001, std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::infinity()}) {
    EstimatorNoise noise;
    noise.planned_contact_delay_s = invalid;
    EXPECT_THROW(KinematicStateEstimator(robot_.pin, robot_.info, noise), std::invalid_argument);
    noise.planned_contact_delay_s = 0.0;
    noise.planned_contact_ramp_s = invalid;
    EXPECT_THROW(KinematicStateEstimator(robot_.pin, robot_.info, noise), std::invalid_argument);
  }
}

TEST_F(KinematicStateEstimatorTest, RollingSphericalStanceRecoversVelocityAndDisplacement) {
  constexpr double radius = 0.026;
  constexpr double dt = 0.001;
  constexpr int steps = 500;
  for (double yaw : {0.0, 0.6}) {
    for (double vx : {0.2, -0.2}) {
      SCOPED_TRACE(testing::Message() << "yaw=" << yaw << " vx=" << vx);
      EstimatorNoise rolling_noise;
      rolling_noise.rolling_foot_radius_m = radius;
      KinematicStateEstimator rolling(robot_.pin, robot_.info, rolling_noise),
          legacy(robot_.pin, robot_.info);
      rolling.Reset(0.34);
      legacy.Reset(0.34);
      const auto& model = robot_.pin.getModel();
      auto& data = robot_.pin.getData();
      ocs2::vector_t q = ocs2::vector_t::Zero(model.nq);
      q(3) = yaw;
      const auto initial = Pose();
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        const auto id = model.getJointId(std::string(kJointNames[joint]));
        q(model.joints[id].idx_q()) = initial.position[joint];
      }
      const Eigen::Vector3d truth_velocity(vx, 0.0, 0.0);
      const double truth_height = FlatGroundHeight(initial);
      auto imu = RestingImu();
      imu.orientation_wxyz = {std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0)};
      EstimatedState corrected, old;
      Eigen::Vector3d corrected_tail = Eigen::Vector3d::Zero();
      Eigen::Vector3d legacy_tail = Eigen::Vector3d::Zero();
      for (int cycle = 0; cycle <= steps; ++cycle) {
        const auto v = RollingStanceVelocity(q, truth_velocity, radius);
        ASSERT_TRUE(v.allFinite());
        // Check the independent no-slip contact constraint on every generated
        // trajectory sample, so arbitrary joint motion cannot satisfy this test.
        pinocchio::forwardKinematics(model, data, q, v);
        pinocchio::updateFramePlacements(model, data);
        for (const auto foot : kFootFrameNames) {
          const auto motion = pinocchio::getFrameVelocity(model, data,
              model.getFrameId(std::string(foot)), pinocchio::LOCAL_WORLD_ALIGNED);
          EXPECT_LT((motion.linear() + motion.angular().cross(
              Eigen::Vector3d(0.0, 0.0, -radius))).norm(), 1e-9);
        }
        const auto feedback = JointFeedback(q, v);
        imu.stamp_seconds = cycle * dt;
        corrected = rolling.Update(feedback, imu, ContactFlags(kStanceMode), dt);
        old = legacy.Update(feedback, imu, ContactFlags(kStanceMode), dt);
        ASSERT_TRUE(FiniteEstimate(corrected));
        ASSERT_TRUE(FiniteEstimate(old));
        if (cycle > steps - 200) {
          corrected_tail += corrected.velocity_world / 200.0;
          legacy_tail += old.velocity_world / 200.0;
        }
        // Midpoint integration changes joints only. Base translation is kept
        // out of estimator FK, while truth x advances independently at vx.
        ocs2::vector_t midpoint = q;
        midpoint.tail<12>() += 0.5 * dt * v.tail<12>();
        const auto midpoint_v = RollingStanceVelocity(midpoint, truth_velocity, radius);
        q.tail<12>() += dt * midpoint_v.tail<12>();
      }
      EXPECT_LT((corrected_tail - truth_velocity).norm(), 5e-4);
      EXPECT_GT((legacy_tail - truth_velocity).norm(), 0.005);
      const Eigen::Vector3d truth_position(vx * steps * dt, 0.0, truth_height);
      EXPECT_LT((corrected.position - truth_position).norm(), 0.001);
      EXPECT_GT((old.position - truth_position).norm(), 0.003);
      // The position bound also detects a partial fix that changes velocity
      // measurements but leaves the KF foot-center prediction stationary.
    }
  }
}

TEST_F(KinematicStateEstimatorTest, ZeroRollingRadiusPreservesPointFootBehaviorExactly) {
  EstimatorNoise point_noise;
  point_noise.rolling_foot_radius_m = 0.0;
  KinematicStateEstimator planned(robot_.pin, robot_.info, point_noise),
      explicit_point(robot_.pin, robot_.info);
  planned.Reset(0.34);
  explicit_point.Reset(0.34);
  auto imu = RestingImu();
  imu.angular_velocity = {0.03, -0.02, 0.01};
  for (int cycle = 0; cycle < 200; ++cycle) {
    auto joints = Pose(0.8 + 0.02 * std::sin(cycle * 0.01), -1.6);
    for (std::size_t joint = 0; joint < kJointCount; ++joint)
      joints.velocity[joint] = 0.04 * std::cos(cycle * 0.01 + joint);
    const auto contacts = ContactFlags(cycle < 50 ? kStanceMode :
        (cycle < 100 ? std::size_t{6} : (cycle < 150 ? std::size_t{9} : std::size_t{0})));
    ContactSupport support;
    support.contact = contacts;
    support.height_valid = contacts;
    support.foot_center_height.fill(RobotModelConfig{}.foot_radius_m);
    imu.stamp_seconds = cycle * 0.002;
    const auto actual = planned.Update(joints, imu, contacts, 0.002);
    const auto expected = explicit_point.Update(joints, imu, support, 0.002);
    ASSERT_TRUE(FiniteEstimate(actual));
    ASSERT_TRUE(FiniteEstimate(expected));
    EXPECT_TRUE(actual.position == expected.position);
    EXPECT_TRUE(actual.velocity_world == expected.velocity_world);
    for (std::size_t leg = 0; leg < kLegCount; ++leg)
      EXPECT_TRUE(actual.foot_position_world[leg] == expected.foot_position_world[leg]);
  }
}

TEST_F(KinematicStateEstimatorTest, ExplicitSupportRetainsCallerVelocityWithRollingConfigured) {
  EstimatorNoise noise;
  noise.rolling_foot_radius_m = 0.026;
  noise.planned_contact_delay_s = 0.020;
  noise.planned_contact_ramp_s = 0.010;
  KinematicStateEstimator configured(robot_.pin, robot_.info, noise),
      reference(robot_.pin, robot_.info);
  configured.Reset(0.34);
  reference.Reset(0.34);
  auto imu = RestingImu();
  imu.angular_velocity = {0.1, -0.2, 0.3};
  for (int cycle = 0; cycle < 200; ++cycle) {
    auto joints = Pose();
    joints.velocity.fill(0.2);
    ContactSupport support;
    support.contact = ContactFlags(cycle < 50 ? kStanceMode :
        (cycle < 100 ? std::size_t{6} : (cycle < 150 ? std::size_t{9} : std::size_t{0})));
    support.height_valid = support.contact;
    support.foot_center_height.fill(RobotModelConfig{}.foot_radius_m);
    for (std::size_t leg = 0; leg < kLegCount; ++leg)
      support.center_velocity[leg] = Eigen::Vector3d(0.04 + leg * 0.001, -0.03, 0.0);
    imu.stamp_seconds = cycle * 0.002;
    const auto actual = configured.Update(joints, imu, support, 0.002);
    const auto expected = reference.Update(joints, imu, support, 0.002);
    ASSERT_TRUE(FiniteEstimate(actual));
    ASSERT_TRUE(FiniteEstimate(expected));
    EXPECT_TRUE(actual.position == expected.position);
    EXPECT_TRUE(actual.velocity_world == expected.velocity_world);
    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
      EXPECT_TRUE(actual.foot_position_world[leg] == expected.foot_position_world[leg]);
      EXPECT_DOUBLE_EQ(configured.plannedContactTrust()[leg], 0.0);
    }
  }
}

TEST_F(KinematicStateEstimatorTest, RollingStaticStancePreservesPlannedContactTrust) {
  EstimatorNoise noise;
  noise.planned_contact_delay_s = 0.020;
  noise.planned_contact_ramp_s = 0.010;
  KinematicStateEstimator point(robot_.pin, robot_.info, noise);
  noise.rolling_foot_radius_m = 0.026;
  KinematicStateEstimator sphere(robot_.pin, robot_.info, noise);
  point.Reset(0.34);
  sphere.Reset(0.34);
  for (int cycle = 0; cycle < 100; ++cycle) {
    const auto contacts = ContactFlags(cycle < 20 || cycle >= 60 ? kStanceMode : std::size_t{0});
    const auto actual = sphere.Update(Pose(), RestingImu(), contacts, 0.002);
    const auto expected = point.Update(Pose(), RestingImu(), contacts, 0.002);
    ASSERT_TRUE(FiniteEstimate(actual));
    ASSERT_TRUE(FiniteEstimate(expected));
    EXPECT_EQ(sphere.plannedContactTrust(), point.plannedContactTrust());
    EXPECT_TRUE(actual.position == expected.position);
    EXPECT_TRUE(actual.velocity_world == expected.velocity_world);
  }
}

TEST_F(KinematicStateEstimatorTest, InvalidRollingFootRadiusIsRejected) {
  for (double invalid : {-0.001, 0.050001, std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::infinity()}) {
    EstimatorNoise noise;
    noise.rolling_foot_radius_m = invalid;
    EXPECT_THROW(KinematicStateEstimator(robot_.pin, robot_.info, noise), std::invalid_argument);
  }
  for (double valid : {0.0, 0.026, 0.05}) {
    EstimatorNoise noise;
    noise.rolling_foot_radius_m = valid;
    EXPECT_NO_THROW(KinematicStateEstimator(robot_.pin, robot_.info, noise));
  }
}

}  // namespace
}  // namespace custom_dog_control
