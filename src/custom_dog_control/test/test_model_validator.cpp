#include <array>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "custom_dog_control/nmpc/ModelValidator.hpp"
#include "custom_dog_control/nmpc/KinematicStateEstimator.hpp"

namespace {

TEST(ModelValidator, CanonicalModelAndHardwareOrderAreValid) {
  const std::vector<std::string> joints = {
      "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
  const std::vector<std::string> feet = {
      "FR_foot", "FL_foot", "RR_foot", "RL_foot"};
  auto pinocchio_interface =
      ocs2::centroidal_model::createPinocchioInterface(
          CUSTOM_DOG_CONTROL_CANONICAL_URDF, joints);
  const auto model_info = ocs2::centroidal_model::createCentroidalModelInfo(
      pinocchio_interface,
      ocs2::CentroidalModelType::FullCentroidalDynamics,
      ocs2::vector_t::Zero(joints.size()), feet, {});

  auto result = custom_dog_control::ModelValidator::Validate(
      pinocchio_interface, model_info);

  EXPECT_TRUE(result.ok) << result.Summary();
  EXPECT_NEAR(result.total_mass_kg, 15.15, 1e-5);
  EXPECT_EQ(
      result.model_joint_slots,
      (std::array<int, 12>{3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8}));
}

TEST(ModelValidator, EstimatorResetDiscardsPreCalibrationFootAnchors) {
  using namespace custom_dog_control;
  std::vector<std::string> joints, feet;
  for (const auto name : kJointNames) joints.emplace_back(name);
  for (const auto name : kFootFrameNames) feet.emplace_back(name);
  auto pin = ocs2::centroidal_model::createPinocchioInterface(
      CUSTOM_DOG_CONTROL_CANONICAL_URDF, joints);
  const auto info = ocs2::centroidal_model::createCentroidalModelInfo(
      pin, ocs2::CentroidalModelType::FullCentroidalDynamics,
      ocs2::vector_t::Zero(joints.size()), feet, {});
  KinematicStateEstimator reused(pin, info), fresh(pin, info);
  JointSample sample;
  sample.valid.fill(1.0);
  sample.position.fill(2.0);  // Raw encoders have no meaningful URDF zero.
  ImuSample imu;
  imu.valid = true;
  imu.linear_acceleration = {0.0, 0.0, 9.81};
  reused.Reset(0.29);
  for (int i = 0; i < 20; ++i)
    reused.Update(sample, imu, ContactFlags(kStanceMode), 0.004);
  for (std::size_t i = 0; i < kJointCount; ++i)
    sample.position[i] = i % 3 == 0 ? 0.0 : (i % 3 == 1 ? 1.253146 : -2.823943);
  reused.Reset(0.29);
  fresh.Reset(0.29);
  const auto after = reused.Update(sample, imu, ContactFlags(kStanceMode), 0.004);
  const auto expected = fresh.Update(sample, imu, ContactFlags(kStanceMode), 0.004);
  ASSERT_TRUE(after.valid);
  ASSERT_TRUE(expected.valid);
  EXPECT_LT((after.position - expected.position).norm(), 1e-12);
  EXPECT_LT((after.velocity_world - expected.velocity_world).norm(), 1e-12);
}

TEST(ModelValidator, FlatGroundEstimatorUsesSphereCenterHeightFromModel) {
  using namespace custom_dog_control;
  std::vector<std::string> joints, feet;
  for (const auto name : kJointNames) joints.emplace_back(name);
  for (const auto name : kFootFrameNames) feet.emplace_back(name);
  auto pin = ocs2::centroidal_model::createPinocchioInterface(
      CUSTOM_DOG_CONTROL_CANONICAL_URDF, joints);
  const auto info = ocs2::centroidal_model::createCentroidalModelInfo(
      pin, ocs2::CentroidalModelType::FullCentroidalDynamics,
      ocs2::vector_t::Zero(joints.size()), feet, {});
  const double radius = RobotModelConfig{}.foot_radius_m;
  KinematicStateEstimator flat(pin, info), explicit_height(pin, info);
  KinematicStateEstimator zero_height(pin, info, {}, 0.0);
  JointSample sample;
  sample.valid.fill(1.0);
  ocs2::vector_t q = ocs2::vector_t::Zero(info.generalizedCoordinatesNum);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    sample.position[i] = i % 3 == 0 ? 0.0 : (i % 3 == 1 ? .8 : -1.6);
    const auto id = pin.getModel().getJointId(joints[i]);
    q(pin.getModel().joints[id].idx_q()) = sample.position[i];
  }
  pinocchio::forwardKinematics(pin.getModel(), pin.getData(), q);
  pinocchio::updateFramePlacements(pin.getModel(), pin.getData());
  double mean_foot_z_at_base_origin = 0.0;
  for (const auto& foot : feet)
    mean_foot_z_at_base_origin +=
        pin.getData().oMf[pin.getModel().getFrameId(foot)].translation().z() / kLegCount;

  ImuSample imu;
  imu.valid = true;
  imu.linear_acceleration = {0.0, 0.0, 9.81};
  ContactSupport support;
  support.contact.fill(true);
  support.height_valid.fill(true);
  support.foot_center_height.fill(radius);
  flat.Reset(.29);
  explicit_height.Reset(.29);
  zero_height.Reset(.29);
  const auto actual = flat.Update(sample, imu, ContactFlags(kStanceMode), .004);
  const auto expected = explicit_height.Update(sample, imu, support, .004);
  const auto point_contact = zero_height.Update(sample, imu, ContactFlags(kStanceMode), .004);
  ASSERT_TRUE(actual.valid);
  ASSERT_TRUE(expected.valid);
  ASSERT_TRUE(point_contact.valid);
  EXPECT_NEAR(actual.position.z(), radius - mean_foot_z_at_base_origin, 1e-10);
  EXPECT_LT((actual.position - expected.position).norm(), 1e-12);
  EXPECT_LT((actual.velocity_world - expected.velocity_world).norm(), 1e-12);
  EXPECT_NEAR(actual.position.z() - point_contact.position.z(), radius, 1e-10);

  // The explicit support overload must retain caller-supplied platform heights.
  support.foot_center_height.fill(.12);
  explicit_height.Reset(.29);
  const auto platform = explicit_height.Update(sample, imu, support, .004);
  ASSERT_TRUE(platform.valid);
  EXPECT_NEAR(platform.position.z(), .12 - mean_foot_z_at_base_origin, 1e-10);
}

TEST(ModelValidator, FlatGroundFootCenterHeightRejectsInvalidConfiguration) {
  using namespace custom_dog_control;
  std::vector<std::string> joints, feet;
  for (const auto name : kJointNames) joints.emplace_back(name);
  for (const auto name : kFootFrameNames) feet.emplace_back(name);
  auto pin = ocs2::centroidal_model::createPinocchioInterface(
      CUSTOM_DOG_CONTROL_CANONICAL_URDF, joints);
  const auto info = ocs2::centroidal_model::createCentroidalModelInfo(
      pin, ocs2::CentroidalModelType::FullCentroidalDynamics,
      ocs2::vector_t::Zero(joints.size()), feet, {});
  for (const double invalid : {-0.001, std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(KinematicStateEstimator(pin, info, {}, invalid), std::invalid_argument);
  }
  EXPECT_NO_THROW(KinematicStateEstimator(pin, info, {}, 0.0));
}

}  // namespace
