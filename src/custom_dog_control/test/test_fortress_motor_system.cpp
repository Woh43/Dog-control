#include <gtest/gtest.h>

#include <map>
#include <limits>
#include <string>
#include <vector>

#include <gz_ros2_control/gz_system_interface.hpp>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <pluginlib/class_loader.hpp>
#include <sdf/JointAxis.hh>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {
TEST(FortressMotorSystem, DelaysPacketsButUsesCurrentPhysicsFeedbackForMotorPd) {
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("motor_physics_contract");
  pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface> loader(
      "gz_ros2_control", "gz_ros2_control::GazeboSimSystemInterface");
  auto motor = loader.createSharedInstance("custom_dog_control/FortressMotorSystem");
  sim::EntityComponentManager ecm;
  std::map<std::string, sim::Entity> entities;
  hardware_interface::HardwareInfo info;
  info.name = "virtual_motors";
  info.type = "system";
  for (const auto name : kJointNames) {
    const auto entity = ecm.CreateEntity();
    entities[std::string(name)] = entity;
    sdf::JointAxis axis;
    axis.SetEffort(23.7);
    ecm.CreateComponent(entity, sim::components::JointAxis(axis));
    hardware_interface::ComponentInfo joint;
    joint.name = std::string(name);
    info.joints.push_back(joint);
  }
  int rate = 0;  // gz_ros2_control assigns this only after initSim.
  ASSERT_TRUE(motor->initSim(node, entities, info, ecm, rate));
  ASSERT_EQ(motor->on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  auto commands = motor->export_command_interfaces();
  const auto set = [&](const std::string& name, double value) {
    for (auto& command : commands) {
      if (command.get_name() == name) { command.set_value(value); return; }
    }
    FAIL() << "missing interface " << name;
  };
  const auto feedback = [&](double q, double dq) {
    for (const auto& entry : entities) {
      ecm.Component<sim::components::JointPosition>(entry.second)->Data() = {q};
      ecm.Component<sim::components::JointVelocity>(entry.second)->Data() = {dq};
    }
  };
  const auto force = [&]() {
    return ecm.Component<sim::components::JointForceCmd>(
        entities.at("FR_hip_joint"))->Data().at(0);
  };
  const auto time = [](int milliseconds) { return rclcpp::Time(milliseconds * 1000000LL); };
  const auto period = rclcpp::Duration::from_seconds(0.004);
  feedback(0.0, 0.0);
  motor->read(time(4), period);
  for (const auto name : kJointNames) {
    set(std::string(name) + "/position", 0.1);
    set(std::string(name) + "/kp", 20.0);
    set(std::string(name) + "/kd", 2.0);
  }
  set("custom_dog/simulation_motor_mode", 2.0);
  motor->write(time(4), period);
  EXPECT_DOUBLE_EQ(force(), 0.0);  // New packet is still in the host pipeline.
  motor->read(time(8), period);
  motor->write(time(8), period);
  EXPECT_NEAR(force(), 2.0, 1e-12);
  feedback(0.02, 0.2);
  motor->write(time(9), rclcpp::Duration::from_seconds(0.001));
  EXPECT_NEAR(force(), 1.2, 1e-12);  // Local PD updates without another host read.

  ecm.Component<sim::components::JointVelocity>(entities.at("RL_calf_joint"))->Data() =
      {std::numeric_limits<double>::quiet_NaN()};
  motor->write(time(10), rclcpp::Duration::from_seconds(0.001));
  for (const auto& entry : entities) {
    EXPECT_DOUBLE_EQ(ecm.Component<sim::components::JointForceCmd>(entry.second)->Data().at(0), 0.0);
  }
  feedback(0.02, 0.2);
  motor->write(time(11), rclcpp::Duration::from_seconds(0.001));
  EXPECT_DOUBLE_EQ(force(), 0.0);  // Restoring feedback cannot replay the old packet.

  motor->read(time(12), period);
  set("custom_dog/simulation_motor_mode", 0.0);
  motor->write(time(12), period);
  EXPECT_DOUBLE_EQ(force(), 0.0);  // Disable bypasses the delayed active packet.
  motor->read(time(16), period);
  motor->write(time(16), period);
  EXPECT_DOUBLE_EQ(force(), 0.0);
}
}  // namespace custom_dog_control
