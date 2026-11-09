#include <array>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <gz_ros2_control/gz_system_interface.hpp>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityReset.hh>
#include <pluginlib/class_list_macros.hpp>

#include "custom_dog_control/control/JointCommandUtils.hpp"

namespace custom_dog_control {

// Models the existing five-field actuator protocol. The outer NMPC/WBC loop
// runs at controller_manager's rate; the held motor-side PD command is evaluated
// using physical joint feedback at every physics step. This is an ideal servo
// model, not a measurement of the real drive's bandwidth or torque accuracy.
class FortressMotorSystem final : public gz_ros2_control::GazeboSimSystemInterface {
 public:
  bool initSim(rclcpp::Node::SharedPtr& node,
               std::map<std::string, sim::Entity>& joints,
               const hardware_interface::HardwareInfo& info,
               sim::EntityComponentManager& ecm, int& /*update_rate*/) override {
    nh_ = node;
    ecm_ = &ecm;
    // gz_ros2_control fills update_rate only after initSim returns and the
    // controller_manager exists. It must not be read during this callback.
    if (info.joints.size() != kJointCount) return false;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto& joint = info.joints[i];
      const auto entity = joints.find(joint.name);
      if (joint.name != kJointNames[i] || entity == joints.end()) return false;
      entities_[i] = entity->second;
      const auto* axis = ecm.Component<sim::components::JointAxis>(entities_[i]);
      if (!axis || !std::isfinite(axis->Data().Effort()) || axis->Data().Effort() <= 0.0) {
        RCLCPP_ERROR(node->get_logger(), "Missing effort limit: %s", joint.name.c_str());
        return false;
      }
      limits_[i] = axis->Data().Effort();
      double initial = 0.0;
      for (const auto& interface : joint.state_interfaces) {
        if (interface.name == "position" && !interface.initial_value.empty()) {
          initial = std::stod(interface.initial_value);
        }
      }
      if (!std::isfinite(initial)) return false;
      state_.position[i] = initial;
      state_.valid[i] = 1.0;
      if (!ecm.Component<sim::components::JointPosition>(entities_[i])) {
        ecm.CreateComponent(entities_[i], sim::components::JointPosition());
      }
      if (!ecm.Component<sim::components::JointVelocity>(entities_[i])) {
        ecm.CreateComponent(entities_[i], sim::components::JointVelocity());
      }
      ecm.CreateComponent(entities_[i], sim::components::JointPositionReset({initial}));
      ecm.CreateComponent(entities_[i], sim::components::JointVelocityReset({0.0}));
    }
    RCLCPP_INFO(node->get_logger(),
        "Motor-PD physics: one outer-cycle delay plus physics-step phase, local PD every physics step");
    return true;
  }

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override {
    return hardware_interface::SystemInterface::on_init(info);
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const std::string name(kJointNames[i]);
      interfaces.emplace_back(name, "position", &state_.position[i]);
      interfaces.emplace_back(name, "velocity", &state_.velocity[i]);
      // Requested simulated torque, not a physical torque sensor.
      interfaces.emplace_back(name, "effort", &state_.effort[i]);
    }
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const std::string name(kJointNames[i]);
      interfaces.emplace_back(name, "position", &command_.position[i]);
      interfaces.emplace_back(name, "velocity", &command_.velocity[i]);
      interfaces.emplace_back(name, "effort", &command_.effort[i]);
      interfaces.emplace_back(name, "kp", &command_.kp[i]);
      interfaces.emplace_back(name, "kd", &command_.kd[i]);
    }
    // 0 = disabled, 1 = immediate simulation pose/safe output,
    // 2 = active command through the same one-cycle pipe as hardware.
    interfaces.emplace_back("custom_dog", "simulation_motor_mode", &command_mode_);
    return interfaces;
  }

  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>&,
      const std::vector<std::string>& stop) override {
    if (!stop.empty()) Reset();
    return hardware_interface::return_type::OK;
  }

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State&) override {
    Reset();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration&) override {
    // Hardware sends the previous write's packet at the next read. The PD
    // itself stays in the motor and does not wait for another host sample.
    active_ = pending_;
    ++read_epoch_;
    last_read_seconds_ = time.seconds();
    Sample(state_);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration&) override {
    if (staged_epoch_ != read_epoch_) {
      staged_epoch_ = read_epoch_;
      if ((command_mode_ != 1.0 && command_mode_ != 2.0) ||
          !BoundedMotorPdCommand(command_, state_, limits_, pending_)) {
        pending_ = {};
        active_ = {};
      } else if (command_mode_ == 1.0) {
        // Passive/FAULT output bypasses and clears the delayed motion packet.
        active_ = pending_;
      }
    }
    if (time.seconds() < last_read_seconds_ || time.seconds() - last_read_seconds_ > 0.02) {
      pending_ = {};
      active_ = {};
    }
    JointSample physical;
    Sample(physical);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (physical.valid[i] < 0.5) {
        pending_ = {};
        active_ = {};
        break;
      }
    }
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const double effort = EquivalentEffort(active_, physical, i);
      // Do not hide inter-sample PD peaks with a second software clamp that
      // the real motor protocol does not provide. Report requested torque;
      // do not assume an engine-side effort clamp has been verified.
      state_.effort[i] = physical.valid[i] > 0.5 && std::isfinite(effort) ? effort : 0.0;
      auto* force = ecm_->Component<sim::components::JointForceCmd>(entities_[i]);
      if (force) force->Data() = {state_.effort[i]};
      else ecm_->CreateComponent(entities_[i], sim::components::JointForceCmd({state_.effort[i]}));
    }
    return hardware_interface::return_type::OK;
  }

 private:
  void Sample(JointSample& sample) {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto* q = ecm_->Component<sim::components::JointPosition>(entities_[i]);
      const auto* dq = ecm_->Component<sim::components::JointVelocity>(entities_[i]);
      const bool valid = q && dq && !q->Data().empty() && !dq->Data().empty() &&
          std::isfinite(q->Data()[0]) && std::isfinite(dq->Data()[0]);
      sample.valid[i] = valid ? 1.0 : 0.0;
      if (valid) {
        sample.position[i] = q->Data()[0];
        sample.velocity[i] = dq->Data()[0];
      }
    }
  }
  void Reset() {
    command_ = {};
    pending_ = {};
    active_ = {};
    command_mode_ = 0.0;
  }
  sim::EntityComponentManager* ecm_ = nullptr;
  std::array<sim::Entity, kJointCount> entities_{};
  std::array<double, kJointCount> limits_{};
  JointSample state_;
  HybridJointCommand command_, pending_, active_;
  double command_mode_ = 0.0;
  double last_read_seconds_ = 0.0;
  std::uint64_t read_epoch_ = 0, staged_epoch_ = 0;
};
}  // namespace custom_dog_control

PLUGINLIB_EXPORT_CLASS(custom_dog_control::FortressMotorSystem,
                      gz_ros2_control::GazeboSimSystemInterface)
