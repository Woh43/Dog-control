#include <gtest/gtest.h>

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <map>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz_ros2_control/gz_system_interface.hpp>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <pluginlib/class_loader.hpp>
#include <rcutils/logging.h>
#include <sdf/JointAxis.hh>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {
namespace {
// Capture diagnostics from the actual dynamically loaded plugin rather than a
// copy of its effort calculation. The previous process-wide handler is restored
// even when a fatal assertion returns from the test.
class ScopedMotorLogCapture {
 public:
  ScopedMotorLogCapture() : previous_(rcutils_logging_get_output_handler()) {
    active_ = this;
    rcutils_logging_set_output_handler(&Capture);
  }
  ~ScopedMotorLogCapture() {
    rcutils_logging_set_output_handler(previous_);
    active_ = nullptr;
  }
  std::vector<std::string> messages;
  std::vector<std::string> statistics;

 private:
  static void Capture(const rcutils_log_location_t* location, int severity,
                      const char* name, rcutils_time_point_value_t timestamp,
                      const char* format, va_list* args) {
    char buffer[2048];
    va_list copy;
    va_copy(copy, *args);
    std::vsnprintf(buffer, sizeof(buffer), format, copy);
    va_end(copy);
    if (active_ && std::string(buffer).find(
            "Motor requested effort threshold crossing:") == 0) {
      active_->messages.emplace_back(buffer);
    }
    if (active_ && std::string(buffer).find(
            "Motor physics cumulative stats:") == 0) {
      active_->statistics.emplace_back(buffer);
    }
    if (active_ && active_->previous_) {
      active_->previous_(location, severity, name, timestamp, format, args);
    }
  }
  inline static ScopedMotorLogCapture* active_ = nullptr;
  rcutils_logging_output_handler_t previous_;
};
}  // namespace

class FortressMotorSystemTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("motor_physics_contract");
    loader_ = std::make_unique<pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface>>(
        "gz_ros2_control", "gz_ros2_control::GazeboSimSystemInterface");
    motor_ = loader_->createSharedInstance("custom_dog_control/FortressMotorSystem");
    info_.name = "virtual_motors";
    info_.type = "system";
    for (const auto name : kJointNames) {
      const auto entity = ecm_.CreateEntity();
      entities_[std::string(name)] = entity;
      sdf::JointAxis axis;
      axis.SetEffort(23.7);
      ecm_.CreateComponent(entity, sim::components::JointAxis(axis));
      hardware_interface::ComponentInfo joint;
      joint.name = std::string(name);
      info_.joints.push_back(joint);
    }
  }

  bool Initialize(const std::unordered_map<std::string, std::string>& parameters = {}) {
    info_.hardware_parameters = parameters;
    int rate = 0;  // gz_ros2_control assigns this only after initSim.
    if (!motor_->initSim(node_, entities_, info_, ecm_, rate)) return false;
    if (motor_->on_init(info_) != hardware_interface::CallbackReturn::SUCCESS) return false;
    commands_ = motor_->export_command_interfaces();
    states_ = motor_->export_state_interfaces();
    Feedback(0.0, 0.0);
    return true;
  }

  void Set(const std::string& name, double value) {
    for (auto& command : commands_) {
      if (command.get_name() == name) { command.set_value(value); return; }
    }
    FAIL() << "missing interface " << name;
  }

  void Feedback(double q, double dq) {
    for (const auto& entry : entities_) {
      ecm_.Component<sim::components::JointPosition>(entry.second)->Data() = {q};
      ecm_.Component<sim::components::JointVelocity>(entry.second)->Data() = {dq};
    }
  }

  double Force(const std::string& name = "FR_hip_joint") {
    return ecm_.Component<sim::components::JointForceCmd>(entities_.at(name))->Data().at(0);
  }

  double Effort(const std::string& name = "FR_hip_joint") {
    for (const auto& state : states_) {
      if (state.get_name() == name + "/effort") return state.get_value();
    }
    ADD_FAILURE() << "missing state " << name;
    return std::numeric_limits<double>::quiet_NaN();
  }

  void PositionCommand(double q, double kp = 20.0, double kd = 2.0) {
    for (const auto name : kJointNames) {
      Set(std::string(name) + "/position", q);
      Set(std::string(name) + "/kp", kp);
      Set(std::string(name) + "/kd", kd);
    }
    Set("custom_dog/simulation_motor_mode", 2.0);
  }

  void Read(int milliseconds) {
    motor_->read(rclcpp::Time(milliseconds * 1000000LL), rclcpp::Duration::from_seconds(0.002));
  }

  void Write(int milliseconds) {
    motor_->write(rclcpp::Time(milliseconds * 1000000LL), rclcpp::Duration::from_seconds(0.001));
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface>> loader_;
  std::shared_ptr<gz_ros2_control::GazeboSimSystemInterface> motor_;
  sim::EntityComponentManager ecm_;
  std::map<std::string, sim::Entity> entities_;
  hardware_interface::HardwareInfo info_;
  std::vector<hardware_interface::CommandInterface> commands_;
  std::vector<hardware_interface::StateInterface> states_;
};

TEST_F(FortressMotorSystemTest, DelaysPacketsButUsesCurrentPhysicsFeedbackForMotorPd) {
  ASSERT_TRUE(Initialize());
  Read(4);
  PositionCommand(0.1);
  Write(4);
  EXPECT_DOUBLE_EQ(Force(), 0.0);  // New packet is still in the host pipeline.
  Read(8);
  Write(8);
  EXPECT_NEAR(Force(), 2.0, 1e-12);
  Feedback(0.02, 0.2);
  Write(9);
  EXPECT_NEAR(Force(), 1.2, 1e-12);  // Local PD updates without another host read.

  ecm_.Component<sim::components::JointVelocity>(entities_.at("RL_calf_joint"))->Data() =
      {std::numeric_limits<double>::quiet_NaN()};
  Write(10);
  for (const auto& entry : entities_) EXPECT_DOUBLE_EQ(Force(entry.first), 0.0);
  Feedback(0.02, 0.2);
  Write(11);
  EXPECT_DOUBLE_EQ(Force(), 0.0);  // Restoring feedback cannot replay the old packet.
  Read(12);
  Set("custom_dog/simulation_motor_mode", 0.0);
  Write(12);
  EXPECT_DOUBLE_EQ(Force(), 0.0);
  Read(16);
  Write(16);
  EXPECT_DOUBLE_EQ(Force(), 0.0);
}

TEST_F(FortressMotorSystemTest, ExtraDelayIsElapsedTimeAndReleasesCommandsInOrder) {
  ASSERT_TRUE(Initialize({{"simulation_command_delay_s", "0.005"}}));
  Read(2);
  PositionCommand(0.1);
  Write(2);
  Read(4);  // First command sent now, due at 9 ms.
  PositionCommand(0.2);
  Write(4);
  Read(6);  // Second command due at 11 ms.
  Write(6);
  Write(8);
  EXPECT_DOUBLE_EQ(Force(), 0.0);
  Write(9);  // No new controller read: delivery is not quantized to its rate.
  EXPECT_NEAR(Force(), 2.0, 1e-12);
  Feedback(0.02, 0.2);
  Write(10);
  EXPECT_NEAR(Force(), 1.2, 1e-12);
  Write(11);
  EXPECT_NEAR(Force(), 3.2, 1e-12);
}

TEST_F(FortressMotorSystemTest, SignedBudgetPreservesCancellationAcrossDelayedMotorPdExecution) {
  ASSERT_TRUE(Initialize({{"motor_pd_signed_budget", "true"},
                          {"motor_pd_budget_horizon_s", "0.024"},
                          {"motor_pd_assumed_acceleration_rad_s2", "100"},
                          {"simulation_command_delay_s", "0.020"}}));
  ScopedMotorLogCapture capture;
  Read(2);
  PositionCommand(-0.2, 50.0, 0.0);
  for (const auto name : kJointNames) Set(std::string(name) + "/effort", 20.0);
  Write(2);  // Total 10 fits; the legacy |FF|+|P|=30 budget would shrink it.
  Read(4);   // This packet is due at 24 ms, not when it is staged.
  Write(4);
  Read(6);   // Keep the existing host watchdog valid through delivery.
  Write(6);
  Read(8);
  Write(8);
  Write(23);
  EXPECT_DOUBLE_EQ(Force(), 0.0);
  Write(24);
  EXPECT_NEAR(Force(), 10.0, 1e-12);
  EXPECT_NEAR(Effort(), 10.0, 1e-12);

  Feedback(0.001, 0.1);
  Write(25);  // Held packet, new physical feedback, without another host read.
  EXPECT_NEAR(Force(), 9.95, 1e-12);
  EXPECT_NEAR(Effort(), 9.95, 1e-12);
  EXPECT_TRUE(capture.messages.empty());
  Set("custom_dog/simulation_motor_mode", 1.0);
  Write(26);  // Report the completed active episode before immediate safe mode.
  ASSERT_EQ(capture.statistics.size(), 1U);
  const auto& statistics = capture.statistics.front();
  EXPECT_NE(statistics.find("over_nominal=0 over_guard=0"), std::string::npos);
  EXPECT_NE(statistics.find("peak_ff=20.000000000 peak_p=-10.000000000"),
            std::string::npos);
  EXPECT_NE(statistics.find("signed_budget=1 envelope_violations=0"),
            std::string::npos);
  EXPECT_NE(statistics.find("stale_window_samples=0 signed_budget_samples=24"),
            std::string::npos);
}

TEST_F(FortressMotorSystemTest, SignedCancellationCannotBypassTheIndependentFeedforwardCap) {
  ASSERT_TRUE(Initialize({{"motor_pd_signed_budget", "true"},
                          {"motor_pd_budget_horizon_s", "0.024"},
                          {"motor_pd_assumed_acceleration_rad_s2", "100"}}));
  Read(2);
  PositionCommand(-0.2, 100.0, 0.0);
  for (const auto name : kJointNames) Set(std::string(name) + "/effort", 40.0);
  Write(2);  // Net 20 fits, but FF 40 alone exceeds its independent 23.7 cap.
  Read(4);
  Write(4);
  EXPECT_NEAR(Force(), 20.0 * (23.7 / 40.0), 1e-12);
  Feedback(-0.2, 0.0);  // Zero P reveals that the transmitted FF itself was capped.
  Write(5);
  EXPECT_NEAR(Force(), 23.7, 1e-12);
}

TEST_F(FortressMotorSystemTest, SignedWindowTelemetryExposesOutOfModelFeedbackWithoutHiddenClamp) {
  ASSERT_TRUE(Initialize({{"motor_pd_signed_budget", "true"},
                          {"motor_pd_budget_horizon_s", "0.024"},
                          {"motor_pd_assumed_acceleration_rad_s2", "100"},
                          {"simulation_command_delay_s", "0.020"}}));
  ScopedMotorLogCapture capture;
  Read(2);
  PositionCommand(-0.2, 50.0, 0.0);
  for (const auto name : kJointNames) Set(std::string(name) + "/effort", 20.0);
  Write(2);
  Read(4);
  Write(4);
  Read(6);
  Write(6);
  Read(8);
  Write(8);
  Write(24);
  ASSERT_NEAR(Force(), 10.0, 1e-12);
  // Deliberately violate the declared acceleration model. A real motor does
  // not acquire continuous saturation merely because the host checked a window.
  Feedback(-0.5, 0.0);
  Write(25);
  EXPECT_NEAR(Force(), 35.0, 1e-12);
  EXPECT_NEAR(Effort(), 35.0, 1e-12);
  EXPECT_EQ(capture.messages.size(), kJointCount);
  Set("custom_dog/simulation_motor_mode", 1.0);
  Write(26);
  ASSERT_EQ(capture.statistics.size(), 1U);
  EXPECT_NE(capture.statistics.front().find("over_nominal=12 over_guard=12"),
            std::string::npos);
  EXPECT_NE(capture.statistics.front().find("signed_budget=1 envelope_violations=12"),
            std::string::npos);
}

TEST_F(FortressMotorSystemTest, LogsFirstEffortExcursionWithDelayedCommandTermsWithoutClamping) {
  ASSERT_TRUE(Initialize({{"simulation_command_delay_s", "0.005"}}));
  ScopedMotorLogCapture capture;
  Read(2);
  PositionCommand(0.1, 60.0, 2.0);
  for (const auto name : kJointNames) Set(std::string(name) + "/effort", 5.0);
  Write(2);  // The packet fits the effort budget at this feedback sample.
  Read(4);
  Write(4);
  Write(9);
  ASSERT_NEAR(Force(), 11.0, 1e-12);
  ASSERT_TRUE(capture.messages.empty());

  const auto hip = entities_.at("FR_hip_joint");
  ecm_.Component<sim::components::JointPosition>(hip)->Data() = {-0.2};
  ecm_.Component<sim::components::JointVelocity>(hip)->Data() = {-4.0};
  Write(10);
  EXPECT_NEAR(Force(), 31.0, 1e-12);  // FF 5 + P 18 + D 8, still not clamped.
  EXPECT_NEAR(Effort(), 31.0, 1e-12);
  ASSERT_EQ(capture.messages.size(), 1U);
  const auto& message = capture.messages.front();
  EXPECT_NE(message.find("sim_time=0.010000000 index=0 joint=FR_hip_joint"),
            std::string::npos);
  EXPECT_NE(message.find("threshold=26.070000000"), std::string::npos);
  EXPECT_NE(message.find("ff=5.000000000 p=18.000000000 d=8.000000000"),
            std::string::npos);
  EXPECT_NE(message.find("command_age_ms=8.000000"), std::string::npos);
  Write(11);
  Feedback(0.0, 0.0);
  Write(12);
  ecm_.Component<sim::components::JointPosition>(hip)->Data() = {-0.2};
  ecm_.Component<sim::components::JointVelocity>(hip)->Data() = {-4.0};
  Write(13);
  EXPECT_EQ(capture.messages.size(), 1U);  // No per-step or repeated crossing flood.

  Set("custom_dog/simulation_motor_mode", 0.0);
  Write(14);
  Feedback(0.0, 0.0);
  Read(16);
  PositionCommand(0.1, 60.0, 2.0);
  Write(16);
  Read(18);
  Write(18);
  Write(23);
  ecm_.Component<sim::components::JointPosition>(hip)->Data() = {-0.2};
  ecm_.Component<sim::components::JointVelocity>(hip)->Data() = {-4.0};
  Write(24);
  EXPECT_NEAR(Force(), 31.0, 1e-12);
  EXPECT_EQ(capture.messages.size(), 2U);  // Disable starts a new logged episode.
}

TEST_F(FortressMotorSystemTest, DisableImmediatelyClearsActivePendingAndQueuedCommands) {
  ASSERT_TRUE(Initialize({{"simulation_command_delay_s", "0.005"}}));
  Read(2);
  PositionCommand(0.1);
  Write(2);
  Read(4);
  Write(4);
  Read(6);
  Write(6);
  Write(9);
  ASSERT_NEAR(Force(), 2.0, 1e-12);
  Set("custom_dog/simulation_motor_mode", 0.0);
  Write(10);  // Disable must not wait for another host read or queue deadline.
  EXPECT_DOUBLE_EQ(Force(), 0.0);
  Write(11);
  EXPECT_DOUBLE_EQ(Force(), 0.0);
  Read(12);
  Write(12);
  EXPECT_DOUBLE_EQ(Force(), 0.0);
}

TEST_F(FortressMotorSystemTest, SafePoseBypassesAndClearsDelayedMotion) {
  ASSERT_TRUE(Initialize({{"simulation_command_delay_s", "0.005"}}));
  Read(2);
  PositionCommand(0.1);
  Write(2);
  Read(4);
  Write(4);
  PositionCommand(-0.1);
  Set("custom_dog/simulation_motor_mode", 1.0);
  Write(5);
  EXPECT_NEAR(Force(), -2.0, 1e-12);
  Read(6);
  Write(6);
  Write(9);  // Old command's deadline must not resurrect positive motion.
  EXPECT_NEAR(Force(), -2.0, 1e-12);
  Write(11);
  EXPECT_NEAR(Force(), -2.0, 1e-12);
  PositionCommand(0.03);  // Return to active mode with a new packet.
  Write(12);
  Read(14);
  Write(14);
  Write(18);
  EXPECT_NEAR(Force(), -2.0, 1e-12);  // Safe pose is held until new packet arrives.
  Write(19);
  EXPECT_NEAR(Force(), 0.6, 1e-12);
}

TEST_F(FortressMotorSystemTest, ClockRollbackAtReadDiscardsAllOldCommands) {
  ASSERT_TRUE(Initialize({{"simulation_command_delay_s", "0.005"}}));
  Read(12);
  PositionCommand(0.1);
  Write(12);
  Read(14);
  Write(14);
  Write(19);
  ASSERT_NEAR(Force(), 2.0, 1e-12);
  Read(2);
  Write(2);
  EXPECT_DOUBLE_EQ(Force(), 0.0);
  Read(14);
  Write(14);
  Write(19);
  EXPECT_DOUBLE_EQ(Force(), 0.0);
  PositionCommand(0.05);  // Only a new post-reset command may resume motion.
  Read(20);
  Write(20);
  Read(22);
  Write(22);
  Write(27);
  EXPECT_NEAR(Force(), 1.0, 1e-12);
}

TEST_F(FortressMotorSystemTest, ClockRollbackAtPhysicsWriteAlsoDiscardsOldCommands) {
  ASSERT_TRUE(Initialize({{"simulation_command_delay_s", "0.005"}}));
  Read(12);
  PositionCommand(0.1);
  Write(12);
  Read(14);
  Write(19);
  ASSERT_NEAR(Force(), 2.0, 1e-12);
  Write(15);  // Still newer than the last read, but older than the last write.
  EXPECT_DOUBLE_EQ(Force(), 0.0);
  Write(21);
  EXPECT_DOUBLE_EQ(Force(), 0.0);
}

TEST_F(FortressMotorSystemTest, MissingHostUpdatesFlushCommandsBeforeFutureDeadlines) {
  ASSERT_TRUE(Initialize({{"simulation_command_delay_s", "0.05"}}));
  Read(2);
  PositionCommand(0.1);
  Write(2);
  Read(4);
  Write(4);
  Write(25);  // Existing 20 ms host watchdog must also clear the delay queue.
  EXPECT_DOUBLE_EQ(Force(), 0.0);
  Read(52);
  Set("custom_dog/simulation_motor_mode", 0.0);
  Write(52);
  Write(54);
  EXPECT_DOUBLE_EQ(Force(), 0.0);
}

TEST_F(FortressMotorSystemTest, RepeatedFrozenTimeReadsCannotGrowAnUnboundedCommandQueue) {
  ASSERT_TRUE(Initialize({{"simulation_command_delay_s", "0.005"}}));
  Read(2);
  PositionCommand(0.1);
  Write(2);
  for (int i = 0; i < 1000; ++i) Read(4);
  Write(4);
  EXPECT_DOUBLE_EQ(Force(), 0.0);
  Read(8);
  Write(8);
  Write(9);
  EXPECT_DOUBLE_EQ(Force(), 0.0);  // Overflow discarded every queued packet.
}

TEST_F(FortressMotorSystemTest, FrictionIsDissipativeJointLoadAndEffortRemainsActuatorRequest) {
  ASSERT_TRUE(Initialize({{"simulation_coulomb_friction", "1 2 3"},
                          {"simulation_viscous_friction", "0.5"},
                          {"simulation_friction_velocity_scale", "0.1"}}));
  Read(2);
  PositionCommand(0.0, 0.0, 0.0);
  for (const auto name : kJointNames) Set(std::string(name) + "/effort", 4.0);
  Write(2);
  Read(4);
  Write(4);
  EXPECT_DOUBLE_EQ(Force(), 4.0);  // Friction cannot generate torque at rest.
  for (const double velocity : {-1.0, -0.01, 0.01, 1.0}) {
    Feedback(0.0, velocity);
    Write(4);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const std::string name(kJointNames[i]);
      const double expected_load = -(1.0 + i % kJointsPerLeg) * std::tanh(velocity / 0.1) -
          0.5 * velocity;
      EXPECT_NEAR(Force(name), 4.0 + expected_load, 1e-12);
      EXPECT_DOUBLE_EQ(Effort(name), 4.0);
      EXPECT_LT((Force(name) - Effort(name)) * velocity, 0.0);
    }
  }
  Set("custom_dog/simulation_motor_mode", 0.0);
  Write(5);
  EXPECT_DOUBLE_EQ(Effort(), 0.0);
  EXPECT_LT(Force(), 0.0);  // A disabled motor still has passive friction.
}

TEST_F(FortressMotorSystemTest, FrictionAcceptsIndividualJointCoefficients) {
  ASSERT_TRUE(Initialize({{"simulation_coulomb_friction", "0 1 2 3 4 5 6 7 8 9 10 11"}}));
  Feedback(0.0, -1.0);
  Read(2);
  Write(2);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    EXPECT_NEAR(Force(std::string(kJointNames[i])), i * std::tanh(20.0), 1e-12);
  }
}

TEST_F(FortressMotorSystemTest, RejectsInvalidDelayAndFrictionParameters) {
  const std::vector<std::pair<std::string, std::string>> invalid = {
      {"simulation_command_delay_s", "-0.001"},
      {"simulation_command_delay_s", "0.051"},
      {"simulation_command_delay_s", "0 0.001"},
      {"simulation_command_delay_s", "nan"},
      {"simulation_command_delay_s", "0junk"},
      {"simulation_coulomb_friction", "-1"},
      {"simulation_coulomb_friction", "0 1"},
      {"simulation_coulomb_friction", "1 2 3 4"},
      {"simulation_coulomb_friction", "inf"},
      {"simulation_coulomb_friction", "0 1e999"},
      {"simulation_viscous_friction", ""},
      {"simulation_viscous_friction", "-0.2"},
      {"simulation_friction_velocity_scale", "0"},
      {"simulation_friction_velocity_scale", "nan"}};
  for (const auto& parameter : invalid) {
    SCOPED_TRACE(parameter.first + "=" + parameter.second);
    EXPECT_FALSE(Initialize({parameter}));
  }
}
}  // namespace custom_dog_control
