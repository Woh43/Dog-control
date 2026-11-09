// Exercise the production hardware plugin through four in-process PTYs only.
// FOC packets are decoded here; no physical UART or actuator is opened.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <linux/serial.h>
#include <limits>
#include <iostream>
#include <mutex>
#include <poll.h>
#include <sched.h>
#include <sstream>
#include <pty.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "custom_dog_control/hardware/UnitreeSystemInterface.hpp"

// The actuator SDK uses a hardware-only custom-baud ioctl even for a PTY.
// Emulate that configuration for /dev/pts only; other descriptors always use
// the genuine kernel ioctl. This symbol is local to this test executable.
extern "C" int ioctl(int fd, unsigned long request, ...) noexcept {
  if (request == TIOCEXCL || request == TIOCNXCL) {
    return static_cast<int>(::syscall(SYS_ioctl, fd, request, 0));
  }
  va_list arguments;
  va_start(arguments, request);
  void* argument = va_arg(arguments, void*);
  va_end(arguments);
  char path[64]{};
  char target[256]{};
  std::snprintf(path, sizeof(path), "/proc/self/fd/%d", fd);
  const auto length = ::readlink(path, target, sizeof(target) - 1);
  if (length > 0 && std::strncmp(target, "/dev/pts/", 9) == 0) {
    if (request == TIOCGSERIAL) {
      auto* serial = static_cast<serial_struct*>(argument);
      std::memset(serial, 0, sizeof(*serial));
      serial->type = PORT_16550A;
      serial->baud_base = 4000000;
      serial->custom_divisor = 1;
      return 0;
    }
    if (request == TIOCSSERIAL) return 0;
  }
  return static_cast<int>(::syscall(SYS_ioctl, fd, request, argument));
}

namespace custom_dog_control {
namespace {
using namespace std::chrono_literals;

std::uint16_t Crc(const std::uint8_t* bytes, std::size_t count) {
  std::uint16_t crc = 0;
  for (std::size_t i = 0; i < count; ++i) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = crc & 1 ? (crc >> 1) ^ 0x8408 : crc >> 1;
    }
  }
  return crc;
}

// Optional test-only placement: emulate UART hardware independently of the
// controller's cores (RK3588: DOG_PTY_RESPONDER_CPUS=0-3, test taskset 4-7).
// An unset variable preserves portable tests' inherited scheduler affinity.
std::vector<int> PtyResponderCpus() {
  const char* value = std::getenv("DOG_PTY_RESPONDER_CPUS");
  if (!value || !*value) return {};
  std::stringstream input(value);
  std::string token;
  std::vector<int> cpus;
  while (std::getline(input, token, ',')) {
    int first = -1, last = -1;
    char extra = 0;
    if (std::sscanf(token.c_str(), "%d-%d%c", &first, &last, &extra) != 2) {
      if (std::sscanf(token.c_str(), "%d%c", &first, &extra) != 1) {
        throw std::runtime_error("invalid DOG_PTY_RESPONDER_CPUS");
      }
      last = first;
    }
    if (first < 0 || last < first || last >= CPU_SETSIZE) {
      throw std::runtime_error("DOG_PTY_RESPONDER_CPUS outside supported CPU set");
    }
    for (int cpu = first; cpu <= last; ++cpu) cpus.push_back(cpu);
  }
  if (cpus.empty()) throw std::runtime_error("empty DOG_PTY_RESPONDER_CPUS");
  return cpus;
}

class MotorPtys {
 public:
  using Packet = std::array<std::uint8_t, 17>;
  struct Seen { std::size_t index; Packet packet; };
  MotorPtys() {
    responder_cpus_ = PtyResponderCpus();
    const char* busy = std::getenv("DOG_PTY_BUSY_POLL");
    busy_poll_ = busy && (std::strcmp(busy, "1") == 0 || std::strcmp(busy, "true") == 0);
    if (busy && *busy && !busy_poll_ && std::strcmp(busy, "0") != 0 &&
        std::strcmp(busy, "false") != 0) {
      throw std::runtime_error("DOG_PTY_BUSY_POLL must be 0/1 or false/true");
    }
    // Keep allocator work outside the two-millisecond synthetic UART response.
    history_.reserve(kJointCount * 512);
    for (auto& pending : pending_) pending.reserve(64);
    try {
      for (std::size_t leg = 0; leg < ports.size(); ++leg) {
        int slave = -1;
        char name[128]{};
        if (::openpty(&masters_[leg], &slave, name, nullptr, nullptr) != 0) {
          throw std::runtime_error("openpty failed");
        }
        ports[leg] = name;
        ::close(slave);
        if (ports[leg].rfind("/dev/pts/", 0) != 0) {
          throw std::runtime_error("refusing a physical device");
        }
        ::fcntl(masters_[leg], F_SETFL, O_NONBLOCK);
      }
      // One independent device peer services all UARTs. Four sleeping peers
      // otherwise compete to wake under CFS inside the real SDK's 2 ms limit.
      worker_ = std::thread([this] { Respond(); });
    } catch (...) {
      for (int fd : masters_) if (fd >= 0) ::close(fd);
      throw;
    }
  }
  ~MotorPtys() {
    stopping_.store(true);
    if (worker_.joinable()) worker_.join();
    for (int fd : masters_) if (fd >= 0) ::close(fd);
  }
  std::array<Packet, kJointCount> packets() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return packets_;
  }
  std::vector<Seen> history() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return history_;
  }
  void ClearHistory() {
    std::lock_guard<std::mutex> lock(mutex_);
    history_.clear();
  }
  bool valid() const { return !invalid_.load(); }
  void CorruptReplyFor(int motor) { corrupt_reply_motor_.store(motor); }
  void DropReplyFor(int motor) { drop_reply_motor_.store(motor); }
  std::array<std::string, 4> ports;

 private:
  void Process(std::size_t leg) {
    auto& pending = pending_[leg];
    std::uint8_t bytes[512];
    const auto count = ::read(masters_[leg], bytes, sizeof(bytes));
    if (count <= 0) return;
    pending.insert(pending.end(), bytes, bytes + count);
    while (pending.size() >= 17) {
      Packet packet{};
      std::copy_n(pending.begin(), packet.size(), packet.begin());
      pending.erase(pending.begin(), pending.begin() + packet.size());
      const auto id = packet[2] & 0x0f;
      if (packet[0] != 0xfe || packet[1] != 0xee || id >= 3 ||
          Crc(packet.data(), 15) != (packet[15] | (packet[16] << 8))) {
        invalid_.store(true);
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        packets_[leg * 3 + id] = packet;
        history_.push_back({leg * 3 + id, packet});
      }
      if (drop_reply_motor_.load() == static_cast<int>(leg * 3 + id)) continue;
      std::array<std::uint8_t, 16> reply{};
      reply[0] = 0xfd;
      reply[1] = 0xee;
      reply[2] = packet[2] & 0x7f;
      // Nonzero speed makes accidental passive damping visible on the wire.
      reply[5] = 64;
      reply[11] = 30;
      const auto crc = Crc(reply.data(), 14);
      reply[14] = crc & 0xff;
      reply[15] = crc >> 8;
      if (corrupt_reply_motor_.load() == static_cast<int>(leg * 3 + id)) reply[14] ^= 1;
      const auto written = ::write(masters_[leg], reply.data(), reply.size());
      if (written != static_cast<ssize_t>(reply.size())) invalid_.store(true);
    }
  }
  void Respond() {
    if (!responder_cpus_.empty()) {
      cpu_set_t requested, actual;
      CPU_ZERO(&requested);
      for (int cpu : responder_cpus_) CPU_SET(cpu, &requested);
      if (::sched_setaffinity(0, sizeof(requested), &requested) != 0 ||
          ::sched_getaffinity(0, sizeof(actual), &actual) != 0) {
        invalid_.store(true);
        std::perror("test PTY responder affinity");
        return;
      }
      std::stringstream recorded;
      for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
        if (CPU_ISSET(cpu, &actual)) recorded << cpu << ',';
      }
      std::cout << "[MotorPtys] single_responder_cpus=" << recorded.str()
                << " busy_poll=" << busy_poll_ << std::endl;
    }
    std::array<pollfd, kLegCount> requests{};
    for (std::size_t leg = 0; leg < requests.size(); ++leg) {
      requests[leg].fd = masters_[leg];
      requests[leg].events = POLLIN;
    }
    while (!stopping_.load()) {
      const auto ready = ::poll(requests.data(), requests.size(), busy_poll_ ? 0 : 2);
      bool serviced = false;
      if (ready > 0) {
        for (std::size_t leg = 0; leg < requests.size(); ++leg) {
          if (requests[leg].revents & POLLIN) { Process(leg); serviced = true; }
        }
      }
      // Optional dedicated-core busy polling models an independently running
      // motor processor. It changes only this test peer, never production IO.
      if (!serviced && !busy_poll_) std::this_thread::sleep_for(50us);
    }
  }
  std::vector<int> responder_cpus_;
  bool busy_poll_ = false;
  std::array<int, 4> masters_{-1, -1, -1, -1};
  std::array<std::vector<std::uint8_t>, kLegCount> pending_;
  std::thread worker_;
  mutable std::mutex mutex_;
  std::array<Packet, kJointCount> packets_{};
  std::vector<Seen> history_;
  std::atomic<bool> stopping_{false}, invalid_{false};
  std::atomic<int> corrupt_reply_motor_{-1}, drop_reply_motor_{-1};
};

hardware_interface::HardwareInfo PtyHardwareInfo(
    const std::array<std::string, kLegCount>& ports) {
  hardware_interface::HardwareInfo info;
  info.name = "CustomDogTest";
  info.type = "system";
  info.hardware_parameters = {
      {"fr_port", ports[0]}, {"fl_port", ports[1]},
      {"rr_port", ports[2]}, {"rl_port", ports[3]},
      {"parallel_leg_io", "false"},
      {"command_timeout_s", "0.1"}};
  info.original_xml = "<robot name='gate_test'><link name='base'/>";
  for (const auto& name : kJointNames) {
    hardware_interface::ComponentInfo joint;
    joint.name = std::string(name);
    info.joints.push_back(joint);
    const std::string child = joint.name + "_link";
    info.original_xml += "<link name='" + child + "'/><joint name='" + joint.name +
        "' type='revolute'><parent link='base'/><child link='" + child +
        "'/><axis xyz='0 1 0'/><limit lower='-4' upper='4' effort='23.7' velocity='30'/></joint>";
  }
  info.original_xml += "</robot>";
  return info;
}

void CheckActivationConfirmation(bool require_confirmation, bool verified,
                                 hardware_interface::CallbackReturn expected,
                                 bool explicit_requirement = true) {
  MotorPtys motors;
  auto info = PtyHardwareInfo(motors.ports);
  info.hardware_parameters["physical_estop_verified"] = verified ? "true" : "false";
  if (explicit_requirement) {
    info.hardware_parameters["require_physical_estop_confirmation"] =
        require_confirmation ? "true" : "false";
  }
  UnitreeSystemInterface hardware;
  rclcpp_lifecycle::State lifecycle;
  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_configure(lifecycle), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_activate(lifecycle), expected);
  const auto states = hardware.export_state_interfaces();
  for (const auto& state : states) {
    if (state.get_name() == "custom_dog/motion_enabled" ||
        state.get_name() == "custom_dog/calibrated") {
      EXPECT_DOUBLE_EQ(state.get_value(), 0.0);
    }
  }
  if (expected == hardware_interface::CallbackReturn::SUCCESS) {
    ASSERT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 4000000)),
              hardware_interface::return_type::OK);
    for (const auto& packet : motors.packets()) {
      EXPECT_EQ(packet[0], 0xfe);
      EXPECT_EQ(packet[1], 0xee);
      EXPECT_EQ((packet[2] >> 4) & 0x07, 0);
      EXPECT_TRUE(std::all_of(packet.begin() + 3, packet.begin() + 15,
                             [](auto byte) { return byte == 0; }));
    }
  } else {
    // Rejected activation never starts the SDK workers or sends a motor frame.
    for (const auto& packet : motors.packets()) {
      EXPECT_TRUE(std::all_of(packet.begin(), packet.end(),
                             [](auto byte) { return byte == 0; }));
    }
  }
  EXPECT_EQ(hardware.on_deactivate(lifecycle), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(motors.valid());
}

TEST(UnitreeActivationConfirmation, DefaultRequirementRejectsUnverifiedActivation) {
  CheckActivationConfirmation(true, false, hardware_interface::CallbackReturn::ERROR, false);
}

TEST(UnitreeActivationConfirmation, RequiredVerifiedActivationRemainsDisabled) {
  CheckActivationConfirmation(true, true, hardware_interface::CallbackReturn::SUCCESS);
}

TEST(UnitreeActivationConfirmation, DesktopCanStartWithoutClaimingPhysicalStopVerified) {
  CheckActivationConfirmation(false, false, hardware_interface::CallbackReturn::SUCCESS);
}

class UnitreeMotionGate : public ::testing::Test {
 protected:
  virtual bool StartupAlignmentEnabled() const { return false; }
  virtual bool ParallelLegIoEnabled() const { return false; }
  void SetUp() override {
    auto info = PtyHardwareInfo(motors_.ports);
    if (StartupAlignmentEnabled()) info.hardware_parameters["startup_alignment_enabled"] = "true";
    if (ParallelLegIoEnabled()) info.hardware_parameters["parallel_leg_io"] = "true";
    info.hardware_parameters["physical_estop_verified"] = "true";
    ASSERT_EQ(hardware_.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
    commands_ = hardware_.export_command_interfaces();
    states_ = hardware_.export_state_interfaces();
    for (const auto& command : commands_) names_.push_back(command.get_name());
    ASSERT_EQ(hardware_.on_configure(lifecycle_), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(hardware_.on_activate(lifecycle_), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  }
  void TearDown() override {
    hardware_.on_deactivate(lifecycle_);
    EXPECT_TRUE(motors_.valid());
  }
  void Set(const std::string& name, double value) {
    for (auto& interface : commands_) {
      if (interface.get_name() == name) {
        interface.set_value(value);
        return;
      }
    }
    FAIL() << "missing command " << name;
  }
  double State(const std::string& name) {
    for (const auto& interface : states_) {
      if (interface.get_name() == name) return interface.get_value();
    }
    ADD_FAILURE() << "missing state " << name;
    return -1.0;
  }
  hardware_interface::return_type Read() { return hardware_.read(time_, period_); }
  hardware_interface::return_type Write() { return hardware_.write(time_, period_); }
  void CalibrateAndClaim() {
    ASSERT_EQ(hardware_.perform_command_mode_switch(names_, {}), hardware_interface::return_type::OK);
    Set("custom_dog/calibrate", 1.0);
    // Calibration deliberately waits for one complete valid feedback cycle.
    // PTY responder threads share an ordinary scheduler with physics/tests, so
    // a missed 2 ms receive deadline may need the same next-frame retry as the
    // real driver. This only prepares the fixture; fault injections below are
    // each asserted once, and production receive timeouts stay unchanged.
    for (int attempt = 0; attempt < 10 && State("custom_dog/calibrated") < 0.5; ++attempt) {
      ASSERT_EQ(Write(), hardware_interface::return_type::OK);
      ASSERT_EQ(Read(), hardware_interface::return_type::OK);
      ExpectDisabled();
      if (State("custom_dog/calibrated") < 0.5) std::this_thread::sleep_for(1ms);
    }
    ASSERT_DOUBLE_EQ(State("custom_dog/calibrated"), 1.0);
    Set("custom_dog/calibrate", 0.0);
  }
  void EnableTorque() {
    for (const auto& name : kJointNames) {
      Set(std::string(name) + "/effort", 1000.0);
      Set(std::string(name) + "/kd", 0.0);
    }
    Set("custom_dog/motion_enabled", 1.0);
    ASSERT_EQ(Write(), hardware_interface::return_type::OK);
    ASSERT_EQ(Read(), hardware_interface::return_type::OK);
    ASSERT_DOUBLE_EQ(State("custom_dog/motion_enabled"), 1.0);
    ASSERT_DOUBLE_EQ(State("custom_dog/communication_ok"), 1.0);
    for (const auto& name : kJointNames) {
      ASSERT_DOUBLE_EQ(State(std::string(name) + "/valid"), 1.0);
    }
  }
  void ExpectDisabled() {
    EXPECT_DOUBLE_EQ(State("custom_dog/motion_enabled"), 0.0);
    for (const auto& packet : motors_.packets()) {
      EXPECT_EQ((packet[2] >> 4) & 0x07, 0);
      EXPECT_TRUE(std::all_of(packet.begin() + 3, packet.begin() + 15,
                             [](auto byte) { return byte == 0; }));
    }
  }
  MotorPtys motors_;
  UnitreeSystemInterface hardware_;
  std::vector<hardware_interface::CommandInterface> commands_;
  std::vector<hardware_interface::StateInterface> states_;
  std::vector<std::string> names_;
  rclcpp_lifecycle::State lifecycle_;
  rclcpp::Time time_{0};
  rclcpp::Duration period_{0, 4000000};
};

TEST_F(UnitreeMotionGate, UncalibratedAndCalibratingRemainDisabled) {
  ASSERT_EQ(hardware_.perform_command_mode_switch(names_, {}), hardware_interface::return_type::OK);
  Set("custom_dog/motion_enabled", 1.0);
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  ExpectDisabled();
  Set("custom_dog/calibrate", 1.0);
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  ASSERT_DOUBLE_EQ(State("custom_dog/calibrated"), 1.0);
  ExpectDisabled();
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  ExpectDisabled();
}

TEST_F(UnitreeMotionGate, PassiveWithNonzeroFeedbackVelocityNeverEnablesDrive) {
  CalibrateAndClaim();
  EXPECT_NE(State("FR_hip_joint/velocity"), 0.0);
  for (const auto& name : kJointNames) Set(std::string(name) + "/kd", 1.0);
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  ExpectDisabled();
}

TEST_F(UnitreeMotionGate, PureFeedforwardIsBoundedAndZeroSwitchStops) {
  CalibrateAndClaim();
  EnableTorque();
  const DriveParameters parameters;
  const auto packets = motors_.packets();
  for (std::size_t i = 0; i < packets.size(); ++i) {
    const auto& packet = packets[i];
    EXPECT_EQ((packet[2] >> 4) & 0x07, 1);
    EXPECT_TRUE(std::all_of(packet.begin() + 5, packet.begin() + 15,
                           [](auto byte) { return byte == 0; }));
    const auto raw = static_cast<std::int16_t>(packet[3] | (packet[4] << 8));
    const double rotor_torque = raw / 256.0;
    const double gear = i % 3 == 2 ? parameters.calf_total_gear_ratio : parameters.base_gear_ratio;
    const double joint_torque = rotor_torque * gear * parameters.motor_directions[i];
    EXPECT_GT(joint_torque, 23.7 - gear / 256.0);
    EXPECT_LE(joint_torque, 23.7);
  }
  Set("custom_dog/motion_enabled", 0.0);
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  ExpectDisabled();
}

TEST_F(UnitreeMotionGate, FiveMotorPdFieldsPreserveLegacyGearDirectionAndCalibration) {
  CalibrateAndClaim();
  const DriveParameters parameters;
  std::array<float, kJointCount> target_positions{}, target_velocities{};
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const std::string name(kJointNames[i]);
    const auto joint = i % kJointsPerLeg;
    ASSERT_NEAR(State(name + "/position"), parameters.calibration_pose[joint], 2e-7);
    target_positions[i] = static_cast<float>(parameters.calibration_pose[joint] + 0.04);
    target_velocities[i] = i % 2 == 0 ? 0.125F : -0.125F;
    Set(name + "/position", target_positions[i]);
    Set(name + "/velocity", target_velocities[i]);
    Set(name + "/effort", 1.25);
    Set(name + "/kp", 24.0);
    Set(name + "/kd", 0.75);
  }
  Set("custom_dog/motion_enabled", 1.0);
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  ASSERT_DOUBLE_EQ(State("custom_dog/communication_ok"), 1.0);
  const auto packets = motors_.packets();
  for (std::size_t i = 0; i < kJointCount; ++i) {
    SCOPED_TRACE(std::string(kJointNames[i]));
    const auto joint = i % kJointsPerLeg;
    const float gear = static_cast<float>(joint == 2
        ? parameters.calf_total_gear_ratio : parameters.base_gear_ratio);
    const float direction = static_cast<float>(parameters.motor_directions[i]);
    const float calibration_pose = static_cast<float>(parameters.calibration_pose[joint]);
    // PTYs report raw rotor position zero. The START zero offset therefore
    // maps the known prone pose back to zero, as in the legacy IOSDK contract.
    MotorCmd expected;
    expected.motorType = MotorType::GO_M8010_6;
    expected.mode = 1;
    expected.id = joint;
    expected.q = static_cast<float>(
        (static_cast<double>(target_positions[i]) - calibration_pose) * gear * direction);
    expected.dq = target_velocities[i] * gear * direction;
    expected.tau = (1.25F / gear) * direction;
    expected.kp = 24.0F / (gear * gear);
    expected.kd = 0.75F / (gear * gear);
    expected.hex_len = 17;
    std::memset(expected.get_motor_send_data(), 0, 17);
    expected.modify_data(&expected);
    MotorPtys::Packet expected_packet{};
    std::copy_n(expected.get_motor_send_data(), expected_packet.size(), expected_packet.begin());
    EXPECT_EQ(packets[i], expected_packet);
    EXPECT_EQ((packets[i][2] >> 4) & 0x07, 1);
    EXPECT_NE(packets[i][3] | (packets[i][4] << 8), 0);  // FF torque
    EXPECT_NE(packets[i][5] | (packets[i][6] << 8), 0);  // velocity
    EXPECT_TRUE(std::any_of(packets[i].begin() + 7, packets[i].begin() + 11,
                            [](auto byte) { return byte != 0; }));  // position
    EXPECT_NE(packets[i][11] | (packets[i][12] << 8), 0);  // Kp
    EXPECT_NE(packets[i][13] | (packets[i][14] << 8), 0);  // Kd
    EXPECT_EQ(Crc(packets[i].data(), 15), packets[i][15] | (packets[i][16] << 8));
  }
}

TEST_F(UnitreeMotionGate, BadFeedbackCrcDisablesTheNextWholeMotorPdFrame) {
  CalibrateAndClaim();
  EnableTorque();
  motors_.CorruptReplyFor(11);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(State("custom_dog/communication_ok"), 0.0);
  EXPECT_DOUBLE_EQ(State("RL_calf_joint/valid"), 0.0);
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  motors_.CorruptReplyFor(-1);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  ExpectDisabled();
}

TEST_F(UnitreeMotionGate, InvalidLastJointCommandStopsAllMotorsWithoutSendingFoc) {
  CalibrateAndClaim();
  EnableTorque();
  Set("RL_calf_joint/position", std::numeric_limits<double>::quiet_NaN());
  ASSERT_EQ(Write(), hardware_interface::return_type::ERROR);
  ExpectDisabled();
}

TEST_F(UnitreeMotionGate, ExplicitEnableDoesNotInferAuthorizationFromZeroTorque) {
  CalibrateAndClaim();
  for (const auto& name : kJointNames) {
    Set(std::string(name) + "/kp", 0.0);
    Set(std::string(name) + "/kd", 0.0);
    Set(std::string(name) + "/effort", 0.0);
  }
  Set("custom_dog/motion_enabled", 1.0);
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(State("custom_dog/motion_enabled"), 1.0);
  EXPECT_DOUBLE_EQ(State("custom_dog/communication_ok"), 1.0);
  for (const auto& packet : motors_.packets()) {
    EXPECT_EQ((packet[2] >> 4) & 0x07, 1);
    EXPECT_TRUE(std::all_of(packet.begin() + 3, packet.begin() + 15,
                           [](auto byte) { return byte == 0; }));
  }
}

TEST_F(UnitreeMotionGate, ReleasingAnyCommandDiscardsLatchedTorqueAndRequiresNewEnable) {
  CalibrateAndClaim();
  EnableTorque();
  ASSERT_EQ(hardware_.perform_command_mode_switch({}, {"FR_hip_joint/effort"}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  ExpectDisabled();
  Set("custom_dog/motion_enabled", 1.0);
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  ExpectDisabled();
  ASSERT_EQ(hardware_.perform_command_mode_switch({"FR_hip_joint/effort"}, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  ExpectDisabled();
}

TEST_F(UnitreeMotionGate, EmergencyStopDisablesDriveInsteadOfApplyingDamping) {
  CalibrateAndClaim();
  EnableTorque();
  Set("custom_dog/emergency_stop", 1.0);
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  ExpectDisabled();
}

TEST_F(UnitreeMotionGate, InvalidMotionValueStopsAndRejectsWrite) {
  CalibrateAndClaim();
  EnableTorque();
  Set("custom_dog/motion_enabled", 0.5);
  ASSERT_EQ(Write(), hardware_interface::return_type::ERROR);
  ExpectDisabled();
}

TEST_F(UnitreeMotionGate, NanMotionValueStopsAndRejectsWrite) {
  CalibrateAndClaim();
  EnableTorque();
  Set("custom_dog/motion_enabled", std::numeric_limits<double>::quiet_NaN());
  ASSERT_EQ(Write(), hardware_interface::return_type::ERROR);
  ExpectDisabled();
}

TEST_F(UnitreeMotionGate, DeactivateClearsMotionAndSendsStop) {
  CalibrateAndClaim();
  EnableTorque();
  ASSERT_EQ(hardware_.on_deactivate(lifecycle_), hardware_interface::CallbackReturn::SUCCESS);
  ExpectDisabled();
  ASSERT_EQ(hardware_.on_activate(lifecycle_), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  ExpectDisabled();
}

// The profile's absolute deadlines are checked independently of scheduler timing.
TEST(StartupAlignmentDeadlines, LateCycleSkipsForceAndFailedReleaseCannotFinish) {
  StartupAlignment alignment(true);
  alignment.BeginCycle(1.0, false, false);
  EXPECT_EQ(alignment.stage(), StartupAlignment::Stage::WAITING);
  alignment.BeginCycle(1.0, true, true);
  ASSERT_EQ(alignment.stage(), StartupAlignment::Stage::HIP_THIGH);
  alignment.FinishCycle(true);
  alignment.BeginCycle(1.7, true, true);
  ASSERT_EQ(alignment.stage(), StartupAlignment::Stage::RELEASE);
  for (const auto& motor : alignment.command().motors) {
    EXPECT_EQ(motor.mode, 0u);
    EXPECT_FLOAT_EQ(motor.effort, 0.0F);
  }
  alignment.FinishCycle(true);
  alignment.BeginCycle(2.0, true, true);
  alignment.FinishCycle(false);
  EXPECT_EQ(alignment.stage(), StartupAlignment::Stage::ABORTED);
  EXPECT_FALSE(alignment.canCalibrate());
  alignment.BeginCycle(3.0, true, true);
  EXPECT_EQ(alignment.stage(), StartupAlignment::Stage::ABORTED);
}

class UnitreeStartupAlignment : public UnitreeMotionGate {
 protected:
  bool StartupAlignmentEnabled() const override { return true; }
  void BeginAlignment() {
    ASSERT_DOUBLE_EQ(State("custom_dog/startup_alignment"), 1.0);
    ASSERT_EQ(hardware_.perform_command_mode_switch(names_, {}), hardware_interface::return_type::OK);
    ASSERT_EQ(Write(), hardware_interface::return_type::OK);
    ASSERT_EQ(Read(), hardware_interface::return_type::OK);
    ASSERT_DOUBLE_EQ(State("custom_dog/startup_alignment"), 2.0);
    ASSERT_DOUBLE_EQ(State("custom_dog/calibrated"), 0.0);
    ASSERT_DOUBLE_EQ(State("custom_dog/motion_enabled"), 0.0);
  }
  void ExpectLegacyProfile(bool calf_enabled) {
    const DriveParameters drive;
    const std::array<std::array<float, 3>, 4> legacy = {{{0.95F, 0.63F, -1.52F},
        {-0.95F, -0.63F, 1.52F}, {-0.95F, 0.63F, -1.52F}, {0.95F, -0.63F, 1.52F}}};
    const auto packets = motors_.packets();
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto joint = i % kJointsPerLeg;
      const float gear = static_cast<float>(joint == 2 ? drive.calf_total_gear_ratio : drive.base_gear_ratio);
      MotorCmd expected;
      expected.motorType = MotorType::GO_M8010_6;
      expected.mode = 1;
      expected.id = joint;
      expected.q = expected.dq = expected.kp = expected.kd = 0.0F;
      expected.tau = (joint == 2 && !calf_enabled) ? 0.0F : legacy[i / 3][joint] / gear;
      expected.hex_len = 17;
      std::memset(expected.get_motor_send_data(), 0, 17);
      expected.modify_data(&expected);
      MotorPtys::Packet bytes;
      std::copy_n(expected.get_motor_send_data(), bytes.size(), bytes.begin());
      EXPECT_EQ(packets[i], bytes) << "legacy rotor command differs for " << kJointNames[i];
    }
  }
};

TEST_F(UnitreeStartupAlignment, ExternalUncalibratedCommandsCannotBypassTheFixedProfile) {
  for (const auto& name : kJointNames) {
    Set(std::string(name) + "/position", 2.0);
    Set(std::string(name) + "/effort", 1000.0);
    Set(std::string(name) + "/kp", 100.0);
    Set(std::string(name) + "/kd", 2.0);
  }
  Set("custom_dog/motion_enabled", 1.0);
  Set("custom_dog/calibrate", 1.0);
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  ExpectDisabled();
  EXPECT_DOUBLE_EQ(State("custom_dog/startup_alignment"), 1.0);
  EXPECT_DOUBLE_EQ(State("custom_dog/calibrated"), 0.0);
  ASSERT_EQ(hardware_.perform_command_mode_switch(names_, {}), hardware_interface::return_type::OK);
  Set("custom_dog/motion_enabled", 1.0);
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  ExpectDisabled();
  EXPECT_DOUBLE_EQ(State("custom_dog/startup_alignment"), 1.0);
  Set("custom_dog/motion_enabled", 0.0);
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  ASSERT_DOUBLE_EQ(State("custom_dog/startup_alignment"), 2.0);
  ExpectLegacyProfile(false);
  EXPECT_DOUBLE_EQ(State("custom_dog/calibrated"), 0.0);
}

TEST_F(UnitreeStartupAlignment, AllPhasesKeepLegacyBytesAndCalibrationWaitsForCompletion) {
  BeginAlignment();
  ExpectLegacyProfile(false);
  Set("custom_dog/calibrate", 1.0);  // Held early request must not be applied on the finishing cycle.
  bool saw_all = false, saw_release = false;
  const auto deadline = std::chrono::steady_clock::now() + 1500ms;
  while (State("custom_dog/startup_alignment") != 5.0 &&
         std::chrono::steady_clock::now() < deadline) {
    ASSERT_EQ(Write(), hardware_interface::return_type::OK);
    ASSERT_EQ(Read(), hardware_interface::return_type::OK);
    const auto stage = State("custom_dog/startup_alignment");
    ASSERT_NE(stage, 6.0);
    if (stage == 2.0) ExpectLegacyProfile(false);
    if (stage == 3.0) { saw_all = true; ExpectLegacyProfile(true); }
    if (stage == 4.0) { saw_release = true; ExpectDisabled(); }
    EXPECT_DOUBLE_EQ(State("custom_dog/calibrated"), 0.0);
    EXPECT_DOUBLE_EQ(State("custom_dog/motion_enabled"), 0.0);
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_DOUBLE_EQ(State("custom_dog/startup_alignment"), 5.0);
  EXPECT_TRUE(saw_all);
  EXPECT_TRUE(saw_release);
  ExpectDisabled();
  Set("custom_dog/calibrate", 0.0);
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(State("custom_dog/calibrated"), 0.0);
  CalibrateAndClaim();
  EXPECT_DOUBLE_EQ(State("custom_dog/calibrated"), 1.0);
}

enum class AlignmentAbortCause { CRC, MISSING, ESTOP, STALE_WRITE, RELEASE_CLAIM, DEACTIVATE, RESET };
class UnitreeStartupAbort : public UnitreeStartupAlignment,
                           public ::testing::WithParamInterface<AlignmentAbortCause> {};

TEST_P(UnitreeStartupAbort, AbortStopsEveryLegAndCannotRetryOrCalibrate) {
  BeginAlignment();
  ExpectLegacyProfile(false);
  motors_.ClearHistory();
  switch (GetParam()) {
    case AlignmentAbortCause::CRC: motors_.CorruptReplyFor(0); break;
    case AlignmentAbortCause::MISSING: motors_.DropReplyFor(0); break;
    case AlignmentAbortCause::ESTOP:
      Set("custom_dog/emergency_stop", 1.0);
      ASSERT_EQ(Write(), hardware_interface::return_type::OK);
      break;
    case AlignmentAbortCause::STALE_WRITE: std::this_thread::sleep_for(120ms); break;
    case AlignmentAbortCause::RELEASE_CLAIM:
      ASSERT_EQ(hardware_.perform_command_mode_switch({}, {"FR_hip_joint/effort"}),
                hardware_interface::return_type::OK);
      break;
    case AlignmentAbortCause::DEACTIVATE:
      ASSERT_EQ(hardware_.on_deactivate(lifecycle_), hardware_interface::CallbackReturn::SUCCESS);
      EXPECT_DOUBLE_EQ(State("custom_dog/startup_alignment"), 6.0);
      ExpectDisabled();
      ASSERT_EQ(hardware_.on_activate(lifecycle_), hardware_interface::CallbackReturn::SUCCESS);
      break;
    case AlignmentAbortCause::RESET:
      Set("custom_dog/calibrate", -1.0);
      ASSERT_EQ(Write(), hardware_interface::return_type::OK);
      break;
  }
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(State("custom_dog/startup_alignment"), 6.0);
  EXPECT_DOUBLE_EQ(State("custom_dog/calibrated"), 0.0);
  ExpectDisabled();
  if (GetParam() == AlignmentAbortCause::CRC || GetParam() == AlignmentAbortCause::MISSING) {
    const auto history = motors_.history();
    ASSERT_GT(history.size(), 1u);
    EXPECT_EQ(history.front().index, 0u);
    EXPECT_EQ((history.front().packet[2] >> 4) & 0x07, 1);
    // The serial fixture is deliberately sequential: after the first failed
    // exchange every subsequent packet in this same round must already stop.
    for (std::size_t i = 1; i < history.size(); ++i) {
      EXPECT_EQ((history[i].packet[2] >> 4) & 0x07, 0);
      EXPECT_TRUE(std::all_of(history[i].packet.begin() + 3, history[i].packet.begin() + 15,
                             [](auto byte) { return byte == 0; }));
    }
  }
  motors_.CorruptReplyFor(-1);
  motors_.DropReplyFor(-1);
  Set("custom_dog/emergency_stop", 0.0);
  Set("custom_dog/motion_enabled", 0.0);
  Set("custom_dog/calibrate", 0.0);
  ASSERT_EQ(hardware_.perform_command_mode_switch(names_, {}), hardware_interface::return_type::OK);
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(State("custom_dog/startup_alignment"), 6.0);
  ExpectDisabled();
  Set("custom_dog/calibrate", 1.0);
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(State("custom_dog/calibrated"), 0.0);
  ExpectDisabled();
}

INSTANTIATE_TEST_SUITE_P(RequiredStops, UnitreeStartupAbort, ::testing::Values(
    AlignmentAbortCause::CRC, AlignmentAbortCause::MISSING, AlignmentAbortCause::ESTOP,
    AlignmentAbortCause::STALE_WRITE, AlignmentAbortCause::RELEASE_CLAIM,
    AlignmentAbortCause::DEACTIVATE, AlignmentAbortCause::RESET));

class UnitreeParallelStartupAlignment : public UnitreeStartupAlignment {
 protected:
  bool ParallelLegIoEnabled() const override { return true; }
};

TEST_F(UnitreeParallelStartupAlignment, AnyWorkerCrcFailureCancelsAndStopsAllFourLegs) {
  BeginAlignment();
  ExpectLegacyProfile(false);
  motors_.CorruptReplyFor(0);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(State("custom_dog/startup_alignment"), 6.0);
  EXPECT_DOUBLE_EQ(State("custom_dog/communication_ok"), 0.0);
  ExpectDisabled();
  motors_.CorruptReplyFor(-1);
  ASSERT_EQ(Write(), hardware_interface::return_type::OK);
  ASSERT_EQ(Read(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(State("custom_dog/startup_alignment"), 6.0);
  ExpectDisabled();
}

}  // namespace
}  // namespace custom_dog_control
