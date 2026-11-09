#ifdef COMPILE_WITH_REAL_ROBOT

#include "custom_dog_control/hardware/IOSDK.hpp"

#include <cmath>
#include <chrono>
#include <cstring>
#include <cerrno>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {

constexpr const char* LEG_NAMES[4] = {"FR", "FL", "RR", "RL"};
constexpr const char* JOINT_NAMES[3] = {"hip", "thigh", "calf"};

}  // namespace

namespace custom_dog_control {

// The SDK keeps its fd private and opens it in its constructor. Keep a second
// fd for advisory ownership first, then set TIOCEXCL after the SDK's own open.
// The /proc check also catches older controllers that never used either lock.
class IOSDK::SerialPortOwnership final {
 public:
  explicit SerialPortOwnership(const std::string& port) : port_(port) {
    descriptor_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (descriptor_ < 0) Fail("open", errno);
    try {
      struct stat status{};
      if (::fstat(descriptor_, &status) != 0) Fail("fstat", errno);
      if (!S_ISCHR(status.st_mode) || ::isatty(descriptor_) != 1) {
        throw std::runtime_error("motor port is not a tty: " + port_);
      }
      device_ = status.st_rdev;
      if (::flock(descriptor_, LOCK_EX | LOCK_NB) != 0) Fail("flock", errno);
      CheckNoOtherOwners(1);
    } catch (...) {
      ::close(descriptor_);
      descriptor_ = -1;
      throw;
    }
  }

  ~SerialPortOwnership() {
    if (descriptor_ >= 0) {
      if (exclusive_) ::ioctl(descriptor_, TIOCNXCL);
      ::close(descriptor_);
    }
  }

  std::unique_ptr<SerialPort> OpenSdk() const {
    try {
      return std::make_unique<SerialPort>(
          port_, 16, 4000000, 2000, BlockYN::NO,
          bytesize_t::eightbits, parity_t::parity_none,
          stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none);
    } catch (...) {
      // This SDK version leaks its private fd if configuration throws after
      // open(). Before construction our guard was the only same-device fd.
      // IOSDK construction/lifecycle must not race other same-port operations.
      try {
        for (int fd : DeviceDescriptors("/proc/self/fd")) {
          if (fd != descriptor_) ::close(fd);
        }
      } catch (...) {}
      throw;
    }
  }

  void MakeExclusive() {
    if (::ioctl(descriptor_, TIOCEXCL) != 0) Fail("TIOCEXCL", errno);
    exclusive_ = true;
    // Detect an uncooperative opener that raced the SDK constructor. Future
    // ordinary opens now fail in the kernel, including opens through aliases.
    CheckNoOtherOwners(2);
  }

 private:
  [[noreturn]] void Fail(const char* operation, int error) const {
    throw std::runtime_error(std::string(operation) + " " + port_ + ": " +
                             std::strerror(error));
  }

  std::vector<int> DeviceDescriptors(const std::filesystem::path& directory) const {
    std::error_code error;
    std::filesystem::directory_iterator entry(directory, error), end;
    if (error) throw std::system_error(error, directory.string());
    std::vector<int> descriptors;
    for (; entry != end; entry.increment(error)) {
      if (error) throw std::system_error(error, directory.string());
      struct stat status{};
      if (::stat(entry->path().c_str(), &status) == 0 &&
          S_ISCHR(status.st_mode) && status.st_rdev == device_) {
        descriptors.push_back(std::stoi(entry->path().filename().string()));
      }
    }
    return descriptors;
  }

  void CheckNoOtherOwners(std::size_t expected_self_fds) const {
    const auto self = std::to_string(::getpid());
    if (DeviceDescriptors("/proc/self/fd").size() != expected_self_fds) {
      throw std::runtime_error("motor port already open in this process: " + port_);
    }
    for (const auto& process : std::filesystem::directory_iterator("/proc")) {
      const std::string pid = process.path().filename().string();
      if (pid == self || pid.find_first_not_of("0123456789") != std::string::npos) continue;
      std::vector<int> descriptors;
      try {
        descriptors = DeviceDescriptors(process.path() / "fd");
      } catch (const std::system_error& error) {
        // Other users' fds may be hidden by procfs permissions; PID exits also
        // race this read-only scan. Kernel exclusivity still protects new opens.
        if (error.code().value() == EACCES || error.code().value() == EPERM ||
            error.code().value() == ENOENT || error.code().value() == ESRCH) continue;
        throw;
      }
      if (!descriptors.empty()) {
        throw std::runtime_error("motor port already open by pid=" + pid +
                                 " fd=" + std::to_string(descriptors.front()) +
                                 ": " + port_);
      }
    }
  }

  std::string port_;
  int descriptor_ = -1;
  dev_t device_{};
  bool exclusive_ = false;
};

IOSDK::IOSDK(const DriveParameters& drive_parameters)
    : serial_ports_(drive_parameters.serial_ports),
      base_gear_ratio_(static_cast<float>(drive_parameters.base_gear_ratio)),
      calf_total_gear_ratio_(static_cast<float>(drive_parameters.calf_total_gear_ratio)),
      use_parallel_leg_io_(drive_parameters.use_parallel_leg_io),
      calibration_pose_(drive_parameters.calibration_pose),
      startup_alignment_(drive_parameters.startup_alignment_enabled) {
    calibration_offsets_.fill(0.0f);
    for (std::size_t i = 0; i < motor_directions_.size(); ++i) {
        motor_directions_[i] = static_cast<float>(drive_parameters.motor_directions[i]);
    }
    active_legs_ = {0, 1, 2, 3};

    std::cout << "[IOSDK] 初始化完成" << std::endl;
    std::cout << "[IOSDK] 电机串口配置"
              << " FR=" << serial_ports_[0]
              << " FL=" << serial_ports_[1]
              << " RR=" << serial_ports_[2]
              << " RL=" << serial_ports_[3]
              << std::endl;
    std::cout << "[IOSDK] 腿部通信模式="
              << (use_parallel_leg_io_ ? "parallel_4workers" : "single_thread")
              << std::endl;

    OpenSerialPorts();
    InitializeMotorMetadata();

}

IOSDK::~IOSDK() {
    Deactivate();
}

void IOSDK::Activate() {
    if (active_) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        workers_stopping_ = false;
        dispatch_epoch_ = 0;
        completed_workers_ = 0;
        active_command_ = nullptr;
        active_state_ = nullptr;
    }
    StartWorkers();
    active_ = true;
}

void IOSDK::Deactivate() {
    CancelStartupAlignment();
    have_valid_feedback_ = false;
    if (!active_) {
        return;
    }
    StopWorkers();
    const auto zero_cmd = BuildZeroTorqueCommand();
    for (int leg : active_legs_) {
        SendDirectLegCommand(leg, zero_cmd);
    }
    active_ = false;
}

void IOSDK::LogStartupAlignmentTransition(StartupAlignment::Stage previous) const {
    const auto stage = startup_alignment_.stage();
    if (stage == previous) return;
    switch (stage) {
        case StartupAlignment::Stage::HIP_THIGH:
            std::cout << "[IOSDK] 启动小力矩预对位：髋/大腿阶段（300 ms）" << std::endl;
            break;
        case StartupAlignment::Stage::ALL_JOINTS:
            std::cout << "[IOSDK] 小力矩预对位：加入小腿（300 ms）" << std::endl;
            break;
        case StartupAlignment::Stage::RELEASE:
            std::cout << "[IOSDK] 小力矩预对位：停机释放（300 ms）" << std::endl;
            break;
        case StartupAlignment::Stage::DONE:
            std::cout << "[IOSDK] 预对位完成，请松开后重新按 START 采集标定零位" << std::endl;
            break;
        case StartupAlignment::Stage::ABORTED:
            std::cout << "[IOSDK][ERROR] 预对位中止，退出小力矩输出；需重新配置或重启后重试"
                      << std::endl;
            break;
        default: break;
    }
}

void IOSDK::CancelStartupAlignment() {
    startup_cycle_cancelled_.store(true);
    const auto previous = startup_alignment_.stage();
    startup_alignment_.Cancel();
    LogStartupAlignmentTransition(previous);
}

void IOSDK::OpenSerialPorts() {
    for (int leg = 0; leg < 4; ++leg) {
        try {
            auto ownership = std::make_unique<SerialPortOwnership>(serial_ports_[leg]);
            auto serial = ownership->OpenSdk();
            ownership->MakeExclusive();
            serials_.push_back(std::move(serial));
            port_ownership_[leg] = std::move(ownership);
            std::cout << "[IOSDK] 电机串口已打开"
                      << " leg=" << LEG_NAMES[leg]
                      << " port=" << serial_ports_[leg]
                      << " baud=4000000"
                      << " receive_timeout_us=2000 (raw SDK/select path)" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[IOSDK][ERROR] 电机串口打开失败"
                      << " leg=" << LEG_NAMES[leg]
                      << " port=" << serial_ports_[leg]
                      << " reason=" << e.what()
                      << std::endl;
            throw;
        }
    }
}

void IOSDK::InitializeMotorMetadata() {
    for (int i = 0; i < 12; ++i) {
        motor_commands_[i].motorType = MotorType::GO_M8010_6;
        motor_commands_[i].mode = static_cast<unsigned short>(MotorMode::FOC);
        motor_commands_[i].hex_len = 17;

        motor_data_[i].motorType = MotorType::GO_M8010_6;
        motor_data_[i].hex_len = 16;
    }
}

std::array<ActuatorCommand, 3> IOSDK::BuildZeroTorqueCommand() const {
    std::array<ActuatorCommand, 3> cmds;
    for (auto& cmd : cmds) {
        cmd.mode = static_cast<std::uint32_t>(ActuatorControlMode::DISABLE);
    }
    return cmds;
}

void IOSDK::SendDirectLegCommand(int leg, const std::array<ActuatorCommand, 3>& user_cmds) {
    SerialPort* serial = serials_[leg].get();
    if (serial == nullptr) {
        return;
    }

    for (int joint = 0; joint < 3; ++joint) {
        PopulateMotorCommand(leg, joint, user_cmds[joint]);
        if (!ExchangeMotor(leg, joint)) {
            std::cerr << "[IOSDK][WARN] 停止命令未收到有效回包"
                      << " leg=" << LEG_NAMES[leg]
                      << " joint=" << JOINT_NAMES[joint]
                      << " port=" << serial_ports_[leg]
                      << std::endl;
        }
    }
}

void IOSDK::StartWorkers() {
    if (!use_parallel_leg_io_) {
        return;
    }

    try {
        for (int leg = 0; leg < 4; ++leg) {
            workers_[leg] = std::thread(&IOSDK::WorkerLoop, this, leg);
        }
    } catch (...) {
        StopWorkers();
        throw;
    }
}

void IOSDK::StopWorkers() {
    if (!use_parallel_leg_io_) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        workers_stopping_ = true;
    }
    dispatch_cv_.notify_all();
    completed_cv_.notify_all();

    for (auto& worker : workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}

void IOSDK::WorkerLoop(int leg) {
    std::size_t local_epoch = 0;

    while (true) {
        const LowLevelCommand* cmd = nullptr;
        LowLevelState* state = nullptr;
        bool internal_alignment_frame = false;

        {
            std::unique_lock<std::mutex> lock(worker_mutex_);
            dispatch_cv_.wait(lock, [this, &local_epoch] {
                return workers_stopping_ || dispatch_epoch_ != local_epoch;
            });

            if (workers_stopping_) {
                return;
            }

            local_epoch = dispatch_epoch_;
            cmd = active_command_;
            state = active_state_;
            internal_alignment_frame = active_alignment_frame_;
        }

        const bool leg_active = (active_legs_.find(leg) != active_legs_.end());
        if (leg_active) {
            try {
                SendReceiveLeg(leg, *cmd, *state, internal_alignment_frame);
            } catch (...) {
                for (int joint = 0; joint < 3; ++joint) {
                    MarkMotorOffline(leg, joint, *state);
                }
            }
        }

        if (leg_active) {
            std::lock_guard<std::mutex> lock(worker_mutex_);
            ++completed_workers_;
            if (completed_workers_ >= active_legs_.size()) {
                completed_cv_.notify_one();
            }
        }
    }
}

float IOSDK::GearRatioForJoint(int joint) const {
    return (joint == 2) ? calf_total_gear_ratio_ : base_gear_ratio_;
}

void IOSDK::CalibrateLeg(int leg) {
    for (int joint = 0; joint < 3; ++joint) {
        const int motor_id = leg * 3 + joint;
        const float gear = GearRatioForJoint(joint);
        const float dir = motor_directions_[motor_id];
        const double q_urdf_uncalibrated = MotorPositionToUncalibratedUrdf(
            motor_data_[motor_id].q, gear, dir);
        calibration_offsets_[motor_id] = static_cast<float>(
            CalibrationOffset(q_urdf_uncalibrated, joint, calibration_pose_));
    }
}

void IOSDK::TryCalibrate(bool calibration_requested) {
    if (calibrated_ || !calibration_requested) {
        return;
    }

    std::cout << "[IOSDK] 检测到 START，开始校准" << std::endl;
    for (int leg : active_legs_) {
        CalibrateLeg(leg);
    }
    calibrated_ = true;
    std::cout << "[IOSDK] 校准完成" << std::endl;
}

void IOSDK::FinalizeCycleCalibration(
    bool calibration_requested, LowLevelState& state) {
    // 只有整轮无通信失败才采纳标定。零点改变后重算同一批回包的关节位置，
    // 避免 calibrated 已置位、输出却仍沿用旧零点的一个周期不一致。
    const int failures = current_cycle_failures_.load(std::memory_order_relaxed);
    last_cycle_failures_.store(failures, std::memory_order_release);
    const bool was_calibrated = calibrated_;
    if (failures == 0) {
        TryCalibrate(calibration_requested);
    }
    if (!was_calibrated && calibrated_) {
        for (int leg : active_legs_) {
            for (int joint = 0; joint < static_cast<int>(kJointsPerLeg); ++joint) {
                UpdateMotorStateFromFeedback(leg, joint, state);
            }
        }
    }
}

void IOSDK::PopulateMotorCommand(int leg, int joint, const ActuatorCommand& user_cmd,
                                bool internal_alignment_frame) {
    const int motor_id = leg * 3 + joint;
    const float gear = GearRatioForJoint(joint);
    const float gear_sq = gear * gear;
    // 角度/速度乘减速比，力矩除减速比；PD 增益除减速比平方，
    // 才能保持输出侧的等效刚度与阻尼。calf 使用包含额外传动的总减速比。
    const float dir = motor_directions_[leg * 3 + joint];
    auto& motor_cmd = motor_commands_[motor_id];
    motor_cmd.id = joint;
    motor_cmd.q = user_cmd.kp == 0.0F ? 0.0F : static_cast<float>(UrdfPositionToMotor(
        user_cmd.position,
        calibrated_ ? calibration_offsets_[motor_id] : 0.0F,
        gear, dir));
    motor_cmd.dq = user_cmd.kd == 0.0F ? 0.0F : user_cmd.velocity * gear * dir;
    motor_cmd.kp = user_cmd.kp / gear_sq;
    motor_cmd.kd = user_cmd.kd / gear_sq;
    motor_cmd.tau = (user_cmd.effort / gear) * dir;
    motor_cmd.mode = user_cmd.mode;
    if (!calibrated_) {
        motor_cmd.q = motor_cmd.dq = motor_cmd.kp = motor_cmd.kd = 0.0F;
        // Only ExchangeCycle's internally generated, fixed profile receives
        // this private authorization. Public caller commands never do.
        if (!internal_alignment_frame) {
            motor_cmd.tau = 0.0F;
            motor_cmd.mode = static_cast<unsigned short>(ActuatorControlMode::DISABLE);
        }
    }
}

void IOSDK::UpdateMotorStateFromFeedback(int leg, int joint, LowLevelState& state) {
    const int motor_id = leg * 3 + joint;
    const float gear = GearRatioForJoint(joint);
    const float dir = motor_directions_[leg * 3 + joint];
    const float q_feedback = static_cast<float>(MotorPositionToUncalibratedUrdf(
        motor_data_[motor_id].q, gear, dir));

    auto& motor_state = state.motors[motor_id];
    motor_state.position = calibrated_
                               ? static_cast<float>(CalibratedUrdfPosition(
                                     q_feedback, calibration_offsets_[motor_id]))
                               : q_feedback;
    motor_state.velocity = (motor_data_[motor_id].dq / gear) * dir;
    motor_state.effort = (motor_data_[motor_id].tau * gear) * dir;
    motor_state.temperature = motor_data_[motor_id].temp;
    motor_state.fault = motor_data_[motor_id].merror;
}

void IOSDK::MarkMotorOffline(int leg, int joint, LowLevelState& state) const {
    // 不把上一帧反馈伪装成本轮有效数据；0xFF 使上层 valid 接口明确失效。
    const int motor_id = leg * 3 + joint;
    auto& motor_state = state.motors[motor_id];
    motor_state = {};
    motor_state.fault = 0xFF;
    current_cycle_failures_.fetch_add(1, std::memory_order_relaxed);
    total_failures_.fetch_add(1, std::memory_order_relaxed);
}

bool IOSDK::ExchangeMotor(int leg, int joint, bool internal_alignment_frame) {
    auto& command = motor_commands_[leg * 3 + joint];
    auto& feedback = motor_data_[leg * 3 + joint];
    // The ARM64 SDK's MotorCmd overload resets the port to a hardcoded 20 ms
    // timeout on every GO-M8010-6 frame. Its byte overload preserves our 2 ms
    // select timeout and avoids a per-frame termios reset. BlockYN::NO is
    // intentional: this SDK uses NO for select-based timed reception; YES
    // returns before a complete frame and can shift motor IDs by one frame.
    // SDK bitfields contain reserved bits; do not transmit uninitialized data.
    std::memset(command.get_motor_send_data(), 0, 17);
    command.modify_data(&command);
    // Recheck immediately at the serial boundary as well as before packing:
    // a worker may have been descheduled while preparing its previous packet.
    if (internal_alignment_frame && !StartupForcePacketAllowed()) {
        const ActuatorCommand stop;
        PopulateMotorCommand(leg, joint, stop, false);
        std::memset(command.get_motor_send_data(), 0, 17);
        command.modify_data(&command);
    }
    feedback.correct = false;
    std::memset(feedback.get_motor_recv_data(), 0, 16);
    const bool received = serials_[leg]->sendRecv(
        command.get_motor_send_data(), feedback.get_motor_recv_data(),
        command.hex_len);
    const bool frame_valid = received &&
        feedback.extract_data(&feedback) && feedback.correct &&
        feedback.motor_id == joint && feedback.mode == command.mode &&
        std::isfinite(feedback.q) && std::isfinite(feedback.dq) &&
        std::isfinite(feedback.tau);
    if (received && !frame_valid) {
        // The raw SDK accepts any 16 bytes and leaves trailing RX bytes queued.
        // One stray byte can otherwise shift every following response forever.
        // Restore synchronization only on this error path, retaining the real
        // 2 ms timeout. A valid motor-fault response is not a framing failure.
        serials_[leg]->resetSerial(
            16, 4000000, 2000, BlockYN::NO, bytesize_t::eightbits,
            parity_t::parity_none, stopbits_t::stopbits_one,
            flowcontrol_t::flowcontrol_none);
    }
    return frame_valid && feedback.merror == 0;
}

bool IOSDK::StartupForcePacketAllowed() {
    if (startup_cycle_cancelled_.load() || current_cycle_failures_.load() != 0) {
        startup_cycle_cancelled_.store(true);
        return false;
    }
    const double now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    if (now > startup_round_deadline_seconds_ || now > startup_feedback_deadline_seconds_) {
        startup_cycle_cancelled_.store(true);
        return false;
    }
    if (now >= startup_force_deadline_seconds_) {
        startup_cycle_force_expired_.store(true);
        return false;
    }
    return !startup_cycle_force_expired_.load();
}

void IOSDK::SendReceiveLeg(
    int leg, const LowLevelCommand& command, LowLevelState& state,
    bool internal_alignment_frame) {
    SerialPort* serial = serials_[leg].get();
    if (serial == nullptr) {
        return;
    }

    for (int joint = 0; joint < 3; ++joint) {
        const int motor_id = leg * 3 + joint;
        const bool force_allowed = !internal_alignment_frame || StartupForcePacketAllowed();
        const ActuatorCommand stop;
        PopulateMotorCommand(leg, joint,
            force_allowed ? command.motors[motor_id] : stop,
            internal_alignment_frame && force_allowed);
        if (ExchangeMotor(leg, joint, internal_alignment_frame && force_allowed)) {
            UpdateMotorStateFromFeedback(leg, joint, state);
        } else {
            MarkMotorOffline(leg, joint, state);
        }
    }
}

void IOSDK::ExchangeCycle(
    const LowLevelCommand& command, LowLevelState& state,
    bool internal_alignment_frame) {
    if (!use_parallel_leg_io_) {
        for (int leg : active_legs_) {
            try {
                SendReceiveLeg(leg, command, state, internal_alignment_frame);
            } catch (...) {
                for (int joint = 0; joint < 3; ++joint) MarkMotorOffline(leg, joint, state);
            }
        }
        return;
    }
    const std::size_t active_workers = active_legs_.size();
    if (active_workers == 0) return;
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        active_command_ = &command;
        active_state_ = &state;
        active_alignment_frame_ = internal_alignment_frame;
        completed_workers_ = 0;
        ++dispatch_epoch_;
    }
    dispatch_cv_.notify_all();
    std::unique_lock<std::mutex> lock(worker_mutex_);
    completed_cv_.wait(lock, [this, active_workers] {
        return workers_stopping_ || completed_workers_ >= active_workers;
    });
}

void IOSDK::SendReceive(
    const LowLevelCommand& command, LowLevelState& state,
    bool calibration_requested, bool startup_alignment_allowed) {
    if (!active_ || serials_.size() != kLegCount) return;
    const auto now_seconds = [] {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    };
    const double now = now_seconds();
    // Initial metadata/default failure counters are not feedback. Each force
    // cycle needs a complete, recent preceding round, independent of claims.
    const bool previous_feedback_fresh = have_valid_feedback_ &&
        now >= last_valid_feedback_seconds_ &&
        now - last_valid_feedback_seconds_ <= 0.020;
    const auto previous_stage = startup_alignment_.stage();
    const bool calibration_allowed_at_entry = startup_alignment_.canCalibrate();
    startup_alignment_.BeginCycle(
        now, startup_alignment_allowed, previous_feedback_fresh);
    const bool internal_force = startup_alignment_.appliesTorque();
    const bool alignment_in_progress = startup_alignment_.running();
    startup_cycle_cancelled_.store(false);
    startup_cycle_force_expired_.store(false);
    startup_round_deadline_seconds_ = now + 0.020;
    startup_feedback_deadline_seconds_ = last_valid_feedback_seconds_ + 0.020;
    startup_force_deadline_seconds_ = startup_alignment_.forceDeadlineSeconds();
    LowLevelCommand effective = startup_alignment_.canCalibrate()
        ? command : startup_alignment_.command();
    current_cycle_failures_.store(0, std::memory_order_relaxed);
    ExchangeCycle(effective, state, internal_force);
    double completed = now_seconds();
    const bool interrupted = current_cycle_failures_.load() != 0 ||
        startup_cycle_cancelled_.load() || completed - now > 0.020;
    const bool force_expired = startup_cycle_force_expired_.load();
    if (alignment_in_progress && (interrupted || force_expired)) {
        // Suppress remaining packets immediately after a known failure/deadline,
        // then explicitly stop legs whose force packets were already in flight.
        // Do not clear the original errors when stop replies happen to succeed.
        const LowLevelCommand stop;
        ExchangeCycle(stop, state, false);
        completed = now_seconds();
    }
    const bool complete_feedback = current_cycle_failures_.load() == 0;
    const bool round_fresh = completed >= now && completed - now <= 0.020;
    if (alignment_in_progress) {
        if (interrupted || !complete_feedback || !round_fresh) {
            startup_alignment_.Cancel();
        } else {
            // Crossing 600 ms inside a normal round requires a stop, not a
            // spurious fault. At 300 ms the existing hip/thigh torques are
            // unchanged; the calf joins on the following ordinary control cycle.
            if (force_expired) startup_alignment_.BeginCycle(completed, true, true);
            startup_alignment_.FinishCycle(true);
        }
    }
    have_valid_feedback_ = complete_feedback && round_fresh;
    // The oldest sample was acquired near the round's start, not its end.
    if (have_valid_feedback_) last_valid_feedback_seconds_ = now;
    // A START received before completion cannot be accepted on the finishing
    // cycle. The controller additionally requires a fresh post-DONE key edge.
    FinalizeCycleCalibration(
        calibration_requested && calibration_allowed_at_entry &&
        startup_alignment_.canCalibrate(), state);
    LogStartupAlignmentTransition(previous_stage);
}

}  // namespace custom_dog_control

#endif  // COMPILE_WITH_REAL_ROBOT
