// Finite timing experiment. Real IO is permanently mode 0/all-zero. Numerical
// outputs never cross the ControlWorkload API. No ROS controller is launched.
#include "custom_dog_control/hardware/IOSDK.hpp"
#include "control_timing_workload.hpp"
#include <csignal>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <fcntl.h>
#include <sched.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <unistd.h>

extern "C" void dog_nmpc_profile_begin() __attribute__((weak));
extern "C" void dog_nmpc_profile_end() __attribute__((weak));

namespace dog = custom_dog_control;
namespace tp = timing_probe;
namespace fs = std::filesystem;
volatile std::sig_atomic_t stopped = 0;
void Stop(int) { stopped = 1; }

// Python layout: <QQQqqqqqq10d. Locked shared memory, not a ROS message.
struct ImuSnapshot {
  std::uint64_t version = 0, imu_frame = 0, ahrs_frame = 0;
  std::int64_t imu_device_us = 0, ahrs_device_us = 0;
  std::int64_t imu_read_ns = 0, ahrs_read_ns = 0;
  std::int64_t imu_parse_ns = 0, ahrs_parse_ns = 0;
  double orientation[4]{}, angular[3]{}, acceleration[3]{};
};
static_assert(sizeof(ImuSnapshot) == 152);

class SnapshotReader {
 public:
  explicit SnapshotReader(const fs::path& path) {
    fd_ = open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd_ < 0) throw std::runtime_error("cannot open IMU snapshot");
    memory_ = mmap(nullptr, sizeof(ImuSnapshot), PROT_READ, MAP_SHARED, fd_, 0);
    if (memory_ == MAP_FAILED) { close(fd_); throw std::runtime_error("cannot map IMU snapshot"); }
  }
  ~SnapshotReader() { munmap(memory_, sizeof(ImuSnapshot)); close(fd_); }
  bool Read(ImuSnapshot& result) {
    if (flock(fd_, LOCK_SH | LOCK_NB)) return false;
    std::memcpy(&result, memory_, sizeof(result));
    flock(fd_, LOCK_UN);
    return result.version && result.imu_frame && result.ahrs_frame;
  }
 private:
  int fd_ = -1;
  void* memory_ = MAP_FAILED;
};

dog::ImuSample ToImu(const ImuSnapshot& snapshot) {
  dog::ImuSample result;
  std::copy_n(snapshot.orientation, 4, result.orientation_wxyz.begin());
  std::copy_n(snapshot.angular, 3, result.angular_velocity.begin());
  std::copy_n(snapshot.acceleration, 3, result.linear_acceleration.begin());
  result.stamp_seconds = snapshot.ahrs_read_ns * 1e-9;
  const auto age = tp::NowNs() - snapshot.ahrs_read_ns;
  result.valid = dog::FiniteImuSample(result) && age >= 0 && age <= 100000000LL;
  return result;
}

struct Round {
  std::uint64_t index = 0, tick = 0, skipped_slots = 0;
  std::int64_t scheduled_ns = 0, start_ns = 0, io_start_ns = 0, io_end_ns = 0;
  std::int64_t shadow_write_ns = 0, previous_shadow_write_ns = 0, end_ns = 0;
  int failures = 0;
  bool imu_available = false;
  tp::ControlStep control;
  ImuSnapshot imu;
  std::array<dog::MotorTimingProbeRecord, dog::kJointCount> motors{};
};

std::string Affinity() {
  cpu_set_t cpus;
  CPU_ZERO(&cpus);
  if (sched_getaffinity(0, sizeof(cpus), &cpus)) throw std::runtime_error("cannot read CPU affinity");
  std::ostringstream out;
  for (int i = 0; i < CPU_SETSIZE; ++i) if (CPU_ISSET(i, &cpus)) {
    if (out.tellp() > 0) out << ',';
    out << i;
  }
  return out.str();
}

std::string Hex(const std::array<std::uint8_t, 17>& bytes) {
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (auto byte : bytes) out << std::setw(2) << unsigned(byte);
  return out.str();
}

void WriteRows(const fs::path& output, const std::vector<Round>& rows, bool io) {
  std::ofstream rounds(output / "cycles.csv");
  rounds << "cycle,tick,scheduled_ns,start_ns,io_start_ns,io_end_ns,shadow_write_ns,previous_shadow_write_ns,end_ns,skipped_slots,failures,"
            "control_start_ns,estimator_start_ns,estimator_end_ns,observation_end_ns,policy_end_ns,wbc_end_ns,control_end_ns,"
            "policy_sequence,policy_age_ms,mpc_solve_ms,wbc_solve_ms,estimate_valid,policy_valid,policy_fresh,wbc_executed,wbc_valid,solver_healthy,new_policy,"
            "imu_available,imu_frame,ahrs_frame,imu_device_us,ahrs_device_us,imu_read_ns,ahrs_read_ns,imu_parse_ns,ahrs_parse_ns\n";
  rounds << std::setprecision(15);
  std::ofstream wbc_file(output / "wbc.csv");
  wbc_file << std::setprecision(15)
      << "cycle,model_ms,constraints_ms,tasks_ms,assembly_ms,solver_ms,validation_ms,total_ms,objective,n_wsr,cold_n_wsr,hot_n_wsr,status,reset_reasons,reuse_enabled,cold_start,hot_start,cold_retry,hot_failed,validation_failed,failed,equality_residual,inequality_violation,compared,comparison_ok,reference_total_ms,reference_solver_ms,reference_objective,reference_n_wsr,reference_failed,reference_equality_residual,reference_inequality_violation,max_solution_difference,max_torque_difference,objective_relative_difference\n";
  std::ofstream motors;
  if (io) {
    motors.open(output / "motors.csv");
    motors << "cycle,motor_index,io_cycle,sdk_start_ns,sdk_return_ns,parsed_or_recovery_end_ns,received,frame_valid,success,motor_error,previous_shadow_write_ns,tx_hex\n";
  }
  for (const auto& r : rows) {
    const auto& c = r.control;
    if (c.wbc_executed) {
      const auto& p = c.wbc_probe;
      const auto& m = p.measured;
      wbc_file << r.index << ',' << m.model_ms << ',' << m.constraints_ms << ',' << m.tasks_ms << ',' << m.assembly_ms << ','
          << m.solver_ms << ',' << m.validation_ms << ',' << m.total_ms << ',' << m.objective << ',' << m.n_wsr << ','
          << m.cold_n_wsr << ',' << m.hot_n_wsr << ',' << m.status << ',' << m.reset_reasons << ',' << m.reuse_enabled << ','
          << m.cold_start << ',' << m.hot_start << ',' << m.cold_retry << ',' << m.hot_failed << ',' << m.validation_failed << ','
          << m.failed << ',' << p.equality_residual << ',' << p.inequality_violation << ',' << p.compared << ',' << p.comparison_ok << ','
          << p.reference.total_ms << ',' << p.reference.solver_ms << ',' << p.reference.objective << ',' << p.reference.n_wsr << ','
          << p.reference.failed << ',' << p.reference_equality_residual << ',' << p.reference_inequality_violation << ','
          << p.max_solution_difference << ',' << p.max_torque_difference << ',' << p.objective_relative_difference << '\n';
    }
    rounds << r.index << ',' << r.tick << ',' << r.scheduled_ns << ',' << r.start_ns << ',' << r.io_start_ns << ',' << r.io_end_ns << ','
           << r.shadow_write_ns << ',' << r.previous_shadow_write_ns << ',' << r.end_ns << ',' << r.skipped_slots << ',' << r.failures << ','
           << c.start_ns << ',' << c.estimator_start_ns << ',' << c.estimator_end_ns << ',' << c.observation_end_ns << ',' << c.policy_end_ns << ','
           << c.wbc_end_ns << ',' << c.end_ns << ',' << c.policy_sequence << ',' << c.policy_age_ms << ',' << c.mpc_solve_ms << ',' << c.wbc_solve_ms << ','
           << c.estimate_valid << ',' << c.policy_valid << ',' << c.policy_fresh << ',' << c.wbc_executed << ',' << c.wbc_valid << ',' << c.solver_healthy << ',' << c.new_policy << ','
           << r.imu_available << ',' << r.imu.imu_frame << ',' << r.imu.ahrs_frame << ',' << r.imu.imu_device_us << ',' << r.imu.ahrs_device_us << ','
           << r.imu.imu_read_ns << ',' << r.imu.ahrs_read_ns << ',' << r.imu.imu_parse_ns << ',' << r.imu.ahrs_parse_ns << '\n';
    if (io) for (std::size_t i = 0; i < r.motors.size(); ++i) {
      const auto& m = r.motors[i];
      motors << r.index << ',' << i << ',' << m.cycle << ',' << m.sdk_start_ns << ',' << m.sdk_return_ns << ',' << m.parsed_ns << ','
             << m.received << ',' << m.frame_valid << ',' << m.success << ',' << m.motor_error << ',' << r.previous_shadow_write_ns << ',' << Hex(m.tx) << '\n';
    }
  }
}

int SelfTest() {
  dog::LowLevelCommand zero;
  if (!dog::TimingProbeZeroCommand(zero)) throw std::runtime_error("zero command rejected");
  for (int field = 0; field < 6; ++field) {
    auto bad = zero;
    auto& m = bad.motors[11];
    if (field == 0) m.mode = 1;
    if (field == 1) m.position = 1;
    if (field == 2) m.velocity = 1;
    if (field == 3) m.effort = 1;
    if (field == 4) m.kp = 1;
    if (field == 5) m.kd = std::numeric_limits<float>::quiet_NaN();
    if (dog::TimingProbeZeroCommand(bad)) throw std::runtime_error("unsafe command accepted");
  }
  for (int id = 0; id < 3; ++id) {
    MotorCmd cmd;
    cmd.motorType = MotorType::GO_M8010_6;
    cmd.hex_len = 17;
    cmd.id = id;
    cmd.mode = 0;
    cmd.q = cmd.dq = cmd.tau = cmd.kp = cmd.kd = 0;
    std::memset(cmd.get_motor_send_data(), 0, 17);
    cmd.modify_data(&cmd);
    if (!dog::TimingProbeZeroPacket(cmd)) throw std::runtime_error("encoded zero rejected");
    cmd.get_motor_send_data()[2] |= 0x10;
    if (dog::TimingProbeZeroPacket(cmd)) throw std::runtime_error("encoded FOC accepted");
    cmd.get_motor_send_data()[2] &= ~0x10;
    cmd.get_motor_send_data()[8] = 1;
    if (dog::TimingProbeZeroPacket(cmd)) throw std::runtime_error("nonzero encoded payload accepted");
  }
  const auto a = tp::NowNs(), b = tp::NowNs();
  if (b < a) throw std::runtime_error("clock is not monotonic");
  std::cout << "Zero-command/encoded-packet guards and clock self-test passed; no device opened.\n";
  return 0;
}

int main(int argc, char** argv) {
  try {
    std::string phase, workspace, output, snapshot, control_cpus, mpc_cpus;
    double duration = 30.0, hz = 1000.0, mpc_hz_override = 0.0;
    bool override_mpc = false;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--self-test") return SelfTest();
      if (i + 1 == argc) throw std::invalid_argument("missing argument value");
      const std::string value = argv[++i];
      if (arg == "--phase") phase = value;
      else if (arg == "--workspace") workspace = value;
      else if (arg == "--output") output = value;
      else if (arg == "--snapshot") snapshot = value;
      else if (arg == "--duration") duration = std::stod(value);
      else if (arg == "--hz") hz = std::stod(value);
      else if (arg == "--mpc-hz") { mpc_hz_override = std::stod(value); override_mpc = true; }
      else if (arg == "--control-cpus") control_cpus = value;
      else if (arg == "--mpc-cpus") mpc_cpus = value;
      else if (arg == "--wbc-reuse" || arg == "--wbc-compare") {
        if (value != "true" && value != "false") throw std::invalid_argument("WBC switches require true|false");
        if (arg == "--wbc-reuse") tp::backend_options.wbc_reuse = value == "true";
        else tp::backend_options.wbc_compare = value == "true";
      }
      else throw std::invalid_argument("unknown argument: " + arg);
    }
    if ((phase != "io" && phase != "control" && phase != "imu" && phase != "offline") || workspace.empty() || output.empty() ||
        !std::isfinite(duration) || duration < 1 || duration > 120 || !std::isfinite(hz) || hz < 1 || hz > 2000)
      throw std::invalid_argument("use --phase io|control|imu|offline --workspace PATH --output NEW_DIR --duration 1..120 --hz 1..2000");
    if (override_mpc && (phase == "io" || !std::isfinite(mpc_hz_override) ||
                         mpc_hz_override < 1 || mpc_hz_override > 200))
      throw std::invalid_argument("--mpc-hz requires phase control|imu and 1..200 Hz");
    if (phase == "imu" && snapshot.empty()) throw std::invalid_argument("imu phase requires --snapshot");
    // Validate both masks before narrowing the control thread. Helpers created
    // in Configure and the NMPC worker then use the same prevalidated mask.
    if (!mpc_cpus.empty()) {
      tp::nmpc_cpu_mask = tp::ParseCpuSet(mpc_cpus);
      tp::split_nmpc_cpus = true;
      tp::backend_options.nmpc_cpus = mpc_cpus;
    }
    if (!control_cpus.empty()) tp::ApplyCpuSet(tp::ParseCpuSet(control_cpus));
    prctl(PR_SET_NAME, "dog-timing-ctrl", 0, 0, 0);
    fs::create_directories(output);
    if (fs::exists(fs::path(output) / "metadata.json") || fs::exists(fs::path(output) / "cycles.csv"))
      throw std::invalid_argument("refusing to overwrite existing capture");
    signal(SIGINT, Stop);
    signal(SIGTERM, Stop);
    std::unique_ptr<SnapshotReader> imu_reader;
    dog::ImuSample initial_imu;
    if (phase == "imu") {
      imu_reader = std::make_unique<SnapshotReader>(snapshot);
      ImuSnapshot first;
      if (!imu_reader->Read(first)) throw std::runtime_error("initial IMU snapshot unavailable");
      initial_imu = ToImu(first);
      if (!initial_imu.valid) throw std::runtime_error("initial IMU snapshot invalid/stale");
    }
    std::unique_ptr<tp::ControlWorkload> workload;
    if (phase != "io") {
      workload = std::make_unique<tp::ControlWorkload>(workspace, mpc_hz_override);
      workload->Initialize(imu_reader ? &initial_imu : nullptr);  // finish codegen before opening UARTs
    }
    std::unique_ptr<dog::IOSDK> sdk;
    const bool motor_io = phase == "io" || phase == "control";
    if (motor_io) {
      dog::DriveParameters drive;
      drive.serial_ports = {"/dev/ttyS3", "/dev/ttyS4", "/dev/ttyS7", "/dev/ttyS8"};
      drive.use_parallel_leg_io = true;
      drive.startup_alignment_enabled = false;
      sdk = std::make_unique<dog::IOSDK>(drive);
      sdk->Activate();
    }
    const dog::LowLevelCommand wire_command{};  // never assigned from numerical output
    dog::LowLevelState state;
    const auto period_ns = std::int64_t(std::llround(1e9 / hz));
    if (dog_nmpc_profile_begin) {
      const auto profile_output = (fs::path(output) / "nmpc_stages.csv").string();
      setenv("DOG_NMPC_PROFILE_OUTPUT", profile_output.c_str(), 1);
      dog_nmpc_profile_begin();
    }
    const auto begin = tp::NowNs(), deadline = begin + std::int64_t(duration * 1e9);
    std::int64_t release = begin, previous_start = begin, previous_shadow = 0;
    std::uint64_t tick = 0;
    int failed_rounds = 0;
    bool safety_failed = false, wbc_failed = false, comparison_failed = false;
    std::vector<Round> rows;
    rows.reserve(std::size_t(std::ceil(duration * hz)) + 2);
    while (!stopped && tp::NowNs() < deadline) {
      timespec wake{release / 1000000000LL, release % 1000000000LL};
      int sleep_error;
      do { sleep_error = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake, nullptr); }
      while (sleep_error == EINTR && !stopped);
      if (sleep_error && sleep_error != EINTR) throw std::runtime_error("absolute sleep failed");
      if (stopped) break;
      Round row;
      row.index = rows.size(); row.tick = tick; row.scheduled_ns = release; row.start_ns = tp::NowNs();
      if (row.start_ns >= deadline) break;
      row.previous_shadow_write_ns = previous_shadow;
      if (sdk) {
        row.io_start_ns = tp::NowNs();
        sdk->SendReceive(wire_command, state, false, false);
        row.io_end_ns = tp::NowNs();
        row.motors = sdk->TimingProbeRecords();
        row.failures = sdk->LastCycleFailures();
        failed_rounds = row.failures ? failed_rounds + 1 : 0;
        for (const auto& trace : row.motors) if (!trace.sdk_start_ns || !trace.parsed_ns) safety_failed = true;
      }
      dog::ImuSample input;
      if (imu_reader) {
        row.imu_available = imu_reader->Read(row.imu);
        if (row.imu_available) {
          input = ToImu(row.imu);
        }
      }
      if (workload && (!imu_reader || row.imu_available))
        row.control = workload->Step((row.start_ns - previous_start) * 1e-9, imu_reader ? &input : nullptr);
      row.shadow_write_ns = tp::NowNs();  // metadata only; no numerical command is sent
      previous_shadow = row.shadow_write_ns;
      previous_start = row.start_ns;
      row.end_ns = tp::NowNs();
      const auto following = release + period_ns;
      // Never catch up with a burst of stale frames. Preserve missed slots in CSV.
      row.skipped_slots = row.end_ns > following ? std::uint64_t((row.end_ns - following) / period_ns + 1) : 0;
      rows.push_back(row);
      tick += 1 + row.skipped_slots;
      release = following + std::int64_t(row.skipped_slots) * period_ns;
      wbc_failed = wbc_failed || (row.control.wbc_executed && !row.control.wbc_valid);
      comparison_failed = comparison_failed ||
          (row.control.wbc_probe.compared && !row.control.wbc_probe.comparison_ok);
      if (safety_failed || failed_rounds >= 3 ||
          (workload && row.control.start_ns && !row.control.solver_healthy) ||
          wbc_failed || comparison_failed) break;
    }
    const auto finish = tp::NowNs();
    const bool solver_failed = workload && !rows.empty() && rows.back().control.start_ns &&
        !rows.back().control.solver_healthy;
    const bool completed_duration = finish >= deadline;
    std::array<dog::MotorTimingProbeRecord, dog::kJointCount> stop_records{};
    if (sdk) { sdk->Deactivate(); stop_records = sdk->TimingProbeRecords(); sdk.reset(); }
    if (workload) workload->Stop();
    if (dog_nmpc_profile_end) dog_nmpc_profile_end();
    WriteRows(output, rows, motor_io);
    if (motor_io) {
      std::ofstream stop_file(fs::path(output) / "shutdown.csv");
      stop_file << "motor_index,sdk_start_ns,sdk_return_ns,parsed_or_recovery_end_ns,success,tx_hex\n";
      for (std::size_t i = 0; i < stop_records.size(); ++i) {
        const auto& m = stop_records[i];
        stop_file << i << ',' << m.sdk_start_ns << ',' << m.sdk_return_ns << ',' << m.parsed_ns << ',' << m.success << ',' << Hex(m.tx) << '\n';
      }
    }
    std::ofstream meta(fs::path(output) / "metadata.json");
    meta << std::setprecision(15) << "{\"phase\":\"" << phase << "\",\"clock\":\"CLOCK_MONOTONIC\",\"cpu_affinity\":\"" << Affinity()
         << "\",\"scheduler_policy\":" << sched_getscheduler(0) << ",\"target_hz\":" << hz << ",\"period_ns\":" << period_ns
         << ",\"requested_duration_s\":" << duration << ",\"begin_ns\":" << begin << ",\"finish_ns\":" << finish
         << ",\"cycles\":" << rows.size() << ",\"stopped_by_signal\":" << (stopped ? "true" : "false")
         << ",\"incomplete_trace\":" << (safety_failed ? "true" : "false") << ",\"consecutive_failed_rounds\":" << failed_rounds
         << ",\"completed_requested_duration\":" << (completed_duration ? "true" : "false")
         << ",\"solver_failed\":" << (solver_failed ? "true" : "false")
         << ",\"wbc_failed\":" << (wbc_failed ? "true" : "false")
         << ",\"comparison_failed\":" << (comparison_failed ? "true" : "false")
         << ",\"all_hardware_commands_mode0_zero\":true,\"startup_alignment\":false,\"calibration\":false,\"ros_controller_started\":false,"
            "\"joint_observations\":\"synthetic nominal stance, zero velocity\",\"imu_observations\":\""
         << (phase == "imu" ? "receive-only device_type=1 conversion via locked shared memory" : "synthetic identity attitude and gravity")
         << "\",\"pipeline_marker\":\"shadow_write is compute completion metadata; next cycle sends identical zero bytes, no firmware command echo\""
         << ",\"mpc_hz\":" << (workload ? workload->config().frequency_hz : 0)
         << ",\"mpc_frequency_test_override\":" << (override_mpc ? "true" : "false")
         << ",\"policy_age_limit_ms\":" << (workload ? 2000.0 / workload->config().frequency_hz : 0)
         << ",\"motor_ports_opened\":" << (motor_io ? "true" : "false")
         << ",\"nmpc_cpu_affinity_request\":\"" << mpc_cpus << "\""
         << ",\"wbc_reuse\":" << (tp::backend_options.wbc_reuse ? "true" : "false")
         << ",\"wbc_paired_comparison\":" << (tp::backend_options.wbc_compare ? "true" : "false")
         << ",\"nmpc_node_profiling\":" << (dog_nmpc_profile_begin ? "true" : "false")
         << ",\"mpc_threads\":" << (workload ? workload->config().solver_threads : 0) << "}\n";
    std::cout << "Timing capture: " << output << ", cycles=" << rows.size() << ", elapsed_s=" << (finish-begin)*1e-9 << '\n';
    return safety_failed || failed_rounds >= 3 || rows.empty() || stopped || solver_failed ||
        wbc_failed || comparison_failed || !completed_duration ? 2 : 0;
  } catch (const std::exception& error) {
    std::cerr << "Timing probe stopped: " << error.what() << '\n';
    return 1;
  }
}
