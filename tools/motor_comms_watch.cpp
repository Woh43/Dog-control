// Continuous communication monitor. Every packet requests BRAKE mode with
// q/dq/tau/kp/kd all zero. This utility has no FOC or calibration command path.
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "crc/crc_ccitt.h"
#include "serialPort/SerialPort.h"

namespace {
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t stop_requested = 0;

void Stop(int) { stop_requested = 1; }

class SerialInputGuard {
 public:
  explicit SerialInputGuard(const std::string& port) {
  // The SDK owns its private fd already. Set exclusivity through another fd
  // after its constructor; the tty remains exclusive while the SDK fd is open.
  // Setting TIOCEXCL before construction would reject the SDK's own open.
  descriptor_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (descriptor_ < 0) {
    throw std::runtime_error("cannot acquire serial exclusivity: " +
                             std::string(std::strerror(errno)));
  }
  const int result = ::ioctl(descriptor_, TIOCEXCL);
  const int saved_errno = errno;
  if (result < 0) {
    ::close(descriptor_);
    throw std::runtime_error("TIOCEXCL failed: " +
                             std::string(std::strerror(saved_errno)));
  }
  }
  ~SerialInputGuard() { ::close(descriptor_); }
  SerialInputGuard(const SerialInputGuard&) = delete;
  SerialInputGuard& operator=(const SerialInputGuard&) = delete;

  void DiscardInput() const {
    if (::tcflush(descriptor_, TCIFLUSH) != 0) {
      throw std::runtime_error("cannot resynchronize serial input: " +
                               std::string(std::strerror(errno)));
    }
  }

 private:
  int descriptor_ = -1;
};

void FlushConsoleStreams() noexcept {
  // Cover both C and C++ output used by different SDK builds. In particular,
  // do not leave a buffered timeout warning to appear after output is restored.
  try { std::cout.flush(); } catch (...) {}
  try { std::cerr.flush(); } catch (...) {}
  try { std::clog.flush(); } catch (...) {}
  std::fflush(stdout);
  std::fflush(stderr);
}

class SdkOutputFiles {
 public:
  SdkOutputFiles() {
    saved_stdout_ = ::fcntl(STDOUT_FILENO, F_DUPFD_CLOEXEC, 3);
    saved_stderr_ = ::fcntl(STDERR_FILENO, F_DUPFD_CLOEXEC, 3);
    discard_ = ::open("/dev/null", O_WRONLY | O_CLOEXEC);
    if (saved_stdout_ < 0 || saved_stderr_ < 0 || discard_ < 0) {
      const int saved_errno = errno;
      Close();
      throw std::runtime_error("cannot prepare quiet SDK output: " +
                               std::string(std::strerror(saved_errno)));
    }
  }
  ~SdkOutputFiles() { Close(); }
  SdkOutputFiles(const SdkOutputFiles&) = delete;
  SdkOutputFiles& operator=(const SdkOutputFiles&) = delete;
  int saved_stdout() const { return saved_stdout_; }
  int saved_stderr() const { return saved_stderr_; }
  int discard() const { return discard_; }

 private:
  void Close() noexcept {
    if (saved_stdout_ >= 0) ::close(saved_stdout_);
    if (saved_stderr_ >= 0) ::close(saved_stderr_);
    if (discard_ >= 0) ::close(discard_);
  }
  int saved_stdout_ = -1;
  int saved_stderr_ = -1;
  int discard_ = -1;
};

class QuietSdkOutput {
 public:
  explicit QuietSdkOutput(const SdkOutputFiles& files) : files_(files) {
    FlushConsoleStreams();
    if (::dup2(files_.discard(), STDOUT_FILENO) < 0) {
      throw std::runtime_error("cannot silence SDK stdout");
    }
    if (::dup2(files_.discard(), STDERR_FILENO) < 0) {
      const int saved_errno = errno;
      ::dup2(files_.saved_stdout(), STDOUT_FILENO);
      throw std::runtime_error("cannot silence SDK stderr: " +
                               std::string(std::strerror(saved_errno)));
    }
  }
  ~QuietSdkOutput() {
    const int saved_errno = errno;
    FlushConsoleStreams();
    ::dup2(files_.saved_stdout(), STDOUT_FILENO);
    ::dup2(files_.saved_stderr(), STDERR_FILENO);
    errno = saved_errno;
  }
  QuietSdkOutput(const QuietSdkOutput&) = delete;
  QuietSdkOutput& operator=(const QuietSdkOutput&) = delete;

 private:
  const SdkOutputFiles& files_;
};

bool QuietSdkExchange(SerialPort& serial, MotorCmd& command,
                      MotorData& feedback) {
  // This monitor is single-threaded. The descriptors are prepared once, and
  // only this process's stdout/stderr are redirected during the SDK call.
  static const SdkOutputFiles files;
  const QuietSdkOutput quiet(files);
  return serial.sendRecv(command.get_motor_send_data(),
                         feedback.get_motor_recv_data(), command.hex_len);
}

std::string JsonString(const std::string& value) {
  std::ostringstream out;
  out << '"';
  for (const unsigned char c : value) {
    switch (c) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (c < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<unsigned int>(c) << std::dec;
        } else {
          out << c;
        }
    }
  }
  out << '"';
  return out.str();
}

std::string UtcNow() {
  const std::time_t now = std::time(nullptr);
  std::tm utc{};
  gmtime_r(&now, &utc);
  char result[32];
  std::strftime(result, sizeof(result), "%Y-%m-%dT%H:%M:%SZ", &utc);
  return result;
}

struct Counts {
  std::uint64_t attempts = 0;
  std::uint64_t received = 0;
  std::uint64_t valid = 0;
  std::uint64_t timeouts = 0;
  std::uint64_t crc_errors = 0;
  std::uint64_t malformed_errors = 0;
  std::uint64_t id_errors = 0;
  std::uint64_t mode_errors = 0;
  std::uint64_t sdk_errors = 0;
  std::uint64_t nonfinite_errors = 0;
  std::uint64_t motor_error_samples = 0;
  std::uint64_t rx_resyncs = 0;
};

void WriteCounts(std::ostream& out, const Counts& current,
                 const Counts& previous = {}) {
  out << "\"attempts\":" << current.attempts - previous.attempts
      << ",\"received\":" << current.received - previous.received
      << ",\"valid\":" << current.valid - previous.valid
      << ",\"timeouts\":" << current.timeouts - previous.timeouts
      << ",\"crc_errors\":" << current.crc_errors - previous.crc_errors
      << ",\"malformed_errors\":" << current.malformed_errors - previous.malformed_errors
      << ",\"id_errors\":" << current.id_errors - previous.id_errors
      << ",\"mode_errors\":" << current.mode_errors - previous.mode_errors
      << ",\"sdk_errors\":" << current.sdk_errors - previous.sdk_errors
      << ",\"nonfinite_errors\":" << current.nonfinite_errors - previous.nonfinite_errors
      << ",\"motor_error_samples\":" << current.motor_error_samples - previous.motor_error_samples
      << ",\"rx_resyncs\":" << current.rx_resyncs - previous.rx_resyncs;
}

struct MotorStats {
  Counts counts;
  Counts previous;
  bool has_feedback = false;
  int error = 0;
  int temperature = 0;
  double q = 0.0;
  double dq = 0.0;
  double tau = 0.0;
  std::string last_invalid_frame_hex;
};

void Snapshot(const std::string& path, const std::string& port, int hz,
              std::array<MotorStats, 3>& stats, Clock::time_point begin,
              Clock::time_point& last_snapshot, bool running) {
  const auto now = Clock::now();
  const double elapsed = std::chrono::duration<double>(now - begin).count();
  const double interval = std::chrono::duration<double>(now - last_snapshot).count();
  const std::string timestamp = UtcNow();
  const std::string temporary = path + ".tmp." + std::to_string(::getpid());
  {
    std::ofstream out(temporary, std::ios::trunc);
    if (!out) throw std::runtime_error("cannot write snapshot: " + temporary);
    out << std::setprecision(10)
        << "{\"running\":" << (running ? "true" : "false")
        << ",\"updated_at_utc\":" << JsonString(timestamp)
        << ",\"elapsed_seconds\":" << elapsed
        << ",\"interval_seconds\":" << interval
        << ",\"pid\":" << ::getpid()
        << ",\"port\":" << JsonString(port)
        << ",\"baud\":4000000,\"cycle_hz\":" << hz
        << ",\"mode\":0,\"q\":0,\"dq\":0,\"tau\":0,\"kp\":0,\"kd\":0"
        << ",\"motors\":[";
    for (std::size_t id = 0; id < stats.size(); ++id) {
      const auto& motor = stats[id];
      if (id) out << ',';
      out << "{\"id\":" << id << ',';
      WriteCounts(out, motor.counts);
      out << ",\"delta\":{";
      WriteCounts(out, motor.counts, motor.previous);
      out << "},\"last_motor_error\":";
      if (motor.has_feedback) out << motor.error;
      else out << "null";
      out << ",\"temperature_c\":";
      if (motor.has_feedback) out << motor.temperature;
      else out << "null";
      out << ",\"q_rotor_rad\":";
      if (motor.has_feedback) out << motor.q;
      else out << "null";
      out << ",\"dq_rotor_rad_s\":";
      if (motor.has_feedback) out << motor.dq;
      else out << "null";
      out << ",\"tau_rotor_nm\":";
      if (motor.has_feedback) out << motor.tau;
      else out << "null";
      out << ",\"last_invalid_frame_hex\":";
      if (motor.last_invalid_frame_hex.empty()) out << "null";
      else out << JsonString(motor.last_invalid_frame_hex);
      out << '}';
    }
    out << "]}\n";
    out.flush();
    if (!out) throw std::runtime_error("snapshot write failed: " + temporary);
  }
  if (std::rename(temporary.c_str(), path.c_str()) != 0) {
    std::remove(temporary.c_str());
    throw std::runtime_error("cannot publish snapshot: " + path);
  }
  std::cout << timestamp << " port=" << port << " mode=0 all_command_values=0";
  for (std::size_t id = 0; id < stats.size(); ++id) {
    auto& motor = stats[id];
    std::cout << " ID" << id << '=' << motor.counts.valid << '/'
              << motor.counts.attempts << " (+"
              << motor.counts.valid - motor.previous.valid << '/'
              << motor.counts.attempts - motor.previous.attempts << ')';
    motor.previous = motor.counts;
  }
  std::cout << " running=" << (running ? "true" : "false") << std::endl;
  last_snapshot = now;
}

MotorCmd BrakeCommand(unsigned int id) {
  if (id > 2) throw std::invalid_argument("only motor IDs 0, 1, 2 are supported");
  MotorCmd command;
  command.motorType = MotorType::GO_M8010_6;
  command.hex_len = 17;
  command.id = id;
  command.mode = 0;
  command.q = command.dq = command.tau = command.kp = command.kd = 0.0F;
  // Also initialize reserved packet bits before the SDK populates its fields.
  std::memset(command.get_motor_send_data(), 0, 17);
  command.modify_data(&command);
  return command;
}

void VerifyBrakeFrame(MotorCmd& command, unsigned int id) {
  const auto* bytes = command.get_motor_send_data();
  if (command.hex_len != 17 || command.mode != 0 || command.id != id ||
      command.q != 0.0F || command.dq != 0.0F || command.tau != 0.0F ||
      command.kp != 0.0F || command.kd != 0.0F ||
      bytes[0] != 0xfe || bytes[1] != 0xee || bytes[2] != id) {
    throw std::runtime_error("refusing to send: frame is not BRAKE/all-zero");
  }
  for (std::size_t index = 3; index < 15; ++index) {
    if (bytes[index] != 0) {
      throw std::runtime_error("refusing to send: nonzero command payload");
    }
  }
  const unsigned int checksum = bytes[15] | (static_cast<unsigned int>(bytes[16]) << 8U);
  if (crc_ccitt(0, bytes, 15) != checksum) {
    throw std::runtime_error("refusing to send: invalid command CRC");
  }
}

void CheckFramesOffline() {
  for (unsigned int id = 0; id < 3; ++id) {
    auto command = BrakeCommand(id);
    VerifyBrakeFrame(command, id);
    std::cout << "ID " << id << " BRAKE/all-zero frame:" << std::hex << std::setfill('0');
    const auto* bytes = command.get_motor_send_data();
    for (std::size_t index = 0; index < 17; ++index) {
      std::cout << ' ' << std::setw(2) << static_cast<unsigned int>(bytes[index]);
    }
    std::cout << std::dec << std::setfill(' ') << " PASS" << std::endl;
  }
  std::cout << "Offline frame checks passed; no serial device was opened." << std::endl;
}

enum class ReplyFrameStatus { OK, MALFORMED, CRC_ERROR, ID_ERROR, MODE_ERROR };

ReplyFrameStatus ValidateReplyFrame(const std::uint8_t* bytes, unsigned int id) {
  if (bytes[0] != 0xfd || bytes[1] != 0xee) return ReplyFrameStatus::MALFORMED;
  const unsigned int checksum = bytes[14] | (static_cast<unsigned int>(bytes[15]) << 8U);
  if (crc_ccitt(0, bytes, 14) != checksum) return ReplyFrameStatus::CRC_ERROR;
  if ((bytes[2] & 0x0fU) != id) return ReplyFrameStatus::ID_ERROR;
  if (((bytes[2] >> 4U) & 0x07U) != 0) return ReplyFrameStatus::MODE_ERROR;
  return ReplyFrameStatus::OK;
}

void RecoverInvalidReply(const SerialInputGuard& input, MotorStats& stats,
                         const std::uint8_t* bytes) {
  std::ostringstream frame;
  frame << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < 16; ++index)
    frame << std::setw(2) << static_cast<unsigned int>(bytes[index]);
  stats.last_invalid_frame_hex = frame.str();
  // The SDK only reads fixed 16-byte chunks. A stray byte can leave every
  // subsequent successful read permanently shifted. Its typed API flushes on
  // every resetSerial; the raw path needs explicit recovery after a bad frame.
  // No other command is in flight here; the next query starts at a clean RX.
  input.DiscardInput();
  ++stats.counts.rx_resyncs;
}

void Query(SerialPort& serial, const SerialInputGuard& input,
           unsigned int id, MotorStats& stats) {
  auto command = BrakeCommand(id);
  // Validate the final serialized bytes on every exchange, not only --check.
  VerifyBrakeFrame(command, id);

  MotorData feedback;
  feedback.motorType = MotorType::GO_M8010_6;
  feedback.hex_len = 16;
  feedback.correct = false;
  auto& count = stats.counts;
  ++count.attempts;
  auto* bytes = feedback.get_motor_recv_data();
  std::memset(bytes, 0, 16);
  // The raw overload preserves the configured 2 ms timeout. The typed SDK
  // overload resets it to 20 ms. A false result means no complete frame was
  // returned; this API does not expose partial byte counts.
  if (!QuietSdkExchange(serial, command, feedback)) {
    ++count.timeouts;
    return;
  }
  ++count.received;
  const auto status = ValidateReplyFrame(bytes, id);
  if (status != ReplyFrameStatus::OK) {
    switch (status) {
      case ReplyFrameStatus::MALFORMED: ++count.malformed_errors; break;
      case ReplyFrameStatus::CRC_ERROR: ++count.crc_errors; break;
      case ReplyFrameStatus::ID_ERROR: ++count.id_errors; break;
      case ReplyFrameStatus::MODE_ERROR: ++count.mode_errors; break;
      case ReplyFrameStatus::OK: break;
    }
    RecoverInvalidReply(input, stats, bytes);
    return;
  }
  if (!feedback.extract_data(&feedback) || !feedback.correct) {
    ++count.sdk_errors;
    RecoverInvalidReply(input, stats, bytes);
    return;
  }
  if (feedback.motor_id != id) {
    ++count.id_errors;
    RecoverInvalidReply(input, stats, bytes);
    return;
  }
  if (feedback.mode != 0) {
    ++count.mode_errors;
    RecoverInvalidReply(input, stats, bytes);
    return;
  }
  if (!std::isfinite(feedback.q) || !std::isfinite(feedback.dq) ||
      !std::isfinite(feedback.tau)) {
    ++count.nonfinite_errors;
    RecoverInvalidReply(input, stats, bytes);
    return;
  }
  ++count.valid;
  count.motor_error_samples += feedback.merror != 0;
  stats.has_feedback = true;
  stats.error = feedback.merror;
  stats.temperature = feedback.temp;
  stats.q = feedback.q;
  stats.dq = feedback.dq;
  stats.tau = feedback.tau;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE) != 0) {
      throw std::runtime_error("SDK BRAKE mapping is not zero; refusing to transmit");
    }
    if (argc == 2 && std::string(argv[1]) == "--check") {
      CheckFramesOffline();
      return 0;
    }
    if (argc != 4) {
      std::cerr << "usage: motor_comms_watch PORT CYCLE_HZ LATEST_JSON\n"
                   "       motor_comms_watch --check\n";
      return 2;
    }
    const std::string port(argv[1]);
    std::size_t consumed = 0;
    const int hz = std::stoi(argv[2], &consumed);
    if (port.rfind("/dev/", 0) != 0 || port.find_first_of("\r\n") != std::string::npos ||
        consumed != std::strlen(argv[2]) || hz < 1 || hz > 100) {
      throw std::invalid_argument("PORT must be under /dev and CYCLE_HZ must be 1..100");
    }
    std::signal(SIGINT, Stop);
    std::signal(SIGTERM, Stop);
    SerialPort serial(port, 16, 4000000, 2000, BlockYN::NO,
                      bytesize_t::eightbits, parity_t::parity_none,
                      stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none);
    SerialInputGuard input(port);
    std::array<MotorStats, 3> stats{};
    const auto begin = Clock::now();
    auto last_snapshot = begin;
    auto next_cycle = begin;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / hz));
    Snapshot(argv[3], port, hz, stats, begin, last_snapshot, true);
    while (!stop_requested) {
      for (unsigned int id = 0; id < stats.size() && !stop_requested; ++id) {
        Query(serial, input, id, stats[id]);
      }
      const auto now = Clock::now();
      if (now - last_snapshot >= std::chrono::seconds(1)) {
        Snapshot(argv[3], port, hz, stats, begin, last_snapshot, true);
      }
      next_cycle += period;
      // Avoid bursts after a scheduler stall. Sleep is bounded to <=1 s so a
      // termination request releases the device promptly even at 1 Hz.
      if (next_cycle < now) next_cycle = now;
      std::this_thread::sleep_until(next_cycle);
    }
    Snapshot(argv[3], port, hz, stats, begin, last_snapshot, false);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "motor_comms_watch stopped: " << error.what() << std::endl;
    return 1;
  }
}
