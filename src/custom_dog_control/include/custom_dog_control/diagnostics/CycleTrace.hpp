#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <limits>
#include <memory>
#include <sched.h>
#include <string>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <thread>
#include <time.h>
#include <type_traits>
#include <unistd.h>

namespace custom_dog_control::diagnostics {

inline constexpr std::uint64_t kCycleTraceMaxFileBytes = 64ULL * 1024ULL * 1024ULL;
inline constexpr std::size_t kCycleTraceRetainedSegments = 4;  // First + newest three.

// Native-layout records, with their exact sizeof/alignment and byte order in the
// header. Format v1 uses the Linux CLOCK_MONOTONIC nanosecond time base.
struct CycleTraceHeader {
  char magic[8] = {'D', 'G', 'C', 'Y', 'C', 'L', '0', '1'};
  std::uint16_t version = 1, header_size = 64;
  std::uint32_t record_size = 0, type_id = 0, endian = 0x01020304;
  std::uint64_t wall_start_ns = 0, monotonic_start_ns = 0;
  std::uint64_t max_file_bytes = kCycleTraceMaxFileBytes;
  // flags: 1 native records, 2 segmented, 4 cumulative footer record/drop counts.
  std::uint32_t ring_capacity = 0, record_alignment = 0, clock_id = 1, flags = 7;
};

enum class CycleTraceStopReason : std::uint32_t {
  kRunning = 0,
  kRequested = 1,
  kCapacity = 2,  // This segment is full; recording continues in the next one.
  kIoError = 3,
};

// Record/drop counts are cumulative across this recorder's segments; deleted
// segments are included. bytes_written counts this segment only, including its
// header and footer. Intermediate rotation footers can include pending accepted
// records: those are not discarded. An interrupted/failed write may lack a footer.
struct CycleTraceFooter {
  char magic[8] = {'D', 'G', 'C', 'E', 'N', 'D', '0', '1'};
  std::uint32_t version = 1, reason = 0;
  std::uint64_t accepted = 0, written = 0, dropped_full = 0, discarded = 0;
  std::uint64_t bytes_written = 0, io_errors = 0;
};
static_assert(sizeof(CycleTraceHeader) == 64 && sizeof(CycleTraceFooter) == 64);
static_assert(offsetof(CycleTraceHeader, wall_start_ns) == 24);
static_assert(offsetof(CycleTraceHeader, ring_capacity) == 48);
static_assert(offsetof(CycleTraceFooter, accepted) == 16);

struct CycleTraceStats {
  std::uint64_t accepted = 0, written = 0, dropped_full = 0, discarded = 0;
  std::uint64_t bytes_written = 0, io_errors = 0;
  CycleTraceStopReason reason = CycleTraceStopReason::kRunning;
  bool enabled = false;
};

struct CycleTraceOptions {
  // A smaller limit is useful for bounded offline tests. Raising the hard limit
  // is deliberately unsupported. Header and footer count towards this limit.
  std::uint64_t max_file_bytes = kCycleTraceMaxFileBytes;
};

namespace cycle_trace_detail {

class FileDescriptor {
 public:
  explicit FileDescriptor(int fd = -1) noexcept : fd_(fd) {}
  ~FileDescriptor() { if (fd_ >= 0) ::close(fd_); }
  FileDescriptor(const FileDescriptor&) = delete;
  FileDescriptor& operator=(const FileDescriptor&) = delete;
  int get() const noexcept { return fd_; }
  int release() noexcept { const int fd = fd_; fd_ = -1; return fd; }
  void reset(int fd) noexcept { if (fd_ >= 0) ::close(fd_); fd_ = fd; }
 private:
  int fd_;
};

// Existing directories only, and no symlink in any path component. Directory
// descriptors keep the target stable if another process renames a component.
inline int OpenDirectory(const std::string& path) {
  if (path.empty()) { errno = EINVAL; return -1; }
  FileDescriptor directory(::open(path.front() == '/' ? "/" : ".",
                                  O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW));
  if (directory.get() < 0) return -1;
  std::size_t begin = 0;
  while (begin < path.size()) {
    const auto end = path.find('/', begin);
    const auto part = path.substr(begin, end == std::string::npos ? end : end - begin);
    if (!part.empty() && part != ".") {
      if (part == "..") { errno = EINVAL; return -1; }
      const int next = ::openat(directory.get(), part.c_str(),
                                O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
      if (next < 0) return -1;
      directory.reset(next);
    }
    if (end == std::string::npos) break;
    begin = end + 1;
  }
  return directory.release();
}

inline std::uint64_t ClockNs(clockid_t clock) noexcept {
  timespec stamp{};
  if (::clock_gettime(clock, &stamp) != 0) return 0;
  return static_cast<std::uint64_t>(stamp.tv_sec) * 1000000000ULL + stamp.tv_nsec;
}

// Exactly one producer and one consumer. Storage is allocated with its owning
// recorder before control starts, and each successful push is one memcpy.
template <class Record, std::size_t Capacity>
class SpscRing {
  static_assert(std::is_trivially_copyable_v<Record>);
  static_assert(Capacity > 0 && (Capacity & (Capacity - 1)) == 0,
                "Cycle trace ring capacity must be a power of two");
  static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
 public:
  bool tryPush(const Record& record) noexcept {
    const auto head = head_.load(std::memory_order_relaxed);
    if (head - tail_.load(std::memory_order_acquire) >= Capacity) return false;
    std::memcpy(storage_.data() + (head & (Capacity - 1)) * sizeof(Record),
                &record, sizeof(Record));
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  bool tryPop(void* record) noexcept {
    const auto tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) return false;
    std::memcpy(record, storage_.data() + (tail & (Capacity - 1)) * sizeof(Record),
                sizeof(Record));
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

 private:
  alignas(64) std::atomic<std::uint64_t> head_{0};
  alignas(64) std::atomic<std::uint64_t> tail_{0};
  alignas(Record) std::array<std::byte, sizeof(Record) * Capacity> storage_{};
};

}  // namespace cycle_trace_detail

// Create/stop/destruct only outside the real-time control callback. Keep this
// object alive until its single producer has stopped calling it. stop() may run
// concurrently with that producer; destruction must follow producer shutdown.
// There is no allocation, lock, clock query, file IO, scheduler call, or thread
// notification in enabled()/tryPush(). No environment means a null recorder and
// no ring allocation/thread. The caller can skip constructing its record then.
template <class Record, std::size_t Capacity = 1024>
class CycleTrace {
  static_assert(std::is_trivially_copyable_v<Record>);
  static_assert(sizeof(Record) <= std::numeric_limits<std::uint32_t>::max());
  static_assert(Capacity <= std::numeric_limits<std::uint32_t>::max());
  static_assert(std::atomic<bool>::is_always_lock_free);
 public:
  static std::unique_ptr<CycleTrace> CreateFromEnvironment(
      const std::string& filename, std::uint32_t type_id,
      CycleTraceOptions options = {}) noexcept {
    try {
      const char* configured = std::getenv("CUSTOM_DOG_CYCLE_TRACE_DIR");
      if (configured == nullptr || *configured == '\0') return nullptr;
      if (filename.empty() || filename == "." || filename == ".." ||
          filename.find('/') != std::string::npos || filename.find('\0') != std::string::npos) {
        std::fprintf(stderr, "Cycle trace disabled: invalid filename.\n");
        return nullptr;
      }
      if (options.max_file_bytes > kCycleTraceMaxFileBytes ||
          options.max_file_bytes < sizeof(CycleTraceHeader) + sizeof(Record) + sizeof(CycleTraceFooter)) {
        std::fprintf(stderr, "Cycle trace disabled: invalid file size limit.\n");
        return nullptr;
      }
      std::string directory_name(configured);
      std::string basename(filename);
      cycle_trace_detail::FileDescriptor directory(cycle_trace_detail::OpenDirectory(directory_name));
      if (directory.get() < 0) {
        std::fprintf(stderr, "Cycle trace disabled: directory %s: %s.\n",
                     directory_name.c_str(), std::strerror(errno));
        return nullptr;
      }
      auto trace = std::unique_ptr<CycleTrace>(
          new CycleTrace(directory.release(), std::move(directory_name), std::move(basename),
                         type_id, options.max_file_bytes));
      if (!trace->openSegment()) {
        trace->reportIoError("create/header");
        return nullptr;
      }
      trace->accepting_.store(true, std::memory_order_seq_cst);
      trace->worker_ = std::thread(&CycleTrace::writerLoop, trace.get());
      std::fprintf(stderr, "Cycle trace enabled: %s; record_size=%zu ring=%zu segment_cap=%llu bytes; "
                           "keep first + newest 3 segments (max %llu bytes), full rings drop records.\n",
                   trace->path_.c_str(), sizeof(Record), Capacity,
                   static_cast<unsigned long long>(options.max_file_bytes),
                   static_cast<unsigned long long>(options.max_file_bytes * kCycleTraceRetainedSegments));
      return trace;
    } catch (const std::exception& error) {
      std::fprintf(stderr, "Cycle trace disabled: %s.\n", error.what());
    } catch (...) {
      std::fprintf(stderr, "Cycle trace disabled: initialization failed.\n");
    }
    return nullptr;
  }

  ~CycleTrace() { stop(); if (fd_ >= 0) ::close(fd_); if (directory_fd_ >= 0) ::close(directory_fd_); }
  CycleTrace(const CycleTrace&) = delete;
  CycleTrace& operator=(const CycleTrace&) = delete;

  bool enabled() const noexcept { return accepting_.load(std::memory_order_relaxed); }

  bool tryPush(const Record& record) noexcept {
    if (!accepting_.load(std::memory_order_seq_cst)) return false;
    // These ordered stores and the second check let a non-RT stop/capacity/error
    // close wait for an in-progress copy, without a mutex or producer wait.
    producer_active_.store(true, std::memory_order_seq_cst);
    if (!accepting_.load(std::memory_order_seq_cst)) {
      producer_active_.store(false, std::memory_order_seq_cst);
      return false;
    }
    const bool accepted = ring_.tryPush(record);
    if (accepted) accepted_.fetch_add(1, std::memory_order_relaxed);
    else dropped_full_.fetch_add(1, std::memory_order_relaxed);
    producer_active_.store(false, std::memory_order_seq_cst);
    return accepted;
  }

  // Single lifecycle owner; repeated calls are safe, simultaneous stop() calls
  // from multiple owners are not supported. Pending accepted records drain.
  void stop() noexcept {
    accepting_.store(false, std::memory_order_seq_cst);
    stop_requested_.store(true, std::memory_order_release);
    if (worker_.joinable()) worker_.join();
  }

  CycleTraceStats stats() const noexcept {
    return {accepted_.load(std::memory_order_relaxed), written_.load(std::memory_order_relaxed),
            dropped_full_.load(std::memory_order_relaxed), discarded_.load(std::memory_order_relaxed),
            bytes_written_.load(std::memory_order_relaxed), io_errors_.load(std::memory_order_relaxed),
            reason_.load(std::memory_order_relaxed), enabled()};
  }

 private:
  CycleTrace(int directory_fd, std::string&& directory, std::string&& basename,
             std::uint32_t type_id, std::uint64_t limit) noexcept
      : directory_(std::move(directory)), basename_(std::move(basename)),
        max_file_bytes_(limit), type_id_(type_id), directory_fd_(directory_fd) {}

  struct OwnedSegment {
    std::string name;
    dev_t device = 0;
    ino_t inode = 0;
    bool valid = false;
  };

  bool openSegment() noexcept {
    try {
      const auto slot = segment_index_ == 0 ? 0 : 1 + (segment_index_ - 1) % 3;
      auto& previous = segments_[slot];
      if (previous.valid) {
        path_ = directory_ + "/" + previous.name;
        struct stat existing{};
        if (::fstatat(directory_fd_, previous.name.c_str(), &existing, AT_SYMLINK_NOFOLLOW) != 0) {
          if (errno != ENOENT) { last_errno_ = errno; return false; }
        } else {
          // Retention only removes an inode created by this exact recorder.
          // A renamed/replaced output must never cause deletion of another file.
          if (!S_ISREG(existing.st_mode) || existing.st_dev != previous.device || existing.st_ino != previous.inode) {
            last_errno_ = ESTALE;
            return false;
          }
          if (::unlinkat(directory_fd_, previous.name.c_str(), 0) != 0) { last_errno_ = errno; return false; }
        }
        previous.valid = false;
      }
      char suffix[32];
      std::snprintf(suffix, sizeof(suffix), ".%05llu.bin", static_cast<unsigned long long>(segment_index_));
      const auto stem_length = basename_.size() >= 4 && basename_.compare(basename_.size() - 4, 4, ".bin") == 0
          ? basename_.size() - 4 : basename_.size();
      std::string name = basename_.substr(0, stem_length) + suffix;
      path_ = directory_ + "/" + name;
      cycle_trace_detail::FileDescriptor file(::openat(
          directory_fd_, name.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600));
      if (file.get() < 0) { last_errno_ = errno; return false; }
      struct stat status{};
      if (::fstat(file.get(), &status) != 0) { last_errno_ = errno; return false; }
      if (!S_ISREG(status.st_mode)) { last_errno_ = EINVAL; return false; }
      previous.name = std::move(name);
      previous.device = status.st_dev;
      previous.inode = status.st_ino;
      previous.valid = true;
      fd_ = file.release();
      segment_bytes_written_ = 0;
      segment_records_written_ = 0;
      CycleTraceHeader header;
      header.record_size = sizeof(Record);
      header.type_id = type_id_;
      header.wall_start_ns = cycle_trace_detail::ClockNs(CLOCK_REALTIME);
      header.monotonic_start_ns = cycle_trace_detail::ClockNs(CLOCK_MONOTONIC);
      header.max_file_bytes = max_file_bytes_;
      header.ring_capacity = Capacity;
      header.record_alignment = alignof(Record);
      return writeAll(&header, sizeof(header));
    } catch (...) { last_errno_ = ENOMEM; return false; }
  }

  bool closeSegment(CycleTraceStopReason reason) noexcept {
    CycleTraceFooter footer;
    const auto state = stats();
    footer.reason = static_cast<std::uint32_t>(reason);
    footer.accepted = state.accepted;
    footer.written = state.written;
    footer.dropped_full = state.dropped_full;
    footer.discarded = state.discarded;
    footer.bytes_written = segment_bytes_written_ + sizeof(footer);
    footer.io_errors = state.io_errors;
    if (!writeAll(&footer, sizeof(footer))) return false;
    if (::close(fd_) != 0) { fd_ = -1; last_errno_ = errno; return false; }
    fd_ = -1;
    return true;
  }

  bool writeAll(const void* data, std::size_t size) noexcept {
    const auto* bytes = static_cast<const std::byte*>(data);
    while (size > 0) {
      const auto count = ::write(fd_, bytes, size);
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) { last_errno_ = count < 0 ? errno : EIO; return false; }
      bytes += count;
      size -= static_cast<std::size_t>(count);
      segment_bytes_written_ += static_cast<std::size_t>(count);
      bytes_written_.fetch_add(static_cast<std::size_t>(count), std::memory_order_relaxed);
    }
    return true;
  }

  void reportIoError(const char* phase) noexcept {
    if (io_errors_.fetch_add(1, std::memory_order_relaxed) == 0) {
      std::fprintf(stderr, "Cycle trace IO failed (%s): %s: %s; control continues without recording.\n",
                   phase, path_.c_str(), std::strerror(last_errno_));
    }
    reason_.store(CycleTraceStopReason::kIoError, std::memory_order_relaxed);
  }

  void closeAcceptance() noexcept {
    accepting_.store(false, std::memory_order_seq_cst);
    while (producer_active_.load(std::memory_order_seq_cst)) std::this_thread::yield();
  }

  void accountPartialRecordWrite() noexcept {
    // write() may complete earlier records before failing midway through the
    // batch. Count only complete records; a decoder can discard the remainder.
    const auto complete = (segment_bytes_written_ - sizeof(CycleTraceHeader)) / sizeof(Record);
    written_.fetch_add(complete - segment_records_written_, std::memory_order_relaxed);
    segment_records_written_ = complete;
  }

  void writerLoop() noexcept {
    // Linux nice is per-thread. This changes only this worker's priority and
    // preserves controller and SDK worker scheduling/affinity.
    const sched_param background_policy{};
    (void)::sched_setscheduler(0, SCHED_OTHER, &background_policy);
    const auto tid = static_cast<id_t>(::syscall(SYS_gettid));
    errno = 0;
    const int current_nice = ::getpriority(PRIO_PROCESS, tid);
    if (errno == 0) (void)::setpriority(PRIO_PROCESS, tid, std::min(19, current_nice + 5));

    const auto record_limit =
        (max_file_bytes_ - sizeof(CycleTraceHeader) - sizeof(CycleTraceFooter)) / sizeof(Record);
    bool failed = false;
    for (;;) {
      if (stop_requested_.load(std::memory_order_acquire)) closeAcceptance();
      const auto remaining = record_limit - segment_records_written_;
      if (remaining == 0) {
        if (!closeSegment(CycleTraceStopReason::kCapacity)) {
          reportIoError("rotation footer/close");
          failed = true;
          break;
        }
        ++segment_index_;
        if (!openSegment()) {
          reportIoError("rotation create/header");
          failed = true;
          break;
        }
        continue;
      }
      std::size_t count = 0;
      const auto batch_limit = static_cast<std::size_t>(std::min<std::uint64_t>(kBatchRecords, remaining));
      while (count < batch_limit && ring_.tryPop(batch_.data() + count * sizeof(Record))) ++count;
      if (count > 0) {
        if (!writeAll(batch_.data(), count * sizeof(Record))) {
          accountPartialRecordWrite();
          reportIoError("records");
          closeAcceptance();
          failed = true;
          break;
        }
        written_.fetch_add(count, std::memory_order_relaxed);
        segment_records_written_ += count;
      } else if (stop_requested_.load(std::memory_order_acquire)) {
        // This iteration closed acceptance before looking at the ring, except
        // when stop arrived during the empty check. Close and check once more.
        closeAcceptance();
        if (ring_.tryPop(batch_.data())) {
          if (!writeAll(batch_.data(), sizeof(Record))) {
            accountPartialRecordWrite();
            reportIoError("drain");
            failed = true;
            break;
          }
          written_.fetch_add(1, std::memory_order_relaxed);
          ++segment_records_written_;
          continue;
        }
        reason_.store(CycleTraceStopReason::kRequested, std::memory_order_relaxed);
        break;
      } else {
        // Polling avoids a producer syscall/notification and batches 250 Hz
        // records into ordinary background IO. Maximum polling delay is 4 ms.
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
      }
    }
    closeAcceptance();
    // Only complete records count as written, including a complete prefix of a
    // failed batch. Parsers can recover those by length when no footer exists.
    discarded_.store(accepted_.load(std::memory_order_relaxed) - written_.load(std::memory_order_relaxed),
                     std::memory_order_relaxed);
    if (!failed && !closeSegment(reason_.load(std::memory_order_relaxed))) reportIoError("footer/close");
    // No fsync in the control path (nor at every record). Close errors also
    // disable only tracing; an abrupt system power loss can lose cached data.
    if (fd_ >= 0 && ::close(fd_) != 0) { last_errno_ = errno; reportIoError("close"); }
    fd_ = -1;
    const auto state = stats();
    std::fprintf(stderr, "Cycle trace closed: %s reason=%u accepted=%llu written=%llu "
                         "dropped_full=%llu discarded=%llu bytes_all_segments=%llu io_errors=%llu "
                         "segment_cap=%llu keep_first_plus_newest=3 last_segment=%llu.\n",
                 path_.c_str(), static_cast<unsigned>(state.reason),
                 static_cast<unsigned long long>(state.accepted), static_cast<unsigned long long>(state.written),
                 static_cast<unsigned long long>(state.dropped_full), static_cast<unsigned long long>(state.discarded),
                 static_cast<unsigned long long>(state.bytes_written), static_cast<unsigned long long>(state.io_errors),
                 static_cast<unsigned long long>(max_file_bytes_), static_cast<unsigned long long>(segment_index_));
  }

  static constexpr std::size_t kBatchRecords = 32;
  const std::string directory_, basename_;
  std::string path_;
  const std::uint64_t max_file_bytes_;
  const std::uint32_t type_id_;
  int directory_fd_ = -1, fd_ = -1, last_errno_ = 0;
  std::uint64_t segment_index_ = 0, segment_bytes_written_ = 0, segment_records_written_ = 0;
  std::array<OwnedSegment, kCycleTraceRetainedSegments> segments_{};
  std::thread worker_;
  cycle_trace_detail::SpscRing<Record, Capacity> ring_;
  alignas(Record) std::array<std::byte, kBatchRecords * sizeof(Record)> batch_{};
  alignas(64) std::atomic<bool> accepting_{false}, producer_active_{false}, stop_requested_{false};
  std::atomic<std::uint64_t> accepted_{0}, written_{0}, dropped_full_{0}, discarded_{0};
  std::atomic<std::uint64_t> bytes_written_{0}, io_errors_{0};
  std::atomic<CycleTraceStopReason> reason_{CycleTraceStopReason::kRunning};
};

}  // namespace custom_dog_control::diagnostics
