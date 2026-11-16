#include "custom_dog_control/diagnostics/CycleTrace.hpp"

#include <gtest/gtest.h>

#include <array>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <sys/wait.h>
#include <vector>

namespace {
using namespace custom_dog_control::diagnostics;
namespace fs = std::filesystem;

struct Record {
  std::uint64_t sequence = 0;
  std::array<std::uint64_t, 15> payload{};
};
static_assert(sizeof(Record) == 128);

Record MakeRecord(std::uint64_t sequence) {
  Record record;
  record.sequence = sequence;
  for (std::size_t i = 0; i < record.payload.size(); ++i) record.payload[i] = sequence ^ (0x123456780000ULL + i);
  return record;
}

bool ValidRecord(const Record& record) {
  const auto expected = MakeRecord(record.sequence);
  return std::memcmp(&record, &expected, sizeof(record)) == 0;
}

class CycleTraceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const char* old = std::getenv("CUSTOM_DOG_CYCLE_TRACE_DIR");
    had_environment_ = old != nullptr;
    if (old != nullptr) old_environment_ = old;
    char pattern[] = "/tmp/custom_dog_cycle_trace_test_XXXXXX";
    const char* created = ::mkdtemp(pattern);
    ASSERT_NE(created, nullptr);
    directory_ = created;
    ASSERT_EQ(::setenv("CUSTOM_DOG_CYCLE_TRACE_DIR", directory_.c_str(), 1), 0);
  }

  void TearDown() override {
    if (had_environment_) ::setenv("CUSTOM_DOG_CYCLE_TRACE_DIR", old_environment_.c_str(), 1);
    else ::unsetenv("CUSTOM_DOG_CYCLE_TRACE_DIR");
    if (!directory_.empty()) fs::remove_all(directory_);
  }

  std::vector<char> Read(const std::string& filename) {
    std::ifstream stream(directory_ / filename, std::ios::binary);
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  }

  fs::path directory_;
  bool had_environment_ = false;
  std::string old_environment_;
};

TEST(CycleTraceRing, FullIsNonBlockingAndSlotsCanBeReused) {
  cycle_trace_detail::SpscRing<Record, 4> ring;
  for (std::uint64_t i = 0; i < 4; ++i) ASSERT_TRUE(ring.tryPush(MakeRecord(i)));
  EXPECT_FALSE(ring.tryPush(MakeRecord(4)));
  Record record;
  ASSERT_TRUE(ring.tryPop(&record));
  EXPECT_EQ(record.sequence, 0U);
  ASSERT_TRUE(ring.tryPush(MakeRecord(4)));
  for (std::uint64_t i = 1; i <= 4; ++i) {
    ASSERT_TRUE(ring.tryPop(&record));
    EXPECT_EQ(record.sequence, i);
    EXPECT_TRUE(ValidRecord(record));
  }
  EXPECT_FALSE(ring.tryPop(&record));
}

TEST(CycleTraceRing, ConcurrentProducerConsumerPreserveEveryRecordAndOrder) {
  cycle_trace_detail::SpscRing<Record, 64> ring;
  constexpr std::uint64_t kRecords = 200000;
  std::atomic<bool> corrupted{false};
  std::thread producer([&] {
    for (std::uint64_t i = 0; i < kRecords; ++i) {
      const auto record = MakeRecord(i);
      while (!ring.tryPush(record)) std::this_thread::yield();
    }
  });
  std::thread consumer([&] {
    for (std::uint64_t i = 0; i < kRecords; ++i) {
      Record record;
      while (!ring.tryPop(&record)) std::this_thread::yield();
      if (record.sequence != i || !ValidRecord(record)) corrupted.store(true);
    }
  });
  producer.join();
  consumer.join();
  EXPECT_FALSE(corrupted.load());
}

TEST_F(CycleTraceTest, UnsetOrEmptyEnvironmentDoesNotCreateRecorderOrFile) {
  ::unsetenv("CUSTOM_DOG_CYCLE_TRACE_DIR");
  EXPECT_EQ(CycleTrace<Record>::CreateFromEnvironment("off.bin", 1), nullptr);
  ::setenv("CUSTOM_DOG_CYCLE_TRACE_DIR", "", 1);
  EXPECT_EQ(CycleTrace<Record>::CreateFromEnvironment("off.bin", 1), nullptr);
  EXPECT_TRUE(fs::is_empty(directory_));
}

TEST_F(CycleTraceTest, HeaderFooterAndAcceptedPayloadMatchAfterOverflow) {
  const int producer_nice_before = ::getpriority(PRIO_PROCESS, 0);
  auto trace = CycleTrace<Record, 2>::CreateFromEnvironment("trace.bin", 73);
  ASSERT_NE(trace, nullptr);
  std::vector<std::uint64_t> accepted;
  constexpr std::uint64_t kAttempts = 100000;
  for (std::uint64_t i = 0; i < kAttempts; ++i) {
    if (trace->tryPush(MakeRecord(i))) accepted.push_back(i);
  }
  trace->stop();
  trace->stop();
  EXPECT_FALSE(trace->enabled());
  EXPECT_FALSE(trace->tryPush(MakeRecord(kAttempts)));
  const auto state = trace->stats();
  EXPECT_GT(state.dropped_full, 0U);
  EXPECT_EQ(state.accepted + state.dropped_full, kAttempts);
  EXPECT_EQ(state.accepted, state.written);
  EXPECT_EQ(state.discarded, 0U);
  EXPECT_EQ(state.reason, CycleTraceStopReason::kRequested);
  EXPECT_EQ(::getpriority(PRIO_PROCESS, 0), producer_nice_before);

  const auto bytes = Read("trace.00000.bin");
  ASSERT_EQ(bytes.size(), 128 + accepted.size() * sizeof(Record));
  CycleTraceHeader header;
  std::memcpy(&header, bytes.data(), sizeof(header));
  EXPECT_EQ(std::string(header.magic, 8), "DGCYCL01");
  EXPECT_EQ(header.version, 1);
  EXPECT_EQ(header.header_size, 64);
  EXPECT_EQ(header.record_size, sizeof(Record));
  EXPECT_EQ(header.type_id, 73U);
  EXPECT_EQ(header.endian, 0x01020304U);
  EXPECT_EQ(header.record_alignment, alignof(Record));
  EXPECT_EQ(header.ring_capacity, 2U);
  EXPECT_EQ(header.flags, 7U);
  EXPECT_EQ(header.clock_id, 1U);
  EXPECT_GT(header.wall_start_ns, 0U);
  EXPECT_GT(header.monotonic_start_ns, 0U);
  for (std::size_t i = 0; i < accepted.size(); ++i) {
    Record record;
    std::memcpy(&record, bytes.data() + 64 + i * sizeof(Record), sizeof(record));
    EXPECT_EQ(record.sequence, accepted[i]);
    EXPECT_TRUE(ValidRecord(record));
  }
  CycleTraceFooter footer;
  std::memcpy(&footer, bytes.data() + bytes.size() - 64, sizeof(footer));
  EXPECT_EQ(std::string(footer.magic, 8), "DGCEND01");
  EXPECT_EQ(footer.written, accepted.size());
  EXPECT_EQ(footer.dropped_full, state.dropped_full);
  EXPECT_EQ(footer.bytes_written, bytes.size());
  EXPECT_EQ(footer.io_errors, 0U);
}

TEST_F(CycleTraceTest, ConcurrentStopDrainsEveryAcceptedRecord) {
  auto trace = CycleTrace<Record, 64>::CreateFromEnvironment("stop.bin", 1);
  ASSERT_NE(trace, nullptr);
  std::atomic<std::uint64_t> accepted{0};
  std::thread producer([&] {
    for (std::uint64_t i = 0; trace->enabled(); ++i) {
      if (trace->tryPush(MakeRecord(i))) accepted.fetch_add(1, std::memory_order_relaxed);
    }
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  trace->stop();
  producer.join();
  const auto state = trace->stats();
  EXPECT_EQ(state.accepted, accepted.load());
  EXPECT_EQ(state.written, accepted.load());
  EXPECT_EQ(state.discarded, 0U);
}

TEST_F(CycleTraceTest, RotationKeepsFirstAndNewestThreeWithinByteLimit) {
  constexpr std::uint64_t kLimit = 128 + 3 * sizeof(Record);
  auto trace = CycleTrace<Record, 64>::CreateFromEnvironment("rotate.bin", 2, {kLimit});
  ASSERT_NE(trace, nullptr);
  for (std::uint64_t i = 0; i < 17; ++i) ASSERT_TRUE(trace->tryPush(MakeRecord(i)));
  trace->stop();
  const auto state = trace->stats();
  EXPECT_EQ(state.written, 17U);
  EXPECT_EQ(state.dropped_full, 0U);
  EXPECT_EQ(state.discarded, 0U);
  std::set<std::string> filenames;
  std::uint64_t retained_bytes = 0;
  for (const auto& file : fs::directory_iterator(directory_)) {
    filenames.insert(file.path().filename());
    EXPECT_LE(file.file_size(), kLimit);
    retained_bytes += file.file_size();
  }
  EXPECT_EQ(filenames, (std::set<std::string>{"rotate.00000.bin", "rotate.00003.bin", "rotate.00004.bin", "rotate.00005.bin"}));
  EXPECT_LE(retained_bytes, 4 * kLimit);
  for (std::uint64_t segment : {0, 3, 4, 5}) {
    char name[64];
    std::snprintf(name, sizeof(name), "rotate.%05llu.bin", static_cast<unsigned long long>(segment));
    const auto bytes = Read(name);
    ASSERT_GE(bytes.size(), 128U);
    const auto count = (bytes.size() - 128) / sizeof(Record);
    for (std::size_t i = 0; i < count; ++i) {
      Record record;
      std::memcpy(&record, bytes.data() + 64 + i * sizeof(Record), sizeof(record));
      EXPECT_EQ(record.sequence, segment * 3 + i);
      EXPECT_TRUE(ValidRecord(record));
    }
    CycleTraceFooter footer;
    std::memcpy(&footer, bytes.data() + bytes.size() - 64, sizeof(footer));
    EXPECT_EQ(footer.written, segment * 3 + count);
    EXPECT_EQ(footer.reason, segment == 5 ? 1U : 2U);
    EXPECT_EQ(footer.bytes_written, bytes.size());
  }
}

TEST_F(CycleTraceTest, RefusesExistingFilesSymlinkComponentsAndNonDirectories) {
  { std::ofstream existing(directory_ / "existing.00000.bin"); existing << "preserve"; }
  EXPECT_EQ(CycleTrace<Record>::CreateFromEnvironment("existing.bin", 1), nullptr);
  EXPECT_EQ(Read("existing.00000.bin"), (std::vector<char>{'p', 'r', 'e', 's', 'e', 'r', 'v', 'e'}));
  fs::create_symlink(directory_ / "existing.00000.bin", directory_ / "linked.00000.bin");
  EXPECT_EQ(CycleTrace<Record>::CreateFromEnvironment("linked.bin", 1), nullptr);
  EXPECT_TRUE(fs::is_symlink(directory_ / "linked.00000.bin"));
  fs::create_directory(directory_ / "real");
  fs::create_directory(directory_ / "real" / "child");
  fs::create_directory_symlink(directory_ / "real", directory_ / "link");
  const auto linked_directory = directory_ / "link" / "child";
  ::setenv("CUSTOM_DOG_CYCLE_TRACE_DIR", linked_directory.c_str(), 1);
  EXPECT_EQ(CycleTrace<Record>::CreateFromEnvironment("trace.bin", 1), nullptr);
  const auto not_directory = directory_ / "existing.00000.bin";
  ::setenv("CUSTOM_DOG_CYCLE_TRACE_DIR", not_directory.c_str(), 1);
  EXPECT_EQ(CycleTrace<Record>::CreateFromEnvironment("trace.bin", 1), nullptr);
  ::setenv("CUSTOM_DOG_CYCLE_TRACE_DIR", directory_.c_str(), 1);
  EXPECT_EQ(CycleTrace<Record>::CreateFromEnvironment("../escape.bin", 1), nullptr);
  EXPECT_EQ(CycleTrace<Record>::CreateFromEnvironment("invalid.bin", 1, {127}), nullptr);
  EXPECT_EQ(CycleTrace<Record>::CreateFromEnvironment("invalid.bin", 1, {kCycleTraceMaxFileBytes + 1}), nullptr);
}

TEST_F(CycleTraceTest, RotationDoesNotOverwriteAConflictingFutureSegment) {
  { std::ofstream existing(directory_ / "conflict.00001.bin"); existing << "keep"; }
  auto trace = CycleTrace<Record, 64>::CreateFromEnvironment("conflict.bin", 1, {128 + sizeof(Record)});
  ASSERT_NE(trace, nullptr);
  for (std::uint64_t i = 0; i < 4; ++i) ASSERT_TRUE(trace->tryPush(MakeRecord(i)));
  trace->stop();
  EXPECT_EQ(trace->stats().reason, CycleTraceStopReason::kIoError);
  EXPECT_EQ(trace->stats().written, 1U);
  EXPECT_EQ(trace->stats().discarded, 3U);
  EXPECT_FALSE(trace->enabled());
  EXPECT_EQ(Read("conflict.00001.bin"), (std::vector<char>{'k', 'e', 'e', 'p'}));
}

TEST_F(CycleTraceTest, RetentionRefusesAnOldSegmentReplacedWithASymlink) {
  const auto wait_written = [](const auto& trace, std::uint64_t count) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (trace->stats().written < count && trace->enabled() && std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return trace->stats().written >= count;
  };
  auto trace = CycleTrace<Record, 64>::CreateFromEnvironment("replace.bin", 1, {128 + sizeof(Record)});
  ASSERT_NE(trace, nullptr);
  ASSERT_TRUE(trace->tryPush(MakeRecord(0)));
  ASSERT_TRUE(trace->tryPush(MakeRecord(1)));
  ASSERT_TRUE(wait_written(trace, 2));
  // Segment 1 has closed; replace its name, while preserving the original for
  // inspection. The recorder must neither follow nor delete this new symlink.
  fs::rename(directory_ / "replace.00001.bin", directory_ / "saved_segment.bin");
  { std::ofstream other(directory_ / "unrelated.txt"); other << "keep"; }
  fs::create_symlink(directory_ / "unrelated.txt", directory_ / "replace.00001.bin");
  ASSERT_TRUE(trace->tryPush(MakeRecord(2)));
  ASSERT_TRUE(trace->tryPush(MakeRecord(3)));
  trace->stop();
  EXPECT_EQ(trace->stats().reason, CycleTraceStopReason::kIoError);
  EXPECT_TRUE(fs::is_symlink(directory_ / "replace.00001.bin"));
  EXPECT_TRUE(fs::exists(directory_ / "saved_segment.bin"));
  EXPECT_EQ(Read("unrelated.txt"), (std::vector<char>{'k', 'e', 'e', 'p'}));
}

TEST_F(CycleTraceTest, DiskWriteFailureDisablesOnlyTracingAndPreservesProducerProgress) {
  // A child process confines RLIMIT_FSIZE and SIGXFSZ handling to this test.
  // Header fits, record write fails with EFBIG: no special device or ROS needed.
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::signal(SIGXFSZ, SIG_IGN);
    const rlimit limit{64, 64};
    if (::setrlimit(RLIMIT_FSIZE, &limit) != 0) _exit(20);
    auto trace = CycleTrace<Record, 4>::CreateFromEnvironment("diskfail.bin", 1);
    if (!trace || !trace->tryPush(MakeRecord(1))) _exit(21);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (trace->enabled() && std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    for (std::uint64_t i = 0; i < 10000; ++i) if (trace->tryPush(MakeRecord(i))) _exit(22);
    trace->stop();
    const auto state = trace->stats();
    if (state.enabled || state.reason != CycleTraceStopReason::kIoError || state.io_errors != 1 ||
        state.accepted != 1 || state.written != 0 || state.discarded != 1) _exit(23);
    _exit(0);
  }
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  EXPECT_EQ(fs::file_size(directory_ / "diskfail.00000.bin"), 64U);
}

TEST_F(CycleTraceTest, PartialDiskWriteCountsCompletePrefixAndLeavesRecoverableBytes) {
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  constexpr std::uint64_t kLimit = 64 + sizeof(Record) + sizeof(Record) / 2;
  if (child == 0) {
    ::signal(SIGXFSZ, SIG_IGN);
    const rlimit limit{kLimit, kLimit};
    if (::setrlimit(RLIMIT_FSIZE, &limit) != 0) _exit(30);
    auto trace = CycleTrace<Record, 64>::CreateFromEnvironment("partial.bin", 1);
    if (!trace) _exit(31);
    for (std::uint64_t i = 0; i < 10; ++i) trace->tryPush(MakeRecord(i));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (trace->enabled() && std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    trace->stop();
    const auto state = trace->stats();
    if (state.enabled || state.reason != CycleTraceStopReason::kIoError || state.io_errors != 1 ||
        state.written != 1 || state.discarded != state.accepted - 1 || state.bytes_written != kLimit) _exit(32);
    _exit(0);
  }
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  EXPECT_EQ(fs::file_size(directory_ / "partial.00000.bin"), kLimit);
  const auto bytes = Read("partial.00000.bin");
  ASSERT_EQ(bytes.size(), kLimit);
  Record recovered;
  std::memcpy(&recovered, bytes.data() + 64, sizeof(recovered));
  EXPECT_EQ(recovered.sequence, 0U);
  EXPECT_TRUE(ValidRecord(recovered));
}
}  // namespace
