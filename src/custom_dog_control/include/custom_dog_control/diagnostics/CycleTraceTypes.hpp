#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <type_traits>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control::diagnostics {

// Binary schema v1: little endian IEEE754, explicit 8-byte fields/padding.
// All joint arrays use kJointNames (FR, FL, RR, RL), never model order.
inline std::uint64_t TraceSteadyNs() noexcept {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

struct TraceCommand {
  std::array<double, kJointCount> position{}, velocity{}, effort{}, kp{}, kd{};
};
inline TraceCommand ToTraceCommand(const HybridJointCommand& c) noexcept {
  return {c.position, c.velocity, c.effort, c.kp, c.kd};
}
struct TraceFeedback {
  std::array<double, kJointCount> position{}, velocity{}, effort{}, valid{};
};
inline TraceFeedback ToTraceFeedback(const JointSample& s) noexcept {
  return {s.position, s.velocity, s.effort, s.valid};
}

// Controller flags: 1 evaluated, 2 policy available, 4 estimate valid,
// 8 communication OK, 16 WBC valid, 32 output active, 64 handoff active,
// 128 host-total, 256 nominal-reference, 512 trace nominal sample valid,
// 1024 old-policy nominal evaluated at this SAME query time is valid.
struct ControllerCycleRecord {
  std::uint64_t cycle = 0, manager_time_ns = 0, start_ns = 0, end_ns = 0;
  std::uint64_t policy_sequence = 0, mode = 0, policy_mode = 0, flags = 0;
  double period_s = 0, query_time_s = 0, policy_age_s = 0, source_observation_s = 0;
  double policy_first_time_s = 0, policy_last_time_s = 0, node_left_time_s = 0, node_right_time_s = 0;
  double handoff_alpha = 0, imu_stamp_s = 0, imu_use_ros_s = 0, wbc_solve_ms = 0;
  double equality_residual = 0, inequality_violation = 0, motor_pd_horizon_s = 0, reserved = 0;
  TraceFeedback feedback;
  TraceCommand wbc_command, output_command;
  std::array<double, kJointCount> policy_position{}, policy_velocity{};
  std::array<double, kJointCount> nominal_position{}, nominal_velocity{};
  std::array<double, kJointCount> previous_nominal_position{}, previous_nominal_velocity{};
  std::array<double, kJointCount> node_left_position{}, node_right_position{};
  std::array<double, kJointCount> node_left_velocity{}, node_right_velocity{};
  std::array<double, kJointCount> qp_torque{}, qp_pd_nominal{}, qp_pd_lower{}, qp_pd_upper{};
  std::array<double, kJointCount> output_p{}, output_d{}, output_total{};
  std::array<double, 3> base_position{}, base_velocity{}, base_euler_zyx{};
};

// Motor flags: 1 attempted, 2 received, 4 decode attempted, 8 decoded,
// 16 SDK correct, 32 ID match, 64 mode match, 128 finite,
// 256 frame valid, 512 success. Non-attempted slots contain no packet.
struct MotorPacketRecord {
  std::uint64_t sdk_start_ns = 0, sdk_return_ns = 0, parsed_ns = 0, flags = 0;
  std::int64_t received_id = -1, received_mode = -1, motor_error = -1;
  double position = 0, velocity = 0, effort = 0, kp = 0, kd = 0;
  double gear = 0, direction = 0, calibration_offset = 0;
  std::array<std::uint8_t, 17> tx{};
  std::array<std::uint8_t, 16> rx{};
  std::array<std::uint8_t, 7> reserved{};
};
struct SdkRoundRecord {
  // kind: 1 ordinary SendReceive, 2 internal alignment, 3 abort/stop.
  std::uint64_t kind = 0, start_ns = 0, end_ns = 0;
  std::array<MotorPacketRecord, kJointCount> motors{};
};
struct SdkCycleRecord {
  std::uint64_t round_count = 0, omitted_rounds = 0;
  std::array<SdkRoundRecord, 2> rounds{};
};
// Hardware flags and switches are documented by the producer and parser.
struct HardwareCycleRecord {
  std::uint64_t read_sequence = 0, manager_time_ns = 0, source_write_sequence = 0, source_write_time_ns = 0;
  std::uint64_t source_write_steady_ns = 0, read_start_ns = 0, read_end_ns = 0, flags = 0;
  double calibration_request = 0, emergency_stop_request = 0, motion_request = 0, motion_enabled = 0;
  double calibrated_before = 0, calibrated_after = 0, alignment_before = 0, alignment_after = 0;
  TraceCommand requested_command, staged_command, sdk_command;
  TraceFeedback before_feedback, after_feedback;
  std::array<double, kJointCount> staged_mode{}, sdk_mode{};
  SdkCycleRecord sdk;
};

static_assert(sizeof(TraceCommand) == 480 && sizeof(TraceFeedback) == 384);
static_assert(sizeof(MotorPacketRecord) == 160 && sizeof(SdkRoundRecord) == 1944);
static_assert(std::is_trivially_copyable_v<ControllerCycleRecord>);
static_assert(std::is_trivially_copyable_v<HardwareCycleRecord>);
}  // namespace custom_dog_control::diagnostics
