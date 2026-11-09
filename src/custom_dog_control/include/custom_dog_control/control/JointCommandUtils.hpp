#pragma once

#include <algorithm>
#include <cmath>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

// 四腿复用 hip/thigh/calf 三个刚度；默认零目标速度、零前馈力矩。
inline HybridJointCommand PositionPdCommand(
    const std::array<double, kJointCount>& position,
    const std::array<double, kJointsPerLeg>& stiffness, double damping) {
  HybridJointCommand command;
  command.position = position;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    command.kp[i] = stiffness[i % kJointsPerLeg];
    command.kd[i] = damping;
  }
  return command;
}

inline double EquivalentEffort(
    const HybridJointCommand& command, const JointSample& measured,
    std::size_t joint) {
  return command.effort[joint] +
         command.kp[joint] * (command.position[joint] - measured.position[joint]) +
         command.kd[joint] * (command.velocity[joint] - measured.velocity[joint]);
}

// Output boundary: clamp the combined PD/feedforward effort.
// A zero gain does not suppress NaN arithmetic (0 * NaN is NaN), so an invalid
// sample must produce a finite disabled output instead of poisoning physics.
inline double BoundedEquivalentEffort(
    const HybridJointCommand& command, const JointSample& measured,
    std::size_t joint, double effort_limit) {
  const double effort = EquivalentEffort(command, measured, joint);
  if (!std::isfinite(effort) || !std::isfinite(effort_limit) || effort_limit < 0.0) {
    return 0.0;
  }
  return std::clamp(effort, -effort_limit, effort_limit);
}

// Keep motor-side PD and its position/velocity targets. At this feedback sample,
// jointly scale FF, Kp and Kd so |FF| + Kp*|position error| + Kd*|velocity error|
// fits the joint effort limit without relying on cancellation between terms.
// This is a sampled command check, not continuous motor torque saturation:
// feedback changes between host samples while the motor continues evaluating PD.
// Reject the entire frame on invalid feedback, command, gain or arithmetic.
// Gain caps are software configuration limits in joint-side SI units.
inline bool BoundedMotorPdCommand(
    const HybridJointCommand& command, const JointSample& measured,
    const std::array<double, kJointCount>& effort_limits,
    HybridJointCommand& output, double max_kp = 200.0, double max_kd = 10.0) {
  output = {};
  if (!std::isfinite(max_kp) || max_kp < 0.0 ||
      !std::isfinite(max_kd) || max_kd < 0.0) {
    return false;
  }
  std::array<double, kJointCount> contribution_bounds{};
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!std::isfinite(measured.valid[i]) || measured.valid[i] < 0.5 ||
        !std::isfinite(measured.position[i]) ||
        !std::isfinite(measured.velocity[i]) ||
        !std::isfinite(command.position[i]) ||
        !std::isfinite(command.velocity[i]) ||
        !std::isfinite(command.effort[i]) ||
        !std::isfinite(command.kp[i]) || command.kp[i] < 0.0 ||
        command.kp[i] > max_kp ||
        !std::isfinite(command.kd[i]) || command.kd[i] < 0.0 ||
        command.kd[i] > max_kd ||
        !std::isfinite(effort_limits[i]) || effort_limits[i] <= 0.0) {
      return false;
    }
    contribution_bounds[i] = std::abs(command.effort[i]) +
        command.kp[i] * std::abs(command.position[i] - measured.position[i]) +
        command.kd[i] * std::abs(command.velocity[i] - measured.velocity[i]);
    if (!std::isfinite(contribution_bounds[i])) return false;
  }
  output = command;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double scale = contribution_bounds[i] > effort_limits[i]
        ? effort_limits[i] / contribution_bounds[i] : 1.0;
    output.effort[i] *= scale;
    output.kp[i] *= scale;
    output.kd[i] *= scale;
  }
  return true;
}

// 同时插值增益和目标会在 PD 力矩中产生交叉项，故用前馈补偿，保证在
// 当前 measured 下总力矩等于两端总力矩的线性插值。调用方提供 [0,1] 的 alpha；
// 此处不限幅，后续输出饱和以及下一周期测量变化都会影响实际力矩。
inline HybridJointCommand BlendHybridCommands(
    const HybridJointCommand& from, const HybridJointCommand& to,
    const JointSample& measured, double alpha) {
  HybridJointCommand command;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    command.position[i] = (1.0 - alpha) * from.position[i] + alpha * to.position[i];
    command.velocity[i] = (1.0 - alpha) * from.velocity[i] + alpha * to.velocity[i];
    command.kp[i] = (1.0 - alpha) * from.kp[i] + alpha * to.kp[i];
    command.kd[i] = (1.0 - alpha) * from.kd[i] + alpha * to.kd[i];
    const double net_effort =
        (1.0 - alpha) * EquivalentEffort(from, measured, i) +
        alpha * EquivalentEffort(to, measured, i);
    command.effort[i] = net_effort -
        command.kp[i] * (command.position[i] - measured.position[i]) -
        command.kd[i] * (command.velocity[i] - measured.velocity[i]);
  }
  return command;
}

}  // namespace custom_dog_control
