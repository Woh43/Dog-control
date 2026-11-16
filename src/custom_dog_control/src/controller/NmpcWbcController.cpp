#include "custom_dog_control/controller/NmpcWbcController.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <pluginlib/class_list_macros.hpp>

#include "custom_dog_control/control/ImuFreshness.hpp"

namespace custom_dog_control {
namespace {

// 限制速度参考领先实测同向速度的幅度，避免机器人尚未跟上时目标位姿持续跑远。
// 减速请求可直接通过；实测速度反向时以零作为基线。这不是按 dt 限制加速度的 Slew。
double LimitAcceleratingReference(
    double requested, double measured, double max_lead) {
  if (requested > 0.0) {
    return std::min(requested, std::max(0.0, measured) + max_lead);
  }
  if (requested < 0.0) {
    return std::max(requested, std::min(0.0, measured) - max_lead);
  }
  return 0.0;
}

}  // namespace

controller_interface::return_type NmpcWbcController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  // 估计、限速及 WBC 使用有界步长；诊断仍记录真实周期，以免掩盖调度抖动或超时。
  const auto trace_start_ns = cycle_trace_ && cycle_trace_->enabled()
      ? diagnostics::TraceSteadyNs() : 0;
  const double dt = std::clamp(period.seconds(), 1e-4, 0.02);
  control_period_ms_ = period.seconds() * 1000.0;
  control_timing_.Add(control_period_ms_);
  ReadHardwareState();
  if (IsRealHardware()) {
    io_timing_.Add(io_period_ms_);
  }
  // 回调只更新输入缓冲；本周期读取快照后统一仲裁速度、推进状态和写入硬件。
  imu_sample_ = *imu_buffer_.readFromRT();
  const JoyInput joy = *joy_buffer_.readFromRT();
  // A real IMU callback can arrive after the manager captured this cycle's
  // time. Read the same ROS clock only AFTER copying the input snapshot;
  // otherwise a newly received sample can be mistaken for a future sample.
  // Simulation retains the physics update timestamp and its clock domain.
  const rclcpp::Time state_time =
      IsRealHardware() ? get_node()->get_clock()->now() : time;
  const double now_seconds = state_time.seconds();
  joints_state_.stamp_valid = !IsRealHardware();
  if (joints_state_.stamp_valid) joints_state_.stamp_seconds = now_seconds;
  const VelocityCommand command = SelectVelocityCommand(now_seconds, dt);

  const bool estimation_ready = !IsRealHardware() ||
      (calibrated_ && !reset_calibration_pending_ &&
       mode_ != OperatingMode::FAULT &&
       (!requires_recalibration_ || mode_ == OperatingMode::CALIBRATION));
  if (calibration_gate_.Update(estimation_ready)) {
    simulation_effort_delay_.Reset();
    estimator_->Reset(nominal_height_m_);
    backend_->InvalidateObservation();
    estimate_ = EstimatedState{};
    last_policy_ = PolicySample{};
    policy_buffer_.writeFromNonRT(PolicySample{});
    measured_rbd_state_.resize(0);
    last_target_update_seconds_ = -1.0;
    target_command_was_moving_ = false;
  }

  // 估计器先使用上一份有效策略的接触计划；首次获得策略前按四足支撑初始化。
  // planned contacts 是模型假设，不能当作真实足端触地测量。
  std::size_t planned_mode = last_policy_.valid ? last_policy_.mode : kStanceMode;
  const auto contacts = ContactFlags(planned_mode);
  const auto imu_freshness = EvaluateImuFreshness(
      imu_sample_, now_seconds, safety_monitor_->limits().imu_timeout_s);
  const bool imu_fresh = imu_freshness == ImuFreshnessStatus::Fresh;
  imu_orientation_prediction_s_ = 0.0;
  const char* estimate_invalid_reason = "kinematic estimator returned invalid state";
  if (!estimation_ready) {
    estimate_invalid_reason = "calibration/operating-mode gate is closed";
    estimate_ = EstimatedState{};
  } else if (IsRealHardware() && !communication_ok_) {
    estimate_invalid_reason = "motor communication is invalid";
    estimate_ = EstimatedState{};
  } else if (IsRealHardware() && !imu_fresh) {
    estimate_invalid_reason = ImuFreshnessReason(imu_freshness);
    estimate_ = EstimatedState{};
  } else if (!IsRealHardware() && use_sim_ground_truth_) {
    estimate_invalid_reason = "simulation ground truth is invalid or stale";
    estimate_ = *ground_truth_buffer_.readFromRT();
    estimate_.valid =
        estimate_.valid &&
        now_seconds - estimate_.stamp_seconds <= ground_truth_timeout_s_;
  } else {
    // Predict only a fresh attitude's known host age, never an assumed device
    // delay. Keep the original IMU timestamp for freshness and diagnostics.
    if (imu_fresh) {
      imu_orientation_prediction_s_ = std::min(
          now_seconds - imu_sample_.stamp_seconds, imu_orientation_prediction_max_s_);
    }
    estimate_ = estimator_->Update(joints_state_, imu_sample_, contacts, dt,
                                   imu_orientation_prediction_s_);
  }
  // 无效估计不推进优化器观测/目标；后面的安全检查和状态机仍会执行。
  if (estimate_.valid) {
    observation_time_ += dt;
    measured_rbd_state_ = backend_->UpdateObservation(
        estimate_, joints_state_, observation_time_, planned_mode);
    // 原始速度请求仍交给步态监督器；在新 Trot 策略确认前，给后端的速度参考保持零。
    // 否则求解器仍按四足站立约束工作时，运动目标已经向前推进，会积累跟踪误差。
    VelocityCommand backend_command = command;
    if (mode_ != OperatingMode::MPC_TROT) {
      backend_command.vx = 0.0;
      backend_command.vy = 0.0;
      backend_command.yaw = 0.0;
    } else {
      // 只按偏航角把世界系水平速度投影到机体前向/侧向，与速度指令使用同一平面基准。
      // 这里不是包含 roll/pitch 的完整三维坐标变换。
      const double yaw = estimate_.euler_zyx.x();
      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      const double measured_vx_body =
          cos_yaw * estimate_.velocity_world.x() +
          sin_yaw * estimate_.velocity_world.y();
      const double measured_vy_body =
          -sin_yaw * estimate_.velocity_world.x() +
          cos_yaw * estimate_.velocity_world.y();
      backend_command.vx = LimitAcceleratingReference(
          command.vx, measured_vx_body, max_reference_lead_xy_m_s_);
      backend_command.vy = LimitAcceleratingReference(
          command.vy, measured_vy_body, max_reference_lead_xy_m_s_);
      backend_command.yaw = LimitAcceleratingReference(
          command.yaw, estimate_.angular_velocity_world.z(),
          max_reference_lead_yaw_rad_s_);
    }
    governed_command_ = backend_command;
    const bool target_command_is_moving =
        std::max({std::abs(backend_command.vx),
                  std::abs(backend_command.vy),
                  std::abs(backend_command.yaw)}) > 1e-3;
    // 连续运动参考至多每 50 ms 更新一次；首次目标、停车和站立漂移重锚定走事件更新。
    const bool moving_target_update_due =
        target_command_is_moving &&
        now_seconds - last_target_update_seconds_ >= 0.05;
    const double stance_anchor_error = std::hypot(
        estimate_.position.x() - stance_anchor_xy_[0],
        estimate_.position.y() - stance_anchor_xy_[1]);
    const bool stance_reanchor_due =
        mode_ == OperatingMode::MPC_STANCE && !target_command_is_moving &&
        stance_anchor_error >= stance_reanchor_distance_m_;
    // 两次更新之间保持参考不变，避免位置控制向 WBC 交接期间不断扰动求解目标。
    if (last_target_update_seconds_ < 0.0 || moving_target_update_due ||
        stance_reanchor_due ||
        (target_command_was_moving_ && !target_command_is_moving)) {
      backend_->SetVelocityCommand(
          backend_command,
          stance_reanchor_due ||
              (target_command_was_moving_ && !target_command_is_moving));
      last_target_update_seconds_ = now_seconds;
      if (!target_command_is_moving) {
        stance_anchor_xy_ = {estimate_.position.x(), estimate_.position.y()};
      }
    }
    target_command_was_moving_ = target_command_is_moving;
  }

  // 只有评估成功才替换策略缓存；序号变化才计入一次 MPC 求解耗时，
  // 因为同一份后台策略会被高频控制循环重复评估。
  PolicySample evaluated_policy;
  const bool evaluated = estimate_.valid &&
                         backend_->EvaluatePolicy(now_seconds, evaluated_policy);
  if (evaluated) {
    policy_buffer_.writeFromNonRT(evaluated_policy);
    if (evaluated_policy.sequence != last_timed_policy_sequence_) {
      mpc_timing_.Add(evaluated_policy.solve_time_ms);
      last_timed_policy_sequence_ = evaluated_policy.sequence;
    }
  }
  last_policy_ = *policy_buffer_.readFromRT();
  // 真机用单调墙钟；仿真用仿真时钟，与对应的求解节拍和物理推进保持一致。
  const double policy_age = backend_->policyAgeSeconds(now_seconds);
  const bool solver_healthy = backend_->solverHealthy();
  const bool policy_available = last_policy_.valid && solver_healthy &&
      policy_age <= safety_monitor_->limits().max_policy_age_s;

  // 起身和 MPC 活动模式都检查估计及求解健康；策略时效用于 MPC 站立和 Trot。
  // 安全故障先于本周期的步态监督处理，防止继续发出正常运动切换请求。
  if (mode_ != OperatingMode::PASSIVE &&
      mode_ != OperatingMode::CALIBRATION &&
      mode_ != OperatingMode::FAULT) {
    if (!estimate_.valid || !solver_healthy) {
      // Preserve the triggering snapshot: the next FAULT cycle intentionally
      // invalidates estimation/policy, so its diagnostics cannot explain why
      // this transition occurred. Do not replace this with later consequences.
      std::ostringstream reason;
      reason << std::setprecision(17)
             << (!estimate_.valid ? "State estimate invalid: " : "NMPC solver failure: ")
             << (!estimate_.valid ? estimate_invalid_reason : "backend unhealthy")
             << "; cycle_time=" << time.seconds()
             << "; check_time=" << now_seconds
             << "; imu_stamp=" << imu_sample_.stamp_seconds
             << "; imu_age_s=" << now_seconds - imu_sample_.stamp_seconds
             << "; imu_valid=" << imu_sample_.valid
             << "; communication_ok=" << communication_ok_
             << "; io_failures=" << consecutive_io_failures_
             << "; solver_healthy=" << solver_healthy
             << "; mpc_error=" << backend_->lastError()
             << "; estimate_position=" << estimate_.position.transpose()
             << "; estimate_velocity=" << estimate_.velocity_world.transpose();
      for (std::size_t i = 0; i < kJointCount; ++i) {
        if (joints_state_.valid[i] < 0.5 ||
            !std::isfinite(joints_state_.position[i]) ||
            !std::isfinite(joints_state_.velocity[i])) {
          reason << "; invalid_joint=" << i
                 << ",valid=" << joints_state_.valid[i]
                 << ",q=" << joints_state_.position[i]
                 << ",dq=" << joints_state_.velocity[i];
        }
      }
      controller_fault_reason_ = reason.str();
      RCLCPP_ERROR(get_node()->get_logger(), "%s", controller_fault_reason_.c_str());
      TransitionTo(OperatingMode::FAULT, now_seconds);
    }
    SafetyInput safety_input;
    safety_input.joints = joints_state_;
    safety_input.imu = imu_sample_;
    safety_input.now_seconds = now_seconds;
    safety_input.policy_age_s = policy_age;
    safety_input.roll = estimate_.euler_zyx.z();
    safety_input.pitch = estimate_.euler_zyx.y();
    safety_input.horizontal_speed_m_s = estimate_.velocity_world.head<2>().norm();
    safety_input.consecutive_io_failures = consecutive_io_failures_;
    safety_input.communication_ok = communication_ok_;
    safety_input.physical_estop = physical_estop_;
    safety_input.solver_valid = solver_healthy;
    safety_input.dynamic_mode = mode_ == OperatingMode::MPC_TROT ||
                                mode_ == OperatingMode::MPC_STANCE;
    if (!safety_monitor_->Evaluate(safety_input)) {
      if (controller_fault_reason_.empty()) {
        // Latch the exact safety input before FAULT invalidates the policy.
        // lastSolveTimeMs is an asynchronous diagnostic sample, not an input
        // to this decision and not necessarily the solve for this policy.
        std::ostringstream reason;
        reason << std::setprecision(17)
               << "Safety fault: " << safety_monitor_->reason()
               << "; mode=" << ToString(mode_)
               << "; cycle_time=" << time.seconds()
               << "; check_time=" << safety_input.now_seconds
               << "; policy_age_s=" << safety_input.policy_age_s
               << "; max_policy_age_s=" << safety_monitor_->limits().max_policy_age_s
               << "; policy_sequence=" << last_policy_.sequence
               << "; policy_valid=" << last_policy_.valid
               << "; policy_evaluated=" << evaluated
               << "; solver_healthy=" << safety_input.solver_valid
               << "; last_solve_ms_async=" << backend_->lastSolveTimeMs()
               << "; control_period_ms=" << control_period_ms_
               << "; io_period_ms=" << io_period_ms_
               << "; io_failures=" << safety_input.consecutive_io_failures
               << "; communication_ok=" << safety_input.communication_ok
               << "; imu_age_s=" << safety_input.now_seconds - safety_input.imu.stamp_seconds
               << "; estimate_valid=" << estimate_.valid;
        controller_fault_reason_ = reason.str();
        RCLCPP_ERROR(get_node()->get_logger(), "%s", controller_fault_reason_.c_str());
      } else {
        // An estimate/solver failure earlier in this cycle remains primary.
        RCLCPP_ERROR(
            get_node()->get_logger(), "Safety fault: %s",
            safety_monitor_->reason().c_str());
      }
      TransitionTo(OperatingMode::FAULT, now_seconds);
    }
  }

  if (mode_ != OperatingMode::FAULT) {
    UpdateLocomotionSupervisor(
        now_seconds, command, joy, last_policy_, policy_age);
  }

  // 状态机负责起身 PD、WBC 输出和短暂失败回退；最后集中决定正常输出或安全输出。
  // state_ok 只说明状态机已处理本周期，不能替代对 PASSIVE/CALIBRATION/FAULT 的判断。
  HybridJointCommand hybrid_command;
  WbcOutput wbc;
  const bool state_ok = ApplyStateMachine(
      now_seconds, dt, joy, last_policy_, policy_available,
      hybrid_command, wbc);
  if (!state_ok || mode_ == OperatingMode::PASSIVE ||
      mode_ == OperatingMode::CALIBRATION ||
      mode_ == OperatingMode::FAULT) {
    WriteSafeCommand();
  } else {
    WriteHybridCommand(hybrid_command);
  }

  if (trace_start_ns != 0) {
    RecordCycleTrace(time, trace_start_ns, now_seconds, evaluated, policy_available, policy_age, wbc);
  }
  PublishDiagnostics(state_time, estimate_, last_policy_, wbc, planned_mode);
  // FAULT 由内部模式和安全命令处理；这里的 OK 表示完成 update，不表示机器人无故障。
  return controller_interface::return_type::OK;
}

void NmpcWbcController::RecordCycleTrace(
    const rclcpp::Time& manager_time, std::uint64_t start_ns,
    double imu_use_ros_s, bool evaluated, bool available, double policy_age, const WbcOutput& wbc) {
  diagnostics::ControllerCycleRecord record;
  record.cycle = ++cycle_trace_sequence_;
  // Keep update's ROS timestamp for context. Some ros2_control versions sample
  // a different ROS timestamp for write; align by the retained monotonic write
  // time within controller end/next-start, verified against all five fields.
  record.manager_time_ns = manager_time.nanoseconds();
  record.start_ns = start_ns;
  record.policy_sequence = last_policy_.sequence;
  record.mode = static_cast<std::uint64_t>(mode_);
  record.policy_mode = last_policy_.mode;
  record.flags = (evaluated ? 1 : 0) | (available ? 2 : 0) |
      (estimate_.valid ? 4 : 0) | (communication_ok_ ? 8 : 0) |
      (wbc.valid ? 16 : 0) | (command_output_active_ ? 32 : 0) |
      (host_total_torque_command_ ? 128 : 0);
  record.period_s = control_period_ms_ * 0.001;
  record.policy_age_s = policy_age;
  record.feedback = diagnostics::ToTraceFeedback(joints_state_);
  if (last_policy_.valid) backend_->FillCycleTraceReferences(record);
  record.wbc_command = diagnostics::ToTraceCommand(wbc.command);
  record.qp_torque = wbc.trace_qp_torque;
  record.qp_pd_nominal = wbc.trace_pd_nominal;
  record.qp_pd_lower = wbc.trace_pd_lower;
  record.qp_pd_upper = wbc.trace_pd_upper;
  record.wbc_solve_ms = wbc.solve_time_ms;
  record.equality_residual = wbc.equality_residual;
  record.inequality_violation = wbc.inequality_violation;
  record.motor_pd_horizon_s = motor_pd_window_model_.horizon_seconds;
  const double handoff_phase = std::clamp(
      (imu_use_ros_s - state_entered_seconds_) / std::max(0.1, handoff_duration_s_), 0.0, 1.0);
  record.handoff_alpha = mode_ == OperatingMode::MPC_STANCE && stance_handoff_active_
      ? handoff_phase * handoff_phase * (3.0 - 2.0 * handoff_phase) : 1.0;
  // The state's latch persists after alpha reaches one. Flag only the actual
  // blending interval, so steady-stance reference checks remain eligible.
  if (record.handoff_alpha < 1.0) record.flags |= 64;
  record.imu_stamp_s = imu_sample_.stamp_seconds;
  record.imu_use_ros_s = imu_use_ros_s;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    auto& c = record.output_command;
    c.position[i] = command_interfaces_[command_interface_indices_[i][0]].get_value();
    c.velocity[i] = command_interfaces_[command_interface_indices_[i][1]].get_value();
    c.effort[i] = command_interfaces_[command_interface_indices_[i][2]].get_value();
    c.kp[i] = command_interfaces_[command_interface_indices_[i][3]].get_value();
    c.kd[i] = command_interfaces_[command_interface_indices_[i][4]].get_value();
    record.output_p[i] = c.kp[i] * (c.position[i] - joints_state_.position[i]);
    record.output_d[i] = c.kd[i] * (c.velocity[i] - joints_state_.velocity[i]);
    record.output_total[i] = c.effort[i] + record.output_p[i] + record.output_d[i];
  }
  for (std::size_t i = 0; i < 3; ++i) {
    record.base_position[i] = estimate_.position(i);
    record.base_velocity[i] = estimate_.velocity_world(i);
    record.base_euler_zyx[i] = estimate_.euler_zyx(i);
  }
  record.end_ns = diagnostics::TraceSteadyNs();
  cycle_trace_->tryPush(record);
}

}  // namespace custom_dog_control

// 与插件描述 XML 配合，使 controller_manager 能按插件名创建本控制器。
PLUGINLIB_EXPORT_CLASS(
    custom_dog_control::NmpcWbcController,
    controller_interface::ControllerInterface)
