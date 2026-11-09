#include "custom_dog_control/controller/NmpcWbcController.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <locale>
#include <sstream>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace custom_dog_control {
namespace {

diagnostic_msgs::msg::KeyValue KeyValue(
    const std::string& key, const std::string& value) {
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

std::string Number(double value) {
  std::ostringstream stream;
  stream.precision(6);
  stream << value;
  return stream.str();
}

// Numeric diagnostic arrays are JSON, including unavailable/non-finite entries.
// Use a fixed locale so a process locale cannot change decimal points to commas.
template <typename Values>
std::string JsonNumberArray(const Values& values) {
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream.precision(std::numeric_limits<double>::max_digits10);
  stream << '[';
  bool first = true;
  for (const double value : values) {
    if (!first) stream << ',';
    first = false;
    if (std::isfinite(value)) stream << value;
    else stream << "null";
  }
  stream << ']';
  return stream.str();
}

const char* JoystickRequestName(JoystickModeRequest request) {
  switch (request) {
    case JoystickModeRequest::NONE: return "NONE";
    case JoystickModeRequest::PASSIVE: return "PASSIVE";
    case JoystickModeRequest::CALIBRATE: return "CALIBRATE";
    case JoystickModeRequest::STANCE: return "STANCE";
    case JoystickModeRequest::TROT: return "TROT";
    case JoystickModeRequest::ESTOP: return "ESTOP";
  }
  return "UNKNOWN";
}

const char* StartupAlignmentName(double value) {
  if (value == 0.0) return "disabled";
  if (value == 1.0) return "waiting";
  if (value == 2.0) return "hip_thigh";
  if (value == 3.0) return "all_joints";
  if (value == 4.0) return "release";
  if (value == 5.0) return "done";
  if (value == 6.0) return "aborted_restart_required";
  return "invalid";
}

Eigen::Quaterniond QuaternionFromZyx(const Eigen::Vector3d& zyx) {
  return Eigen::AngleAxisd(zyx.x(), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(zyx.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(zyx.z(), Eigen::Vector3d::UnitX());
}

}  // namespace

void NmpcWbcController::PublishDiagnostics(
    const rclcpp::Time& stamp, const EstimatedState& estimate,
    const PolicySample& policy, const WbcOutput& wbc,
    std::size_t estimator_contact_mode) {
  // Optional simulation-only trace: one sample per controller update, before
  // 10 Hz diagnostic throttling. Default off; never publishes on real hardware.
  if (simulation_trace_publisher_) {
    std_msgs::msg::Float64MultiArray trace;
    trace.data.reserve(146);
    trace.data = {stamp.seconds(), static_cast<double>(mode_),
                  static_cast<double>(policy.mode), static_cast<double>(policy.sequence),
                  static_cast<double>(estimator_contact_mode), policy.observation_time,
                  estimate.valid ? 1.0 : 0.0,
                  estimate.position.x(), estimate.position.y(), estimate.position.z(),
                  estimate.velocity_world.x(), estimate.velocity_world.y(), estimate.velocity_world.z(),
                  wbc.valid ? 1.0 : 0.0};
    const auto append = [&trace](const auto& values) {
      trace.data.insert(trace.data.end(), values.begin(), values.end());
    };
    append(joints_state_.position);
    append(joints_state_.velocity);
    append(joints_state_.effort);
    append(policy.state);
    append(policy.input);
    append(wbc.command.position);
    append(wbc.command.velocity);
    append(wbc.command.effort);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      trace.data.push_back(command_interfaces_[command_interface_indices_[i][0]].get_value());
    }
    trace.layout.dim.resize(1);
    trace.layout.dim[0].label = "custom_dog_sim_control_trace_v1";
    trace.layout.dim[0].size = trace.data.size();
    trace.layout.dim[0].stride = trace.data.size();
    simulation_trace_publisher_->publish(trace);
  }
  // 诊断按 ROS 时间限频到约 10 Hz；统计窗口在控制循环中持续采样。
  // 这里仍有消息分配、排序和发布开销，并非独立的实时安全发布线程。
  if (last_diagnostics_seconds_ >= 0.0 &&
      stamp.seconds() - last_diagnostics_seconds_ < 0.10) {
    return;
  }
  last_diagnostics_seconds_ = stamp.seconds();

  std_msgs::msg::String mode_message;
  mode_message.data = std::string(ToString(mode_));
  mode_publisher_->publish(mode_message);

  std_msgs::msg::Float64MultiArray contact_message;
  // 发布的是策略接触计划；无有效策略时显示全支撑，不表示检测到了触地。
  const auto contacts = ContactFlags(policy.valid ? policy.mode : kStanceMode);
  contact_message.data.reserve(kLegCount);
  for (const bool contact : contacts) {
    contact_message.data.push_back(contact ? 1.0 : 0.0);
  }
  contact_publisher_->publish(contact_message);

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "custom_dog_control/nmpc_wbc";
  status.hardware_id = IsRealHardware() ? "custom_dog_rs485" : "gazebo";
  status.level =
      mode_ == OperatingMode::FAULT
          ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
          : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = std::string(ToString(mode_));
  if (mode_ == OperatingMode::FAULT) {
    if (!controller_fault_reason_.empty()) {
      status.message = controller_fault_reason_;
    } else if (safety_monitor_->faulted()) {
      status.message = safety_monitor_->reason();
    }
  }
  if (IsRealHardware() && startup_alignment_status_ == 6.0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "Startup alignment aborted; restart required";
  }
  const auto control_timing = control_timing_.Snapshot();
  const auto io_timing = io_timing_.Snapshot();
  const auto mpc_timing = mpc_timing_.Snapshot();
  double wbc_tau_max_abs = 0.0;
  for (const double effort : wbc.command.effort) {
    wbc_tau_max_abs = std::max(wbc_tau_max_abs, std::abs(effort));
  }
  // These are the controller's exported requests after WriteHybridCommand or
  // WriteSafeCommand, before the hardware write boundary scales PD/FF or encodes
  // SDK packets. Effort-only simulation has no position/velocity/gain interfaces.
  const bool hybrid_interfaces = IsRealHardware() || IsHybridSimulation();
  std::array<std::array<double, kJointCount>, 5> interface_requests;
  for (auto& values : interface_requests) {
    values.fill(std::numeric_limits<double>::quiet_NaN());
  }
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    if (hybrid_interfaces) {
      for (std::size_t component = 0; component < interface_requests.size(); ++component) {
        interface_requests[component][joint] =
            command_interfaces_[command_interface_indices_[joint][component]].get_value();
      }
    } else {
      interface_requests[2][joint] =
          command_interfaces_[command_interface_indices_[joint][0]].get_value();
    }
  }
  // Compare against the final configured stand-up target, not its current
  // interpolated target. Invalid feedback does not participate in this maximum.
  double stand_up_final_target_max_error = 0.0;
  std::size_t stand_up_final_target_max_error_joint = kJointCount;
  std::size_t stand_up_final_target_valid_joints = 0;
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    if (!std::isfinite(joints_state_.valid[joint]) || joints_state_.valid[joint] < 0.5 ||
        !std::isfinite(joints_state_.position[joint]) ||
        !std::isfinite(stand_up_joint_positions_[joint])) {
      continue;
    }
    const double error =
        std::abs(joints_state_.position[joint] - stand_up_joint_positions_[joint]);
    if (!std::isfinite(error)) continue;
    ++stand_up_final_target_valid_joints;
    if (stand_up_final_target_max_error_joint == kJointCount ||
        error > stand_up_final_target_max_error) {
      stand_up_final_target_max_error = error;
      stand_up_final_target_max_error_joint = joint;
    }
  }
  // This latest buffered snapshot may arrive after the update's earlier Joy
  // snapshot. It reports the received request, not a latched/accepted mode.
  const JoyInput diagnostic_joy = *joy_buffer_.readFromRT();
  const double diagnostic_joy_age = stamp.seconds() - diagnostic_joy.stamp_seconds;
  status.values = {
      KeyValue("mode", std::string(ToString(mode_))),
      KeyValue("mode_elapsed_s", Number(stamp.seconds() - state_entered_seconds_)),
      KeyValue("joint_array_order", "FR,FL,RR,RL; each hip,thigh,calf"),
      KeyValue("joint_array_encoding", "JSON numeric arrays; null means unavailable or non-finite"),
      KeyValue("joint_position_rad", JsonNumberArray(joints_state_.position)),
      KeyValue("joint_velocity_rad_s", JsonNumberArray(joints_state_.velocity)),
      KeyValue("joint_feedback_valid", JsonNumberArray(joints_state_.valid)),
      KeyValue("joint_effort_feedback", JsonNumberArray(joints_state_.effort)),
      KeyValue(
          "joint_effort_feedback_semantics",
          IsRealHardware()
              ? "SDK effort feedback mapped by gear ratio/direction; physical torque meaning and calibration unverified; not measured foot force"
              : (IsHybridSimulation()
                     ? "simulated motor-PD requested joint effort; not measured hardware torque or foot force"
                     : "simulation joint effort state interface; not measured hardware torque or foot force")),
      KeyValue("joint_effort_limits_nm", JsonNumberArray(safety_monitor_->limits().effort_limit)),
      KeyValue("stand_up_final_target_position_rad", JsonNumberArray(stand_up_joint_positions_)),
      KeyValue(
          "stand_up_final_target_max_error_rad",
          stand_up_final_target_valid_joints > 0 ? Number(stand_up_final_target_max_error) : "null"),
      KeyValue(
          "stand_up_final_target_max_error_joint_index",
          stand_up_final_target_valid_joints > 0
              ? std::to_string(stand_up_final_target_max_error_joint) : "null"),
      KeyValue(
          "stand_up_final_target_max_error_joint_name",
          stand_up_final_target_valid_joints > 0
              ? joints_[stand_up_final_target_max_error_joint] : "unavailable"),
      KeyValue("stand_up_final_target_error_valid_joint_count", std::to_string(stand_up_final_target_valid_joints)),
      KeyValue("stand_up_final_target_error_semantics", "maximum over valid finite joint positions only; final target, not interpolated target"),
      KeyValue("controller_request_interface_layout", hybrid_interfaces ? "position,velocity,effort,kp,kd" : "effort_only"),
      KeyValue("controller_request_position_rad", JsonNumberArray(interface_requests[0])),
      KeyValue("controller_request_velocity_rad_s", JsonNumberArray(interface_requests[1])),
      KeyValue("controller_request_effort_nm", JsonNumberArray(interface_requests[2])),
      KeyValue("controller_request_kp_nm_rad", JsonNumberArray(interface_requests[3])),
      KeyValue("controller_request_kd_nm_s_rad", JsonNumberArray(interface_requests[4])),
      KeyValue("controller_request_semantics", "controller command-interface request; not final motor packet or measured torque; hardware may scale/disable it"),
      KeyValue("joy_requested_mode", JoystickRequestName(diagnostic_joy.requested_mode)),
      KeyValue("joy_age_s", Number(diagnostic_joy_age)),
      KeyValue("joy_request_snapshot", "latest buffered Joy at diagnostic sampling; may postdate the control update snapshot; not an accepted mode"),
      KeyValue("limited_command_exceeds_trot_entry_threshold", ExceedsTrotEntryThreshold(limited_command_) ? "true" : "false"),
      KeyValue("controller_fault_reason", controller_fault_reason_),
      KeyValue("calibrated", calibrated_ ? "true" : "false"),
      KeyValue("startup_alignment", Number(startup_alignment_status_)),
      KeyValue("startup_alignment_phase", StartupAlignmentName(startup_alignment_status_)),
      KeyValue("requires_recalibration", requires_recalibration_ ? "true" : "false"),
      KeyValue("estimate_valid", estimate.valid ? "true" : "false"),
      KeyValue("imu_valid", imu_sample_.valid ? "true" : "false"),
      KeyValue("imu_age_s", Number(stamp.seconds() - imu_sample_.stamp_seconds)),
      KeyValue("trot_enabled", trot_enabled_ ? "true" : "false"),
      KeyValue(
          "gait_transition",
          gait_transition_ == GaitTransition::STARTING_TROT
              ? "STARTING_TROT"
              : (gait_transition_ == GaitTransition::STOPPING_TROT
                     ? "STOPPING_TROT"
                     : "NONE")),
      KeyValue("policy_mode", std::to_string(policy.mode)),
      KeyValue("nmpc_backend", "legged::LeggedInterface"),
      KeyValue("wbc_backend", "legged::WeightedWbc"),
      KeyValue(
          "legged_control_revision",
          "a7f381c0367e98e31c01336e678eef47e304d40d"),
      KeyValue("control_period_ms", Number(control_period_ms_)),
      KeyValue("control_period_mean_ms", Number(control_timing.mean)),
      KeyValue("control_period_p95_ms", Number(control_timing.p95)),
      KeyValue("control_period_p99_ms", Number(control_timing.p99)),
      KeyValue("io_period_ms", Number(io_period_ms_)),
      KeyValue("io_period_p99_ms", Number(io_timing.p99)),
      KeyValue("io_timeout_count", Number(io_timeout_count_)),
      KeyValue("communication_ok", communication_ok_ ? "true" : "false"),
      KeyValue("base_x_m", Number(estimate.position.x())),
      KeyValue("base_y_m", Number(estimate.position.y())),
      KeyValue("base_z_m", Number(estimate.position.z())),
      KeyValue("base_yaw_rad", Number(estimate.euler_zyx.x())),
      KeyValue("base_roll_rad", Number(estimate.euler_zyx.z())),
      KeyValue("base_pitch_rad", Number(estimate.euler_zyx.y())),
      KeyValue("base_vx_mps", Number(estimate.velocity_world.x())),
      KeyValue("base_vy_mps", Number(estimate.velocity_world.y())),
      KeyValue("base_vz_mps", Number(estimate.velocity_world.z())),
      KeyValue("requested_vx_mps", Number(limited_command_.vx)),
      KeyValue("requested_vy_mps", Number(limited_command_.vy)),
      KeyValue("requested_yaw_rad_s", Number(limited_command_.yaw)),
      KeyValue("governed_vx_mps", Number(governed_command_.vx)),
      KeyValue("governed_vy_mps", Number(governed_command_.vy)),
      KeyValue("governed_yaw_rad_s", Number(governed_command_.yaw)),
      KeyValue("fr_hip_position_rad", Number(joints_state_.position[0])),
      KeyValue("fr_thigh_position_rad", Number(joints_state_.position[1])),
      KeyValue("fr_calf_position_rad", Number(joints_state_.position[2])),
      KeyValue("fr_hip_velocity_rad_s", Number(joints_state_.velocity[0])),
      KeyValue("fr_thigh_velocity_rad_s", Number(joints_state_.velocity[1])),
      KeyValue("fr_calf_velocity_rad_s", Number(joints_state_.velocity[2])),
      KeyValue("wbc_q_fr_hip_rad", Number(wbc.command.position[0])),
      KeyValue("wbc_q_fr_thigh_rad", Number(wbc.command.position[1])),
      KeyValue("wbc_q_fr_calf_rad", Number(wbc.command.position[2])),
      KeyValue("wbc_qd_fr_hip_rad_s", Number(wbc.command.velocity[0])),
      KeyValue("wbc_qd_fr_thigh_rad_s", Number(wbc.command.velocity[1])),
      KeyValue("wbc_qd_fr_calf_rad_s", Number(wbc.command.velocity[2])),
      KeyValue("mpc_policy_age_s", Number(backend_->policyAgeSeconds(stamp.seconds()))),
      KeyValue("mpc_solver_healthy", backend_->solverHealthy() ? "true" : "false"),
      KeyValue("mpc_last_error", backend_->lastError()),
      KeyValue("mpc_solve_ms", Number(backend_->lastSolveTimeMs())),
      KeyValue("mpc_solve_mean_ms", Number(mpc_timing.mean)),
      KeyValue("mpc_solve_p95_ms", Number(mpc_timing.p95)),
      KeyValue("mpc_solve_p99_ms", Number(mpc_timing.p99)),
      KeyValue("wbc_solve_ms", Number(wbc.solve_time_ms)),
      KeyValue("wbc_valid", wbc.valid ? "true" : "false"),
      KeyValue(
          "wbc_consecutive_failures",
          std::to_string(consecutive_wbc_failures_)),
      KeyValue("wbc_tau_max_abs", Number(wbc_tau_max_abs)),
      KeyValue("wbc_tau_fr_hip", Number(wbc.command.effort[0])),
      KeyValue("wbc_tau_fr_thigh", Number(wbc.command.effort[1])),
      KeyValue("wbc_tau_fr_calf", Number(wbc.command.effort[2])),
      KeyValue("wbc_equality_residual", Number(wbc.equality_residual)),
      KeyValue("wbc_inequality_violation", Number(wbc.inequality_violation)),
      KeyValue("policy_fz_fr", Number(policy.input[2])),
      KeyValue("policy_fz_fl", Number(policy.input[5])),
      KeyValue("policy_fz_rr", Number(policy.input[8])),
      KeyValue("policy_fz_rl", Number(policy.input[11]))};
  diagnostics.status.push_back(std::move(status));
  diagnostics_publisher_->publish(diagnostics);

  if (!estimate.valid) {
    return;
  }
  const Eigen::Quaterniond quaternion = QuaternionFromZyx(estimate.euler_zyx);
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = "odom";
  odometry.child_frame_id = "base";
  odometry.pose.pose.position.x = estimate.position.x();
  odometry.pose.pose.position.y = estimate.position.y();
  odometry.pose.pose.position.z = estimate.position.z();
  odometry.pose.pose.orientation.w = quaternion.w();
  odometry.pose.pose.orientation.x = quaternion.x();
  odometry.pose.pose.orientation.y = quaternion.y();
  odometry.pose.pose.orientation.z = quaternion.z();
  // Odometry 的 pose 位于 odom，twist 位于 child_frame_id=base；
  // 估计器输出的是世界系速度，发布前必须旋转到机体系。
  const Eigen::Vector3d velocity_body =
      quaternion.inverse() * estimate.velocity_world;
  const Eigen::Vector3d angular_body =
      quaternion.inverse() * estimate.angular_velocity_world;
  odometry.twist.twist.linear.x = velocity_body.x();
  odometry.twist.twist.linear.y = velocity_body.y();
  odometry.twist.twist.linear.z = velocity_body.z();
  odometry.twist.twist.angular.x = angular_body.x();
  odometry.twist.twist.angular.y = angular_body.y();
  odometry.twist.twist.angular.z = angular_body.z();
  odom_publisher_->publish(odometry);

  geometry_msgs::msg::TransformStamped transform;
  transform.header = odometry.header;
  transform.child_frame_id = "base";
  transform.transform.translation.x = estimate.position.x();
  transform.transform.translation.y = estimate.position.y();
  transform.transform.translation.z = estimate.position.z();
  transform.transform.rotation = odometry.pose.pose.orientation;
  tf_broadcaster_->sendTransform(transform);
}

}  // namespace custom_dog_control
