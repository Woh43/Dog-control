#pragma once

#include <array>
#include <limits>
#include <memory>
#include <string>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "custom_dog_control/control/ControlTypes.hpp"
#include "custom_dog_control/control/JointCommandWindowBudget.hpp"
#include "custom_dog_control/control/SignedMotorPdBudget.hpp"
#include "custom_dog_control/diagnostics/CycleTraceTypes.hpp"
#include "custom_dog_control/nmpc/KinematicStateEstimator.hpp"
#include "custom_dog_control/model/ModelValidator.hpp"

namespace custom_dog_control {

struct NmpcBackendConfig {
  // INFO 配置与规范 URDF 的文件路径；由控制器生命周期负责解析默认路径。
  std::string task_file;
  std::string reference_file;
  std::string urdf_file;
  // 后台 SQP 的目标频率；WBC 同步运行于 ros2_control update，频率独立配置。
  double frequency_hz = 50.0;
  // Pace slower-than-real-time physics by observation time, so 20 Hz means
  // twenty solves per simulated second rather than per wall-clock second.
  bool use_sim_time = false;
  int solver_threads = 0;  // 0 keeps task.info; real RK3588 overrides to 2.
  double trot_period_s = kTrotPeriodSeconds;
  double trot_double_support_s = 0.0;
  double contact_force_ramp_s = 0.0;
  double target_horizon_s = 1.0;  // 速度指令生成两点参考轨迹的前视时间，不是求解器时域配置。
  // Zero retains full XY reanchoring during velocity commands. A positive
  // value preserves bounded position memory perpendicular to linear motion.
  double max_cross_track_error_m = 0.0;
  double nominal_height_m = 0.28;
  // WBC 前先把 NMPC 关节目标投影到 URDF 限位内，并额外收紧髋关节活动范围。
  double max_hip_angle_rad = 0.20;
  double joint_limit_margin_rad = 0.03;
  bool use_joint_acceleration_decision = false;
  double joint_velocity_acceleration_limit = 0.0;
  // Keep feedback contact forces, but pair nominal q with nominal dq rather
  // than treating the joint-velocity feedback action as a trajectory tangent.
  bool nominal_joint_reference = false;
  // Observation only; no change to policy selection, reference or solver.
  bool cycle_trace_enabled = false;
  MotorPdWindowModel motor_pd_window_model;
  // Opt-in host composition: QP torque is total actuator torque. The backend
  // exports an internal FF residual so the final handoff can compose PD once.
  // Mutually exclusive with the predictive inner-motor-PD window/center.
  bool host_total_torque_command = false;
  std::array<double, kJointCount> motion_joint_kp{};
  double motion_joint_kd = 0.0;
  double max_nominal_height_correction_mps2 = 0.0;
  std::array<double, kJointCount> nominal_joint_positions{};  // 硬件顺序，单位 rad。
};

// valid=false 时 command 不可下发，由状态机决定短暂站立保持或进入 FAULT。
struct WbcOutput {
  // q/dq use the accepted policy reference after safety projection; effort follows the configured WBC/host mode.
  HybridJointCommand command;
  // 约束残差是求解质量诊断，不是关节跟踪误差。
  double equality_residual = 0.0;
  double inequality_violation = 0.0;
  double solve_time_ms = 0.0;
  // Actual policy state passed to WBC, not the nominal height reference.
  // QP acceleration is the solved world-frame base z generalized acceleration;
  // it is a model prediction, not an IMU measurement or realized acceleration.
  // NaN means no corresponding WBC evaluation / valid QP solution this update.
  double desired_base_z_m = std::numeric_limits<double>::quiet_NaN();
  double task_base_az_mps2 = std::numeric_limits<double>::quiet_NaN();
  double qp_base_az_mps2 = std::numeric_limits<double>::quiet_NaN();
  // Signed unweighted task residual: QP-predicted az minus task target az.
  double qp_base_az_residual_mps2 = std::numeric_limits<double>::quiet_NaN();
  double raw_base_az_task_rhs_mps2 = std::numeric_limits<double>::quiet_NaN();
  double joint_acceleration_coupling_az_mps2 = std::numeric_limits<double>::quiet_NaN();
  double nominal_height_correction_mps2 = std::numeric_limits<double>::quiet_NaN();
  std::array<double, kJointCount> pd_window_reservation_nm{};
  std::array<double, kJointCount> joint_pd_gain_scale{};
  // Trace-only accepted QP decision and modeled PD, hardware joint order.
  std::array<double, kJointCount> trace_qp_torque{}, trace_pd_nominal{}, trace_pd_lower{}, trace_pd_upper{};
  std::size_t velocity_reference_limited_joints = 0;
  bool valid = false;
};

// 封装 OCS2 策略交换与 WBC。Configure/Start/Stop 由生命周期串行管理；
// UpdateObservation、SetVelocityCommand、EvaluatePolicy、ComputeWbc 在同一控制线程调用。
// 后台线程只推进 NMPC；目标和步态请求通过专用同步入口交给求解器，
// 不能把本类的整个公共接口视为可任意并发调用或保证无锁的实时接口。
class NmpcBackend {
 public:
  NmpcBackend();
  ~NmpcBackend();

  NmpcBackend(const NmpcBackend&) = delete;
  NmpcBackend& operator=(const NmpcBackend&) = delete;

  // 停止旧求解线程并加载模型/优化问题；仅在返回 ok=true 后使用计算与模型访问接口。
  ModelValidationResult Configure(const NmpcBackendConfig& config);
  void Start();
  // 唤醒并 join 后台线程，可能阻塞；不在 update 的周期路径中调用。
  void Stop();
  // Non-blocking zero-epoch barrier. Worker resets SQP on the first new
  // observation + target; policies from an earlier epoch cannot be evaluated.
  void InvalidateObservation();

  // 将硬件关节顺序转换为模型顺序并发布质心观测。返回供 WBC 使用的 36 维刚体状态：
  // [ZYX 欧拉角, 世界系位置, q_j, 世界系角速度, 世界系线速度, dq_j]。
  // observation_time 是控制器累计的模型时间（秒）；planned_mode 是接触计划位掩码。
  ocs2::vector_t UpdateObservation(
      const EstimatedState& estimate,
      const JointSample& joints,
      double observation_time,
      std::size_t planned_mode);
  // vx/vy 沿机体航向的前向/侧向，yaw 为偏航角速度；目标生成时转换到世界系。
  // 两个 reanchor 标志分别把位置/朝向锚点重置到最新观测；请求合并为最新一份，
  // 由后台线程在下一次求解前应用，不直接跨线程改写 ReferenceManager。
  // 进入停车的调用方用 reanchor_position 捕获实际位置；否则静止时保持原锚点。
  void SetVelocityCommand(
      const VelocityCommand& command, bool reanchor_position = false,
      bool reanchor_heading = false);
  // true 请求对角 Trot，false 请求四足站立；发出请求不代表新接触策略已经生效。
  void RequestGait(bool trot);
  // 接收并评估策略；成功时完整填写 output，失败时调用方不得使用本次 output。
  // sequence 仅在接收新策略时递增；四足站立使用前馈输入，Trot 保留策略反馈。
  bool EvaluatePolicy(double now_seconds, PolicySample& output);
  void FillCycleTraceReferences(diagnostics::ControllerCycleRecord& output) const;
  // 同步求解 WBC 并检查残差/力矩限位。period_seconds 用于动力学任务和预测限位。
  // 调用前须已成功 Configure，并提供维数正确的策略及 UpdateObservation 返回的状态。
  WbcOutput ComputeWbc(
      const PolicySample& policy,
      const ocs2::vector_t& measured_rbd_state,
      double period_seconds);

  bool configured() const;
  bool solverHealthy() const;
  double lastSolveTimeMs() const;
  // 真机使用 steady_clock；仿真使用 ROS 仿真时间，与求解频率的时间基准一致。
  // 尚无策略时返回 +inf。
  double policyAgeSeconds(double now_seconds) const;
  // Retained cross-track reference correction at the latest target update,
  // in estimator/world coordinates; zero when this optional path is inactive.
  double targetCrossTrackErrorMeters() const;
  std::string lastError() const;

  // 返回后端持有的对象引用；未建立模型时抛出 logic_error，重新配置后旧引用不可再用。
  const ocs2::PinocchioInterface& pinocchioInterface() const;
  const ocs2::CentroidalModelInfo& modelInfo() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace custom_dog_control
