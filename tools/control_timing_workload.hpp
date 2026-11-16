#pragma once

// Numerical shadow workload: synthetic standing joints, no actuator command API.
#include "custom_dog_control/nmpc/NmpcBackend.hpp"
#include "timing_backend_hooks.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <thread>
#include <time.h>

namespace timing_probe {
namespace dog = custom_dog_control;
inline std::int64_t NowNs() {
  timespec value{};
  if (clock_gettime(CLOCK_MONOTONIC, &value)) throw std::runtime_error("CLOCK_MONOTONIC failed");
  return std::int64_t(value.tv_sec) * 1000000000LL + value.tv_nsec;
}

struct ControlStep {
  std::int64_t start_ns = 0, estimator_start_ns = 0, estimator_end_ns = 0;
  std::int64_t observation_end_ns = 0, policy_end_ns = 0, wbc_end_ns = 0, end_ns = 0;
  std::uint64_t policy_sequence = 0;
  double policy_age_ms = -1, mpc_solve_ms = -1, wbc_solve_ms = -1;
  bool estimate_valid = false, policy_valid = false, policy_fresh = false;
  bool wbc_executed = false, wbc_valid = false, solver_healthy = false, new_policy = false;
  WbcProbeResult wbc_probe;
};

class ControlWorkload {
 public:
  explicit ControlWorkload(const std::filesystem::path& root, double mpc_hz_override = 0.0) {
    const auto package = root / "src/custom_dog_control";
    auto doc = YAML::LoadFile((package / "config/controllers.yaml").string());
    const auto real = YAML::LoadFile((package / "config/real_controller.yaml").string());
    for (const auto& node : real) {
      const auto key = node.first.as<std::string>();
      for (const auto& item : node.second["ros__parameters"])
        doc[key]["ros__parameters"][item.first.as<std::string>()] = item.second;
    }
    p_ = doc["nmpc_wbc_controller"]["ros__parameters"];
    config_.use_sim_time = p_["use_sim_time"].as<bool>();
    if (config_.use_sim_time) throw std::runtime_error("shadow workload requires real wall clock");
    config_.task_file = (package / "config/nmpc/task.info").string();
    config_.reference_file = (package / "config/nmpc/reference.info").string();
    config_.urdf_file = (root / "src/custom_dog_description/urdf/custom_dog.urdf").string();
    config_.frequency_hz = p_["mpc_frequency_hz"].as<double>();
    // Test-only override: never rewrite source/installed controller configuration.
    if (mpc_hz_override > 0.0) config_.frequency_hz = mpc_hz_override;
    config_.solver_threads = p_["mpc_solver_threads"].as<int>();
    config_.trot_period_s = p_["trot_period_s"].as<double>();
    config_.trot_double_support_s = Read("trot_double_support_s", 0.0);
    config_.contact_force_ramp_s = Read("wbc_contact_force_ramp_s", 0.0);
    config_.nominal_height_m = p_["nominal_height_m"].as<double>();
    config_.max_hip_angle_rad = p_["locomotion_max_hip_angle_rad"].as<double>();
    config_.joint_limit_margin_rad = p_["locomotion_joint_limit_margin_rad"].as<double>();
    config_.max_cross_track_error_m = p_["nmpc_cross_track_position_error_limit_m"].as<double>();
    config_.joint_velocity_acceleration_limit = p_["wbc_joint_velocity_acceleration_limit"].as<double>();
    config_.max_nominal_height_correction_mps2 = p_["wbc_nominal_height_correction_limit_mps2"].as<double>();
    config_.use_joint_acceleration_decision = p_["wbc_use_joint_acceleration_decision"].as<bool>();
    config_.nominal_joint_reference = p_["wbc_nominal_joint_reference"].as<bool>();
    config_.host_total_torque_command = p_["host_total_torque_command"].as<bool>();
    config_.motor_pd_window_model.horizon_seconds = p_["motor_pd_budget_horizon_s"].as<double>();
    config_.motor_pd_window_model.nominal_time_seconds = p_["motor_pd_nominal_execution_time_s"].as<double>();
    const auto acceleration = p_["motor_pd_assumed_acceleration_rad_s2"]
        ? p_["motor_pd_assumed_acceleration_rad_s2"].as<std::vector<double>>()
        : std::vector<double>{100.0};
    if (acceleration.size() != 1 && acceleration.size() != 3 && acceleration.size() != 12)
      throw std::runtime_error("invalid acceleration bound count");
    const auto nominal = p_["nominal_joint_positions"].as<std::vector<double>>();
    if (nominal.size() != dog::kJointCount) throw std::runtime_error("invalid nominal joint count");
    std::copy(nominal.begin(), nominal.end(), config_.nominal_joint_positions.begin());
    for (std::size_t i = 0; i < dog::kJointCount; ++i) {
      config_.motion_joint_kp[i] = p_[i % 3 == 0 ? "wbc_hip_stiffness" :
          i % 3 == 1 ? "wbc_joint_stiffness" : "wbc_calf_stiffness"].as<double>();
      config_.motor_pd_window_model.assumed_acceleration_bound_rad_s2[i] = acceleration[i % acceleration.size()];
    }
    config_.motion_joint_kd = p_["wbc_joint_damping"].as<double>();
  }

  void Initialize(const dog::ImuSample* initial_imu = nullptr) {
    const auto validation = [&] {
      NmpcConstructionAffinity affinity;
      const auto result = backend_.Configure(config_);
      affinity.Restore();
      return result;
    }();
    if (!validation.ok) throw std::runtime_error(validation.Summary());
    dog::EstimatorNoise noise;
    noise.planned_contact_delay_s = Read("estimator_planned_contact_delay_s", 0.0);
    noise.planned_contact_ramp_s = Read("estimator_planned_contact_ramp_s", 0.0);
    noise.rolling_foot_radius_m = Read("estimator_rolling_foot_radius_m", 0.0);
    estimator_ = std::make_unique<dog::KinematicStateEstimator>(backend_.pinocchioInterface(), backend_.modelInfo(), noise);
    estimator_->Reset(config_.nominal_height_m);
    joints_.position = config_.nominal_joint_positions;
    joints_.valid.fill(1.0);
    joints_.temperature.fill(25.0);
    joints_.stamp_valid = true;
    imu_.valid = true;
    imu_.linear_acceleration = {0., 0., 9.81};
    // Anchor the isolated world's heading to the real initial attitude in the
    // IMU phase; avoid an artificial yaw jump from the synthetic warmup pose.
    if (initial_imu) imu_ = *initial_imu;
    for (int i = 0; i < 100; ++i) {
      joints_.stamp_seconds = imu_.stamp_seconds = NowNs() * 1e-9;
      estimator_->Update(joints_, imu_, dog::ContactFlags(dog::kStanceMode), 0.001);
    }
    epoch_ns_ = NowNs();
    backend_.Start();
    const auto deadline = NowNs() + 30000000000LL;
    while (NowNs() < deadline) {
      auto result = Step(0.002);
      if (result.wbc_probe.compared && !result.wbc_probe.comparison_ok)
        throw std::runtime_error("shadow WBC cold/hot comparison failed during warmup");
      if (result.wbc_valid && result.policy_fresh && result.solver_healthy) return;
      if (!result.solver_healthy) throw std::runtime_error(backend_.lastError());
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    throw std::runtime_error("shadow NMPC/WBC warmup timed out");
  }

  ControlStep Step(double dt, const dog::ImuSample* received_imu = nullptr) {
    ControlStep sample;
    sample.start_ns = NowNs();
    const double t = (sample.start_ns - epoch_ns_) * 1e-9;
    dt = std::clamp(dt, 0.0001, 0.02);  // same integration bounds as production
    joints_.stamp_seconds = sample.start_ns * 1e-9;
    dog::ImuSample input = received_imu ? *received_imu : imu_;
    if (!received_imu) input.stamp_seconds = joints_.stamp_seconds;
    sample.estimator_start_ns = NowNs();
    const auto estimate = estimator_->Update(joints_, input, dog::ContactFlags(dog::kStanceMode), dt);
    sample.estimator_end_ns = NowNs();
    sample.estimate_valid = estimate.valid;
    if (estimate.valid) {
      auto measured = backend_.UpdateObservation(estimate, joints_, t, dog::kStanceMode);
      if (!target_ready_) {
        backend_.SetVelocityCommand(dog::VelocityCommand{}, true, true);
        backend_.RequestGait(false);
        target_ready_ = true;
      }
      sample.observation_end_ns = NowNs();
      dog::PolicySample policy;
      sample.policy_valid = backend_.EvaluatePolicy(t, policy);
      sample.policy_age_ms = backend_.policyAgeSeconds(t) * 1000;
      sample.policy_sequence = policy.sequence;
      sample.policy_fresh = sample.policy_valid && sample.policy_age_ms <= 2000.0 / config_.frequency_hz;
      sample.new_policy = sample.policy_valid && policy.sequence != last_sequence_;
      if (sample.new_policy) last_sequence_ = policy.sequence;
      sample.mpc_solve_ms = policy.solve_time_ms;
      sample.policy_end_ns = NowNs();
      if (sample.policy_fresh) {
        sample.wbc_executed = true;
        const auto output = backend_.ComputeWbc(policy, measured, dt);
        sample.wbc_valid = output.valid;
        sample.wbc_solve_ms = output.solve_time_ms;
        sample.wbc_probe = last_wbc_probe;
        sample.wbc_end_ns = NowNs();
        // output.command is deliberately discarded; it cannot leave this API.
      }
    }
    sample.solver_healthy = backend_.solverHealthy();
    sample.end_ns = NowNs();
    return sample;
  }

  void Stop() { backend_.Stop(); }
  const dog::NmpcBackendConfig& config() const { return config_; }
 private:
  double Read(const char* key, double fallback) const { return p_[key] ? p_[key].as<double>() : fallback; }
  YAML::Node p_;
  dog::NmpcBackendConfig config_;
  dog::NmpcBackend backend_;
  std::unique_ptr<dog::KinematicStateEstimator> estimator_;
  dog::JointSample joints_;
  dog::ImuSample imu_;
  std::int64_t epoch_ns_ = 0;
  std::uint64_t last_sequence_ = 0;
  bool target_ready_ = false;
};
}  // namespace timing_probe
