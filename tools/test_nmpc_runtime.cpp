// Offline numerical runtime validation: synthetic static observations, no IO.
// This verifies cache/model/epoch/gait/solver plumbing, not robot dynamics.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "custom_dog_control/nmpc/NmpcBackend.hpp"

// Test-only linker wrapper. The production archive is unchanged; each normal
// call invokes the genuine OCS2 implementation. A one-shot throw exercises the
// worker's exception boundary without feeding NaNs or invalid robot states.
namespace {
std::atomic_bool inject_solver_exception{false};
std::atomic<unsigned int> advance_calls{0};
std::atomic<unsigned int> injected_exceptions{0};
}
extern "C" void __real__ZN4ocs217MPC_MRT_Interface10advanceMpcEv(void* self);
extern "C" void __wrap__ZN4ocs217MPC_MRT_Interface10advanceMpcEv(void* self) {
  ++advance_calls;
  if (inject_solver_exception.exchange(false)) {
    ++injected_exceptions;
    throw std::runtime_error("offline injected advanceMpc exception");
  }
  __real__ZN4ocs217MPC_MRT_Interface10advanceMpcEv(self);
}

using Clock = std::chrono::steady_clock;
namespace dog = custom_dog_control;
struct Stats {
  std::vector<double> values;
  void Json(std::ostream& out) const {
    auto v = values;
    std::sort(v.begin(), v.end());
    if (v.empty()) { out << "null"; return; }
    out << "{\"samples\":" << v.size()
        << ",\"mean_ms\":" << std::accumulate(v.begin(), v.end(), 0.0) / v.size()
        << ",\"p95_ms\":" << v[static_cast<std::size_t>(std::ceil(.95 * v.size())) - 1]
        << ",\"p99_ms\":" << v[static_cast<std::size_t>(std::ceil(.99 * v.size())) - 1]
        << ",\"max_ms\":" << v.back() << '}';
  }
};
void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: test_nmpc_runtime WORKSPACE TASK_FILE OUTPUT_JSON\n";
    return 2;
  }
  try {
    const std::filesystem::path workspace(argv[1]);
    const auto package = workspace / "src/custom_dog_control";
    auto parameters = YAML::LoadFile((package / "config/controllers.yaml").string())
                          ["nmpc_wbc_controller"]["ros__parameters"];
    const auto real_config = YAML::LoadFile((package / "config/real_controller.yaml").string());
    const auto overlay = real_config["nmpc_wbc_controller"]["ros__parameters"];
    for (const auto& item : overlay) parameters[item.first.as<std::string>()] = item.second;
    const double update_rate = real_config["controller_manager"]["ros__parameters"]["update_rate"].as<double>();
    const double dt = 1.0 / update_rate;
    dog::NmpcBackendConfig config;
    config.task_file = argv[2];
    config.reference_file = (package / "config/nmpc/reference.info").string();
    config.urdf_file = (workspace / "src/custom_dog_description/urdf/custom_dog.urdf").string();
    config.frequency_hz = parameters["mpc_frequency_hz"].as<double>();
    config.solver_threads = parameters["mpc_solver_threads"].as<int>();
    config.nominal_height_m = parameters["nominal_height_m"].as<double>();
    auto nominal = parameters["stand_up_joint_positions"].as<std::vector<double>>();
    Require(nominal.size() == dog::kJointCount, "nominal joint count");
    std::copy(nominal.begin(), nominal.end(), config.nominal_joint_positions.begin());
    dog::NmpcBackend backend;
    const auto validation = backend.Configure(config);
    Require(validation.ok, validation.Summary());
    dog::JointSample joints;
    joints.position = config.nominal_joint_positions;
    joints.valid.fill(1.0);
    joints.temperature.fill(25.0);
    joints.stamp_valid = true;
    dog::ImuSample imu;
    imu.valid = true;
    imu.linear_acceleration = {0., 0., 9.81};
    dog::KinematicStateEstimator estimator(backend.pinocchioInterface(), backend.modelInfo());
    dog::EstimatedState estimate;
    auto reset_estimator = [&] {
      estimator.Reset(config.nominal_height_m);
      for (int step = 1; step <= 100; ++step) {
        joints.stamp_seconds = imu.stamp_seconds = step * dt;
        estimate = estimator.Update(joints, imu, dog::ContactFlags(dog::kStanceMode), dt);
      }
      Require(estimate.valid && estimate.position.allFinite(), "invalid synthetic estimate");
    };
    reset_estimator();
    const auto begin = Clock::now();
    auto model_time = [&] { return std::chrono::duration<double>(Clock::now() - begin).count(); };
    dog::PolicySample policy;
    backend.Start();
    auto require_no_policy = [&](double duration_s, const std::string& context) {
      const auto deadline = Clock::now() + std::chrono::duration<double>(duration_s);
      while (Clock::now() < deadline) {
        Require(!backend.EvaluatePolicy(model_time(), policy), context + ": obsolete policy accepted");
        Require(backend.solverHealthy(), context + ": " + backend.lastError());
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
      }
      std::cout << "BARRIER_PASS " << context << std::endl;
    };
    require_no_policy(.2, "startup_without_observation_or_target");
    auto measured = backend.UpdateObservation(estimate, joints, model_time(), dog::kStanceMode);
    require_no_policy(.2, "observation_without_first_target");
    backend.SetVelocityCommand(dog::VelocityCommand{}, true, true);
    backend.RequestGait(false);
    auto await_policy = [&](bool recovering_fault = false) {
      const auto deadline = Clock::now() + std::chrono::seconds(10);
      while (Clock::now() < deadline) {
        joints.stamp_seconds = imu.stamp_seconds = model_time();
        measured = backend.UpdateObservation(estimate, joints, model_time(), dog::kStanceMode);
        if (!recovering_fault) Require(backend.solverHealthy(), backend.lastError());
        if (backend.EvaluatePolicy(model_time(), policy)) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
      }
      throw std::runtime_error("new epoch policy timed out: " + backend.lastError());
    };
    await_policy();
    const auto before_reset_sequence = policy.sequence;
    backend.InvalidateObservation();
    Require(!backend.EvaluatePolicy(model_time(), policy), "reset failed immediate barrier");
    Require(std::isinf(backend.policyAgeSeconds(model_time())), "reset policy age not infinite");
    require_no_policy(.2, "reset_without_new_inputs");
    const double idle_solve_time = backend.lastSolveTimeMs();
    require_no_policy(.2, "reset_worker_idle");
    Require(backend.lastSolveTimeMs() == idle_solve_time, "worker solved without new observation");
    reset_estimator();
    measured = backend.UpdateObservation(estimate, joints, model_time(), dog::kStanceMode);
    require_no_policy(.2, "reset_observation_without_target");
    Require(backend.lastSolveTimeMs() == idle_solve_time, "worker solved new epoch without target");
    backend.SetVelocityCommand(dog::VelocityCommand{}, true, true);
    await_policy();
    Require(policy.sequence > before_reset_sequence, "new epoch did not advance sequence");
    const auto recovered_sequence = policy.sequence;
    std::cout << "EPOCH_RECOVERY sequence_before=" << before_reset_sequence
              << " sequence_after=" << recovered_sequence << std::endl;

    // Repeated invalidation while the worker may still be solving an older
    // epoch. Interleave targets that are intentionally invalidated, then leave
    // the final epoch with observation only: no old target may revive it.
    constexpr int epoch_stress_resets = 20;
    for (int round = 0; round < epoch_stress_resets; ++round) {
      backend.InvalidateObservation();
      measured = backend.UpdateObservation(estimate, joints, model_time(), dog::kStanceMode);
      Require(!backend.EvaluatePolicy(model_time(), policy), "stress: old epoch policy accepted");
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
      Require(!backend.EvaluatePolicy(model_time(), policy), "stress: missing target revived old policy");
      if (round % 3 == 0 && round + 1 < epoch_stress_resets)
        backend.SetVelocityCommand(dog::VelocityCommand{}, true, true);
      std::this_thread::sleep_for(std::chrono::milliseconds(6));
    }
    require_no_policy(.2, "stress_final_epoch_missing_target");
    backend.SetVelocityCommand(dog::VelocityCommand{}, true, true);
    await_policy();
    Require(policy.sequence > recovered_sequence, "stress: final epoch did not recover");
    std::cout << "EPOCH_STRESS_PASS resets=" << epoch_stress_resets
              << " recovered_sequence=" << policy.sequence << std::endl;

    // Inject exactly one exception at the real worker's advanceMpc boundary.
    // Keep feeding this epoch afterward: it must stay latched until explicit
    // invalidation, then recover in the same surviving background worker.
    const auto pre_fault_sequence = policy.sequence;
    inject_solver_exception.store(true);
    const auto fault_deadline = Clock::now() + std::chrono::seconds(2);
    while (backend.solverHealthy() && Clock::now() < fault_deadline) {
      measured = backend.UpdateObservation(estimate, joints, model_time(), dog::kStanceMode);
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
    Require(!backend.solverHealthy(), "injected solver fault did not latch");
    Require(injected_exceptions.load() == 1, "fault wrapper did not execute exactly once");
    Require(backend.lastError().find("offline injected") != std::string::npos,
            "unexpected solver failure during injection");
    const auto calls_when_faulted = advance_calls.load();
    const auto latch_deadline = Clock::now() + std::chrono::milliseconds(200);
    while (Clock::now() < latch_deadline) {
      measured = backend.UpdateObservation(estimate, joints, model_time(), dog::kStanceMode);
      backend.SetVelocityCommand(dog::VelocityCommand{}, true, true);
      Require(!backend.EvaluatePolicy(model_time(), policy), "faulted epoch returned a policy");
      Require(!backend.solverHealthy(), "faulted epoch automatically resumed");
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
    Require(advance_calls.load() == calls_when_faulted, "faulted epoch retried solver");
    backend.InvalidateObservation();
    reset_estimator();
    measured = backend.UpdateObservation(estimate, joints, model_time(), dog::kStanceMode);
    const auto no_target_deadline = Clock::now() + std::chrono::milliseconds(200);
    while (Clock::now() < no_target_deadline) {
      Require(!backend.EvaluatePolicy(model_time(), policy), "fault recovery without a new target");
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
    Require(advance_calls.load() == calls_when_faulted, "new fault epoch solved before its target");
    backend.SetVelocityCommand(dog::VelocityCommand{}, true, true);
    await_policy(true);
    Require(backend.solverHealthy(), "solver health did not recover");
    Require(policy.sequence > pre_fault_sequence, "fault recovery reused an old policy sequence");
    const auto fault_recovered_sequence = policy.sequence;
    std::cout << "FAULT_RECOVERY_PASS sequence_before=" << pre_fault_sequence
              << " sequence_after=" << fault_recovered_sequence << std::endl;

    Stats solver_stats, wbc_stats;
    std::map<std::string, Stats> phase_solver, phase_wbc;
    std::map<std::size_t, std::size_t> valid_wbc_by_mode, invalid_wbc_by_mode;
    std::set<std::uint64_t> seen_sequences;
    std::set<std::size_t> all_modes;
    double max_tau = 0., max_eq = 0., max_ineq = 0.;
    std::size_t missed_update_periods = 0;
    auto phase = [&](const std::string& name, bool trot, std::size_t target_count) {
      backend.RequestGait(trot);
      backend.SetVelocityCommand(dog::VelocityCommand{}, true, true);
      std::set<std::size_t> modes;
      const auto deadline = Clock::now() + std::chrono::seconds(30);
      auto next = Clock::now();
      while (Clock::now() < deadline) {
        const auto tick = Clock::now();
        const auto now = model_time();
        joints.stamp_seconds = imu.stamp_seconds = now;
        measured = backend.UpdateObservation(estimate, joints, now, policy.mode);
        Require(backend.solverHealthy(), name + ": " + backend.lastError());
        if (backend.EvaluatePolicy(now, policy)) {
          all_modes.insert(policy.mode);
          modes.insert(policy.mode);
          if (seen_sequences.insert(policy.sequence).second) {
            solver_stats.values.push_back(policy.solve_time_ms);
            phase_solver[name].values.push_back(policy.solve_time_ms);
            std::cout << "POLICY phase=" << name << " sequence=" << policy.sequence
                      << " mode=" << policy.mode << " solve_ms=" << policy.solve_time_ms << std::endl;
          }
          const auto wbc = backend.ComputeWbc(policy, measured, dt);
          if (wbc.valid) {
            ++valid_wbc_by_mode[policy.mode];
            wbc_stats.values.push_back(wbc.solve_time_ms);
            phase_wbc[name].values.push_back(wbc.solve_time_ms);
            max_eq = std::max(max_eq, wbc.equality_residual);
            max_ineq = std::max(max_ineq, wbc.inequality_violation);
            for (auto torque : wbc.command.effort) {
              Require(std::isfinite(torque), "nonfinite WBC torque");
              max_tau = std::max(max_tau, std::abs(torque));
            }
          } else {
            ++invalid_wbc_by_mode[policy.mode];
          }
        }
        if (Clock::now() - tick > std::chrono::duration<double>(dt)) ++missed_update_periods;
        const bool matched = trot
            ? modes.count(dog::kTrotFrRlMode) && modes.count(dog::kTrotFlRrMode)
            : policy.mode == dog::kStanceMode;
        if (phase_solver[name].values.size() >= target_count && matched) break;
        next += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(dt));
        next = std::max(next, Clock::now());
        std::this_thread::sleep_until(next);
      }
      Require(phase_solver[name].values.size() >= target_count, name + ": insufficient distinct policies");
      if (trot) Require(modes.count(dog::kTrotFrRlMode) && modes.count(dog::kTrotFlRrMode), "both Trot modes not reached");
      else Require(policy.mode == dog::kStanceMode, "did not return to stance");
    };
    phase("stance_before", false, 40);
    phase("trot", true, 40);
    phase("stance_after", false, 40);
    backend.Stop();
    Require(seen_sequences.size() >= 100, "fewer than 100 distinct policies");
    for (auto mode : {dog::kStanceMode, dog::kTrotFrRlMode, dog::kTrotFlRrMode})
      Require(valid_wbc_by_mode[mode] > 0, "no valid WBC for mode " + std::to_string(mode));
    std::ofstream out(argv[3]);
    out << "{\"passed\":true,\"hardware_io\":false,\"dynamic_simulation\":false,\"synthetic_static_observation\":true,\"mass_kg\":"
        << validation.total_mass_kg << ",\"mpc_frequency_hz\":" << config.frequency_hz
        << ",\"solver_threads\":" << config.solver_threads << ",\"wbc_period_s\":" << dt
        << ",\"epoch_before_sequence\":" << before_reset_sequence << ",\"epoch_recovered_sequence\":" << recovered_sequence
        << ",\"epoch_stress_resets\":" << epoch_stress_resets
        << ",\"injected_solver_exceptions\":" << injected_exceptions.load()
        << ",\"fault_recovered_sequence\":" << fault_recovered_sequence
        << ",\"distinct_policies\":" << seen_sequences.size() << ",\"solver\":";
    solver_stats.Json(out); out << ",\"wbc\":"; wbc_stats.Json(out);
    out << ",\"max_tau_nm\":" << max_tau << ",\"max_equality_residual\":" << max_eq
        << ",\"max_inequality_violation\":" << max_ineq << ",\"ticks_exceeding_period\":" << missed_update_periods
        << ",\"modes\":["; bool first = true;
    for (auto mode : all_modes) { if (!first) out << ','; first = false;
      out << "{\"mode\":" << mode << ",\"valid_wbc\":" << valid_wbc_by_mode[mode]
          << ",\"invalid_wbc\":" << invalid_wbc_by_mode[mode] << '}'; }
    out << "],\"phases\":["; first = true;
    for (const auto& entry : phase_solver) { if (!first) out << ','; first = false;
      out << "{\"name\":\"" << entry.first << "\",\"solver\":"; entry.second.Json(out);
      out << ",\"wbc\":"; phase_wbc[entry.first].Json(out); out << '}'; }
    out << "]}\n";
    std::cout << "OFFLINE_RUNTIME_PASS policies=" << seen_sequences.size()
              << " hardware_io=none dynamic_simulation=false" << std::endl;
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "OFFLINE_RUNTIME_FAIL " << error.what() << std::endl;
    return 1;
  }
}
