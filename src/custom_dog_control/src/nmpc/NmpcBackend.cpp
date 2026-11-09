#include "custom_dog_control/nmpc/NmpcBackend.hpp"
#include "custom_dog_control/nmpc/ModelCache.hpp"
#include "custom_dog_control/control/ContactForceRamp.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <pinocchio/multibody/model.hpp>
#include <legged_wbc/WeightedWbc.h>
#include <legged_interface/LeggedInterface.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_sqp/SqpMpc.h>

namespace custom_dog_control {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

// The pinned upstream constructor only loads settings. Namespace its AD cache
// before setupOptimalControlProblem can load any generated model libraries.
class ModelCachedLeggedInterface final : public legged::LeggedInterface {
 public:
  ModelCachedLeggedInterface(const std::string& task_file,
                            const std::string& urdf_file,
                            const std::string& reference_file)
      : legged::LeggedInterface(task_file, urdf_file, reference_file, true) {
    // Override only the cache root. The fingerprint still hashes the unchanged
    // URDF/task/reference bytes and native ABI/compiler tag.
    const char* cache_override = std::getenv("CUSTOM_DOG_CPPAD_CACHE_DIR");
    const std::string cache_base =
        cache_override && *cache_override ? cache_override : modelSettings_.modelFolderCppAd;
    modelSettings_.modelFolderCppAd = ModelCacheDirectory(
        cache_base, urdf_file, task_file, reference_file);
    std::cerr << "[custom_dog_control] CppAD cache: "
              << modelSettings_.modelFolderCppAd << std::endl;
    std::cerr << "[custom_dog_control] Preparing NMPC/WBC model libraries. "
                 "First initialization may compile native code; wait for 'CppAD models ready'."
              << std::endl;
  }
};

double SteadyNowSeconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// 控制线程只提交步态意图；求解器在 preSolverRun 的同步点修改接触计划。
// 请求是最新值语义，不承诺逐个执行短时间内连续到来的切换。
class GaitCommandModule final : public ocs2::SolverSynchronizedModule {
 public:
  explicit GaitCommandModule(
      std::shared_ptr<ocs2::legged_robot::GaitSchedule> schedule,
      double period_s, double double_support_s)
      : schedule_(std::move(schedule)), period_s_(period_s),
        double_support_s_(double_support_s) {}

  void Request(bool trot, bool force = false) {
    if (!force && trot == requested_trot_.load(std::memory_order_acquire) &&
        !pending_.load(std::memory_order_acquire)) {
      return;
    }
    requested_trot_.store(trot, std::memory_order_release);
    pending_.store(true, std::memory_order_release);
  }

  void preSolverRun(
      ocs2::scalar_t init_time, ocs2::scalar_t final_time,
      const ocs2::vector_t&,
      const ocs2::ReferenceManagerInterface& reference_manager) override {
    if (!pending_.exchange(false, std::memory_order_acq_rel)) {
      return;
    }
    const bool trot = requested_trot_.load(std::memory_order_acquire);
    const auto& active_schedule = reference_manager.getModeSchedule();
    // 优先沿用下一接触事件的时刻和对角腿顺序，避免截断当前支撑相。
    const auto next_event = std::upper_bound(
        active_schedule.eventTimes.begin(), active_schedule.eventTimes.end(),
        init_time + 1e-6);
    const std::size_t next_event_index = static_cast<std::size_t>(
        std::distance(active_schedule.eventTimes.begin(), next_event));
    std::size_t first_diagonal = kTrotFrRlMode;
    if (next_event != active_schedule.eventTimes.end()) {
      for (std::size_t i = next_event_index + 1;
           i < active_schedule.modeSequence.size(); ++i) {
        if (IsTrotMode(active_schedule.modeSequence[i])) {
          first_diagonal = active_schedule.modeSequence[i];
          break;
        }
      }
    }
    const auto contact_schedule =
        MakeTrotContactSchedule(period_s_, double_support_s_, first_diagonal);
    const ocs2::legged_robot::ModeSequenceTemplate gait =
        !trot
            ? ocs2::legged_robot::ModeSequenceTemplate(
                  {0.0, period_s_}, {kStanceMode})
            : ocs2::legged_robot::ModeSequenceTemplate(
                  contact_schedule.times, contact_schedule.modes);
    const double transition_start =
        next_event != active_schedule.eventTimes.end()
            ? *next_event
            : init_time + std::min(0.05, 0.25 * (final_time - init_time));
    schedule_->insertModeSequenceTemplate(gait, transition_start, final_time);
  }

  void postSolverRun(const ocs2::PrimalSolution&) override {}

 private:
  std::shared_ptr<ocs2::legged_robot::GaitSchedule> schedule_;
  const double period_s_;
  const double double_support_s_;
  std::atomic_bool requested_trot_{false};
  std::atomic_bool pending_{true};
};

}  // namespace

class NmpcBackend::Impl {
 public:
  ~Impl() { Stop(); }

  ModelValidationResult Configure(const NmpcBackendConfig& new_config) {
    Stop();
    config = new_config;
    try {
      if (!std::isfinite(config.frequency_hz) || config.frequency_hz <= 0.0 ||
          config.solver_threads < 0 || config.solver_threads > 8) {
        throw std::invalid_argument("invalid MPC frequency or thread count");
      }
      if (!std::isfinite(config.trot_period_s) || config.trot_period_s < 0.2 ||
          config.trot_period_s > 2.0) {
        throw std::invalid_argument("trot_period_s must be in [0.2, 2.0]");
      }
      MakeTrotContactSchedule(
          config.trot_period_s, config.trot_double_support_s, kTrotFrRlMode);
      if (!std::isfinite(config.contact_force_ramp_s) ||
          config.contact_force_ramp_s < 0.0 || config.contact_force_ramp_s > 1.0) {
        throw std::invalid_argument("wbc_contact_force_ramp_s must be in [0, 1]");
      }
      contact_force_ramp.Reset();
      const auto model_prepare_start = std::chrono::steady_clock::now();
      interface = std::make_unique<ModelCachedLeggedInterface>(
          config.task_file, config.urdf_file, config.reference_file);
      interface->setupOptimalControlProblem(
          config.task_file, config.urdf_file, config.reference_file, false);
      std::cerr << "[custom_dog_control] CppAD models ready (loaded or compiled) in "
                << std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - model_prepare_start).count()
                << " s; validating the model and configuring NMPC/WBC." << std::endl;
      const auto validation = ModelValidator::Validate(
          interface->getPinocchioInterface(),
          interface->getCentroidalModelInfo());
      if (!validation.ok) {
        last_error = validation.Summary();
        return validation;
      }
      model_joint_slots = validation.model_joint_slots;

      const auto& info = interface->getCentroidalModelInfo();
      ocs2::CentroidalModelPinocchioMapping mapping(info);
      end_effector_kinematics =
          std::make_unique<ocs2::PinocchioEndEffectorKinematics>(
              interface->getPinocchioInterface(), mapping,
              std::vector<std::string>{
                  std::string(kFootFrameNames[0]), std::string(kFootFrameNames[1]),
                  std::string(kFootFrameNames[2]), std::string(kFootFrameNames[3])});
      wbc = std::make_unique<legged::WeightedWbc>(
          interface->getPinocchioInterface(), info, *end_effector_kinematics);
      wbc->loadTasksSetting(config.task_file, false);

      auto sqp_settings = interface->sqpSettings();
      if (config.solver_threads > 0) sqp_settings.nThreads = config.solver_threads;
      mpc = std::make_unique<ocs2::SqpMpc>(
          interface->mpcSettings(), sqp_settings,
          interface->getOptimalControlProblem(), interface->getInitializer());
      mpc->getSolverPtr()->setReferenceManager(
          interface->getReferenceManagerPtr());
      gait_module = std::make_shared<GaitCommandModule>(
          interface->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(),
          config.trot_period_s, config.trot_double_support_s);
      mpc->getSolverPtr()->addSynchronizedModule(gait_module);

      mrt = std::make_unique<ocs2::MPC_MRT_Interface>(*mpc);
      mrt->initRollout(&interface->getRollout());
      rbd_conversions = std::make_unique<ocs2::CentroidalModelRbdConversions>(
          interface->getPinocchioInterface(), info);

      observation.state = interface->getInitialState();
      observation.input = ocs2::vector_t::Zero(info.inputDim);
      observation.mode = kStanceMode;
      observation.time = 0.0;
      target_position_world = observation.state.segment<2>(6);
      target_heading_yaw = observation.state(9);
      target_heading_update_time = observation.time;
      target_reference_initialized = true;
      pending_target.reset();
      mrt->setCurrentObservation(observation);
      mrt->resetMpcNode(TargetFromCommand(VelocityCommand{}));
      configured_flag = true;
      InvalidateObservation();
      solver_healthy.store(true, std::memory_order_release);
      last_policy_steady_seconds.store(-1.0, std::memory_order_release);
      last_error.clear();
      return validation;
    } catch (const std::exception& exception) {
      configured_flag = false;
      solver_healthy.store(false, std::memory_order_release);
      last_error = exception.what();
      ModelValidationResult result;
      result.errors.push_back(last_error);
      return result;
    }
  }

  void Start() {
    if (!configured_flag || running.exchange(true)) {
      return;
    }
    solver_healthy.store(true, std::memory_order_release);
    last_policy_steady_seconds.store(-1.0, std::memory_order_release);
    {
      std::lock_guard<std::mutex> lock(error_mutex);
      last_error.clear();
    }
    worker = std::thread([this] {
      const auto period = std::chrono::duration<double>(
          1.0 / std::max(1.0, config.frequency_hz));
      auto next = std::chrono::steady_clock::now();
      std::optional<std::uint64_t> failed_epoch;
      std::optional<std::uint64_t> pacing_epoch;
      double next_sim_solve = -1.0;
      while (running.load(std::memory_order_acquire)) {
        if (config.use_sim_time) next = std::chrono::steady_clock::now();
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        if (config.use_sim_time) {
          std::unique_lock<std::mutex> lock(worker_mutex);
          const bool ready = worker_cv.wait_for(lock, std::chrono::milliseconds(20),
              [this, &pacing_epoch, &next_sim_solve] {
            return !running.load() ||
                (observation_ready.load(std::memory_order_acquire) &&
                 (!pacing_epoch || *pacing_epoch != requested_epoch.load(std::memory_order_acquire) ||
                  latest_observation_time.load(std::memory_order_acquire) + 1e-9 >= next_sim_solve));
          });
          if (!running.load()) break;
          if (!ready) continue;
        }
        if (!observation_ready.load(std::memory_order_acquire)) {
          std::unique_lock<std::mutex> lock(worker_mutex);
          worker_cv.wait_until(lock, next, [this] { return !running.load(); });
          continue;
        }
        const auto start = std::chrono::steady_clock::now();
        std::uint64_t epoch = 0;
        try {
          std::optional<ocs2::TargetTrajectories> target;
          {
            std::lock_guard<std::mutex> lock(target_mutex);
            // Capture the epoch together with its target. Otherwise a reset
            // can make an old solve consume the new epoch's only first target.
            epoch = requested_epoch.load(std::memory_order_acquire);
            if (pending_target && (!failed_epoch || *failed_epoch != epoch)) {
              target.emplace(std::move(*pending_target));
              pending_target.reset();
            }
          }
          if (failed_epoch && *failed_epoch == epoch) {
            std::unique_lock<std::mutex> lock(worker_mutex);
            worker_cv.wait_until(lock, next, [this] { return !running.load(); });
            continue;
          }
          if (epoch != completed_epoch.load(std::memory_order_acquire)) {
            if (!target) {
              std::unique_lock<std::mutex> lock(worker_mutex);
              worker_cv.wait_until(lock, next, [this] { return !running.load(); });
              continue;
            }
            // resetMpcNode resets SQP only, leaving MRT evaluation buffers
            // untouched. The epoch barrier prevents consuming those buffers.
            mrt->resetMpcNode(*target);
            gait_module->Request(false, true);
          } else if (target) {
            // ReferenceManager 只在求解线程更新，避免与 advanceMpc 并发修改。
            // 交换区只保留最新目标，控制循环无需等待每个旧目标被求解。
            mrt->getReferenceManager().setTargetTrajectories(
                std::move(*target));
          }
          const double solve_observation_time =
              latest_observation_time.load(std::memory_order_acquire);
          mrt->advanceMpc();
          if (config.use_sim_time) {
            // Keep the nominal phase for 250 Hz / 20 Hz fractional periods,
            // but never replay several missed solve deadlines in a burst.
            if (!pacing_epoch || *pacing_epoch != epoch ||
                solve_observation_time >= next_sim_solve + period.count()) {
              next_sim_solve = solve_observation_time + period.count();
            } else {
              next_sim_solve += period.count();
            }
            pacing_epoch = epoch;
          }
          const auto elapsed = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - start).count();
          last_solve_time_ms.store(elapsed, std::memory_order_release);
          completed_epoch.store(epoch, std::memory_order_release);
          solver_healthy.store(true, std::memory_order_release);
          failed_epoch.reset();
        } catch (const std::exception& exception) {
          {
            std::lock_guard<std::mutex> lock(error_mutex);
            last_error = exception.what();
          }
          solver_healthy.store(false, std::memory_order_release);
          // Latch this epoch instead of destroying the worker. A new explicit
          // calibration/reset epoch retries through resetMpcNode; no RT join.
          failed_epoch = epoch;
          std::cerr << "[custom_dog_control NMPC] solver paused until reset: "
                    << exception.what() << std::endl;
        }
        // Do not replay missed deadlines as a burst of back-to-back solves.
        next = std::max(next, std::chrono::steady_clock::now());
        if (!config.use_sim_time) {
          std::unique_lock<std::mutex> lock(worker_mutex);
          worker_cv.wait_until(lock, next, [this] { return !running.load(); });
        }
      }
    });
  }

  void Stop() {
    running.store(false, std::memory_order_release);
    worker_cv.notify_all();
    if (worker.joinable()) {
      worker.join();
    }
  }

  void InvalidateObservation() {
    contact_force_ramp.Reset();
    std::lock_guard<std::mutex> lock(target_mutex);
    observation_ready.store(false, std::memory_order_release);
    latest_observation_time.store(-1.0, std::memory_order_release);
    requested_epoch.fetch_add(1, std::memory_order_acq_rel);
    active_policy_seconds = -1.0;
    last_policy_steady_seconds.store(-1.0, std::memory_order_release);
    has_yaw = false;
    target_reference_initialized = false;
    yaw_command_was_active = false;
    pending_target.reset();
  }

  ocs2::TargetTrajectories TargetFromCommand(const VelocityCommand& command) {
    // 生成两个带时间戳的参考点；这是参考预瞄长度，不修改 MPC 求解时域。
    // 静止时维持锚定的 xy/yaw，运动时由 SetVelocityCommand 更新参考锚点。
    const double horizon = std::max(0.2, config.target_horizon_s);
    ocs2::vector_t current_pose = observation.state.segment<6>(6);
    if (target_reference_initialized) {
      current_pose.head<2>() = target_position_world;
      current_pose(3) = target_heading_yaw;
    }
    current_pose(2) = config.nominal_height_m;
    current_pose(4) = 0.0;
    current_pose(5) = 0.0;
    const double yaw = current_pose(3);
    const Eigen::Matrix2d rotation =
        (Eigen::Matrix2d() << std::cos(yaw), -std::sin(yaw),
                              std::sin(yaw), std::cos(yaw)).finished();
    const Eigen::Vector2d velocity_world =
        rotation * Eigen::Vector2d(command.vx, command.vy);

    ocs2::vector_t target_pose = current_pose;
    target_pose(0) += velocity_world.x() * horizon;
    target_pose(1) += velocity_world.y() * horizon;
    target_pose(2) = config.nominal_height_m;
    target_pose(3) = target_heading_yaw + command.yaw * horizon;
    target_pose(4) = 0.0;
    target_pose(5) = 0.0;

    const auto& info = interface->getCentroidalModelInfo();
    auto centroidal_state = [&](const ocs2::vector_t& pose) {
      // 与上游目标发布器一致，直接指定归一化线动量、令角动量参考为零。
      // 若从虚构的基座速度反算，会因本机基座与质心偏置引入额外动量。
      ocs2::vector_t state = ocs2::vector_t::Zero(info.stateDim);
      state.head<3>() =
          Eigen::Vector3d(velocity_world.x(), velocity_world.y(), 0.0);
      state.segment<6>(6) = pose;
      for (std::size_t i = 0; i < kJointCount; ++i) {
        state(12 + model_joint_slots[i]) = config.nominal_joint_positions[i];
      }
      return state;
    };
    ocs2::vector_array_t states{
        centroidal_state(current_pose), centroidal_state(target_pose)};
    ocs2::vector_array_t inputs(
        2, ocs2::vector_t::Zero(observation.input.size()));
    return {
        {observation.time, observation.time + horizon},
        std::move(states), std::move(inputs)};
  }

  ocs2::vector_t UpdateObservation(
      const EstimatedState& estimate,
      const JointSample& joints,
      double observation_time,
      std::size_t planned_mode) {
    const auto& info = interface->getCentroidalModelInfo();
    ocs2::vector_t measured =
        ocs2::vector_t::Zero(2 * info.generalizedCoordinatesNum);
    measured.head<3>() = estimate.euler_zyx;
    measured.segment<3>(3) = estimate.position;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      measured(6 + model_joint_slots[i]) = joints.position[i];
    }
    measured.segment<3>(info.generalizedCoordinatesNum) =
        estimate.angular_velocity_world;
    measured.segment<3>(info.generalizedCoordinatesNum + 3) =
        estimate.velocity_world;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      measured(info.generalizedCoordinatesNum + 6 + model_joint_slots[i]) =
          joints.velocity[i];
    }

    observation.time = observation_time;
    observation.state =
        rbd_conversions->computeCentroidalStateFromRbdModel(measured);
    if (has_yaw) {
      // 跨越 +/-pi 时选取离上一帧最近的等价角度，保持优化器参考连续。
      const double raw_yaw = observation.state(9);
      observation.state(9) =
          previous_yaw + std::remainder(raw_yaw - previous_yaw, kTwoPi);
    }
    previous_yaw = observation.state(9);
    has_yaw = true;
    observation.mode = planned_mode;
    mrt->setCurrentObservation(observation);
    latest_observation_time.store(observation.time, std::memory_order_release);
    observation_ready.store(true, std::memory_order_release);
    if (config.use_sim_time) worker_cv.notify_one();
    return measured;
  }

  void SetVelocityCommand(
      const VelocityCommand& command, bool reanchor_position,
      bool reanchor_heading) {
    if (!configured_flag) {
      return;
    }
    const bool moving =
        std::max({std::abs(command.vx), std::abs(command.vy),
                  std::abs(command.yaw)}) > 1e-3;
    const bool yaw_command_active = std::abs(command.yaw) > 1e-3;
    if (!target_reference_initialized) {
      target_position_world = observation.state.segment<2>(6);
      target_heading_yaw = observation.state(9);
      target_reference_initialized = true;
    }
    if (reanchor_position) {
      target_position_world = observation.state.segment<2>(6);
    }
    if (reanchor_heading) {
      target_heading_yaw = observation.state(9);
    }
    if (moving) {
      // 移动目标每次从最新实测位置出发，只在参考预瞄区间外推。
      // 独立积分世界系目标会累积打滑/速度误差，使机器人追赶不可达的位置。
      target_position_world = observation.state.segment<2>(6);
    }
    if (yaw_command_active || yaw_command_was_active) {
      // 转向参考相对最新航向生成；松开转向的第一帧捕获实际航向，随后保持。
      target_heading_yaw = observation.state(9);
    }
    yaw_command_was_active = yaw_command_active;
    target_heading_update_time = observation.time;
    auto target = TargetFromCommand(command);
    {
      std::lock_guard<std::mutex> lock(target_mutex);
      pending_target = std::move(target);
    }
  }

  bool EvaluatePolicy(double now_seconds, PolicySample& output) {
    if (!configured_flag || !observation_ready.load(std::memory_order_acquire) ||
        completed_epoch.load(std::memory_order_acquire) !=
            requested_epoch.load(std::memory_order_acquire) ||
        !mrt->initialPolicyReceived() ||
        !solver_healthy.load(std::memory_order_acquire)) {
      return false;
    }
    try {
      if (mrt->updatePolicy()) {
        // 只有新策略被控制线程接收时才更新时间和序号；重复评估旧策略
        // 不能使其重新变“新鲜”，否则后台停滞会被高频评估掩盖。
        active_policy_seconds = now_seconds;
        last_policy_steady_seconds.store(
            SteadyNowSeconds(), std::memory_order_release);
        ++policy_sequence;
      }
      if (active_policy_seconds < 0.0) {
        return false;
      }
      ocs2::vector_t state;
      ocs2::vector_t input;
      std::size_t mode = kStanceMode;
      mrt->evaluatePolicy(
          observation.time, observation.state, state, input, mode);
      if (mode == kStanceMode) {
        // 本机四足受约束站立时，SQP 反馈易给出过大的关节速度修正。
        // 在名义状态上重评估以取前馈输入；对角 Trot 仍使用实际状态反馈。
        ocs2::vector_t feedforward_state;
        ocs2::vector_t feedforward_input;
        std::size_t feedforward_mode = mode;
        mrt->evaluatePolicy(
            observation.time, state, feedforward_state,
            feedforward_input, feedforward_mode);
        if (feedforward_mode == mode) {
          input = std::move(feedforward_input);
        }
      }
      if (state.size() != kCentroidalStateDim ||
          input.size() != kCentroidalInputDim ||
          !state.allFinite() || !input.allFinite()) {
        return false;
      }
      for (std::size_t i = 0; i < kCentroidalStateDim; ++i) {
        output.state[i] = state(i);
      }
      for (std::size_t i = 0; i < kCentroidalInputDim; ++i) {
        output.input[i] = input(i);
      }
      output.sequence = policy_sequence;
      output.mode = mode;
      output.observation_time = observation.time;
      output.generated_at_seconds = active_policy_seconds;
      output.solve_time_ms = last_solve_time_ms.load(std::memory_order_acquire);
      output.valid = true;
      return true;
    } catch (const std::exception& exception) {
      {
        std::lock_guard<std::mutex> lock(error_mutex);
        last_error = exception.what();
      }
      solver_healthy.store(false, std::memory_order_release);
      std::cerr << "[custom_dog_control NMPC] policy evaluation failed: "
                << exception.what() << std::endl;
      return false;
    }
  }

  WbcOutput ComputeWbc(
      const PolicySample& policy,
      const ocs2::vector_t& measured_rbd_state,
      double period_seconds) {
    WbcOutput output;
    if (!std::isfinite(period_seconds) || period_seconds <= 0.0 || period_seconds > 0.1) {
      return output;
    }
    ocs2::vector_t desired_state(kCentroidalStateDim);
    ocs2::vector_t desired_input(kCentroidalInputDim);
    for (std::size_t i = 0; i < kCentroidalStateDim; ++i) {
      desired_state(i) = policy.state[i];
    }
    for (std::size_t i = 0; i < kCentroidalInputDim; ++i) {
      desired_input(i) = policy.input[i];
    }

    // 在 WBC 前投影 NMPC 目标到关节安全区间，并收紧髋外展角以避免交叉落足。
    // 位置被截断时同步清零该关节目标速度，避免继续朝越界方向运动。
    auto joint_positions = ocs2::centroidal_model::getJointAngles(
        desired_state, interface->getCentroidalModelInfo());
    auto joint_velocities = ocs2::centroidal_model::getJointVelocities(
        desired_input, interface->getCentroidalModelInfo());
    const auto& model = interface->getPinocchioInterface().getModel();
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto joint_id = model.getJointId(std::string(kJointNames[i]));
      const auto q_index = model.joints[joint_id].idx_q();
      const std::size_t slot = static_cast<std::size_t>(model_joint_slots[i]);
      double lower = model.lowerPositionLimit(q_index) + config.joint_limit_margin_rad;
      double upper = model.upperPositionLimit(q_index) - config.joint_limit_margin_rad;
      if (i % kJointsPerLeg == 0) {
        lower = std::max(lower, -config.max_hip_angle_rad);
        upper = std::min(upper, config.max_hip_angle_rad);
      }
      const double projected = std::clamp(joint_positions(slot), lower, upper);
      if (projected != joint_positions(slot)) {
        joint_positions(slot) = projected;
        joint_velocities(slot) = 0.0;
      }
      desired_state(12 + slot) = joint_positions(slot);
      desired_input(12 + slot) = joint_velocities(slot);
    }

    const auto force_bounds = contact_force_ramp.Update(
        ContactFlags(policy.mode), period_seconds, config.contact_force_ramp_s,
        interface->getCentroidalModelInfo().robotMass * 9.81);
    wbc->setContactNormalForceUpperBounds(Eigen::Map<const ocs2::vector_t>(
        force_bounds.data(), force_bounds.size()));
    const auto start = std::chrono::steady_clock::now();
    const ocs2::vector_t solution = wbc->update(
        desired_state, desired_input, measured_rbd_state,
        policy.mode, period_seconds);
    output.solve_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    output.equality_residual = wbc->lastEqualityResidual();
    output.inequality_violation = wbc->lastInequalityViolation();
    // 求解器返回成功还不足以使用结果：同时检查有限性和约束残差，
    // 再逐关节复核力矩。失败时由上层决定短时回退或锁存故障。
    output.valid = wbc->lastSolverSucceeded() &&
                   solution.size() >= static_cast<Eigen::Index>(kJointCount) &&
                   solution.allFinite() &&
                   std::isfinite(output.equality_residual) &&
                   std::isfinite(output.inequality_violation) &&
                   output.equality_residual <= 1e-3 &&
                   output.inequality_violation <= 1e-4;
    if (!output.valid) {
      return output;
    }

    const auto command_joint_positions =
        ocs2::centroidal_model::getJointAngles(
            desired_state, interface->getCentroidalModelInfo());
    const auto command_joint_velocities =
        ocs2::centroidal_model::getJointVelocities(
            desired_input, interface->getCentroidalModelInfo());
    // QP 解的尾部是模型顺序力矩；q/dq 仍取投影后的 NMPC 目标。
    // 在这里统一转换回硬件顺序，kp/kd 留给控制器的模式与交接逻辑设置。
    const auto joint_torques = solution.tail(kJointCount);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto joint_id = model.getJointId(std::string(kJointNames[i]));
      const std::size_t model_slot =
          static_cast<std::size_t>(model_joint_slots[i]);
      if (std::abs(joint_torques(model_slot)) >
          model.effortLimit(model.joints[joint_id].idx_v()) + 1e-6) {
        output.valid = false;
        return output;
      }
      output.command.position[i] = command_joint_positions(model_slot);
      output.command.velocity[i] = command_joint_velocities(model_slot);
      output.command.effort[i] = joint_torques(model_slot);
    }
    return output;
  }

  NmpcBackendConfig config;
  bool configured_flag = false;
  std::atomic_bool running{false};
  std::atomic_bool solver_healthy{false};
  std::atomic_bool observation_ready{false};
  std::atomic<double> latest_observation_time{-1.0};
  std::atomic<std::uint64_t> requested_epoch{0};
  std::atomic<std::uint64_t> completed_epoch{0};
  std::atomic<double> last_solve_time_ms{0.0};
  std::atomic<double> last_policy_steady_seconds{-1.0};
  std::uint64_t policy_sequence = 0;
  double active_policy_seconds = -1.0;
  std::thread worker;
  std::mutex worker_mutex;
  std::condition_variable worker_cv;
  std::mutex target_mutex;
  std::optional<ocs2::TargetTrajectories> pending_target;
  mutable std::mutex error_mutex;
  std::string last_error;

  std::unique_ptr<legged::LeggedInterface> interface;
  std::unique_ptr<ocs2::SqpMpc> mpc;
  std::unique_ptr<ocs2::MPC_MRT_Interface> mrt;
  std::shared_ptr<GaitCommandModule> gait_module;
  std::unique_ptr<ocs2::CentroidalModelRbdConversions> rbd_conversions;
  std::unique_ptr<ocs2::PinocchioEndEffectorKinematics> end_effector_kinematics;
  std::unique_ptr<legged::WeightedWbc> wbc;
  ContactForceRamp contact_force_ramp;
  std::array<int, kJointCount> model_joint_slots{};
  ocs2::SystemObservation observation;
  bool has_yaw = false;
  double previous_yaw = 0.0;
  bool target_reference_initialized = false;
  bool yaw_command_was_active = false;
  Eigen::Vector2d target_position_world = Eigen::Vector2d::Zero();
  double target_heading_yaw = 0.0;
  double target_heading_update_time = 0.0;
};

NmpcBackend::NmpcBackend() : impl_(std::make_unique<Impl>()) {}
NmpcBackend::~NmpcBackend() = default;

ModelValidationResult NmpcBackend::Configure(const NmpcBackendConfig& config) {
  return impl_->Configure(config);
}
void NmpcBackend::Start() { impl_->Start(); }
void NmpcBackend::Stop() { impl_->Stop(); }
void NmpcBackend::InvalidateObservation() { impl_->InvalidateObservation(); }
ocs2::vector_t NmpcBackend::UpdateObservation(
    const EstimatedState& estimate, const JointSample& joints,
    double observation_time, std::size_t planned_mode) {
  return impl_->UpdateObservation(estimate, joints, observation_time, planned_mode);
}
void NmpcBackend::SetVelocityCommand(
    const VelocityCommand& command, bool reanchor_position,
    bool reanchor_heading) {
  impl_->SetVelocityCommand(
      command, reanchor_position, reanchor_heading);
}
void NmpcBackend::RequestGait(bool trot) {
  if (impl_->gait_module) {
    impl_->gait_module->Request(trot);
  }
}
bool NmpcBackend::EvaluatePolicy(double now_seconds, PolicySample& output) {
  return impl_->EvaluatePolicy(now_seconds, output);
}
WbcOutput NmpcBackend::ComputeWbc(
    const PolicySample& policy, const ocs2::vector_t& measured_rbd_state,
    double period_seconds) {
  return impl_->ComputeWbc(policy, measured_rbd_state, period_seconds);
}
bool NmpcBackend::configured() const { return impl_->configured_flag; }
bool NmpcBackend::solverHealthy() const {
  return impl_->solver_healthy.load(std::memory_order_acquire);
}
double NmpcBackend::lastSolveTimeMs() const {
  return impl_->last_solve_time_ms.load(std::memory_order_acquire);
}
double NmpcBackend::policyAgeSeconds(double now_seconds) const {
  if (impl_->config.use_sim_time) {
    const double stamp = impl_->active_policy_seconds;
    return stamp < 0.0 || !std::isfinite(now_seconds) || now_seconds < stamp
        ? std::numeric_limits<double>::infinity() : now_seconds - stamp;
  }
  const double stamp =
      impl_->last_policy_steady_seconds.load(std::memory_order_acquire);
  return stamp < 0.0 ? std::numeric_limits<double>::infinity()
                     : std::max(0.0, SteadyNowSeconds() - stamp);
}
std::string NmpcBackend::lastError() const {
  std::lock_guard<std::mutex> lock(impl_->error_mutex);
  return impl_->last_error;
}
const ocs2::PinocchioInterface& NmpcBackend::pinocchioInterface() const {
  if (!impl_->interface) {
    throw std::logic_error("NMPC backend is not configured");
  }
  return impl_->interface->getPinocchioInterface();
}
const ocs2::CentroidalModelInfo& NmpcBackend::modelInfo() const {
  if (!impl_->interface) {
    throw std::logic_error("NMPC backend is not configured");
  }
  return impl_->interface->getCentroidalModelInfo();
}

}  // namespace custom_dog_control
