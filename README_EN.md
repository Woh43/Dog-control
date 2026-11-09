# Dog-control · NMPC + WBC

[简体中文](README.md) | **English**

RC2027 now contains the basic locomotion stack for the 12-DOF quadruped: ROS 2 Humble, OCS2 NMPC, Weighted WBC, state estimation, and Unitree motor/IMU drivers. Precision foothold control and its `qr_*` packages have been removed. The target is the local Orange Pi 5 Plus / RK3588 running Ubuntu 22.04 ARM64.

## Validation status

**Builds, offline solves, and serial communication checks do not establish hardware locomotion readiness. Physical calibration, loaded standing, and Trot have not been accepted.** See the current [hardware instructions](docs/hardware.md).

**01:25 update:** the latest hardware run faulted about 0.201 seconds after entering MPC_STANCE because its NMPC policy exceeded the 100 ms age limit; communication was valid immediately before the fault. It overlapped the local physics simulation, so CPU contention remains a confounding factor. Historical main-ancestor gains 6.2/0.2 were motor-side values, whereas current 45/55 gains are joint-side values before gear-ratio conversion. Rear-leg tracking and handoff criteria remain unresolved; see the hardware investigation above.

**Latest hardware log, November 10, 2026:** the user reports needing to lift the rear during stand-up, followed by unsupported stance. The 01:03:54 launch ran in MPC_STANCE for about 48 seconds before invalid right-rear hip feedback (RR ID 0) triggered a communication fault; IMU and NMPC were healthy at that instant. The earlier rear-lift difficulty remains unexplained. LT+X with centered sticks retains stance, and this run never entered Trot. A subsequent 60-second all-zero communication check returned 72000/72000 valid replies, which does not exclude intermittent faults during loaded control. Automatic diagnostic recording and gait-arming notices have been added. See the [investigation](docs/hardware.md).

The earlier estimator expression-lifetime defect and IMU check-time race were fixed; the complete 15.15 kg model and persistent CppAD cache are local. At that stage, 18 control-package checks and one model check passed. Fortress with ground-truth assistance disabled and a 100 Hz IMU passed legacy-gamepad stand-up, forward/backward/lateral Trot, both yaw directions, stop/resume, and a final 30-second stance. This does not resolve or validate physical loaded stand-up.

The installed diagnostic update passed a new stand-up/forward/stop/resume physics run with the same model and a 100 Hz IMU, recording all 12 joints and controller requests with unchanged runtime fingerprints. Initially 18/19 control tests passed; the PTY suite passed separately after one receive timeout. Recorder ROS integration and the model check passed. Initial failures and the interrupted run remain recorded in the investigation above.

The **15.15 kg model passed low-speed, flat-ground Gazebo Fortress physics tests with 1000 Hz control** on this board: stand-up, WBC stance, forward/backward/lateral Trot, positive yaw, stop, and resume. The IMU/joint estimator passed with ground-truth feedback disabled, retaining the original stand-up gains and gait. That round used 20 Hz NMPC and two solver threads; 50 Hz triggered a stale-policy fault. Eight software calibration regression groups and 45 control-package checks passed at that stage. See the [simulation and calibration report](docs/simulation-validation-20261102.md).

The later **250 Hz host-side PD with torque-only output** comparisons are historical experiments; see the [historical report](docs/real-control-validation-20261102.md). The current implementation restores five-field motor-side PD. Hardware and Fortress share 250 Hz control, 20 Hz NMPC, two solver threads, gains, gait, and low-speed limits. **Before restoring startup alignment, the full low-speed physics regression passed with ground-truth assistance disabled**: legacy gamepad START/combinations, stand-up, forward motion, stop/resume, backward/lateral motion, both yaw directions, and a final 60-second stance without FAULT. Forward displacement was 0.551 m in 10 seconds and 0.326 m during the 6-second resume. Subsequent changes add hardware startup assistance, the real START wait, and visualization without changing the NMPC/WBC algorithm configuration. Build fingerprints have changed; those earlier results do not validate the new assistance. See the [compatibility validation report](docs/compatibility-validation-20261103.md). Hardware locomotion remains unaccepted.

A policy timeout occurred during concurrent colcon compilation. The isolated repeat passed without relaxing the timeout threshold; this does not establish real-time performance under arbitrary concurrent workloads.

**FR feedback recovered in the November 3 zero-output communication check.** Sequential probing returned 500/500 valid replies from each of the 12 motors, 6000/6000 in total, with zero CRC, ID, mode, or motor errors. Four simultaneous ports then polled each ID at 100 Hz for approximately five seconds, returning 6677/6678 valid replies; the final snapshot records one incomplete reply/timeout for FR ID 2, with its cause undetermined. This does not establish zero packet loss throughout the test or acceptance of a 250 Hz control cycle. The first IMU capture included one earlier AHRS timestamp. After clearing only the host RX queue, a fresh capture contained 500 IMU and 500 AHRS frames at 100 Hz by device timestamps, with zero CRC errors and no repeated timestamp anomaly; the original anomaly's source remains unconfirmed. All probes have stopped and released the serial ports. See the [sequential check](log/communication-recheck-20261103-2347/summary.json) and [parallel results](log/communication-recheck-20261103-2347/parallel/result.json). **These probes sent only mode0 commands with all fields zero; they do not validate physical startup assistance, mechanical calibration, loaded standing, or walking.**

Earlier checks recorded 100/100 replies from each FL/RR/RL motor and 0/100 from each FR motor; old SDK typed and raw 2/20 ms comparisons also returned no valid FR replies. Those historical results remain recorded, and the successful repeat does not identify the earlier failure's cause. The original driver has published input from the Xbox 360 gamepad. See the [compatibility report](docs/compatibility-validation-20261103.md).

The current migration keeps the existing motor protocol, IMU transforms, and START encoder-offset sampling, adapts them to the URDF coordinates required by NMPC/WBC, and restores the previous gamepad layout. Runtime comparison confirms the same compiled controller core, canonical model/task, and algorithm parameters on both paths; differences are limited to clocks, hardware interfaces, and simulation-specific settings. Earlier host-PD, rate, and delay changes were control-timing choices; the battery mass increase did not require all of them. Full loaded-cycle timing remains unverified. The communication probes did not request stand-up or walking; the user separately launched the control program, which is not part of this motion acceptance. Earlier results remain in the [historical baseline](docs/validation-baselines.md) and [battery model report](docs/battery-model-validation-20261102.md).

The model mass is **15.15 kg**: 13.84916 kg CAD mass plus a 1.30084 kg battery at the base origin. Base mass, center of mass, and parallel-axis inertia have been updated. Battery dimensions are unavailable, so its intrinsic inertia remains a point-mass approximation. The complete model now lives in [src/custom_dog_description](src/custom_dog_description/README_EN.md): three URDFs, 17 full STL files, joint conventions, and battery configuration. Its source is pinned to your `Woh43/custom_dog_stack` commit `6c776de8`, with the previously verified local battery correction. It requires neither a neighboring RL repository nor the old model underlay.

## Build and launch

Run from `/home/orangepi/qr_ws`:

```bash
# Build only; does not start nodes or open motor ports.
src/custom_dog_control/scripts/build_real.sh

# Inspect arguments without starting control nodes.
src/custom_dog_control/scripts/run_real.sh --show-args
```

Build scripts build `custom_dog_description` in this workspace before the controller. Runtime scripts verify that the resolved model belongs to this workspace and matches its source. Alternative OCS2/SDK locations can use `CUSTOM_DOG_CONTROL_DEPS_WS` and `UNITREE_ACTUATOR_SDK_ROOT`; model paths no longer use `CUSTOM_DOG_DESCRIPTION_DIR` or `CUSTOM_DOG_MODEL_WS`. Dependency instructions are in [setup.md](docs/setup.md).

Standard runtime scripts keep NMPC's generated CppAD libraries in `.cache/cppad` across reboots, isolated by model/configuration content and compiler ABI fingerprints. First generation can take several minutes; wait for `CppAD models ready` and controller activation. Several November 9–10 launches ended before cold compilation in `/tmp` finished and never reached normal stand-up control; see the [startup investigation](docs/hardware.md).

The desktop **Run Dog** shortcut invokes `src/custom_dog_control/scripts/run_dog.sh` without a `YES` prompt and explicitly selects `startup_alignment_enabled:=true` and `use_rviz:=true`. Before launching, it stops this tool's own mode 0 communication monitors to release the motor ports. `--help`/`--show-args` only inspect information and leave monitors running. The controller stays in PASSIVE while the hardware waits for valid feedback from all 12 motors, controller ownership, and ongoing command writes, then automatically performs one approximately 0.9-second alignment. Press START again after completion to capture offsets; LT+A requests stand-up and LT+X enables Trot.

Alignment preserves the previous rotor directions and joint-side torque magnitudes of `0.95/0.63/1.52 N·m` for hip/thigh/calf: hip/thigh for 300 ms, all three joints for 300 ms, then mode0 release for 300 ms. It is nonblocking and runs once. Communication/CRC failure, software stop, stale command writes, released ownership, or a calibration reset during alignment aborts it to zero output, without automatic retry; recovery requires restarting. It sends nonzero torque before START, so PASSIVE is not always mode0 while assistance is active. Incomplete or stale feedback from any motor prevents it from starting. **The restored profile passed PTY validation; these communication probes did not send the nonzero assistance, and its physical execution remains unaccepted.**

Hardware RViz uses fixed frame `base`, RobotModel joint feedback, and foot TF from forward kinematics. Before calibration, encoder angles are not aligned with the URDF zeros, so the displayed pose does not establish mechanical calibration. Foot TF does not measure foot forces.

The desktop entry passes `require_physical_estop_confirmation:=false` to remove the extra confirmation condition; it does not claim `physical_estop_verified=true`. Calling `run_real.sh` directly retains the confirmation requirement and defaults to startup assistance and RViz disabled. FR feedback has recovered, with one incomplete reply/timeout recorded in the final parallel snapshot; restored communication does not validate loaded standing or walking.

| Hardware gamepad button | Behavior |
| --- | --- |
| START | If assistance is enabled, wait for completion, then press START anew to capture offsets while passive; no automatic stand-up |
| LT+A | Explicit stand-up request after valid calibration |
| LT+X | Enable Trot from standing; valid velocity input and a new gait policy are still required |
| LT+B | Return to passive; acknowledge a fault, then recalibrate |
| LT+Y | Software emergency stop, latching FAULT; replaces the removed legacy STEP_TEST |
| Left stick vertically | Forward/backward velocity |
| Right stick horizontally | Lateral velocity |
| Left stick horizontally | Yaw velocity |

LT is the previous L2 trigger. A/X alone do not change modes. LT+X arms walking; centered sticks retain stance without stepping in place. All three axes retain a 0.08 deadband and map to REP-103 as `vx=-LY, vy=-RX, yaw=-LX`. Shared speed limits are 0.05 m/s forward/lateral and 0.2 rad/s yaw. Calibration equivalence after the coordinate conversion is documented in the [compatibility report](docs/compatibility-validation-20261103.md).

Hardware launches now save approximately 10 Hz diagnostics to `control-diagnostics.jsonl` beside `launch.log`, including all 12 joints, stand-up targets, controller requests, gamepad gates, and faults. Controller requests are not final motor packets. Each segment is limited to 16 MiB; the initial segment and latest three subsequent segments are retained. Set `record_diagnostics:=false` to disable recording.

Device timestamps in the existing raw capture confirm approximately 100 Hz each for IMU and AHRS output; the driver has no fixed 100 Hz cap. The user will adjust the device output rate later. No IMU configuration command is sent this round.

`src/qr_guide/src/interface/IOSDK.cpp` is obsolete. The active motor implementation is [IOSDK.cpp](src/custom_dog_control/src/hardware/IOSDK.cpp).

## Workspace

| Directory | Purpose |
| --- | --- |
| `src/custom_dog_control` | Controller, NMPC/WBC, state machine, hardware plugin, tests |
| `src/fdilink_ahrs` | IMU driver publishing `/imu` |
| `src/serial_ros2` | ROS package `serial` for the IMU; motors use the Unitree SDK |
| `src/unitree_guide` | Historical reference excluded by `COLCON_IGNORE` |
| `src/custom_dog_description` | Self-contained 15.15 kg URDFs, full STL assets, and provenance |
| `external/ocs2_ws` | Pinned OCS2 sources and ARM64 dependencies |

Control flow: IMU/joints → kinematic state estimator → NMPC policy → Weighted WBC → five fields: desired position/velocity, feedforward torque, Kp/Kd → motor-side PD. At each host sample, `abs(tau_ff)+kp*abs(q_des-q)+kd*abs(dq_des-dq)` determines a common scale for feedforward and both gains. This bounds their contributions at that sample, without guaranteeing continuous torque bounds as feedback changes.

See the [controller guide](src/custom_dog_control/README.md) and [architecture](docs/architecture.md). The robot has IMU and joint feedback, without foot contact sensors; planned or estimated contacts are not measured foot forces.

## Checks

```bash
source src/custom_dog_control/scripts/env.sh
colcon test --packages-select custom_dog_control --event-handlers console_direct+
colcon test-result --verbose
```

Native Fortress on this board (headless by default; add `gui:=true` in a desktop terminal):

```bash
src/custom_dog_control/scripts/run_fortress.sh
```

The entry disables ground-truth assistance and uses the common 250 Hz/20 Hz profile on RK3588 large cores. Initial AD generation may take several minutes. The default motor-PD simulation evaluates PD every 1 ms physics step and models one outer-loop cycle plus physics-step phase delay. This ideal servo model is not a measurement of real motor bandwidth. Add `legacy_joy_layout:=true` to reproduce the accepted legacy gamepad flow. Full low-speed motion and 60-second stance results are in the [compatibility report](docs/compatibility-validation-20261103.md); the earlier 1000 Hz passes remain in the [historical simulation report](docs/simulation-validation-20261102.md).

The previous Gazebo Classic entry remains available for development machines:

```bash
CUSTOM_DOG_BUILD_ONLY=1 src/custom_dog_control/scripts/build_simulation.sh
src/custom_dog_control/scripts/run_simulation.sh
src/custom_dog_control/scripts/test_simulation.sh
```

This round uses Fortress for the current configuration; the Classic entry above was not tested.

Removed precision sources remain recoverable from Git history. Prior documents, the worktree patch, and retired artifacts are archived at `/home/orangepi/qr_ws_backups/remove_precision_20261102/`. The working branch remains `RC2027`, tracking `origin/RC2027`; these changes have not been automatically committed or pushed.
