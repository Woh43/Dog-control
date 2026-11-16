# Dog-control · NMPC + WBC

[简体中文](README.md) | **English**

A ROS 2 Humble workspace for model-based quadruped locomotion: state estimation, OCS2 NMPC, Weighted WBC, Gazebo Fortress simulation, and Unitree motor / FDILink IMU interfaces. The target platform is an Orange Pi 5 Plus / RK3588 running Ubuntu 22.04 ARM64; desktop development requires the corresponding dependencies.

Development and integration target **`RC2027` / `origin/RC2027`**. Precision foothold control and its `qr_*` packages have been removed. RL training, policies, and deployment belong to the separate `himloco_custom_dog` repository. Documentation reviewed **2026-11-16**; the latest motion data below was collected **2026-11-15**.

## Current configuration

The sources of truth are [controllers.yaml](src/custom_dog_control/config/controllers.yaml), [task.info](src/custom_dog_control/config/nmpc/task.info), and their launch files.

| Item | Default / meaning |
| --- | --- |
| Shared YAML / direct hardware entry: main control / WBC, NMPC | 1000 Hz / 50 Hz; two NMPC solver threads |
| Desktop Run Dog | 250 Hz main control / WBC, 50 Hz NMPC, RViz off; explicit arguments override these defaults |
| Trot | Full cycle 0.40 s; each diagonal swing 0.20 s; four-foot support overlap defaults to 0 |
| Planned swing clearance | 0.10 m; actual clearance requires measurement, and short swings scale the planned amplitude |
| Nominal height | 0.34 m from ground to the `base` origin, not underside clearance |
| PD stand-up / settling | Hip/thigh/calf Kp = 66/66/99 N·m/rad; Kd = 1.2 N·m·s/rad |
| MPC_STANCE / TROT joint PD | Hip/thigh/calf Kp = 38.4/38.4/57.6 N·m/rad; Kd = 2.2 N·m·s/rad |
| WBC task-space PD | Swing 500/45; base height 80/15; yaw/pitch/roll Kp = 150/100/100, Kd = 30/20/20 |
| WBC task residual multipliers | Swing 100, base acceleration 1, contact force 0.01; their numerical ratio alone does not establish task priority |
| Main NMPC weights | Normalized forward linear momentum `Q(0,0)=100`, height `Q(8,8)=2000`; see the task file for full Q/R |
| Shared velocity limits | Forward/lateral 0.05 m/s each, yaw 0.20 rad/s; faster tests use isolated simulation profiles |
| Model / transmission | 15.15 kg; internal motor ratio 6.33, total calf ratio 12.66; nominal joint torque limits 23.7/23.7/45 N·m |
| Simulation stress defaults | 100 Hz IMU; 5 ms additional command delay; equal added calf Coulomb friction 0.45 N·m and viscous coefficient 0.0175 N·m·s/rad; zero added hip/thigh friction |

1000 Hz is a configured target, not proof of a 1 ms hardware deadline. The additional 5 ms excludes one outer-loop cycle and physics-step phase. Neither delay nor friction has been identified on the robot. The default Fortress profile uses a 5 ms planned-contact confidence delay and a 10 ms ramp; a custom `controllers_file` preserves its values unless explicitly overridden. Attitude prediction defaults to disabled in simulation and hardware; optional rolling-foot estimation defaults to radius 0.

Other retained defaults are a joint velocity-reference slew limit of 400 rad/s², nominal-height acceleration correction capped at 2 m/s², and a cross-track reference position error cap of 0.04 m. Windowed torque budgeting, fixed-total-torque execution, nominal joint references, and joint-acceleration coupling remain disabled experimental options.

## Algorithm and model

The chain is IMU/joint feedback → kinematic state estimation → NMPC state/input policy → Weighted WBC → `q_des, dq_des, tau_ff, Kp, Kd` → motor-side FF + PD.

NMPC plans centroidal momentum, base/joint states, contact forces, and joint velocities. WBC uses current feedback and the contact schedule to optimize tasks subject to rigid-body dynamics, stance-foot, friction-cone, joint, and torque constraints. The default host boundary evaluates `abs(tau_ff)+Kp*abs(q_des-q)+Kd*abs(dq_des-dq)` at the current sample and scales FF and both gains together. This does not guarantee the total request stays within limits during delayed execution or between host samples.

The robot has IMU and joint feedback, without foot contact sensors. Scheduled/estimated contacts, NMPC forces, and simulation effort are not measured sole forces. The physical meaning and calibration of hardware SDK torque feedback remain unverified.

The local [custom_dog_description](src/custom_dog_description/README_EN.md) contains three URDFs, 17 STL assets, and provenance. No neighboring RL repository or old model underlay is required. Total mass includes a 1.30084 kg battery correction; missing battery dimensions leave its intrinsic inertia as a point-mass approximation. See the [controller guide](src/custom_dog_control/README.md) and [architecture](docs/architecture.md).

## Build and simulate

Run from the repository root. Prepare OCS2, the SDK, and system dependencies using [setup.md](docs/setup.md); pinned sources are listed in [dependencies.lock.yaml](src/custom_dog_control/config/dependencies.lock.yaml).

```bash
# Build only: no nodes or motor ports are opened. Requires a configured Unitree SDK.
src/custom_dog_control/scripts/build_real.sh

# Inspect hardware launch arguments without starting control.
src/custom_dog_control/scripts/run_real.sh --show-args

# Native Gazebo Fortress with GUI; omit gui:=true for the default headless mode.
src/custom_dog_control/scripts/run_fortress.sh gui:=true
```

Default Fortress uses motor-side PD, disables ground-truth feedback, and applies the stress settings above. Truth is reserved for independent evaluation. Initial CppAD generation can take several minutes; wait for `CppAD models ready` and controller activation. The `.cache/cppad` cache is separated by model, configuration, and ABI fingerprints.

While simulation is running, use a second terminal with matching `ROS_DOMAIN_ID` and `IGN_PARTITION` for the low-speed check. Only one command-producing test may control the instance:

```bash
source src/custom_dog_control/scripts/env.sh
python3 src/custom_dog_control/scripts/simulation_fortress_test.py \
  --output log/fortress-check.json
```

This script requests simulated stand-up, forward motion, stop, and resume; keep its ROS domain isolated from hardware. Faster tests require explicit limits in an isolated simulation profile; scripts do not relax shared limits. Overrides, speed sweeps, and historical Gazebo Classic entries are documented in the [controller README](src/custom_dog_control/README.md).

## Latest evidence and limitations

On 2026-11-16, the default extra delay and matching planned-contact confidence delay were reduced to 5 ms. A new 20 ms baseline used the same sources, libraries and input profiles. Both completed sequential 20-second forward phases at 0.5 and 1.0 m/s, then 30 seconds standing, without recorded nominal torque excursions. At 1 m/s, 5 ms reduced roll peak-to-peak to 2.08° (20 ms: 4.48°) and body-frame swing-foot FK error RMS to 6.52 mm (13.60 mm), but mean height rose to 0.3461 m. Height oscillation increased at 0.5 m/s, so improvement is not universal. See the [5/20 ms comparison](docs/validation/delay5-comparison-20261116.json). Each condition has one sequential trial, without repeatability acceptance.

The **2026-11-15 visual showcase** used one fresh world, sequentially running forward at 0.5 and 1.0 m/s for 20 seconds each, followed by a 30-second stance. These are not independent fresh-world repetitions per speed. Source, library, and input fingerprints remained unchanged during recording; hardware IO was disabled.

| Metric | Target 0.5 m/s | Target 1.0 m/s |
| --- | --- | --- |
| Actual body forward speed, final 2-second mean | 0.519 m/s | 0.961 m/s |
| Actual height, phase seconds 6–20 mean | 0.33479 m | 0.33512 m |
| Roll / pitch peak-to-peak, phase seconds 6–20 | 1.65° / 1.63° | 4.24° / 2.61° |
| Height peak-to-peak, phase seconds 6–20 | 5.04 mm | 10.70 mm |
| Lateral displacement over 20 seconds, initial-heading frame | 0.0156 m | 0.1264 m |
| Peak delayed FF + PD torque request | 15.534 N·m, RR thigh | 24.119 N·m, FL thigh |

Both scripted checks passed without FAULT, and final stance height was 0.340062–0.340090 m. However, **one 1.0 m/s joint sample exceeded the 23.7 N·m nominal limit, failing the stricter total-torque budget** while remaining below the 26.07 N·m protection threshold. Torque here is a simulation request, not measured motor output. Roll oscillates mainly at 2.5 Hz; height/pitch mainly at 5 Hz. Body smoothness remains inadequate, and matching gait frequencies do not independently prove causation. Methods, metrics, and raw-file fingerprints are in the [showcase summary](docs/validation/simulation-showcase-20261115.json).

In the preceding 16 independent-world weight/PD/rolling-estimator cases, 14 ±1 m/s cases failed lateral-drift checks and three candidates exceeded nominal torque limits. No new weights or PD were adopted; rolling compensation also failed to improve the overall result and remains disabled. See the [comparison evidence](docs/validation/wbc-weight-rolling-20261115.json). One successful showcase does not supersede those failures or establish all-direction, terrain, repeatability, onboard deadline, or hardware locomotion acceptance.

Results at 500/50 Hz, 250/20 Hz, and older gains/heights are historical profiles. They do not validate current defaults. See the [execution-boundary audit](docs/validation/execution-budget-consistency-20261115.md), [historical baselines](docs/validation-baselines.md), and [validation reports](docs/validation/). This documentation update does not rerun motion experiments.

## Hardware entry and logs

The current hardware-check scope is zero-torque, zero-gain communication and IMU inspection. Builds, offline solves, PTY tests, and communication success do not establish loaded-motion acceptance. Historical user stand-up/stance logs do not formally validate the current profile; see the [hardware guide](docs/hardware.md).

On 2026-11-16, this board's zero-command timing probe measured **1.77 ms P99** for four-leg IO and **3.05 ms P99** for IO plus isolated estimator/WBC computation with background NMPC, missing the configured 1 ms deadline. It used CPU 4–7 and ordinary scheduling, without a production ROS controller or real motion. See the [timing evidence and limits](docs/validation/timing-identification-20261116.json).

Follow-up comparisons at 250 Hz WBC with 50/25/20 Hz NMPC favor 250/50 Hz as a board timing candidate, with actual new policies at about 35.6–37.2 Hz. Occasional deadline misses remained in those tests; shared defaults were kept at 1000/50 Hz, without dynamic acceptance of the candidate. See the [frequency candidate evidence](docs/validation/control-frequency-candidates-20261116.json).

Optimization checks found equivalent WBC hot/cold results without a substantial speedup. A CPU-layout candidate measured about 2.06–2.19 ms loop P99 but still had stale policies and was not deployed. NMPC node-stage profiling and a 500 Hz IO-only check are included in the [optimization evidence](docs/validation/control-optimization-20261116.json).

Two operator standing attempts that evening faulted on stale policies about 1.585/0.140 seconds after entering MPC_STANCE, before completing the initial handoff or entering Trot. The controller now retains the triggering timing snapshot. Hardware launch accepts an explicit `control_frequency_hz:=250` candidate, retaining 50 Hz NMPC and the 40 ms policy guard while tightening the WBC failure count from 5 to 1. Shared YAML and direct `run_real.sh` remain at 1000/50 Hz. Desktop Run Dog now defaults to 250/50 Hz with RViz off; sustained standing validation is still required. See the [standing fault procedure](docs/hardware.md#2026-11-16站立交接策略超时与处理入口).

The desktop **Run Dog** entry invokes the controller package's `scripts/run_dog.sh`, passing `control_frequency_hz:=250 use_rviz:=false` and retaining approximately 0.9 seconds of startup alignment. Explicit operator arguments override these defaults. Alignment can produce nonzero torque before START, so this entry is not a zero-output communication probe. Direct `run_real.sh` defaults to alignment and RViz disabled; confirmation-parameter differences are documented in the hardware guide.

| Existing gamepad operation | Behavior |
| --- | --- |
| START | Capture encoder offsets while passive; if alignment is enabled, wait for completion and press START anew; no automatic stand-up |
| LT+A / LT+X | Request stand-up/stance after calibration; enable walking from stance, still requiring valid velocity input |
| LT+B / LT+Y | Passive/fault acknowledgement; software emergency stop latching FAULT |
| Left stick vertical / right stick horizontal / left stick horizontal | Forward / lateral / yaw velocity; centered sticks retain stance |

Hardware launches record approximately 10 Hz diagnostics to `control-diagnostics.jsonl` beside `launch.log` in the current ROS log directory. Each segment is capped at 16 MiB, retaining the initial segment and latest three subsequent segments. Set `record_diagnostics:=false` to disable. Logged controller requests may differ from final motor packets.

Timing percentiles cover the latest 512 control samples. `io_period_ms` measures a complete host SDK exchange, not a single motor's one-way delay. Existing 10 Hz diagnostics cannot accurately identify 5–20 ms dynamics, and the simulation-only high-rate trace is rejected on hardware. Per-motor monotonic timestamps, final-packet logging, asynchronous high-rate recording, and bench response/friction identification are **proposed work**, without an implemented new recorder, identified datasets, or resulting hardware compensation. The shaft-side and current convention of the 0.63895 N·m/A constant also remain unconfirmed.

## Packages, dependencies, and checks

See the [source-package index](src/README_EN.md) and [external workspace guide](external/README.md). `src/unitree_guide` is historical reference excluded by `COLCON_IGNORE`; the active motor interface is [IOSDK.cpp](src/custom_dog_control/src/hardware/IOSDK.cpp). Pinned upstream sources retain their authorship and licenses. Generated dependency workspaces, build products, and bulk logs are excluded from commits.

```bash
source src/custom_dog_control/scripts/env.sh
colcon test --packages-select custom_dog_control custom_dog_description \
  --event-handlers console_direct+
colcon test-result --verbose
```

The latest recorded algorithm regression passed 26 CTest groups as documented in the reports above; that does not mean this documentation update reran them. Open work includes delayed total-torque budgeting, body smoothness, foot/velocity tracking, transmission and torque calibration, onboard deadlines, and hardware locomotion acceptance.
