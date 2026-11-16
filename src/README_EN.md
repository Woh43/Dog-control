# ROS 2 source packages

[简体中文](README.md) | **English**

This workspace maintains the basic NMPC + WBC velocity-control chain. Development and integration target `RC2027`; configuration, setup, launch instructions, and validation limits are maintained in the [workspace README](../README_EN.md). Documentation reviewed 2026-11-16.

| Directory | Role |
| --- | --- |
| [custom_dog_control](custom_dog_control/README.md) | State estimation, OCS2 NMPC, Weighted WBC, state machine, ros2_control hardware, Fortress simulation, and tests |
| [custom_dog_description](custom_dog_description/README_EN.md) | Complete local 15.15 kg model, three URDFs, 17 STLs, battery correction, and provenance |
| [fdilink_ahrs](fdilink_ahrs/README.md) | Serial IMU driver; this robot's device currently outputs approximately 100 Hz, not a fixed driver cap |
| [serial_ros2](serial_ros2/README.md) | ROS package `serial`, used by the IMU driver, not the motor interface |
| `unitree_guide` | Historical reference excluded from current colcon builds by `COLCON_IGNORE` |

The active motor entry is [custom_dog_control/src/hardware/IOSDK.cpp](custom_dog_control/src/hardware/IOSDK.cpp), using the Unitree SDK. Total hip/thigh ratios are 6.33 and the calf ratio is 12.66. `third_party/legged_control`, `legged_wbc`, and qpOASES compile with the control package; the independent OCS2 underlay resides in [external/ocs2_ws](../external/README.md). Sources and revisions are recorded in the [dependency lock file](custom_dog_control/config/dependencies.lock.yaml).

Shared defaults are 1000 Hz main control/WBC, 50 Hz NMPC, a 0.40 s gait cycle, 0.10 m planned clearance, and 0.34 m nominal height; active-stage joint Kp is 60/60/90 with Kd 2. A configured rate or successful simulation does not establish hardware deadlines or loaded-motion acceptance. Shared limits remain 0.05 m/s forward/lateral and 0.2 rad/s yaw; faster experiments use isolated simulation profiles.

Precision foothold control and the `qr_*` packages were removed. RL training, policies, and deployment belong to the separate `himloco_custom_dog` repository. The robot has no sole contact sensors; planned/estimated contacts and SDK effort must not be described as measured sole forces.

Generated `build/`, `install/`, `log/`, CppAD caches, and external workspaces are not committed. Package READMEs describe their own interfaces; use the root README and [setup guide](../docs/setup.md) for cross-package operations.
