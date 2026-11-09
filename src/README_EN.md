# ROS 2 source packages

[中文](README.md) | **English**

This workspace contains the basic NMPC + WBC locomotion stack. Build instructions, operator controls, and validation limits are maintained in the [workspace README](../README_EN.md).

| Directory | Purpose |
| --- | --- |
| `custom_dog_control` | State estimation, NMPC, WBC, state machine, ros2_control hardware |
| `custom_dog_description` | Complete 15.15 kg model, 17 STL files, joint and battery configuration |
| `fdilink_ahrs` | Serial IMU driver publishing `/imu` |
| `serial_ros2` | ROS package `serial`, used by the IMU driver |
| `unitree_guide` | Historical reference excluded by `COLCON_IGNORE` |

Motors use the Unitree SDK through [IOSDK.cpp](custom_dog_control/src/hardware/IOSDK.cpp). The old `src/qr_guide` path is obsolete; the desktop shortcut now uses `run_real.sh`.

Precision foothold control and its `qr_*` packages have been removed. The local `custom_dog_description` source package contains the complete 15.15 kg battery-corrected model; OCS2 builds separately in `external/ocs2_ws`. No RL runtime is loaded.
