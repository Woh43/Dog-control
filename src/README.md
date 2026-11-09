# ROS 2 源码包

**中文** | [English](README_EN.md)

当前工作区仅保留基础 NMPC＋WBC 运动链路。构建、操作流程和验证边界统一见 [工作区 README](../README.md)。

| 目录 | 用途 |
| --- | --- |
| `custom_dog_control` | 状态估计、NMPC、WBC、状态机及 ros2_control 硬件插件 |
| `custom_dog_description` | 完整的 15.15 kg 模型、17 个 STL、关节与电池配置 |
| `fdilink_ahrs` | IMU 串口驱动，发布 `/imu` |
| `serial_ros2` | ROS 包名为 `serial`，供 IMU 驱动使用 |
| `unitree_guide` | 历史参考，使用 `COLCON_IGNORE` 排除 |

电机使用 Unitree SDK；实际接口在 [custom_dog_control/src/hardware/IOSDK.cpp](custom_dog_control/src/hardware/IOSDK.cpp)。旧 `src/qr_guide` 已退出当前分支，桌面入口已更新到 `run_real.sh`。

精准落足和对应 `qr_*` 包已移除。模型在本工作区 `custom_dog_description` 源码包中，含电池总质量 15.15 kg；OCS2 在独立 `external/ocs2_ws` 构建。本工作区不引入 RL 运行环境。
