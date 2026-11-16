# ROS 2 源码包

**简体中文** | [English](README_EN.md)

本工作区维护基础 NMPC＋WBC 速度控制链路。当前开发与合并目标为 `RC2027`；配置、安装、启动和最新验证边界统一见 [工作区 README](../README.md)。文档核对日期：2026-11-16。

| 目录 | 用途与说明 |
| --- | --- |
| [custom_dog_control](custom_dog_control/README.md) | 状态估计、OCS2 NMPC、Weighted WBC、状态机、ros2_control 硬件插件、Fortress 仿真及测试 |
| [custom_dog_description](custom_dog_description/README.md) | 本地完整 15.15 kg 模型、三份 URDF、17 个 STL、电池修正及来源记录 |
| [fdilink_ahrs](fdilink_ahrs/README.md) | IMU 串口驱动；当前机器人设备约 100 Hz，不代表驱动固定限频 |
| [serial_ros2](serial_ros2/README.md) | ROS 包名为 `serial`，供 IMU 驱动使用；不是电机串口实现 |
| `unitree_guide` | 历史参考，由 `COLCON_IGNORE` 排除，不参与当前 colcon 构建 |

当前电机入口是 [custom_dog_control/src/hardware/IOSDK.cpp](custom_dog_control/src/hardware/IOSDK.cpp)，使用 Unitree SDK；髋／大腿总减速比为 6.33，小腿为 12.66。`third_party/legged_control`、`legged_wbc` 和 qpOASES 随控制包编译，独立 OCS2 underlay 位于 [external/ocs2_ws](../external/README.md)。来源与版本由 [依赖锁定文件](custom_dog_control/config/dependencies.lock.yaml) 管理。

当前公共配置为主控／WBC 1000 Hz、NMPC 50 Hz、周期 0.40 s、规划抬脚 0.10 m、名义高度 0.34 m；主动阶段关节 Kp 60/60/90、Kd 2。配置目标与仿真通过不证明真机实时期限或承重运动已验收。公共限速仍为前后／侧向 0.05 m/s、偏航 0.2 rad/s；高速试验使用独立仿真 profile。

精准落足与对应 `qr_*` 包已移除；RL 训练、策略和部署属于独立 `himloco_custom_dog`，不要引入本工作区。机器人没有足底接触传感器，计划／估计接触及 SDK effort 不能标成实测足底力。

生成的 `build/`、`install/`、`log/`、CppAD 缓存和 external 工作区不提交。每个包 README 说明本包职责与接口，跨包操作以根 README 和 [安装说明](../docs/setup.md) 为准。
