# 2026-11-02：基础 NMPC＋WBC 物理仿真与标定检查

本机 Orange Pi 5 Plus / RK3588 已完成原生 Gazebo Fortress 6.18 物理测试，模型质量 **15.15 kg**。使用现有 `NmpcWbcController`、OCS2 NMPC 与 Weighted WBC，通过 `gz_ros2_control` 驱动实际物理关节，没有访问真实电机串口。

本报告保留 **1000 Hz 主控、原起身增益与原步态**的通过基线，证明补电池后的模型可以完成本轮平地低速动作。后续 250 Hz 主控、一周期输出延迟及关节 PD 调整是另一组部署实验，结果与剩余问题见 [真机链路与后续仿真记录](real-control-validation-20261102.md)；不能把不同控制条件的失败归为电池增重导致原参数失效。

## 通过的范围

控制周期目标 1000 Hz（仿真时间），NMPC 20 Hz、2 求解线程，绑定 RK3588 大核 CPU 4–7；物理步长 1 ms，目标实时因子 0.5，实际测试约 0.39。保留原起身 Kp 45→55、Kd 1.5→2.5、0.25 s Trot 周期和名义抬脚参数。控制器关闭真值辅助，只接收仿真 IMU 和关节反馈；仿真真值仅供独立评价。

| 检查 | 结果 |
| --- | --- |
| 趴姿起身、WBC 站立 | 通过 |
| 前进 0.05 m/s、10 仿真秒 | 实际前进 0.519 m |
| 零速停车、再次行走 | 通过，无需重复使能 |
| A 显式站立后保持非零速度输入 | 保持站立，不自动重启 Trot |
| 后退、左右侧移 | 各 5 秒，分别约 −0.300 m、+0.275 m、−0.275 m |
| 原地正向转动 0.2 rad/s | 5 秒实际约 1.004 rad |
| 行走中 WBC | 验收采样无无效输出、无 FAULT |

独立真值/关节 FK 记录共 1001 帧：行走时机身高度约 0.279–0.289 m，最大 roll/pitch 约 0.0131 rad（0.75°）；四腿采样最大离地高度均约 6.4 cm，确有交替抬脚。脚离地高度根据实际关节与仿真位姿计算，不是规划器声明的目标，也不是实测足底力。

真值辅助模式也单独通过了同样的前进/停车/恢复测试。上述是平地、低速范围；没有据此验收高速、复杂地形或真机运动。

## 本轮修复与参数限制

- Fortress 版本的 odometry 线速度在机体系，角速度为 RPY 导数；新增适配器转换为控制器需要的世界系速度，并过滤启动瞬态。
- SDFormat 与 URDFDOM 同名 XML 导出函数返回类型不同。已局部绑定正确的 URDFDOM 函数，保持自碰撞检测；补丁保存在 `ocs2-coal-self-collision.patch`。
- 显式 canonical URDF 文件优先于仿真器注入的装饰描述，避免临时插件路径触发重复生成 AD 模型。
- 修复进入站立时反复重设行走使能标志的问题，保留自动停车后的恢复，同时尊重 A 显式停步。
- 修正估计器将球足中心当作地面接触点的问题：平地足心高度应为 0.026 m。
- **50 Hz NMPC 基线在本机物理仿真中曾触发策略超时**；通过的是 20 Hz 配置。物理仿真以低于实时速度运行，不代表真机完整控制周期已满足实时预算。

测试使用已有内容匹配的 AD 缓存 task 副本；与仓库 task 的区别仅为生成缓存目录和线程数。Fortress 入口显式设置 2 求解线程，其他动态控制参数一致。默认新 task 首次启动仍可能需要数分钟编译 AD 模型。

## 标定检查

仿真关节已处于 URDF 坐标，不需要真实编码器零点标定；仿真 START 直接执行起身。原生仿真插件已实际将 12 个 `initial_value` 写入物理关节，而非仅修改显示值。

另用真实生产 IOSDK 与 Unitree SDK、四个模拟串口完成 **8 组软件标定回归**。684 个发出报文均为 mode 0、全零命令，覆盖 12 个编码器的方向/减速比、任意单电机坏 CRC 或掉包拒绝标定、成功同周期更新角度、Reset 后不复用旧零位以及显式重新标定。最大数值误差约 1.2e−7 rad，这是软件回归误差，**不代表真实机械标定精度**。

真实机械趴姿、安装方向和编码器零位仍未验收。真机 START 只标定，完成回到 PASSIVE；A 才明确请求起身。

## 使用和证据

本机已安装 `ignition-fortress`、`ros-humble-gz-ros2-control`、`ros-humble-ros-gz-sim` 和 `ros-humble-ros-gz-bridge`，并完成控制包构建。此次验收进程已停止；启动一个新的仿真实例即可操作。

```bash
cd /home/orangepi/qr_ws
src/custom_dog_control/scripts/run_fortress.sh
# 桌面终端可加 gui:=true；use_sim_ground_truth:=true 可切换诊断基线。
```

在另一个终端运行键盘控制，以下限制与本轮低速测试相符：

```bash
source src/custom_dog_control/scripts/env.sh
python3 src/custom_dog_control/scripts/keyboard_teleop.py --ros-args \
  -p max_speed_x:=0.05 -p max_speed_y:=0.05 \
  -p max_speed_yaw:=0.2 -p increment:=1.0
```

仿真 `2` 起身，`3` 保持站立，`4` 使能 Trot，W/S、A/D、J/L 调节速度，空格清零，`1` 返回被动。原键盘默认最大速度高于本轮验证范围。

自动验证：

```bash
source src/custom_dog_control/scripts/env.sh
python3 src/custom_dog_control/scripts/simulation_fortress_test.py --output /tmp/fortress-result.json
python3 src/custom_dog_control/scripts/simulation_planar_test.py --output /tmp/fortress-planar-result.json
```

完整测试从 PASSIVE 开始，运行成功后停在 MPC_STANCE；随后可运行 planar 测试。重复完整测试前重启仿真，或先清零速度并用 `1` 返回 PASSIVE。自动测试与键盘不要同时发布命令。

本轮测试使用隔离的 `ROS_DOMAIN_ID=57`；标准入口不强制设置 domain，启动与操作终端应使用相同值。同一 `partition`/world 只启动一个实例。`run_fortress.sh` 默认选择 RK3588 可用的大核，可用 `DOG_CPU_AFFINITY=none` 关闭或设置显式 CPU 列表。控制包 45 项检查全部通过。

- [估计器模式完整结果](../log/fortress-validation-20261102/estimator-result.json)
- [汇总、运行参数与源码/二进制指纹](../log/fortress-validation-20261102/final-summary.json)
- [后退、侧移与转向](../log/fortress-validation-20261102/planar-result.json)
- [独立四足离地记录](../log/fortress-validation-20261102/estimator-kinematics.summary.json)
- [软件标定回归](../log/fortress-validation-20261102/calibration_pty/result.json)
- [实际物理数据曲线](../log/fortress-validation-20261102/physics-validation.png)
