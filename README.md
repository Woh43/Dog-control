# Dog-control · NMPC + WBC

**简体中文** | [English](README_EN.md)

RC2027 当前只保留 12 自由度四足机器人的基础运动控制：ROS 2 Humble、OCS2 NMPC、Weighted WBC、状态估计和 Unitree 电机/IMU 驱动。精准落足及对应 `qr_*` 包已移除。目标设备是本机 Orange Pi 5 Plus / RK3588（Ubuntu 22.04 ARM64）。

## 当前验证状态

**构建、离线求解及串口通信检查不等于真机运动验收。当前尚未完成真实标定姿态、起身承重站立或 Trot 验收。** 当前操作与验证边界见 [真机说明](docs/hardware.md)。

**01:25 更新：** 最新真机在切入 MPC_STANCE 约 0.201 秒后因 NMPC 策略超过 100 ms 门限而保护停机，故障前通信有效。该轮与本机物理仿真时间重叠，CPU 竞争影响未排除。旧 main 祖先的 6.2/0.2 是电机侧增益；当前45/55是换算前关节侧增益，起身力度不能按数字直接比较。后腿起身误差和交接条件仍需处理，详见上述真机说明。

**2026-11-10 最新真机日志：** 用户报告起身时后部需托一下，随后能自行站住。01:03:54 这次启动在 MPC_STANCE 运行约 48 秒后，右后髋关节（RR ID 0）反馈失效而保护停机；当时 IMU 和 NMPC 正常。后部抬起困难发生得更早，原因尚未确定。只按 LT+X 而不推摇杆会保持站立，日志没有 Trot 切换。随后四路全零通信 60 秒共 72000/72000 有效回复，但不能排除承重控制中的偶发超时。已增加默认自动诊断记录及行走使能提示，见 [排查记录](docs/hardware.md)。

此前估计器临时对象生命周期缺陷和 IMU 检查时刻竞争已修复，完整 15.15 kg 模型和持久 CppAD 缓存已在本工作区。当时控制包 18 项与模型包 1 项检查通过；关闭真值辅助、100 Hz IMU 的 Fortress 完成旧手柄起身、前后/左右 Trot、双向转向、停车恢复及最后 30 秒站立。该结果不替代真实起身困难的诊断或承重运动验收。

新增诊断后的安装版本已重新通过相同模型、100 Hz IMU 的起身/前进/停车恢复仿真，完整记录各阶段的 12 关节及控制器请求，测试期间源码与库保持不变。控制包首轮 18/19 项通过，模拟串口套件一次超时后单独重跑通过；记录器 ROS 集成和模型包检查通过。首次失败及中止的测试保留，详见上述排查记录。

本机已通过 **15.15 kg 模型、1000 Hz 主控的 Gazebo Fortress 平地低速物理仿真**：起身、WBC 站立、前后与侧向 Trot、转向、停车及恢复。关闭真值反馈、仅用 IMU 与关节估计时同样通过；保留原起身增益和步态。该轮使用 NMPC 20 Hz、2 求解线程，50 Hz 曾触发策略超时；软件标定完成 8 组模拟串口回归，当时控制包 45 项检查通过。详见 [本轮仿真与标定记录](docs/simulation-validation-20261102.md)。

后续 **250 Hz 主机合成 PD、发送纯力矩**的对照是历史实验，见 [当时的记录](docs/real-control-validation-20261102.md)。当前已恢复电机侧五分量 PD，真机与 Fortress 共用 250 Hz 主控、20 Hz NMPC、2 求解线程、增益、步态及低速限制。**恢复启动预对位之前，关闭真值辅助的完整低速物理回归已通过**：使用旧手柄 START/组合键完成起身、前进、停车与恢复，后退、左右侧移、双向转向，以及最后 60 秒站立；全程未进入 FAULT。10 秒前进 0.551 m，6 秒恢复 0.326 m。此后新增硬件启动辅助、真实 START 等待和可视化，NMPC/WBC 算法配置未改，但构建指纹已经变化；旧结果不覆盖新增辅助的验收。完整结果见 [兼容移植验收记录](docs/compatibility-validation-20261103.md)；真机运动仍未验收。

并发 colcon 编译时曾发生策略超时，独立重跑上述场景通过，超时门限保持不变；这不构成有任意并发负载时的实时性保证。

**11 月 3 日零输出通信复测中，FR 反馈已恢复。** 12 个电机顺序探测各 500/500，合计 6000/6000，CRC、ID、模式及电机错误均为 0。随后四路同时按每 ID 100 Hz 轮询约 5 秒，取得 6677/6678 个有效回复；结束快照记录 FR ID 2 一次未收齐回包/超时，原因未定位。因此不能称为全程零丢包或 250 Hz 控制周期验收通过。首次 IMU 捕获含一条较早的 AHRS 时间戳；仅清空主机 RX 队列后重新捕获，IMU/AHRS 各 500 帧、设备时间戳均为 100 Hz，CRC 为 0，时间戳异常未复现，原异常来源未确证。所有探针已停止并释放串口。证据见 [通信复测](log/communication-recheck-20261103-2347/summary.json)及[并发结果](log/communication-recheck-20261103-2347/parallel/result.json)。**这些探针只发送 mode0 全零命令，不能证明真实启动辅助、机械标定或承重站立行走已验收。**

早期检查曾记录 FL/RR/RL 各 100/100、FR 各 0/100；旧 SDK typed 与 raw 2/20 ms 对照也未取得 FR 有效回复。这些历史结果保留，先前无反馈的原因没有由此次成功复测确定。原驱动已实际发布 Xbox 360 手柄输入；详细边界见 [兼容验收记录](docs/compatibility-validation-20261103.md)。

本轮保留旧电机协议、IMU 数据变换与 START 采样零点流程，在 URDF 坐标下接入 NMPC/WBC，并恢复旧手柄布局。运行参数对照确认两条路径使用同一编译控制核心和模型/task，算法参数一致，差异限于时钟、硬件接口和仿真专用项。此前主机 PD、降频与延迟候选的修改属于控制时序选择，不能全部归因于电池增重。完整带负载时序仍待验证；本次通信探针没有触发起身或行走，用户期间自行启动过控制程序，不作为本次运动验收。历史 13.84916 kg 仿真和 15.15 kg 离线测试分别见 [验收基线](docs/validation-baselines.md) 与 [电池模型记录](docs/battery-model-validation-20261102.md)。

机器人总质量为 **15.15 kg**：裸 CAD 13.84916 kg，加 base 中心电池 1.30084 kg。躯干质量、质心和平行轴惯量已同步；缺少电池尺寸，电池自身惯量暂按点质量近似。完整模型已纳入 [src/custom_dog_description](src/custom_dog_description/README.md)，包含三份 URDF、17 个实体 STL、标定约定和电池配置。来源固定为你的 `Woh43/custom_dog_stack` 仓库提交 `6c776de8`，并保留本项目已验证的电池修正；不再依赖相邻 RL 目录或旧模型 underlay。

## 构建和启动

在 `/home/orangepi/qr_ws` 执行：

```bash
# 只构建，不启动节点或打开电机串口
src/custom_dog_control/scripts/build_real.sh

# 查看启动参数，不启动控制节点
src/custom_dog_control/scripts/run_real.sh --show-args
```

构建脚本先在本工作区构建 `custom_dog_description`，再构建控制器。运行入口检查实际加载的模型属于本工作区，且与源码模型一致。OCS2 和 SDK 的其他目录布局可设置 `CUSTOM_DOG_CONTROL_DEPS_WS`、`UNITREE_ACTUATOR_SDK_ROOT`；模型固定使用本仓库源码，不再读取 `CUSTOM_DOG_DESCRIPTION_DIR` 或 `CUSTOM_DOG_MODEL_WS`。安装与构建依赖见 [setup.md](docs/setup.md)。

标准运行入口将 NMPC 的 CppAD 模型库保存在 `.cache/cppad`，跨重启复用；按模型、配置和编译 ABI 的内容指纹隔离。首次生成可能持续数分钟，请等待 `CppAD models ready` 及控制器激活。此前 11 月 9–10 日的多次启动在 `/tmp` 冷编译完成前结束，尚未进入正常起身流程，详见 [启动排查](docs/hardware.md)。

桌面 **Run Dog** 调用 `src/custom_dog_control/scripts/run_dog.sh`，无需输入 `YES`，并显式启用 `startup_alignment_enabled:=true`、`use_rviz:=true`。启动前自动停止本工具自身的 mode 0 通信监视进程，释放电机串口；`--help`/`--show-args` 仅查看信息，不停止监视。启动后控制器保持 PASSIVE，硬件在 12 路反馈有效、控制器已接管并持续写入后，自动执行一次约 0.9 秒的小力矩预对位。完成后重新按 START 采零，LT+A 请求起身，LT+X 使能 Trot。

预对位沿用旧版转子方向，关节侧力矩幅值为 hip/thigh/calf `0.95/0.63/1.52 N·m`：先 hip/thigh 300 ms，再三关节 300 ms，最后 mode0 释放 300 ms。辅助为非阻塞、单次执行；通信或 CRC 失败、软件急停、写入心跳过期、撤销控制器接管或过程中的标定复位会中止并退回零输出，不自动重试，须重启后恢复。它会在启动后、START 之前发送非零力矩，PASSIVE 不能一概理解为恒定 mode0；任一电机反馈不完整或陈旧时不会开始。**恢复后的 profile 已经 PTY 验证；本次通信探针没有发送该非零辅助，真实辅助执行仍未验收。**

真机 RViz 固定坐标系为 `base`，RobotModel 使用关节反馈，TF 显示足端正运动学结果。标定前编码器角尚未对齐 URDF 零位，显示姿态不代表机械标定已完成；足端 TF 也不是实测足底力。

桌面入口通过 `require_physical_estop_confirmation:=false` 取消额外确认条件，不将 `physical_estop_verified` 写成已验证；直接调用 `run_real.sh` 仍默认要求该确认条件，并默认关闭启动辅助和 RViz。FR 反馈已恢复，并发结束快照的一次未收齐回包/超时仍有记录；通信恢复不能代替承重站立或行走验收。

真机手柄流程：

| 按键 | 行为 |
| --- | --- |
| START | 若启用启动辅助，先等其完成；随后在被动状态新按 START 采集零点，完成后回到 PASSIVE；不会自动起身 |
| LT+A | 标定有效后，明确请求起身并进入 WBC 站立 |
| LT+X | 已站立后使能 Trot；仍需有效速度指令和新步态策略 |
| LT+B | 返回被动；故障后确认复位，随后需要重新标定 |
| LT+Y | 软件急停，进入锁存 FAULT；替代已移除的旧 STEP_TEST |
| 左摇杆前后 | 前后速度 |
| 右摇杆左右 | 侧向速度 |
| 左摇杆左右 | 转向速度 |

LT 即旧 L2 扳机；单按 A/X 不切换状态。LT+X 只使能行走，摇杆回中仍站立，不原地踏步。三轴保留 0.08 死区，在 REP-103 下映射为 `vx=-LY、vy=-RX、yaw=-LX`。共享限速为前后/侧向各 0.05 m/s、偏航 0.2 rad/s。START 标定公式与旧流程在坐标变换后的等价关系见 [兼容验收记录](docs/compatibility-validation-20261103.md)。

真实启动默认在本次 ROS 日志目录保存 `control-diagnostics.jsonl`，自动记录约 10 Hz 的全 12 关节反馈、起身目标、控制器请求、手柄门限及故障，无需另开记录终端。控制器请求不是最终电机报文。每段最多 16 MiB，保留初始段和最近三个分段；`record_diagnostics:=false` 可关闭。

现有原始帧的设备时间戳证实 IMU/AHRS 当前各输出约 100 Hz，驱动没有固定 100 Hz 限频。用户之后自行调整设备输出率；本轮不发送 IMU 配置命令。

旧 `src/qr_guide/src/interface/IOSDK.cpp` 已不属于当前分支。现在的电机入口是 [IOSDK.cpp](src/custom_dog_control/src/hardware/IOSDK.cpp)，请关闭 IDE 中的旧文件标签。

## 软件结构

| 目录 | 内容 |
| --- | --- |
| `src/custom_dog_control` | 控制器、NMPC/WBC、状态机、硬件插件及测试 |
| `src/fdilink_ahrs` | IMU 驱动，发布 `/imu` |
| `src/serial_ros2` | ROS 包 `serial`，供 IMU 使用；电机通信使用 Unitree SDK |
| `src/unitree_guide` | 历史参考，`COLCON_IGNORE` 排除 |
| `src/custom_dog_description` | 自包含的 15.15 kg URDF、完整 STL 与模型来源记录 |
| `external/ocs2_ws` | 固定版本 OCS2 及 ARM64 构建依赖 |

算法链路是：IMU/关节反馈 → 运动学状态估计 → NMPC 策略 → Weighted WBC → 目标位置/速度、前馈力矩、Kp/Kd 五分量 → 电机侧 PD。主机按当前采样的 `abs(tau_ff)+kp*abs(q_des-q)+kd*abs(dq_des-dq)` 对前馈和两项增益共同缩放；这只约束采样时的贡献上界，不能保证反馈变化后的连续力矩上限。

详细接口见 [控制包说明](src/custom_dog_control/README.md) 和 [架构说明](docs/architecture.md)。机器人只有 IMU 和编码器，没有足底接触传感器；接触计划和估计不能标为实测足底力。

## 验证

```bash
source src/custom_dog_control/scripts/env.sh
colcon test --packages-select custom_dog_control --event-handlers console_direct+
colcon test-result --verbose
```

本机原生 Fortress 入口（默认无界面；桌面终端可加 `gui:=true`）：

```bash
src/custom_dog_control/scripts/run_fortress.sh
```

默认关闭真值辅助，使用公共 250 Hz/20 Hz 配置和 RK3588 大核；首次生成 AD 模型可能需要数分钟。当前默认电机 PD 仿真在每个 1 ms 物理步计算 PD，并模拟外环一周期与物理步相位延迟；这是理想伺服模型，没有测得真实电机带宽。复现本轮通过的旧手柄流程可加 `legacy_joy_layout:=true`。低速完整动作及 60 秒站立结果见 [兼容验收记录](docs/compatibility-validation-20261103.md)，此前 1000 Hz 的已通过结果见 [历史仿真记录](docs/simulation-validation-20261102.md)。

有 Gazebo Classic 的开发机仍保留旧入口：

```bash
CUSTOM_DOG_BUILD_ONLY=1 src/custom_dog_control/scripts/build_simulation.sh
src/custom_dog_control/scripts/run_simulation.sh
src/custom_dog_control/scripts/test_simulation.sh
```

本轮使用 Fortress 进行当前配置回归；未验证上述 Classic 入口。

精准落足源码可从 Git 历史恢复；移除前文档、工作区补丁及旧安装产物保存在 `/home/orangepi/qr_ws_backups/remove_precision_20261102/`。当前开发分支为 `RC2027`，远端为 `origin/RC2027`；未自动提交或推送本轮修改。
