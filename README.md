# Dog-control · NMPC + WBC

**简体中文** | [English](README_EN.md)

ROS 2 Humble 四足机器人模型式运控工作区，维护状态估计、OCS2 NMPC、Weighted WBC、Gazebo Fortress 仿真及 Unitree 电机／FDILink IMU 接口。目标平台为 Ubuntu 22.04 ARM64 的 Orange Pi 5 Plus / RK3588，也支持已配置依赖的桌面开发环境。

当前开发及合并目标是 **`RC2027` / `origin/RC2027`**。精准落足和对应 `qr_*` 包已移除；RL 训练、策略和部署属于独立的 `himloco_custom_dog`。文档核对日期：**2026-11-16**；下述最近运动数据采集于 **2026-11-15**。

## 当前配置

配置以 [controllers.yaml](src/custom_dog_control/config/controllers.yaml)、[task.info](src/custom_dog_control/config/nmpc/task.info) 和对应 launch 为准。

| 项目 | 当前默认值／含义 |
| --- | --- |
| 主控／WBC、NMPC | 1000 Hz / 50 Hz；NMPC 两个求解线程 |
| Trot | 完整周期 0.40 s；每次对角摆动 0.20 s；四足支撑重叠默认 0 |
| 规划抬脚高度 | 0.10 m；实际足端净空仍需测量，短摆动会触发规划器幅值缩放 |
| 名义高度 | 地面到 `base` 原点 0.34 m，不是机身底面净空 |
| 起身、稳定、MPC_STANCE/TROT 关节 PD | 髋／大腿／小腿 Kp = 60/60/90 N·m/rad；Kd = 2 N·m·s/rad |
| WBC 任务空间 PD | 摆足 500/45；机身高度 80/15；姿态 yaw/pitch/roll Kp = 150/100/100、Kd = 30/20/20 |
| WBC 任务残差乘数 | 摆足 100、机身加速度 1、接触力 0.01；不能直接按数值比解释任务重要性 |
| NMPC 主要权重 | 归一化前向线动量 `Q(0,0)=100`，高度 `Q(8,8)=2000`；完整 Q/R 见 task 文件 |
| 公共速度限制 | 前后／侧向各 0.05 m/s，偏航 0.20 rad/s；高速测试使用独立仿真配置 |
| 模型与传动 | 15.15 kg；电机内部减速比 6.33，小腿总减速比 12.66；关节名义力矩上限 23.7/23.7/45 N·m |
| 当前仿真非理想因素 | 100 Hz IMU；额外命令延迟 5 ms；四条小腿统一附加库仑摩擦 0.45 N·m、黏性系数 0.0175 N·m·s/rad；髋／大腿附加摩擦为 0 |

1000 Hz 是配置目标，不代表真机已满足 1 ms 实时期限。额外 5 ms 之外还有一个外环周期及物理步相位；这些延迟与摩擦数值尚未经过真机辨识。默认 Fortress profile 的计划接触信任延后／渐变时间为 5／10 ms，自定义 `controllers_file` 保留文件值，除非显式覆盖。仿真和真机默认关闭姿态前推；球足滚动估计补偿默认半径为 0。

当前还保留关节目标速度变化率限制 400 rad/s²、名义高度加速度修正上限 2 m/s² 和横向参考位置误差上限 0.04 m。窗口力矩预算、固定总力矩执行、名义关节参考及关节加速度耦合决策均为默认关闭的实验选项。

## 算法与模型

控制链路为：IMU／关节反馈 → 运动学状态估计 → NMPC 状态／输入策略 → Weighted WBC → `q_des、dq_des、tau_ff、Kp、Kd` → 电机侧 FF＋PD。

NMPC 规划质心动量、机身／关节状态、接触力及关节速度；WBC 根据当前反馈和接触计划求解刚体动力学、支撑足、摩擦锥、关节／力矩约束下的任务优化。默认主机边界按当前反馈计算 `abs(tau_ff)+Kp*abs(q_des-q)+Kd*abs(dq_des-dq)`，共同缩放 FF 和两项增益；这不保证延迟执行及两个主机采样之间的总请求力矩始终不越限。

机器人只有 IMU 与关节反馈，没有足底接触传感器。接触计划、接触估计、NMPC 接触力和仿真 effort 均不能标成实测足底力。真机 SDK 回报扭矩的物理含义与标定仍需核实。

完整模型由本地 [custom_dog_description](src/custom_dog_description/README.md) 提供，含三份 URDF、17 个 STL 和来源记录，不需要相邻 RL 仓库或旧模型 underlay。15.15 kg 包含 1.30084 kg 电池修正；电池尺寸未知，其本体惯量仍按点质量近似。详细实现见 [控制包 README](src/custom_dog_control/README.md) 和 [架构文档](docs/architecture.md)。

## 构建与仿真

在仓库根目录执行。首次安装 OCS2、SDK 和系统依赖见 [安装说明](docs/setup.md)；固定版本见 [依赖锁定文件](src/custom_dog_control/config/dependencies.lock.yaml)。

```bash
# 仅构建，不启动节点、不打开电机串口；需要已配置 Unitree SDK。
src/custom_dog_control/scripts/build_real.sh

# 查看真机入口参数，不启动控制。
src/custom_dog_control/scripts/run_real.sh --show-args

# 原生 Gazebo Fortress 可视化仿真；去掉 gui:=true 则默认无界面。
src/custom_dog_control/scripts/run_fortress.sh gui:=true
```

默认 Fortress 使用电机侧 PD 模型、关闭真值反馈，并使用上表的额外延迟与摩擦。真值仅用于独立评估。首次 CppAD 模型生成可能需要数分钟；等待 `CppAD models ready` 和控制器激活。缓存按模型、配置和 ABI 指纹保存在 `.cache/cppad`。

仿真保持运行时，可在使用相同 `ROS_DOMAIN_ID`、`IGN_PARTITION` 的另一终端运行低速检查；同一实例只运行一个发指令的测试：

```bash
source src/custom_dog_control/scripts/env.sh
python3 src/custom_dog_control/scripts/simulation_fortress_test.py \
  --output log/fortress-check.json
```

该脚本会向仿真实例请求起身、前进、停车和恢复，必须与真机 ROS 域隔离。高速测试需独立提高仿真 profile 的限速；测试脚本不会自动放开公共限速。参数覆盖、速度扫描和 Gazebo Classic 历史入口见 [控制包 README](src/custom_dog_control/README.md)。

## 最近验证与不足

2026-11-16 将默认额外延迟及配套计划接触信任延后改为 5 ms，并使用相同源码、库和输入配置补跑 20 ms 对照。两组均完成 0.5／1.0 m/s 各 20 秒及最后 30 秒站立，未记录名义力矩越限。1 m/s 下，5 ms 的横滚峰峰值为 2.08°（20 ms 为 4.48°），摆动足机身坐标 FK 误差 RMS 为 6.52 mm（13.60 mm）；平均高度却升至 0.3461 m，仍有偏差。0.5 m/s 的高度波动也增大，不能称为所有指标均改善。详见[5／20 ms 对照记录](docs/validation/delay5-comparison-20261116.json)。每个条件仅一次顺序测试，尚非重复性验收。

**2026-11-15 可视化展示**使用一个新世界，顺序执行前进 0.5、1.0 m/s，各 20 秒，最后站立 30 秒。它不是两档速度各自独立的新世界重复实验。记录期间代码、库及输入配置指纹保持不变；没有真机 IO。

| 指标 | 目标 0.5 m/s | 目标 1.0 m/s |
| --- | --- | --- |
| 实际机身前向速度，末 2 秒均值 | 0.519 m/s | 0.961 m/s |
| 实际高度，第 6～20 秒均值 | 0.33479 m | 0.33512 m |
| 横滚／俯仰峰峰值，第 6～20 秒 | 1.65° / 1.63° | 4.24° / 2.61° |
| 高度峰峰值，第 6～20 秒 | 5.04 mm | 10.70 mm |
| 20 秒横向位移，按起始航向计算 | 0.0156 m | 0.1264 m |
| 延迟执行 FF＋PD 请求力矩峰值 | 15.534 N·m，RR 大腿 | 24.119 N·m，FL 大腿 |

两档脚本检查均通过、无 FAULT，最后站立高度 0.340062～0.340090 m；但 **1.0 m/s 出现一个关节样本超过 23.7 N·m 名义上限，严格总力矩预算未通过**，未达到 26.07 N·m 保护门限。这里的力矩是仿真请求，不是实测电机输出。横滚主要以 2.5 Hz 摆动，高度／俯仰主要以 5 Hz 波动，机身平顺性仍需改善；频率与步态一致不能独立证明根因。完整方法、指标和原始文件指纹见 [本次展示摘要](docs/validation/simulation-showcase-20261115.json)。

此前 16 个独立世界的权重／PD／滚动补偿对照中，14 个 ±1 m/s 用例未通过侧偏检查，3 个候选出现名义力矩越限，未采纳新权重或 PD；滚动补偿也未获得整体更优结果，仍默认关闭。详见 [对照证据](docs/validation/wbc-weight-rolling-20261115.json)。单次展示通过不覆盖这些失败，也不证明全方向、复杂地面、重复性、板载实时期限或真机运动能力。

500 Hz/50 Hz、250 Hz/20 Hz、旧 PD 及旧高度的结果属于历史配置，不能移作当前配置的验收。历史索引见 [执行边界复核](docs/validation/execution-budget-consistency-20261115.md)、[验证基线](docs/validation-baselines.md)及各 [验证报告](docs/validation/)。本次 README 更新不重新执行运动实验。

## 真机入口与日志

当前真机检查范围是零力矩、零增益通信和 IMU 数据检查；构建、离线求解、伪终端测试或通信成功均不等于承重运动验收。已有用户起身与站立日志不能替代当前配置的正式验证，详见 [真机说明及历史问题](docs/hardware.md)。

桌面 **Run Dog** 实际调用控制包的 `scripts/run_dog.sh`，默认启用 RViz 和约 0.9 秒的小力矩预对位。预对位在 START 前可以产生非零力矩，不能把该入口当作零输出通信探针。直接 `run_real.sh` 默认关闭预对位和 RViz；两种入口的确认参数区别见真机说明。

| 既有手柄操作 | 行为 |
| --- | --- |
| START | 被动状态采集编码器零点；启用预对位时须先等完成，再新按 START；不自动起身 |
| LT+A / LT+X | 标定有效后请求起身／站立；已站立后使能行走，仍需有效速度输入 |
| LT+B / LT+Y | 被动／故障复位；软件急停并锁存 FAULT |
| 左摇杆前后／右摇杆左右／左摇杆左右 | 前后／侧移／偏航速度；回中保持站立 |

默认真机入口把约 10 Hz 的诊断写入本次 ROS 日志目录的 `control-diagnostics.jsonl`，与 `launch.log` 同目录。每段最多 16 MiB，保留初始段及最近三个分段；`record_diagnostics:=false` 可关闭。记录的控制器请求不一定是最终电机报文。

周期及 IO 分位数来自最近 512 次控制采样；`io_period_ms` 是整轮主机 SDK 收发耗时，不是单电机单向延迟。现有 10 Hz 诊断不足以准确辨识 5～20 ms 动态，仿真高频 trace 也禁止用于真机。逐电机单调时钟打点、最终报文记录、后台高频落盘与台架响应／摩擦辨识目前是**待实施方案**；尚未新增记录模块、采集辨识数据或依据辨识结果补偿真机。扭矩常数 0.63895 N·m/A 的轴侧与电流约定仍待确认。

## 包、依赖与检查

包目录见 [src/README.md](src/README.md)，依赖工作区见 [external/README.md](external/README.md)。`src/unitree_guide` 只作历史参考并由 `COLCON_IGNORE` 排除，当前电机接口是 [IOSDK.cpp](src/custom_dog_control/src/hardware/IOSDK.cpp)。上游源码、作者与许可证按固定来源保留；生成的依赖工作区、构建产物和大规模日志不提交。

```bash
source src/custom_dog_control/scripts/env.sh
colcon test --packages-select custom_dog_control custom_dog_description \
  --event-handlers console_direct+
colcon test-result --verbose
```

最近算法回归的 26 组 CTest 结果见上述验证记录，不等于本次文档更新又执行了这些测试。待解决项包括延迟执行总力矩预算、机身平顺性、足端及速度跟踪、真实传动／力矩标定，以及板载实时与真机运动验收。
