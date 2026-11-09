# 2026-11-02：真机控制链路检查与 250 Hz 仿真进展

本文为 2026-11-02 的历史实验记录，以下“当前”均指当时版本和检查条件。2026-11-03 最新零输出复测已恢复 FR 反馈：顺序探测 6000/6000，四路并发 6677/6678，结束快照保留一次未收齐回包/超时、原因未定位；真实启动辅助、标定和承重运动仍未验收。最新配置和证据见 [兼容移植验收记录](compatibility-validation-20261103.md)及[硬件说明](hardware.md)，不以旧候选或历史阻塞代替当前状态。

**当前不能认定机器狗已具备真机起身、站立或 Trot 的使用条件。** 15.15 kg 模型在原增益、1000 Hz 仿真下已通过；后续 250 Hz 加一周期命令延迟的若干候选也完成了短时物理动作测试，但存在关节速度尖峰或后续策略超时，尚未完成最终候选的持续、重复验收。最近一次真实零输出检查中，右前腿三个电机均未取得有效反馈。生产 `real_controller.yaml` 的旧增益尚未替换，不能将任一候选结果用于推荐该配置直接上机。

本报告记录 `RC2027`、Orange Pi 5 Plus / RK3588、ROS 2 Humble、15.15 kg 模型的本轮证据。实际硬件检查只发送 mode0、零力矩、零增益命令，并接收 IMU；没有执行真实 START 标定、起身或行走。四 PTY 测试与 Fortress 仿真均不访问真实串口。

## 真实设备：反馈范围与当前阻断

最近一次生产 IOSDK 零输出检查，每个电机采样 10 次，顺序为 FR / FL / RR / RL，每腿 hip / thigh / calf：

| 设备 | 此次有效反馈 | 能确认的范围 |
| --- | --- | --- |
| 右前 FR，`/dev/ttyS3`，ID 0/1/2 | 各 0/10 | 当前 mode0 测试条件下未收到有效电机反馈 |
| 左前 FL，`/dev/ttyS4`，ID 0/1/2 | 各 10/10 | 本次短时零输出通信通过 |
| 右后 RR，`/dev/ttyS7`，ID 0/1/2 | 各 10/10 | 本次短时零输出通信通过 |
| 左后 RL，`/dev/ttyS8`，ID 0/1/2 | 各 10/10 | 本次短时零输出通信通过 |
| IMU，`/dev/ttyUSB0`，921600 baud | 约 5 s 内 IMU 501 帧、AHRS 502 帧，约 100 Hz，无解析错误 | 接收和协议解析正常 |

FR 的结果不能单独定位为模块、接线、电机或软件原因，也不能把收发 LED 亮起等同于已收到 ID、长度和 CRC 正确的反馈。当前缺少 FR 有效帧，因此不能完成 12 电机完整反馈条件的真机标定，更不能进行后续运动验收。本次受 FR 超时影响的完整通信循环约 6.47–6.86 ms，也不能据此声称已经满足 250 Hz 的 4 ms 周期。

IMU 沿用实际驱动 `device_type=1` 的坐标变换后，单个静止样本的机体加速度约为 `[0.177,-0.244,9.881] m/s²`，世界系重力补偿残差范数约 `0.076 m/s²`；这支持静止数据和变换基本自洽，实际安装轴的正方向仍需人工转动检查。

现场支撑与可用急停条件、断电后交叉接线定位的两个确认问题尚未收到回复。本报告不把先前“趴地且无人触碰”的回答视为本轮运动条件确认；当前不继续访问真实串口。

证据：[电机结果](../log/real-readiness-20261102-193721/motors.json)、[IMU 结果](../log/real-readiness-20261102-193721/imu.json)、[IMU 变换检查](../log/real-readiness-20261102-193721/imu-transformation-check.json)、[真实检查汇总与指纹](../log/real-readiness-20261102-193721/summary.json)。这些硬件记录产生于本轮后续软件修复之前，不能自动视为修复后生产库的重新硬件验收。

## 软件标定与真实启动链路

完整 `real.launch.py` 已在隔离 ROS domain 164 下，用四个 `/dev/pts/*` 模拟串口运行。测试启动实际 `controller_manager`、`UnitreeSystemInterface`、生产 IOSDK / Unitree SDK 和 NMPC/WBC 控制插件；真实 IMU 和手柄驱动关闭，由测试提供有效 IMU。测试子进程另设物理设备路径拒绝保护，未触发真实设备访问。

| 集成检查 | 结果 |
| --- | --- |
| 未标定直接按 A | 保持 PASSIVE；原始编码器角不初始化估计器 |
| START | 进入 CALIBRATION 后回到 PASSIVE，没有自动起身 |
| 12 个标定角 | 回到每腿 `[0°,71.8°,-161.8°]`，最大数值误差 `1.70×10⁻⁷ rad` |
| 标定后状态估计 | 基座高度 `0.092497741 m`，与独立 URDF FK 的 `0.092497740 m` 一致 |
| PASSIVE 注入非零反馈速度 | 最高约 `1.985 rad/s` 时仍发送 mode0 全零命令 |
| Y 故障、B 复位后再次 A | 旧标定失效、估计无效，A 不能绕过重新标定 |
| 更换 12 个原始编码器值后重新 START | 新零位替换旧零位，最大角度误差 `6.84×10⁻⁸ rad`，估计重新初始化 |

包含退出停机包，共检查 **24876 帧**：无坏 CRC、无非零命令载荷、无 FOC 报文。主库与测试文件的 SHA-256 在测试前后相同；测试进程已退出。PTY 期间有 17 次可恢复 IO 超时，不能将该测试用作真实 RS485 线缆可靠性或实时性验收。

另以新头文件链接当时最新生产共享库，重跑 **8 组 IOSDK 软件标定回归**，684 帧均为 mode0 全零命令；覆盖各电机坏 CRC / 缺包拒绝标定、同周期更新角度、方向与减速比、清零和重新标定。**8 项串口所有权回归**也通过，覆盖外部占用、重复端口、内核独占、构造异常和 FD 释放，未激活电机且无发送字节。PTY 的数值精度不代表真实机械趴姿或编码器零点精度。

证据：[完整 real.launch PTY 集成](../log/real-launch-pty-20261102/run1/result.json)、[完整启动日志](../log/real-launch-pty-20261102/run1/launch.log)、[新生产库标定回归](../log/real-readiness-20261102-193721/calibration_pty/result.json)、[标定回归指纹](../log/real-readiness-20261102-193721/calibration_pty/provenance.json)、[串口所有权回归](../log/real-readiness-20261102-193721/serial-ownership/result.json)。后续仅修改的采样、安全及控制器代码仍需以各自构建和仿真证据判断，不能扩大本次 PTY 报告的二进制覆盖范围。

## 250 Hz 物理仿真：旧增益不可直接沿用

前一轮通过的是仿真控制器 1000 Hz。本轮按真实入口的控制目标改为 **250 Hz**，NMPC 20 Hz、2 求解线程、策略容许年龄两周期。关闭真值辅助时，控制器只用 IMU 和关节反馈；独立物理里程计用于验收。以下结果均来自 Fortress 接触动力学，而非固定状态或仅离线求解。

| 对照配置 | 结果及限制 |
| --- | --- |
| 250 Hz，旧起身 Kp 45→55 / Kd 1.5→2.5，WBC Kp 髋 50 / 腿 20、Kd 2，无真值辅助 | 站立后 Trot 失败：`0.05 m/s` 前进指令下，10 s 独立真值位移约 **−9.315 m**，反馈速度触及 15 rad/s、力矩饱和；当时控制器没有进入 FAULT |
| 同频率旧增益，改用真值辅助 | 前进阶段约 0.433 m，随后停车时 roll/pitch 超出验收阈值，失败；不能将前一项完全归因于状态估计 |
| 250 Hz，无额外命令延迟；保持 Kp，将起身 Kd 改为 0.6→0.8、WBC Kd 改为 0.8，无真值辅助 | 起身、站立、10 s 前进、停车、6 s 恢复、A 显式停步均通过；前进 **0.5568 m / 10 s**，恢复 **0.3251 m / 6 s**，验收采样无无效 WBC |

最后一项只证明该次无额外延迟的平地低速物理仿真通过。真实硬件 `read` 发送上一轮打包命令的管线还有周期延迟，不能据此直接选定真机增益。

证据：[旧增益无真值失败](../log/real-readiness-20261102-193721/fortress-250hz-result.json)、[真值辅助失败](../log/real-readiness-20261102-193721/fortress-250hz-gt-result.json)、[降低阻尼的无延迟通过结果](../log/real-readiness-20261102-193721/fortress-250hz-low-result.json)、[该次实际参数](../log/real-readiness-20261102-193721/fortress-250hz-low-operative.yaml)。

## 采样延迟诊断及首个延迟候选

检查本机 gz_ros2_control 0.7.21 和 Fortress 6.18 的对应源码、二进制热路径：插件每个 1 ms 物理步重写当前 effort，250 Hz 仅节流 read/update；effort 是赋值，不按频率倍增或累加。没有发现“250 Hz 只施力四分之一步”这类实现错误。[gz_ros2_control 源码](https://github.com/ros-controls/gz_ros2_control/blob/0.7.21/gz_ros2_control/src/gz_ros2_control_plugin.cpp#L486)、[effort 赋值](https://github.com/ros-controls/gz_ros2_control/blob/0.7.21/gz_ros2_control/src/gz_system.cpp#L796)、[Fortress 物理力矩应用](https://github.com/gazebosim/gz-sim/blob/ignition-gazebo6_6.18.0/src/systems/physics/Physics.cc#L1814)。

原模型名义姿态的最小有效关节惯量约 `0.002617 kg·m²`。常质量、无接触的 PD 子系统诊断中，旧 Kd=2 的离散谱半径在 1 ms 为约 0.990，在 4 ms 为约 2.079，支持采样保持引起失稳的解释。加入一周期延迟后，可用阻尼范围进一步变窄。此计算省略完整 WBC、接触切换和饱和，**属于候选筛选依据，不能作为物理稳定性保证**。

新仿真延迟辅助在最终前馈＋PD 合成及限幅后，将力矩延迟一个控制器周期；安全输出立即旁路并清空队列，活动模式间保持管线，真实硬件输出不经过此仿真辅助。另新增关节和基座速度的持续超限保护，并在仿真验收中直接用独立物理速度提前发现超速或持续反向运动。

当时的首个延迟候选为：起身及落稳各关节 Kp=25、Kd=0.3；WBC Kp 为髋 40 / 大腿 15 / 小腿 15、Kd=0.4；一周期模拟延迟；关节速度保护 8 rad/s、水平速度保护 0.6 m/s，持续时间 0.1 s。模型扫描中该候选在若干 4–5 ms 常周期样本下线性谱半径小于 1，不等于已覆盖真实抖动或非线性运动。

该候选**尚未通过**：首次运行因 IMU timeout 进入 FAULT，未完成动作验收；domain 176 重试完成初始站立，但 Trot 初段触发 `joint velocity limit: index=8 velocity=15 limit=8`，最终进入 FAULT。后一个结果证明新速度保护触发，不能写成行走通过，也不能为了通过而取消该保护。以下追加的后续实验采用其他参数，不能回写成此候选通过。

证据：[采样 PD 诊断](../log/real-readiness-20261102-193721/gz-effort-audit/sampled-pd.json)、[一周期延迟诊断](../log/real-readiness-20261102-193721/gz-effort-audit/sampled-pd-one-cycle-delay.json)、[4–6 ms 候选扫描](../log/real-readiness-20261102-193721/pd-sampling-stability/delay-jitter-candidate-scan.json)、[候选实际参数](../log/real-readiness-20261102-193721/fortress-250hz-delay1-operative.yaml)、[首次 IMU 超时结果](../log/real-readiness-20261102-193721/fortress-250hz-delay1-result.json)、[重试速度保护结果](../log/real-readiness-20261102-193721/fortress-250hz-delay1-retry-result.json)。

## 后续候选：短时动作通过与仍未关闭的问题

本节补充晚于上述失败记录的实际日志，最后一份 planar 结果的 UTC 时间为 `2026-11-02T15:56:24`（北京时间 23:56）。这些实验均为 15.15 kg、250 Hz 控制、NMPC 20 Hz / 2 线程、关闭真值辅助和一周期模拟命令延迟。起身/落稳 Kp=25、Kd=0.3；WBC 之后叠加的关节跟踪 Kp/Kd 均置零，WBC 内部的运动任务和动力学约束仍在运行。它们已改变原控制参数，不能归为“只增重、原参数通过”。

| 候选 | 附加设置 | 已完成的动作结果 | 限制 |
| --- | --- | --- | --- |
| no-pd | 原 0.25 s Trot 周期 | 完整前进/停车/恢复/显式停步测试通过；10 s 前进 0.5375 m，6 s 恢复 0.3185 m | 另录的约 250 Hz 关节反馈在换相附近仍达到 15 rad/s |
| slow | Trot 周期 0.5 s | 完整测试通过：0.6795 m / 10 s、恢复 0.4024 m / 6 s；后退、左右侧移与正向转动也通过 | 同次 planar 后退 −0.2214 m、左移 +0.2875 m、右移 −0.2700 m、转动 +0.9910 rad；另次完整控制轨迹的 Trot 速度峰仍为 15 rad/s |
| overlap | 周期 0.5 s；双支撑 0.05 s | 完整测试通过：0.4849 m / 10 s、恢复 0.2982 m / 6 s | Trot 速度峰 15 rad/s，67 个记录帧超过 8 rad/s |
| ramp | overlap 设置；支撑力渐变 0.1 s | 完整测试通过：0.5635 m / 10 s、恢复 0.3358 m / 6 s | Trot 速度峰 9.7375 rad/s，5 个记录帧超过 8 rad/s |
| gentle | 周期 0.6 s；双支撑 0.075 s；支撑力渐变 0.15 s | 完整测试通过：0.5768 m / 10 s、恢复 0.3459 m / 6 s | 4310 个 Trot 记录帧的最大关节速度 7.2221 rad/s、无帧超过 8 rad/s；该实例后续在站立中因策略超时进入 FAULT，planar 预检失败 |

动作测试的 `passed: true` 表示完成相应短时动作及该脚本的验收条件。控制器关节速度保护还要求超限持续 0.1 s；因此存在瞬时尖峰与动作测试通过并不矛盾，不能把“未触发 FAULT”写成“所有采样均低于 8 rad/s”。上述峰值来自关节反馈或完整控制轨迹，未把约 10 Hz 诊断采样当作峰值上界。slow 完整控制轨迹只记录了 28 s，未覆盖完整恢复段。

gentle 的 40 s 完整控制轨迹共 9991 帧，其中 Trot 4310 帧；Trot 的每帧最大关节速度 P99 为 5.7617 rad/s，实际施加力矩命令峰值为 18.9539 N·m，Trot 的 WBC 无无效帧。后续启动日志明确记录 `NMPC policy stale or invalid` 和 `MPC_STANCE -> FAULT`；随后 planar 尚未发出动作即因预检发现 FAULT 退出。现有日志不能定位这次策略超时的根因，也不支持把该候选写成持续稳定或完整多方向验收通过。

结果：[no-pd](../log/real-readiness-20261102-193721/fortress-250hz-no-pd-result.json)、[slow 完整动作](../log/real-readiness-20261102-193721/fortress-250hz-slow-result.json)、[slow 多方向](../log/real-readiness-20261102-193721/fortress-250hz-slow-planar-result.json)、[overlap](../log/real-readiness-20261102-193721/fortress-overlap-result.json)、[ramp](../log/real-readiness-20261102-193721/fortress-ramp-result.json)、[gentle 完整动作](../log/real-readiness-20261102-193721/fortress-gentle-result.json)、[gentle 后续 planar 失败](../log/real-readiness-20261102-193721/fortress-gentle-planar-result.json)、[gentle 启动与故障日志](../log/real-readiness-20261102-193721/fortress-gentle-launch.log)。

参数：[no-pd 实际参数](../log/real-readiness-20261102-193721/fortress-250hz-no-pd-operative.yaml)、[slow 实际参数](../log/real-readiness-20261102-193721/fortress-250hz-slow-operative.yaml)、[overlap 实际参数](../log/real-readiness-20261102-193721/fortress-overlap-operative.yaml)、[ramp 实际参数](../log/real-readiness-20261102-193721/fortress-ramp-operative.yaml)、[gentle 输入配置](../log/real-readiness-20261102-193721/fortress-250hz-gentle.yaml)。输入配置中的真值辅助和 NMPC 频率还会被 launch 参数覆盖，本文实际条件以运行参数及结果记录为准，不能只读输入 YAML 推定。

速度与命令证据：[no-pd 关节记录汇总](../log/real-readiness-20261102-193721/no-pd-telemetry.summary.json)、[slow 完整控制轨迹](../log/real-readiness-20261102-193721/internal-slow-trace.csv)、[overlap 完整控制轨迹](../log/real-readiness-20261102-193721/internal-overlap-trace.csv)、[ramp 完整控制轨迹](../log/real-readiness-20261102-193721/internal-ramp-trace.csv)、[gentle 完整控制轨迹](../log/real-readiness-20261102-193721/internal-gentle-trace.csv)。本次只重新读取既有 CSV 计算，没有新增实测。轨迹统计取 `mode=4` 的 Trot 行，关节速度取十二列 `dq_*` 的绝对值最大值，施加力矩命令取十二列 `applied_tau_*` 的绝对值最大值；这些是仿真关节状态与命令，不是真实电机或实测足底力。

## 增重与移植修改的来源

增重本身是躯干增加 1.30084 kg（整机约 +9.393%），需要同步质量、质心、惯量、模型契约和动力学导数缓存。NMPC/WBC 从模型重新计算重力等项，无需将所有增益按质量比缩放。[15.15 kg 的 1000 Hz 原增益物理仿真](simulation-validation-20261102.md)已经通过，说明本轮没有证据把后续全部修改归因于电池增重。

另外发生的变化是桌面 Gazebo Classic 向 RK3588 / Fortress 的环境移植、真实编码器与 URDF 坐标及标定链路适配、控制周期从 1 ms 变为 4 ms，以及本轮在主机合成并限制前馈＋PD 后只发送力矩的实现选择。主机 PD 按通信/控制周期采样并经历命令管线延迟，电机内部 PD 的运行位置和时序不同；当前 delay 仿真对应前一种实现，不能将两者视为相同被控对象，也不能仅从仿真结果认定真实电机内部 PD 已验收。后续降低阻尼、关闭外层关节 PD 和调整步态/支撑力渐变，均属于这些新时序条件下的候选实验，尚未据此更新生产配置。

## 构建状态与仍待完成的事项

采样修复阶段统一构建及安装成功，当时测试汇总为 **75 tests、0 失败 / 错误**；其中 CTest 的 13 个测试入口全部通过，汇总计数同时包含测试入口和底层 gtest 用例，不应解释为 75 次独立物理动作。回归覆盖模拟延迟旁路、超速持续时间与复位、状态机门禁及串口资源管理等。该计数不覆盖后续新增测试，构建成功不代表运动条件已满足。

证据：[构建记录](../log/real-readiness-20261102-193721/build-sampling-fix.log)、[测试运行记录](../log/real-readiness-20261102-193721/tests-sampling-fix.log)、[该次 CTest 明细](../build/custom_dog_control/Testing/20261102-1206/Test.xml)、[真实 launch 参数合并检查](../log/real-readiness-20261102-193721/launch-parameter-merge-check.json)。

仍待完成的是：定位并恢复 FR 的有效反馈；确认现场支撑、急停及后续动作条件；在已有 250 Hz 加一周期延迟短时通过结果上，关闭关节速度尖峰和策略超时问题，完成最终候选的持续、重复及多方向验收，再决定生产配置；之后才有条件安排实际方向、机械趴姿、零位、完整 IO 周期和分阶段真机动作检查。当前生产 [real_controller.yaml](../src/custom_dog_control/config/real_controller.yaml) 仍保留旧增益，未采用 no-pd/slow/overlap/ramp/gentle 的完整参数；本报告没有批准或建议使用它起身、站立或行走。

后续更新应追加最终候选的实际配置、完整结果路径和生产配置是否变更；不能把短时物理仿真通过改述为持续稳定或已经完成真机验收。
