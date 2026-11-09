# 旧版 START 标定与 RC2027 的兼容性核查（2026-11-02）

本次对照的是旧版实际保留的代码与当前 RC2027 工作树。结论是：新旧 START 都用于在**已知机械姿态下采样编码器偏移**；旧版另有启动时主动施加力矩的预对位，当前版没有。当前标定公式适用于 URDF 坐标，不能直接换回旧版角度或全 `+1` 方向。本次没有访问真实串口，没有执行真机预对位、起身或行走，不能据此宣称机械零位、安装方向或真机运动已经通过。

## 证据来源

- 旧工作区备份：`/home/orangepi/qr_ws_backups/before_RC2027_20261102_134538`。
- 旧代码 Git ref：`legacy-origin/qr_ws`，提交 `d4adbbafecd8e60fceb5f820d444090bf3a3e9d5`。
- 旧备份的 `changes.patch` 和 `modified_files/` 只涉及配置、RobotConfig 与 State_Trotting；IOSDK 不在这四个原有修改文件中。旧驱动证据取自上述 Git 对象，避免误读已删除的路径或新分支编译产物。
- 当前代码取自 RC2027 工作树；当前未提交修改也包含在核查范围中。

可按以下方式复核旧文件行号：

```bash
git show legacy-origin/qr_ws:src/qr_guide/src/interface/IOSDK.cpp | nl -ba
git show legacy-origin/qr_ws:src/qr_guide/include/interface/IOSDK.h | nl -ba
git show legacy-origin/qr_ws:src/qr_guide/src/model/LegKinematics.cpp | nl -ba
```

## 旧版启动、标定与起身是三个阶段

1. 旧 `main.cpp:101` 构造 IOSDK。旧 `IOSDK.cpp:42-45` 在构造函数里依次打开串口、初始化电机、执行 `runStartupPoseAlignment()`、启动工作线程。此时还没有等待 START。
2. 旧 `IOSDK.cpp:154-184` 的预对位先给 hip/thigh 力矩、等待 300 ms，再加入 calf 力矩、等待 300 ms，最后发送零力矩并等待 300 ms。参数在旧 `IOSDK.h:81-88`：关节侧幅值 hip 0.95 Nm、thigh 0.63 Nm、calf 1.52 Nm；对应电机侧约 0.15、0.10、0.12 Nm。
3. 预对位后，旧 `IOSDK.cpp:48` 才提示“预对位完成，请确认趴下姿态后按 START 触发校准”。旧 `JoystickMapper.cpp:39-45` 把未标定时的 START（button 7）映射为 `L1_X`，旧 `IOSDK.h:71-72` 将其作为标定触发。旧 `IOSDK.cpp:300-310` 只采集当时缓存的编码器读数并设置标定标志，不主动寻找零位。
4. 旧 `State_Passive.cpp:23-28` 只有 `L2_A` 才进入 FixedStand；对应旧 `JoystickMapper.cpp:53-59` 的 L2+A 组合。因此旧版 START 本身也没有自动站起来。

这段预对位是开环施力和定时等待，没有验证每个关节确实到达限位、稳定不动或与目标角吻合。它可能是用户过去“按 START 就标定正常”的机械前提，不能把它当成已验证的自动标定程序。

当前 `src/custom_dog_control/src/hardware/IOSDK.cpp` 的构造和 `Activate()` 仅配置通信、建立工作线程，没有主动预对位。当前真机状态机 `ControllerStateMachine.cpp:193-211` 为 START→CALIBRATION→PASSIVE，再由 A 进入 STAND_UP；`ControllerInputs.cpp:93-98` 对应 START/A。当前取消了旧 L2 组合键，但继续将标定与起身分开。

**当前机械前提**：按 START 前，实物必须已处于配置所描述的完整参考姿态。仅知道“趴在地上”不足以确认三关节角，尤其不能自动推定 hip 为 0°、大腿/小腿折叠到同一位置。Gazebo 为避免碰撞使用的初始 hip 姿态也不能直接当作真机编码器标定参考。

## 顺序、减速比、方向和姿态

两版均为 FR、FL、RR、RL，每腿 hip、thigh、calf，电机 ID 在每个腿串口上都是 0、1、2。旧 `IOSDK.h:93-99` 给出了该顺序；旧 `IOSDK.cpp:321` 设置电机 ID。旧备份实际配置 `modified_files/src/qr_guide/config/custom_quadruped.yaml:43-51` 使用 FR=`/dev/ttyS3`、FL=`/dev/ttyS4`、RR=`/dev/ttyS7`、RL=`/dev/ttyS8`，hip/thigh 减速比 6.33、calf 总减速比 12.66。当前保持这组端口与减速比。

| 项目 | 旧版软件坐标 | 当前 URDF 坐标 |
| --- | --- | --- |
| 12 电机方向 | 四腿均 `[+1,+1,+1]` | FR `[+1,-1,+1]`，FL `[+1,+1,-1]`，RR `[-1,-1,+1]`，RL `[-1,+1,-1]` |
| FR/RR 标定角（度） | `[0,-161.8,-71.8]` | `[0,+71.8,-161.8]` |
| FL/RL 标定角（度） | `[0,+161.8,+71.8]` | `[0,+71.8,-161.8]` |

旧标定参数见旧 `IOSDK.h:76-80`，左右镜像见旧 `IOSDK.cpp:262-276`；当前默认值见 `include/custom_dog_control/hardware/ActuatorTypes.hpp:13-21`，真机 launch 同时传入方向和标定姿态。

旧 `LegKinematics.cpp:29-49` 使用平面角 `q1_abs=q1`、`q2_abs=pi/2-q2`；`LegKinematics.cpp:179-199` 对左右腿和前后 hip 做镜像。将它转换到当前 URDF，关节角的关系为：

```text
q_new = D_leg * q_old + [0, -pi/2, -pi/2]
```

其中 `D_leg` 为上表当前方向的对角矩阵。代入旧四腿参考角，均得到当前 `[0,+71.8,-161.8]` 度。数值差异来自坐标定义，不是把 thigh/calf 名称写反。因此不能直接把旧 `-161.8/-71.8` 数值填回新 URDF 标定参数。

离线几何核查保留在忽略提交的 `log/deployment/legacy_mapping_check.py` 和 `.json`：每腿 150 组扰动姿态，最佳符号与当前 D 一致。旧 calf 长 0.213 m、新模型约 0.207646 m，两模型足端位置仍约有 5.4 mm 几何差异。这份数学核查只证明软件坐标映射的合理性，没有验证真实电机安装方向。

## 编码器零位公式

记电机编码器角为 `q_m`（电机侧 rad），减速比为正数 `g`，方向为 `d∈{+1,-1}`，参考姿态为 `q_ref`（关节输出侧 URDF rad）。当前 `Calibration.hpp:18-40` 使用：

```text
未标定关节角：u = d * q_m / g
偏移：        offset = u_at_calibration - q_ref
标定后反馈：  q = u - offset
位置指令逆变换：q_m_cmd = (q_cmd + offset) * g * d
速度反馈：    dq = d * dq_m / g
力矩反馈：    tau = d * tau_m * g
```

位置正反变换在 `d=-1` 时也互为逆运算。当前偏移在关节输出侧保存，单位为 rad。

旧 `IOSDK.cpp:291-297` 用 `offset_old=q_m/g-q_ref_old`，但旧 `IOSDK.cpp:330-341` 的反馈是 `d*q_m/g-offset_old`；旧 `IOSDK.cpp:313-327` 的位置指令是 `(q_cmd+offset_old)*g*d`。旧版本全 `d=+1` 时可工作，但把该偏移公式与当前的负方向直接混用会产生错误，不能机械复制。

## 当前反馈与标定完成门槛更严格

旧 `IOSDK.cpp` 的 `sendRecv()` 在本轮收发前调用 `tryCalibrate()`，读的是上一轮缓存；旧 `tryCalibrate()` 不检查 12 个电机本轮均有有效反馈。即使个别电机缓存陈旧或未更新，也可能显示“已标定”。当前 `FinalizeCycleCalibration()` 要求本轮全部 12 个电机成功，才更新偏移，并在同一轮重新导出标定后的关节角。CRC、ID、故障与有限数值检查保留在通信链中。

因此，旧版能置标定标志不能反证当前报告的丢包。如果仍有电机无有效回包，应排查通信和帧解析；不应删掉完整反馈门槛来重现旧“标定成功”的界面。

旧 `RobotRunner.cpp:79-87` 在标定边沿调用估计器 `resetAfterCalibration()`。当前 `NmpcWbcController.cpp:43-55` 同样在标定门控边沿重置估计器与后台策略 epoch，避免旧编码器角初始化的锚点跨越零位跳变。

## 已有软件验证与剩余实物验证

已运行的真实生产 IOSDK + Unitree SDK PTY 回归位于 `log/fortress-validation-20261102/calibration_pty/`。测试只连接 4 个伪终端，逐包检查 mode 0、零力矩、零增益；12 个随机编码器读数、12 个方向和 6.33/12.66 减速比均参与实际生产代码路径。验证了参考姿态同轮导出、每个电机坏 CRC/无回包均阻止标定、Reset 后不复用旧零位、必须再次显式请求，以及重新标定替换偏移。该轮共检查 684 个零输出报文，最大关节角误差约 `1.19e-7 rad`。这是受控模拟回包的软件回归，不是真机标定验收。

串口 RAII/独占保护加入并统一重建后，使用新头文件重新编译测试，链接当前 `build/custom_dog_control/libcustom_dog_control_unitree_system.so`，在 `log/real-readiness-20261102-193721/calibration_pty/` 完整复跑上述 8 组检查：684 个全零 mode 0 报文、0 次瞬态恢复、最大误差 `1.19209e-7 rad`，全部通过。相同当前生产库的 8 组独占/异常释放回归也通过，结果和 SHA-256 记录位于同目录下的 `serial-ownership/`。

仍需在真实硬件上分别确认完整 12 电机有效反馈、人工机械参考姿态、各电机正方向、标定后实际关节角，并在独立授权与保护条件下验收起身/站立/行走。仿真可验证状态机、估计器和 NMPC/WBC 动作链，无法替代上述机械零位检查。

## 串口所有权保护的边界

当前 IOSDK 为各端口保留 RAII guard fd，先用 `flock(LOCK_EX|LOCK_NB)` 协调当前驱动实例，再检查 `/proc/*/fd` 中指向同一字符设备的既有 fd，连同未使用锁的旧控制器也会被拒绝。SDK 自行打开端口后设置 `TIOCEXCL`，阻止后续普通进程通过相同路径或别名打开；析构先关闭 SDK fd，再清除独占并关闭 guard。部分端口构造失败时，已打开的 SDK 对象与 guard 由 RAII 释放。针对本 SDK 配置异常后内部 fd 未关闭的情况，也回收本次串行构造新增的同设备 fd。

这要求 IOSDK 的构造/生命周期与其他同端口操作串行执行，延续该类既有调用约定。普通用户无法检查 procfs 隐藏的其他用户 fd；有 `CAP_SYS_ADMIN` 的进程也可绕过 Linux 的 TIOCEXCL。因此此保护不声称能约束不可见的既有特权进程或恶意竞争者。测试用的 metadata-only PTY shim 仅模拟 `TIOCGSERIAL/TIOCSSERIAL`，`TIOCEXCL/TIOCNXCL` 仍交给真实内核；测试 PTY 必须在 SDK 构造前关闭创建时保留的 slave fd。

独占与资源释放的独立回归保留在 `log/fortress-validation-20261102/serial-ownership/`，使用同一份生产 IOSDK 源码与真实 ARM SDK，仅连 4 个 PTY、不激活驱动。8 组检查均通过：正常构造/析构与内核拒绝额外打开、重复端口、既有无锁旧进程、既有独占进程、第二端口非 tty、第二端口权限拒绝、SDK 配置抛异常时内部 fd 回收、连续三次重新构造。每组前后 fd 数量一致，释放后 4 端口均可重开，4 个 PTY 主端均无发送字节。该测试不构成真实串口或电机通信结论。
