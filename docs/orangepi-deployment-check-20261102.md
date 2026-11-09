# Orange Pi 5 Plus 部署检查（2026-11-02）

> 历史快照：本报告记录移除精准落足及修复真机控制链路之前的检查条件。测试数量、配置和未修复项描述均属于当时版本；当前入口及验证边界见 [硬件说明](hardware.md)。本报告不声明真机起身、站立或 Trot 已通过。


后续已按实物确认补入电池，将总质量修正为 15.15 kg；新结果见
[电池模型修正与参数检查](battery-model-validation-20261102.md)。下文的 13.84916 kg 记录保留为修正前证据。

## 检查范围

- 本机：RK3588 / ARM64，Ubuntu 22.04，ROS 2 Humble。
- 新代码：`RC2027`，`445436784e3a4249c92d3d7b98c6f7094ae53325`。
- 旧代码：`qr_ws` 的 `d4adbbafecd8e60fceb5f820d444090bf3a3e9d5`，并参考
  `/home/orangepi/qr_ws_backups/before_RC2027_20261102_134538/modified_files` 中的本机配置。
- 模型：[`Woh43/custom_dog_stack`](https://github.com/Woh43/custom_dog_stack)，
  `6c776de8deb76b8fd565cd80f16ac6a4cc8b635d` 的
  `ros2/src/custom_dog_description`。本机源位于
  `/home/orangepi/Dog_RL/custom_dog_stack/ros2/src/custom_dog_description`。

本次安装、源码比较、构建及测试均不启动真实电机控制器。

## 电机、IMU 与启动入口对比

| 项目 | 检查结果 |
| --- | --- |
| 四腿串口 | 新旧均为 FR/FL/RR/RL → `/dev/ttyS3/S4/S7/S8` |
| 电机 ID | 每腿 hip/thigh/calf 均为 0/1/2 |
| 电机协议 | GO-M8010-6、4 Mbps、23 字节发送 / 31 字节接收 |
| 减速比及换算 | 新旧代码均使用 6.33 / 12.66；位置和速度乘减速比、力矩除减速比、PD 增益除减速比平方 |
| Unitree SDK | 复用本机 `unitree_actuator_sdk-main/unitree_actuator_sdk-main` 的 AArch64 库 |
| IMU 设备 | `/dev/ttyUSB0`、921600、`device_type_=1`、`/imu` 均一致 |
| IMU 数值变换 | 保留 `q_r * q_ahrs * q_rr` 和角速度/加速度的 `x,-y,-z` 变换 |
| IMU frame | 从 `gyro_link` 改为 `base`；标签更改不会纠正实际安装方向 |
| 桌面 Run Dog | 仍指向已移除的 `qr_guide dog.launch.py`，不能用于 RC2027 |
| 手柄 | 事件读取脚本相同；新版 START 标定后自动起身，A 站立、X Trot、B 被动、Y 急停，不再要求 L2 |

底层设备协议具有复用基础，但新版控制器使用 URDF / REP-103 坐标和 ros2_control 接口，
不能把旧关节命令原样交给新版控制器后认定实机行为相同。

## 真实运动前尚需处理或确认的项目

1. **关节方向与标定。** 旧标定的 thigh/calf 定义含左右镜像；新标定为每腿统一
   `0°, +71.8°, -161.8°`，默认 12 个 `motor_directions` 全为 `+1`，
   与旧分支的软件关节方向不兼容。离线映射和验证结果见下一节；实际极性还需悬空单关节确认。
2. **控制和 IO 周期。** 旧实机是 500 Hz、串行腿通信；新默认 1000 Hz、四腿并行。
   若使用旧串行模式，不能同时沿用 1000 Hz；完整链路周期和超时率仍须测量。
3. **质量。** 旧本机 YAML 总质量为 15.15 kg，新规范 CAD 模型为 13.84916 kg，
   相差约 1.30 kg。需核对当前实际质量和模型惯量，不能仅放宽模型检查阈值。
4. **真实硬件总力矩限制。** `ControllerHardware.cpp` 中真机传递混合命令，
   仿真分支才合成并限制前馈加 PD 的总力矩。发送前的总力矩保护仍需单独审查。
5. **校准后的估计状态。** 新主循环在未标定时也运行状态估计；校准后未见与旧
   `RobotRunner::resetAfterCalibration` 等效的估计器重置，需处理零位跳变。
6. **IMU 方向。** 新控制器直接按机体坐标解释 IMU，尚需确认静止重力、roll/pitch/yaw 正方向。

`physical_estop_verified` 保持默认 `false`。代码构建和离线测试不能证明上述实机项目已通过。

## 旧硬件方向的离线推导

旧 FK 的输出先用 `diag(1,-1,1)` 转为 REP-103，再将旧、新标定姿态对齐。
对规范 URDF 每条腿枚举 8 种轴符号，使用 150 个随机姿态比较，得到唯一最优方向：

| 腿 | hip | thigh | calf | 平均足端误差 | 全 +1 的平均误差 |
| --- | ---: | ---: | ---: | ---: | ---: |
| FR | +1 | -1 | +1 | 5.43 mm | 111.53 mm |
| FL | +1 | +1 | -1 | 5.47 mm | 186.61 mm |
| RR | -1 | -1 | +1 | 5.50 mm | 134.85 mm |
| RL | -1 | +1 | -1 | 6.00 mm | 191.01 mm |

按 FR/FL/RR/RL、每腿 hip/thigh/calf 的候选 `motor_directions` 为：

```text
1 -1 1  1 1 -1  -1 -1 1  -1 1 -1
```

正确映射下 Jacobian 对应列方向余弦约 0.997–1.000，最大足端残差 6.17 mm；
每腿次优符号的平均误差为 60–64 mm，具有明确区分。
解析角度关系为：

```text
q_new_hip   = (前腿 +1 / 后腿 -1) * q_old_hip
q_new_thigh = -(右腿 +1 / 左腿 -1) * q_old_thigh - π/2
q_new_calf  =  (右腿 +1 / 左腿 -1) * q_old_calf - π/2
```

几何也存在差异：l0 均为 84.15 mm、l1 约为 213 mm；旧 l2 统一为 213 mm，
新模型 FR/FL/RR/RL 分别为 207.646/207.580/207.539/207.098 mm。
新模型还有约 0.004 rad 的安装轴倾斜和 0.18264 mm 的 hip mount 高度偏移，
因此正确符号下仍有 5–6 mm 残差。本次未修改默认电机方向，未将软件映射当作实物极性验收。

## 依赖安装与可复现入口

系统 ROS 源原有签名密钥已过期。本次从 ROS 官方 rosdistro 获取更新后的公钥，核对完整
指纹后，改为 HTTPS 和限定 `signed-by`，保留原配置于 `/var/backups/qr_ws-dependencies/`。
APT 本次安装 66 个新包，更新 6 个依赖包。

已安装 Pinocchio 4.0.0、Coal 3.0.3、ros2_control 2.54.3、ros2_controllers 2.54.1、
URDF/Xacro、vcstool、Git LFS 等真机构建依赖。精确版本记录在
`log/deployment/system-packages.txt`，APT 安装日志为 `log/deployment/install-system.log`。

OCS2 / robotic_assets、BLASFEO、HPIPM 使用仓库锁定版本。OCS2 的 16 个目标包均在本机
编译成功。补充 HPIPM 的 ARM64 `TARGET=GENERIC` 配置，避免默认 AVX 编译参数；
BLASFEO 保留 ARM Cortex-A57 优化，并补上上游 CMake 遗漏的 LU 辅助源文件，解决最终
程序链接时的三个未定义符号。修改保存在 `ocs2-humble-custom-dog.patch`，重新获取依赖可复现。

HPIPM 上游 partial-condensing 完整示例实际链接并退出 0，求解状态 0，最大残差
`5.78e-10`；结果位于 `external/ocs2_ws/.cache/hpipm-partial-condensing-runtime-test.log`。
上游另外一个示例的可选 `get_ric_K` 调用使用了未实现的 BLASFEO 操作，本项目实际 OCS2
路径使用其他 getter 和 Eigen 三角求解，不调用该方法。

模型的 17 个网格文件已逐个按 Git LFS 指针的 SHA-256 和长度校验，合计 40,059,728 字节。
未引入 RL 模型、训练框架或策略运行依赖。

新增入口：

```bash
# 只构建真机控制包和模型，不启动控制器
src/custom_dog_control/scripts/build_real.sh

# 查看启动参数，不启动节点
src/custom_dog_control/scripts/run_real.sh --show-args
```

## 本机验证记录

- 规范 URDF 契约通过：12 个关节、4 个足端、总质量 13.84916 kg。
- 安装后的 Pinocchio 4.0.0 可以读取规范 URDF，构建浮动基模型并计算四足 FK。
- 模型 underlay 构建通过。
- `serial` 和 `fdilink_ahrs` 在本机 ARM64 构建通过。
- IMU 虚拟串口测试 `device_type=0/1` 均通过：有效数据发布、CRC 拒绝、姿态超时、
  非有限四元数、短帧和恢复发布。结果为 `log/deployment/imu-pty-type0.json`、
  `log/deployment/imu-pty-type1.json`；只访问 PTY。
- `qr_interfaces`、`qr_planning` 构建通过；规划包 2 组 CTest 通过。
- 旧/新 FK/Jacobian 比较可用 `log/deployment/legacy_mapping_check.py` 复现，数值结果为
  `log/deployment/legacy_mapping_check.json`。
- `build_real.sh` 运行成功：真机目标涉及的 5 个包全部通过，控制包构建约 6 分 18 秒。
- 控制包 12 组 CTest 和规划包 2 组 CTest 全部通过；colcon 汇总为
  **69 tests, 0 errors, 0 failures, 0 skipped**。结果为 `log/deployment/test-results.txt`。
- 两个已安装插件均确认为 AArch64，`ldd` 无缺库，`dlopen` 成功；结果为
  `log/deployment/plugin-check.json`。
- `run_real.sh --show-args` 成功，确认急停参数默认 `false`；未启动节点。结果为
  `log/deployment/real-launch-args.txt`。
- 完整 NMPC + WBC 离线 smoke 退出 0：合成静止 IMU/关节数据先经过真实状态估计器，
  得到有效的四足站立策略（mode 15）；首次 NMPC 求解 55.3308 ms，WBC 0.925446 ms，
  等式残差 `8.53e-14`、不等式违反量 0、最大前馈力矩约 5.805 Nm。全程不创建硬件接口。
  记录为 `log/deployment/nmpc-smoke-runtime-continued.log`，复现程序和源代码也在
  `log/deployment/nmpc_smoke*`。
- CppAD 本机冷缓存生成耗时数分钟；本次使用隔离的日志目录，默认真机配置首次运行仍可能
  需要重新生成缓存。上述求解耗时不含代码生成时间。
- 首次 55.33 ms 超过默认 50 Hz 的 20 ms 求解预算；不能据一次静态 smoke 认定稳态实时性达标。
- 暖缓存下推进合成观测时间，获取 20 份不同 sequence 策略并运行 20 次 WBC，均有效。
  本次使用原任务数值、2 个优化线程、普通调度，结果如下；不含串口 IO，不能视为整机基准：

  | 计算 | 首次 | 后续平均 / 全部平均 | P95 | 最大 |
  | --- | ---: | ---: | ---: | ---: |
  | NMPC | 40.411 ms | 后续 25.065 ms | 27.865 ms | 40.411 ms |
  | WBC | — | 全部 1.672 ms | 4.576 ms | 4.655 ms |

  该配置下 NMPC 后续平均值已超过 50 Hz 的 20 ms 预算，WBC 也没有满足 1000 Hz 的证据。
  记录为 `log/deployment/nmpc-smoke-warm-timed-observations.log`。实际部署仍需按控制线程、
  通信及调度组合重新测量，不能只调高频率参数。
- 本次会话的实时优先级软/硬限制均为 95，memlock 无限制，但 OCS2 申请 FIFO 优先级 50
  返回 EPERM；尚未定位该运行环境限制，也未修改系统调度权限。记录为
  `log/deployment/scheduler-check.json`。

当前 ARM64 软件源没有 Gazebo Classic / `gazebo_ros2_control` 安装候选，本次验证覆盖真机
目标构建和离线计算，未运行 Gazebo 运动回归。
