# 软件架构与维护入口

控制器插件仍是一个 `ros2_control::ControllerInterface` 生命周期实例，内部状态由它持有；
按职责拆分实现文件，避免把高频链路分散到额外节点。仿真和真机使用同一套 NMPC/WBC。

当前范围为基础速度控制。精准落足模块及其 `qr_*` 包已移除。机器人只有 IMU 和关节反馈，没有足底接触传感器。

```text
/imu + 关节反馈 ──> KinematicStateEstimator ──> observation
Gazebo ground truth ────────────────────────> observation（仿真配置）
/joy、/cmd_vel ──> 输入仲裁/限幅 ──> 状态机与目标参考
                                           │
                         NmpcBackend / OCS2 SQP（独立线程）
                                           │ policy
                              ros2_control update（配置周期）
                                           │
                              WeightedWbc + safety checks
                                           │ HybridJointCommand
                          ┌────────────────┴────────────────┐
                FortressMotorSystem                UnitreeSystemInterface
                  物理步执行 PD                      RS485 / 电机内部 PD
```

## 文件职责

以下路径相对于 `src/custom_dog_control/`：

| 文件或目录 | 负责什么 | 何时修改 |
| --- | --- | --- |
| `src/controller/NmpcWbcController.cpp` | `update()` 周期编排、插件导出 | 修改控制执行顺序 |
| `src/controller/ControllerLifecycle.cpp` | 参数声明/校验、后端与订阅创建、启停 | 新增控制参数或资源 |
| `src/controller/ControllerInputs.cpp` | IMU/真值/手柄/速度回调、输入仲裁 | 接入上层指令 |
| `src/controller/ControllerHardware.cpp` | 接口名称解析、关节读取、混合命令输出 | 修改 ros2_control 接口 |
| `src/controller/ControllerStateMachine.cpp` | 模式切换、起身、Trot 门控、WBC 交接与短暂故障处理 | 修改动作流程 |
| `src/controller/ControllerDiagnostics.cpp` | 10 Hz 诊断、里程计和 TF | 增加观测指标 |
| `include/.../controller/NmpcWbcController.hpp` | 生命周期接口与控制器私有状态 | 与实现文件配套修改 |
| `include/.../control/JointCommandUtils.hpp` | 纯关节 PD、等效力矩与连续力矩交接计算 | 修改混合控制数学逻辑 |
| `src/nmpc/` | 模型验证、状态估计、NMPC 与 WBC 适配 | 修改控制算法或估计器 |
| `src/nmpc/NmpcBackend.cpp` 内的 `GaitCommandModule` | 运行时接触计划 | 修改步态调度 |
| `src/safety/` | 输入有效性、限位、超时与故障锁存 | 修改安全条件 |
| `src/hardware/` | Unitree 电机协议、串口、方向与标定 | 接入/维护实机 |
| `third_party/` | 固定版本的上游算法 | 升级时同步来源哈希与验证 |

`include/...` 表示 `include/custom_dog_control`。修改参数时同时维护声明、读取校验和 YAML；
不要仅在 YAML 中添加一个运行时不会读取的键。

## 数据与执行边界

- ROS 订阅回调把最新输入写入 `RealtimeBuffer`；`update()` 读取快照。
- 控制周期读取硬件、更新估计和目标参考、交给后端 observation，再读取最新 policy，
  运行状态机/WBC、安全检查并输出命令。NMPC 求解线程通过后端管理，停用时停止。
- 真机与默认 Fortress 五分量模型均保留目标位置/速度、前馈力矩和 Kp/Kd。
  电机（或仿真物理步）计算 PD；主机在采样时检查各项绝对值贡献之和，
  超限共同缩放 FF/Kp/Kd。这不能保证两次采样之间的连续力矩上限。
  旧 effort 仿真路径仅用于历史对照，可由 `actuator_model:=effort` 显式选择。
- 起身交接同时改变目标位置和增益，因此补偿前馈项，确保当前测量状态下的总力矩连续。
  `test_joint_command_utils` 验证交接端点及中间阶段的力矩性质。
- 真机与默认 Fortress 共用主控 250 Hz、NMPC 20 Hz、2 个求解线程。
  仿真以仿真时钟推进求解，真机以单调墙钟推进；实际硬件完整时序需独立测量。

## 状态机与故障行为

```text
PASSIVE -- START --> CALIBRATION -- 完成 --> PASSIVE
PASSIVE -- LT+A，已标定 --> STAND_UP --> MPC_STANCE
                                      ↕ LT+X/速度/新策略
                                   MPC_TROT
故障/LT+Y --> FAULT -- LT+B --> PASSIVE（重新标定）
```

START 只标定，不自动起身。标定完成后重置估计器，并使旧 NMPC 观测/策略失效，由后台线程重置求解器。
未标定编码器不初始化脚端锚点。LT+A 是独立于 START 的起身请求，LT+X 不能从被动状态直接起身。

`STAND_UP` 由关节 PD 插值起身，等待姿态稳定和可用 policy 后交接 WBC。
速度超过进入阈值并满足驻留时间后请求 Trot；归零并满足退出条件后恢复四足站立。
切换等待新 policy，避免接触计划和控制输出错位。

短暂 WBC 无效时使用站立 PD 保持；连续失败达到 `max_consecutive_wbc_failures`
（默认 5 次）进入 FAULT。安全监控中的过期 policy 故障判断针对 `MPC_STANCE` 和 `MPC_TROT`；
硬件通信、IMU、关节限位等有各自检查。具体阈值见配置及 `SafetyMonitor`。
FAULT 锁存后需 PASSIVE 确认复位，真机需重新标定。

## 配置定位

| 配置 | 内容 |
| --- | --- |
| `config/controllers.yaml` | 仿真/真机共享频率、限幅、起身、交接、输入超时 |
| `config/real_controller.yaml` | 硬件接口、时钟、被动输出与限位容差差异 |
| `config/nmpc/task.info` | NMPC 代价、约束、WBC 设置 |
| `include/.../control/ControlTypes.hpp` | 固定关节/足端顺序、Trot 周期与切换阈值 |
| `config/nmpc/gait.info` | 参考步态配置；当前运行时周期由 C++ 固定常量决定 |
| `config/dependencies.lock.yaml` | 第三方版本 |

## 模型与标定约定

唯一运动学和动力学模型为公共完整 URDF：

```text
/path/to/Dog/Dog-control/src/custom_dog_description/urdf/custom_dog.urdf
```

碰撞模型按仿真器显式分离，但质量、惯量、视觉、关节树和限位都继承上述公共模型：

| 使用方 | URDF | 碰撞约定 |
| --- | --- | --- |
| 真机、Pinocchio、模型契约 | `custom_dog.urdf` | 完整 primitive 碰撞体 |
| Gazebo 传统控制 | `custom_dog_gazebo_point_foot.urdf` | 去除 thigh/calf 地面碰撞，只保留机身、hip 和足端 |
| HimLoco/Isaac | `custom_dog_selective_collision.urdf` | 标定后的 thigh/calf 代理碰撞体 |

Gazebo launch 必须显式加载点足版本，不能再通过修改公共 URDF 绕开起身阶段的腿段
接触。点足版本仅匹配当前 NMPC/WBC 的四足端接触假设，不代表真实腿部不存在碰撞。

| 项目 | 约定 |
| --- | --- |
| 总质量 | 含电池 `15.15 kg`（裸 CAD `13.84916 kg` + 电池 `1.30084 kg`） |
| 坐标系 | REP-103：x 向前、y 向左、z 向上 |
| 腿顺序 | `FR、FL、RR、RL` |
| 单腿关节顺序 | `hip、thigh、calf` |
| 接触帧 | `FR_foot、FL_foot、RR_foot、RL_foot` |
| 名义趴姿 | 每腿 `hip=0 deg、thigh=71.8 deg、calf=-161.8 deg` |

电池质量为 `1.30084 kg`，中心按实物说明置于 `base` 原点，合并到躯干惯性：
躯干质量从 `6.55` 改为 `7.85084 kg`，质心和惯量按刚体合成与平行轴公式重算。
电池尺寸尚未提供，因此其自身惯量采用点质量近似，详见描述包 `config/battery_payload.yaml`。
三份碰撞变体共享同一组连杆惯性。模型来源及回归结果见
[电池模型检查](battery-model-validation-20261102.md)。

NMPC 的 CppAD 缓存使用 URDF、任务和参考配置的内容指纹，并区分本机编译器及架构。
质量或惯量发生变化后会进入新缓存子目录，避免 `recompileLibrariesCppAd=false` 时沿用旧动力学。

运行时 FK、Jacobian、惯量和关节限位均来自 URDF Pinocchio 模型。原 `qr_guide`
的 `RobotModel/LegKinematics` 已退出运行路径，避免旧机械尺寸和旧运动学零点污染模型。

真机按下 START 前，操作者需要将四条腿置于名义趴姿。START 表示“当前机械姿态对应
上述 URDF 角度”：硬件层仅在 12 个电机反馈全部有效时计算零点偏移。该角度是当前
人工折叠姿态的名义值，后续应通过机械基准或标定工装修正；尤其是 hip 的 `0 deg`
不能由贴地几何单独确定。

Gazebo 中看到的趴姿是碰撞和重力稳定后的生成姿态，髋关节会有左右对称的外展；它由
仿真专用 `passive_joint_positions` 描述，不等同于真机 START 建立的逻辑标定角。
二者最终都转换到同一套 URDF 关节坐标后再进入 NMPC-WBC。

## ROS 2 接口

### 输入

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/imu` | `sensor_msgs/msg/Imu` | 姿态与角速度测量 |
| `/joy` | `sensor_msgs/msg/Joy` | 手柄状态和速度输入 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 上层速度指令 |

非零手柄速度有效时优先于 `/cmd_vel`，两者共用速度和加速度限幅器；仅按模式键且摇杆
保持零位时，不会遮蔽 `/cmd_vel`。

### 输出

- `/joint_states`
- `/odom` 和 TF
- `/nmpc_wbc_controller/control_mode`
- `/nmpc_wbc_controller/contact_plan`
- `/nmpc_wbc_controller/diagnostics`

诊断数据包含 NMPC 状态、WBC 残差、控制周期、求解时间和硬件错误计数。

## 工作区边界

`custom_dog_control` 是主包；`fdilink_ahrs`、`serial_ros2` 是驱动依赖。
`src/unitree_guide` 为历史参考，使用 `COLCON_IGNORE` 排除。
`src/custom_dog_description` 保存完整的 15.15 kg 模型和网格，随本工作区构建。
`external/ocs2_ws` 为独立算法 underlay；旧 `external/model_ws` 仅保留历史产物，当前入口不再加载。
上游算法固定提交及本地修改由 provenance 测试校验。
