# 电池模型修正与原参数检查（2026-11-02）

> 历史快照：本报告记录移除精准落足及修复真机控制链路之前的检查条件。测试数量、配置和未修复项描述均属于当时版本；当前入口及验证边界见 [硬件说明](hardware.md)。本报告不声明真机起身、站立或 Trot 已通过。


## 模型修正

实物总质量确认为 **15.15 kg**。裸 CAD 模型为 13.84916 kg，差额 **1.30084 kg** 为电池。
电池位于躯干内部、`base` 原点；其质量合并进现有 `base`，腿部质量、惯量、关节、轴和几何保留。

| 项目 | 修正前 | 修正后 |
| --- | ---: | ---: |
| 整机质量 | 13.84916 kg | 15.15 kg |
| 躯干质量 | 6.55 kg | 7.85084 kg |
| 躯干质心 x | -4.252589 mm | -3.547959 mm |
| 躯干质心 y | 1.281213 mm | 1.068923 mm |
| 躯干质心 z | -1.919146 mm | -1.601155 mm |

电池尺寸未知，其自身转动惯量采用**点质量近似**，没有虚构箱体尺寸。并非只修改 mass 标签：
新质心为质量加权平均，躯干惯量按原躯干和电池相对新质心的平行轴项合成。
后续获得电池长宽高后，应补入电池自身惯量，再重新做动态回归。

本机模型源：

```text
/home/orangepi/Dog_RL/custom_dog_stack/ros2/src/custom_dog_description/
```

`custom_dog.urdf`、`custom_dog_gazebo_point_foot.urdf`、`custom_dog_selective_collision.urdf`
均同步修正，配置记录在 `config/battery_payload.yaml`。主工作区的模型 underlay 和安装目录
也已同步。原模型备份在 `/home/orangepi/qr_ws_backups/before_battery_20261102_144349/`。

可复现源模型差异保存在
`src/custom_dog_control/patches/custom-dog-battery-model.patch`，用于尚未修正的
`custom_dog_stack` 源仓库。本机已应用对应修改。

## 原来调好的参数能否继续使用

当前 RC2027 的 NMPC/WBC 参数保留原数值，作为加电池后动态回归的基线，未统一乘质量比。
模型质量增加约 9.393%，重力从 135.8603 增至 **148.6215 N**；四足均载从 33.9651 增至
**37.1554 N/足**。NMPC/WBC 从 URDF 计算质量矩阵、重力和支撑力，无需另加一次质量倍率。

对原 `stand_up_joint_positions` 和**仿真配置的 settle Kp=55**做固定四足的静力求解，同时满足
浮动基座力/力矩平衡、关节 PD 力矩和足端位置约束：

| 项目 | 原 13.84916 kg 模型 | 新 15.15 kg 模型 |
| --- | ---: | ---: |
| 平衡机身高度 | 0.285184 m | 0.283285 m |
| 最大关节力矩 | 6.087 N·m | 6.776 N·m |
| 最小摩擦锥余量（μ=0.5） | 13.442 N | 13.927 N |

新模型静态高度降低约 **1.900 mm**。四足法向力约为
`[32.981, 33.361, 40.967, 41.313] N`，满足单边支撑、摩擦锥及当前关节力矩上限。
原名义姿态与 PD 目标均在关节限位内。结果支持保留原参数进行后续仿真，不代表起身或 Trot
已经通过动态验收。

现有 PD 目标可由旧模型 `mg/4、Kp=80.3` 的静态补偿重现，误差小于 `1e-5 rad`；
当前**仿真**起身/settle 增益为 45/55，这可能是历史动态调参组合，不能仅凭静态注释差异
判定原参数无效。用新模型、仿真 settle Kp=55 算出的另一组目标只记录为对照，**未覆盖原参数**。
数值和复现脚本为 `log/battery-20261102/static_parameter_check.json`、`.py`。

真机的 `real_controller.yaml` 会覆盖为髋 Kp=8、腿 Kp=12、Kd=0.8；上述 1.900 mm 和
6.776 N·m 的结果未使用这组真机参数，不能作为实机起身、站立能力的证据。

如果“之前参数”指旧 `qr_ws` 的 VMC：旧 YAML 原本已经使用 15.15 kg，不应再次放大其重力补偿。
VMC 的足端力级弹簧参数与新版 WBC 的加速度任务增益含义不同，新控制器也不读取旧 YAML，
不能整份复制。旧足端站姿可转换坐标后做 IK，四腿在新版几何中可达；这只证明几何可用。
电机能力对应的力矩上限继续使用原值，不随电池质量增加。

## 配套修复与验证

严格质量契约同步为 15.15 kg：NMPC 默认校验、公共 RobotModelConfig、precision_model 和现有模型测试。
历史 13.84916 kg 验证记录及模型指纹保留，不能当作新质量的验收记录。

新增 CppAD 内容指纹缓存隔离。原逻辑只检查固定路径下的 `.so` 是否存在，会让新 URDF 的
Pinocchio/WBC 与旧质量的动力学/足端速度 AD 混用。现在先对 URDF、任务、参考文件和本机
编译环境生成指纹，再创建 OCP；供应商代码及来源校验保留。

已完成的独立物理检查：20 个不同基座姿态和关节姿态下，质量矩阵增量与
`m_b J_baseᵀJ_base` 的最大误差约 `1.11e-15`，重力增量误差约 `2.84e-14`。
这确认新增惯性确实对应 base 原点的点质量，三份源 URDF 总质量均为 15.15 kg。
结果为 `log/battery-20261102/payload-dynamics-check.json`。

修正后模型 underlay 与真机控制目标重新构建通过。控制包 13 组 CTest 和规划包 2 组
CTest 均通过，colcon 汇总为 **75 tests, 0 errors, 0 failures, 0 skipped**，包含新增的
5 项缓存内容隔离测试。启动参数检查也通过，物理急停确认仍为默认 false。
结果为 `log/battery-20261102/test-results.txt`。

使用全新缓存的完整 NMPC/WBC 离线验收退出 0：20 份不同 sequence 的四足站立策略和
20 次 WBC 均有效，最大等式残差 `2.84e-14`，不等式违反量 0，20 次最大力矩峰值
`6.527 N·m`。13 个新 AArch64 导数库均在本次创建的模型指纹子目录生成，运行前后模型、
代码和配置指纹一致，确认未沿用漏算电池的旧模型。冷缓存生成和本次验收共约 565.5 秒。

短时离线计时使用 2 个优化线程、普通调度，不含串口 IO：NMPC 首次 53.14 ms、后续平均
29.16 ms；WBC 平均 0.837 ms、P95 0.906 ms、最大 0.929 ms。NMPC 仍超出默认 50 Hz 的
20 ms 预算，不能把求解有效解读成整机实时性已达标。

规划接触力的竖直分量总和为 147.94–151.965 N；初始高度目标会引入加速度，因此此值
不要求逐帧等于静力 `mg=148.6215 N`。这些是算法规划值，不是实测足底力。
完整结果、指纹和各样本位于 `log/battery-20261102/nmpc-smoke-result.json`。

本机 ARM64 环境没有 Gazebo Classic 安装候选，也没有现成物理仿真器。上述静力与优化测试
不含接触积分、摩擦滑移和起身瞬态；完整动态回归应在已有 Gazebo 开发环境加载修正模型后运行：

```bash
export CUSTOM_DOG_DESCRIPTION_DIR=/absolute/path/to/battery-corrected/custom_dog_description
CUSTOM_DOG_BUILD_ONLY=1 src/custom_dog_control/scripts/build_simulation.sh
src/custom_dog_control/scripts/test_simulation.sh --scenario all
```

本次不启动真实电机。
