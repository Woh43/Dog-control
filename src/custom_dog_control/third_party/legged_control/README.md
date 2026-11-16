# legged_control 算法集成

[中文](#legged_control-算法集成) | [English](#legged_control-algorithm-integration)

本目录的 `legged_interface` 与同级 `../legged_wbc` 源自
[`qiayuanl/legged_control`](https://github.com/qiayuanl/legged_control)，固定提交为
`a7f381c0367e98e31c01336e678eef47e304d40d`。保留上游作者信息与
[LICENSE](LICENSE)。本文是本仓库的集成说明，并非上游原始 README。

`legged_interface` 编译为 `custom_dog_control_legged_interface`，WBC 编译为
`custom_dog_control_wbc`，由 `custom_dog_control` 的 NMPC 后端直接使用。
上游 ROS 1 控制器、消息、硬件抽象、launch 和可视化未引入；ROS 2 生命周期、状态估计、
状态机、安全检查及执行接口由主包提供。当前链路维护基础速度控制，不包含 RL 或精准落足模块。

## 来源与本地适配

来源版本记录在 [dependencies.lock.yaml](../../config/dependencies.lock.yaml)。
[legged_control_upstream.sha256](../../config/legged_control_upstream.sha256)
记录列入清单的源码及本地适配后的校验值；它不表示全部文件都与上游逐字一致。
校验入口是 [check_legged_control_provenance.py](../../test/check_legged_control_provenance.py)。
本地算法修改保存在此源码覆盖层，不应编辑生成的 `external/ocs2_ws` 或改变固定来源版本
来配置这些扩展。

### NMPC 与摆动参考

最优控制问题、切换参考管理、摆动轨迹、约束、代价与初始化器基于固定上游算法。
本地增加了配置式水平**足端坐标系中心**平面和仅摆动期的绝对高度恢复：

```text
swing_reference
{
  flatGroundFootCenterHeight  0.0  ; world z [m]，配置平面，非地形实测
  positionErrorGain          0.0  ; swing-only [1/s]，0 为仅速度跟踪
}
```

上面是扩展的缺省值；实际机器人设置以 [task.info](../../config/nmpc/task.info) 为准。
本机半径 0.026 m 的球足在 z=0 平地上使用 0.026 m 的足端中心平面。
例如恢复增益 20 1/s 在高度差 1 cm 时增加 0.2 m/s 的向上速度要求：
`vz - vz_ref + gain * (z - z_ref) = 0`。
该增益不修改支撑腿的 `ZeroVelocityConstraint`；上游共享的
`model_settings.positionErrorGain` 保持为零，不能用它代替仅摆动期恢复。

轨迹在该绝对平面之上叠加 `swingHeight`。保留上游短摆动缩放：
只有摆动时间不小于 `swingTimeScale`（当前为 0.15 s）时，0.10 m 才是完整规划抬脚量。
观测位姿重锚定不会移动配置平面或规划顶点。计划接触、规划轨迹均不是实测足底接触、
足底力或地形高度；实际净空仍受模型、执行延迟和力矩边界影响。

### WBC 与执行预算

当前运行 `WeightedWbc`，由 qpOASES 求解加权二次规划。保留浮动基动力学、
支撑足加速度、摆动足零力、摩擦锥、摆动足跟踪、机身运动与接触力跟踪的基本结构，
但本地版本已经包含算法扩展：

- **预测关节限位与松弛变量**：用位置、速度和预测时域约束关节加速度，添加非负限位松弛
  与代价。当前 12 关节、4 个三维接触下，内部求解有 42 个物理变量加 24 个松弛变量；
  返回上层的仍是物理解 `[广义加速度, 接触力, 前馈力矩]`。
- **机身反馈**：在动力学前馈上加入机身位置/速度与 ZYX 姿态/姿态速度反馈；
  有界名义高度修正只作用于机身加速度任务，不整体抬高期望足端。
- **可选执行预算**：支持每关节 FF 上限，以及已知电机 PD 名义力矩和区间；启用后可在
  动力学中计入名义 PD，并对声明区间内的 FF＋PD 施加力矩边界。区间为零时保留 FF
  形式。它依赖所声明的反馈与预测范围，不是对未知延迟下实际总力矩的无条件保证。
- **可选接触载荷上限**：接受计划接触法向力上限，用于载荷交接实验；不是足底力反馈。
- **可选质心耦合任务**：允许使用 QP 关节加速度变量代替 NMPC 关节速度差分形成质心
  动量率目标。此项改变任务定义，默认关闭，需与基线单独比较。
- **求解检查与观测**：报告求解状态、等式残差、不等式越限，以及机身垂直加速度目标和
  高度修正；失败解由上层拒绝。观测值不是实测机身加速度。

各可选分支是否启用，以 [controllers.yaml](../../config/controllers.yaml) 和后端接线为准；
任务空间 PD、权重与约束见 [task.info](../../config/nmpc/task.info)。
不要把关节电机 PD、任务空间 PD 或 WBC 力矩变量视为同一参数。
安全与模型一致性仍需检查实际执行的 FF＋PD、延迟、限幅和关节参考。
构建或离线 QP 求解通过不代表仿真运动或真机运动通过。

---

## legged_control algorithm integration

[中文](#legged_control-算法集成) | [English](#legged_control-algorithm-integration)

`legged_interface` and the sibling `../legged_wbc` are based on
[`qiayuanl/legged_control`](https://github.com/qiayuanl/legged_control) revision
`a7f381c0367e98e31c01336e678eef47e304d40d`. Original author notices and the
[license](LICENSE) are retained. This is the repository's integration document,
not an unmodified upstream README.

The algorithms compile directly into `custom_dog_control_legged_interface` and
`custom_dog_control_wbc`. ROS 1 controllers, messages, hardware, launch files and
visualization are not vendored. The main ROS 2 package provides estimation,
lifecycle, state transitions, safety and execution for baseline velocity control.

The [dependency lock](../../config/dependencies.lock.yaml) pins the source;
the [checksum manifest](../../config/legged_control_upstream.sha256) describes
the listed local overlay files, not universal byte identity with upstream.
Use [the provenance check](../../test/check_legged_control_provenance.py).
Configure local extensions here rather than editing generated OCS2 sources.

The NMPC reference manager supports a configured world-z **foot-frame center**
plane and a swing-only absolute-z recovery gain via `swing_reference` in
[task.info](../../config/nmpc/task.info). Defaults are plane 0 m and gain 0 1/s.
This robot uses a 0.026 m plane for its spherical feet on flat z=0 ground. The
upstream shared stance position gain stays zero. The planner adds `swingHeight`
above the configured plane, retaining short-swing scaling: full 0.10 m clearance
requires swing duration at least `swingTimeScale` (currently 0.15 s). Observation
re-anchoring does not move that plane or apex. Planned contact and clearance are
not measured foot contact, force or terrain height.

`WeightedWbc` uses qpOASES, retaining rigid-body dynamics, stance acceleration,
swing zero-force, friction cones and weighted swing/base/contact-force tasks.
Local adaptations add predictive joint-position bounds, nonnegative limit
slacks and their penalty; base position/velocity and ZYX attitude feedback;
bounded nominal-height correction; optional FF bounds and motor-PD torque
intervals; optional planned normal-force upper bounds; and an optional QP joint
acceleration coupling for the centroidal task (disabled by default).
The current model has 42 physical variables plus 24 joint-limit slacks. Only the
physical solution `[generalized acceleration, contact force, feedforward torque]`
is returned to the adapter.

The PD interval model can include nominal motor PD in dynamics and constrain
FF plus that interval. Its guarantee depends on the declared prediction bounds;
it does not establish unconditional limits for unknown delayed execution.
Solver status, residuals and base-z target telemetry support upper-level
rejection and diagnosis; targets are not measured accelerations. See
[controllers.yaml](../../config/controllers.yaml) for enabled adapter options and
[task.info](../../config/nmpc/task.info) for tasks and weights. Joint motor PD and
task-space PD have different roles. Builds and offline QP success do not establish
simulation or hardware motion acceptance.
