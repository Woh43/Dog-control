# Custom Dog 机器人描述

**简体中文** | [English](README_EN.md)

本包包含四足机器人的完整 ROS 2 描述资源：三份 URDF、17 个实体 STL、关节顺序与电池配置，以及独立显示入口。全部模型和网格都在本包内，不依赖相邻仓库、外部模型工作区或 RL 运行环境。

本仓库维护 `RC2027` 的模型式 NMPC/WBC 控制链路。RL 训练与部署位于独立的 `himloco_custom_dog`，复用模型资源不引入 RL 依赖。控制和仿真入口见 [控制包说明](../custom_dog_control/README.md)。

## 模型与控制约定

模型包含 17 个 link、16 个 joint，其中 12 个为驱动关节。控制／硬件接口使用 URDF 关节侧的 rad、rad/s、N·m；接口数组顺序为 **FR → FL → RR → RL**，每腿 **hip → thigh → calf**。Pinocchio／NMPC 内部顺序通过控制适配器映射，不能直接套用接口数组顺序。

| 关节 | URDF 力矩上限 | URDF 速度上限 | 真机接口总减速比 |
| --- | ---: | ---: | ---: |
| 髋 hip | 23.7 N·m | 30 rad/s | 6.33 |
| 大腿 thigh | 23.7 N·m | 30 rad/s | 6.33 |
| 小腿 calf | 45 N·m | 15 rad/s | 12.66 |

小腿包含电机内部 6.33 和额外 2:1 传动。减速比由控制包的 [ros2_control 描述](../custom_dog_control/urdf/custom_dog.ros2_control.xacro) 和硬件接口用于换算，不是这三份规范 URDF 中的 transmission 定义。URDF 上限是模型约束，不代表已标定的持续输出能力；电流、SDK effort 含义及传动效率仍需核对。

当前控制配置的名义高度为 **0.34 m**，指平地上 `base` 原点的目标高度，并非腹部离地间隙，也不保证运动中始终达到该值。真机只有 IMU 和关节反馈，没有足底接触传感器；模型中的足端和仿真接触不能写成实测足底力。

## 资源与来源

总质量为 **15.15 kg**：上游 CAD 模型 13.84916 kg，加位于 `base` 原点的 1.30084 kg 电池。电池质量已经合并进 `base`，其质量为 7.85084 kg；质心与惯量按平行轴定理同步。电池尺寸未知，自身惯量仍为点质量近似，见 [battery_payload.yaml](config/battery_payload.yaml)。不要再次加一次电池质量。

| 文件 | 用途 |
| --- | --- |
| `urdf/custom_dog.urdf` | 规范质量、惯量、视觉、运动学和限位；供真机与 NMPC/WBC 使用 |
| `urdf/custom_dog_gazebo_point_foot.urdf` | Fortress 接触模型；仅删除四腿 thigh/calf 的八个碰撞体 |
| `urdf/custom_dog_selective_collision.urdf` | 保留上游的另一组碰撞代理；不引入对应运行框架 |
| `meshes/*.STL` | 17 个完整二进制网格，不是 Git LFS 指针 |
| `config/joint_contract.yaml` | FR、FL、RR、RL，每腿 hip、thigh、calf 的关节顺序及足端名称 |
| `config/model_manifest.json` | 来源提交、LFS 内容标识、文件大小/SHA-256 和本地电池修正 |

三份 URDF 的质量、惯量、视觉和关节树一致，区别限于碰撞代理。保留的规范 URDF SHA-256 为 `bf01f817cfb7fe3d0112333b7c6e767c441a80c03bd78f73a82c5c3b36540741`，与 2026-11-03 已测 15.15 kg 模型的字节一致。这里的资源迁入与静态校验不新增物理仿真或真机运动验收。

来源为 [Woh43/custom_dog_stack](https://github.com/Woh43/custom_dog_stack)，固定提交 `6c776de8deb76b8fd565cd80f16ac6a4cc8b635d` 的 `ros2/src/custom_dog_description`。该提交的原始质量仍为 13.84916 kg；15.15 kg 来自本工作区先前的电池修正。本包复制当前已测 `external/model_ws` 中的资源，保留三份修正后 URDF 的字节。上游 STL 通过 Git LFS 保存，实体文件已逐个核对其 SHA-256 内容标识与大小；没有复制训练代码、策略或部署程序。

上游许可证声明不一致：根目录 [LICENSE.upstream](LICENSE.upstream) 为 MIT，版权为 `Copyright (c) 2026 h43`；描述包原 `package.xml` 声明为 `Proprietary`，维护者为 `hsc`。本次保留这两项原声明，不据此重新许可模型或网格。新增的构建/测试配置和依赖声明属于本仓库集成；详细文件来源及原始摘要见 manifest。

## 构建、显示和校验

在工作区根目录构建并加载环境：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select custom_dog_description
source install/setup.bash
```

仅显示模型（不会打开电机串口，也不运行物理仿真或控制器）：

```bash
ros2 launch custom_dog_description display.launch.py
```

该入口启动 `robot_state_publisher`、`joint_state_publisher_gui` 和 RViz。可用 `use_rviz:=false` 关闭 RViz，但关节滑块 GUI 仍会启动；它不是无显示服务端入口。

独立离线资源校验不需要 ROS、仿真器或网格下载：

```bash
python3 src/custom_dog_description/test/check_model_resources.py src/custom_dog_description
```

启用 `BUILD_TESTING` 构建本包时，同一检查注册为 CTest `check_model_resources`。检查覆盖实体网格与来源摘要、所有资源引用、15.15 kg 质量、惯量物理条件、电池质心/惯量合成、关节契约及三份碰撞变体的一致性。以后修改资源时须同时更新来源说明和 manifest，并根据物理变化重新验证控制模型。
