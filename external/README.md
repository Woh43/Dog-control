# 外部依赖工作区

[中文](#外部依赖工作区) | [English](#external-dependency-workspaces)

这里存放可复现的依赖源码与独立构建产物。基础 NMPC/WBC 速度控制使用 OCS2
underlay；机器人模型由主工作区的 `src/custom_dog_description` 提供。
首次安装与完整构建命令见 [安装说明](../docs/setup.md)。

| 目录 | 内容 | 当前用途与准备入口 |
| --- | --- | --- |
| `ocs2_ws/` | 固定版本 OCS2、robotic assets、本项目补丁及独立构建产物 | [fetch_ocs2.sh](../src/custom_dog_control/scripts/fetch_ocs2.sh) 获取源码和应用补丁，再在此工作区构建 |
| `perception_ws/` | 固定来源的 MID360 驱动与 FAST-LIO ROS 2 | 可选感知依赖；[tools/fetch_perception.sh](../tools/fetch_perception.sh) 只导出源码，不构建、不启动 |
| `model_ws/` | 可能残留的历史模型 underlay | 已停用；当前入口会排除其模型路径，不再准备或加载 |

## 固定来源与构建边界

- OCS2 固定为 `cc73189408d4931d4d9b467a5e280a56ad7ab492`，robotic assets 固定为
  `9feab348a1d9d10a0573ca43c7fc4ac2e4565cf2`。获取清单见
  [ocs2_humble.repos](../src/custom_dog_control/repos/ocs2_humble.repos)，补丁随主仓库保存在
  [patches](../src/custom_dog_control/patches)。
- 感知脚本仅从 `hsc13576717115/ROBOCON_NBUT_R2` 的
  `300d07241372180c5267a129e80ad9a3410e2997` 导出 `livox_ros_driver2` 与
  `FAST_LIO_ROS2`，保留上游许可证并生成 `SOURCE.yaml`。它拒绝覆盖已有工作区，
  也不会引入已移除的精准落足包。感知并非基础速度控制的启动前提。
- 其他固定依赖版本与模型来源见
  [dependencies.lock.yaml](../src/custom_dog_control/config/dependencies.lock.yaml)。
  完整 URDF、网格和模型修改在主仓库内维护，无需相邻 RL 工作区。
- `third_party/legged_control`、`third_party/legged_wbc` 与 qpOASES 位于
  `src/custom_dog_control` 内，随主包直接编译；它们不是独立 underlay。
  LeggedInterface/WBC 的本地适配及校验入口见
  [集成说明](../src/custom_dog_control/third_party/legged_control/README.md)。

`external/COLCON_IGNORE` 防止从主仓库根目录递归发现依赖工作区。保留该标记；
构建 OCS2 时进入 `external/ocs2_ws`，显式使用 `colcon build --base-paths src ...`。
不要运行上游感知包中会清理其他工作区 `build/install` 的构建脚本。

## 环境与生成文件

主包构建完成后，在仓库根目录执行：

```bash
source src/custom_dog_control/scripts/env.sh
```

加载顺序是 ROS 2 Humble → OCS2 underlay → 主工作区。可通过
`CUSTOM_DOG_CONTROL_DEPS_WS` 覆盖 OCS2 工作区或安装前缀；感知工作区不会由该入口
自动加载。入口校验本地模型，拒绝使用其他工作区模型替代当前 URDF。

三个依赖目录均由根 `.gitignore` 排除；源码副本、构建产物和大型日志不提交。
CppAD 原生库缓存默认位于主工作区 `.cache/cppad`，也不提交。
工作区搬迁后应重新生成带绝对路径的 CMake/colcon 产物。复现依赖、构建成功与离线测试
均不代表真机运动已经验收。

---

## External dependency workspaces

[中文](#外部依赖工作区) | [English](#external-dependency-workspaces)

This directory holds reproducible dependency sources and isolated build outputs.
The baseline NMPC/WBC velocity controller uses the OCS2 underlay; the robot model
is built from `src/custom_dog_description` in the main workspace. See
[setup](../docs/setup.md) for installation and build commands.

| Directory | Role |
| --- | --- |
| `ocs2_ws/` | OCS2 and robotic assets, fetched and patched by [fetch_ocs2.sh](../src/custom_dog_control/scripts/fetch_ocs2.sh), then built separately |
| `perception_ws/` | Optional MID360 and FAST-LIO ROS 2 sources exported by [fetch_perception.sh](../tools/fetch_perception.sh); the script does not build or launch them |
| `model_ws/` | Retired model underlay; current entry points exclude its model paths |

OCS2 is pinned to `cc73189408d4931d4d9b467a5e280a56ad7ab492`, and robotic
assets to `9feab348a1d9d10a0573ca43c7fc4ac2e4565cf2`. The
[repository manifest](../src/custom_dog_control/repos/ocs2_humble.repos), local
[patches](../src/custom_dog_control/patches), and
[dependency lock](../src/custom_dog_control/config/dependencies.lock.yaml) are
tracked in the main repository. The perception export is limited to
`livox_ros_driver2` and `FAST_LIO_ROS2` from `hsc13576717115/ROBOCON_NBUT_R2`
revision `300d07241372180c5267a129e80ad9a3410e2997`; it preserves upstream
licenses, writes `SOURCE.yaml`, and refuses to overwrite an existing workspace.
It does not restore the removed precision-foot-placement packages.

Keep `COLCON_IGNORE`. Build OCS2 inside its workspace with an explicit
`--base-paths src`. The vendored LeggedInterface, WBC and qpOASES are compiled
with `custom_dog_control`; see their
[integration notes](../src/custom_dog_control/third_party/legged_control/README.md).
The checked-in URDF and meshes need no adjacent RL workspace.

After building the main workspace, source
`src/custom_dog_control/scripts/env.sh` from the repository root. It loads ROS 2
Humble, the OCS2 underlay, and the main workspace in that order, and checks the
local model. `CUSTOM_DOG_CONTROL_DEPS_WS` can override the OCS2 workspace or
installation prefix. Optional perception is not automatically sourced.

Dependency copies, build outputs, logs and `.cache/cppad` are generated files,
not commit inputs. Rebuild outputs containing absolute paths after relocating
this workspace. Dependency reproduction, builds and offline tests do not establish
hardware motion acceptance.
