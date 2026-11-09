# 外部依赖工作区

传统控制依赖集中存放在这里，分别构建后作为主工作区的 underlay 加载。

| 目录 | 内容 | 准备方式 |
| --- | --- | --- |
| `ocs2_ws/` | 固定版本 OCS2、robotic assets、本项目补丁及构建产物 | 主仓库 `fetch_ocs2.sh` + 独立 colcon 构建 |
| `model_ws/` | 历史模型 underlay；保留旧产物，当前入口不再加载 | 当前模型已迁入 `src/custom_dog_description` |
| `perception_ws/` | 用户仓库固定版本 MID360 驱动与 FAST-LIO ROS 2 | `tools/fetch_perception.sh` 只获取源码，单独构建 |

这些目录由根 `.gitignore` 排除。版本清单、补丁、模型生成逻辑保存在主仓库，
无需把依赖副本或二进制提交到 Git。首次准备步骤见 [安装说明](../docs/setup.md)。
完整模型、实体网格与固定 GitHub 来源记录位于主仓库 `src/custom_dog_description`，无需相邻 RL 目录。

`COLCON_IGNORE` 防止在 `Dog-control` 根目录运行 colcon 时把依赖工作区递归扫描进来。
构建 OCS2 时进入 `external/ocs2_ws`，使用 `colcon build --base-paths src ...`；
不要删除此标记；本仓库的 description 包随主工作区构建，OCS2 等外部算法依赖仍独立构建。

在主仓库根目录使用：

```bash
source src/custom_dog_control/scripts/env.sh
```

加载顺序为 ROS Humble → ocs2_ws → 主工作区（包含 description）。
OCS2 的自定义位置可通过 `CUSTOM_DOG_CONTROL_DEPS_WS` 覆盖；模型固定使用本工作区。
搬迁后必须重建包含绝对路径的 CMake/colcon 产物。

`src/custom_dog_control/third_party` 中的 LeggedInterface、WBC 和 qpOASES 保持原位置：
它们随主包直接编译，并由来源校验管理，与这里独立构建的依赖工作区用途不同。
