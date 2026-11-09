# 首次安装

适用于 Ubuntu 22.04 + 已安装的 ROS 2 Humble。命令中的 `/path/to/Dog` 替换为实际目录。

## 安装与构建

### 1. 系统依赖

```bash
sudo apt update
sudo apt install \
  python3-colcon-common-extensions python3-vcstool \
  python3-evdev python3-numpy python3-yaml \
  libeigen3-dev libyaml-cpp-dev libtinyxml-dev liburdfdom-dev \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-urdf ros-humble-urdfdom ros-humble-xacro \
  ros-humble-pinocchio ros-humble-coal ros-humble-rviz2
```

### 2. OCS2 依赖工作区

```bash
cd /path/to/Dog/Dog-control
source /opt/ros/humble/setup.bash
git submodule update --init --recursive

export CUSTOM_DOG_CONTROL_DEPS_WS="$PWD/external/ocs2_ws"
src/custom_dog_control/scripts/fetch_ocs2.sh "$CUSTOM_DOG_CONTROL_DEPS_WS"

cd "$CUSTOM_DOG_CONTROL_DEPS_WS"
export LIBRARY_PATH="/opt/ros/humble/lib/$(gcc -print-multiarch):${LIBRARY_PATH:-}"
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --base-paths src --executor sequential \
  --packages-up-to ocs2_legged_robot ocs2_self_collision ocs2_sqp \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

依赖版本记录在
[`dependencies.lock.yaml`](../src/custom_dog_control/config/dependencies.lock.yaml)。

`external/COLCON_IGNORE` 只隔离主工作区的递归发现；上面的命令在依赖工作区内显式扫描 `src`。

完成 OCS2 编译后返回 `Dog-control`，运行根 README 的 `build_real.sh`（本机真机/Fortress）
或 `build_simulation.sh`（Gazebo Classic 开发机）。脚本先将仓库内的
`src/custom_dog_description` 构建到本工作区 `install`，再构建控制器。
完整 URDF 与 17 个 STL 都在源码包内，无需另行准备模型 underlay。

运行脚本和 `env.sh` 将 CppAD 生成库保存在工作空间 `.cache/cppad`，
重启后继续复用。首次启动或模型/task/reference、编译器 ABI 改变时仍需生成对应库，
在本机可能持续数分钟；等待 `CppAD models ready` 及控制器激活后再操作手柄。
可通过 `CUSTOM_DOG_CPPAD_CACHE_DIR` 显式指定其他可写目录。
缓存按输入内容隔离，不会因复用而忽略质量或惯量修改；生成文件不提交。
直接使用 `ros2 launch` 前先 source `src/custom_dog_control/scripts/env.sh`。

## 常见问题

- 缺失模型：检查本仓库 `src/custom_dog_description` 是否完整，再运行构建脚本；运行入口拒绝回退到旧 `external/model_ws` 或其他工作区模型。
- 缺失 OCS2：检查 `CUSTOM_DOG_CONTROL_DEPS_WS` 及其 `install/local_setup.bash`，依赖检查失败会停止构建。
- 搬迁工作区：colcon/CMake 产物可能记录旧绝对路径；重新生成受影响工作区的构建与安装产物，不要只改 launch 路径。
- 非 ASCII 路径：Gazebo 的 SDF 参数解析有兼容问题；launch 会在 `/tmp` 创建 ASCII 配置路径和模型链接，退出时清理。
- 测试启动超时：先读输出目录的 `launch.log`，区分模型生成、控制器激活和运动阶段；首次 CppAD 编译较慢，可增大 `--startup-timeout`，不要在编译中反复关闭并重启。
- 构建内存不足：设置 `CMAKE_BUILD_PARALLEL_LEVEL=1`，并确保 `MAKEFLAGS` 没有更高并行数。
