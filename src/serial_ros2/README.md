# serial：ROS 2 串口基础库

**简体中文** | [English](#english)

目录名为 `serial_ros2`，实际 ROS 2 package 名、CMake 项目名及导出库名均为 **`serial`**，当前包版本为 `1.2.1`。它提供 C++ 串口打开、配置、读写、超时与端口枚举，不发布 ROS 话题，也没有本包可运行节点。

本仓库中主要供 `fdilink_ahrs` 读取 IMU 使用。十二电机的通信由 `custom_dog_control` 的厂家 SDK / 硬件接口负责，不经过这个 IMU 串口库；库的超时设置也不等于电机命令的执行延迟。

```text
fdilink_ahrs → serial::Serial → IMU 串口
custom_dog_control → 电机 SDK / 硬件接口 → 四腿电机串口
```

## 构建和使用

在工作区根目录构建，不使用目录名作为 package 名：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select serial
source install/setup.bash
```

需要同时构建 IMU 驱动及依赖时：

```bash
colcon build --symlink-install --packages-up-to fdilink_ahrs
```

C++ 消费方使用 `#include <serial/serial.h>`、`find_package(serial REQUIRED)`，并通过 `ament_target_dependencies(target serial)` 导入头文件路径和库。典型接口包括 `serial::Serial`、`setPort`、`setBaudrate`、`setTimeout`、`open`、`read`、`write`、`close` 及 `serial::list_ports()`。

串口路径、波特率和超时由消费方指定，本库没有 ROS 参数服务器或默认设备配置。本仓库 IMU 驱动当前设置 `/dev/ttyUSB0`、921600 baud、8N1、无流控和 20 ms 串口读写超时；这 **20 ms 是 IO 超时阈值**，不是设备固定延迟。驱动使用说明见 [fdilink_ahrs](../fdilink_ahrs/README.md)。

## 平台和验证边界

当前 [CMakeLists.txt](CMakeLists.txt) 固定编译 `src/impl/unix.cc` 与 Linux 端口枚举实现，链接 `rt` 和 `pthread`。虽然目录保留 Windows/macOS 源文件，当前构建配置面向 Linux，不能据此声明 Windows/macOS 已支持或验证。

当前 CMake 中测试入口被注释，没有注册本库自己的测试。IMU 的 PTY 回归间接覆盖所使用的串口读取路径，不等于全部 API、平台或真实设备测试通过。单独编译该库不会打开串口或驱动电机；本文更新没有新增硬件检查。

## 第三方来源、作者和许可证

[package.xml](package.xml) 记录 ROS 2 移植来源为 [jinmenglei/serial](https://github.com/jinmenglei/serial)，维护者为 `jinmenglei <841549111@qq.com>`，原作者为 **William Woodall** 和 **John Harrison**。源码保留这些作者的版权信息，部分 Linux 端口枚举代码还保留 **Craig Lilley** 的版权声明；基础串口库可追溯至 [wjwwood/serial](https://github.com/wjwwood/serial)。当前目录没有记录移入时的固定上游提交，后续升级需记录明确版本与来源，不自动跟随上游分支。

许可证元数据存在原有差异：`package.xml` 声明 `Apache License 2.0`，而 [serial.h](include/serial/serial.h)、平台头文件及端口枚举文件保留 MIT 声明；[v8stdint.h](include/serial/v8stdint.h) 另有 V8 项目自己的版权与许可。本文保留并说明这些声明，不将整个目录重新许可，也不覆盖第三方源文件中的许可证。

维护时优先保持此库稳定。FDILink 协议解析、IMU 坐标系和数据新鲜度应在 `fdilink_ahrs` 处理；电机时序、命令和力矩换算应在对应硬件接口处理。

## English

[返回中文](#serialros-2-串口基础库)

The source directory is `serial_ros2`, but the ROS 2 package, CMake project, and exported library are named **`serial`** (package version `1.2.1`). This C++ library provides serial configuration, reading/writing, timeouts, and port enumeration. It has no ROS node, topics, or ROS parameters.

The workspace uses it for `fdilink_ahrs` IMU IO. Motor communication uses the control package's vendor SDK/hardware interface instead. An IO timeout is not a measured motor execution delay.

From the workspace root, source ROS 2 Humble, run `colcon build --symlink-install --packages-select serial`, then source `install/setup.bash`. Use `--packages-up-to fdilink_ahrs` to include the IMU driver. C++ consumers include `<serial/serial.h>`, call `find_package(serial REQUIRED)`, and link via `ament_target_dependencies(target serial)`.

Ports and settings belong to the consuming application. The IMU driver currently uses `/dev/ttyUSB0`, 921600 baud, 8N1, no flow control, and a **20 ms IO timeout**, not an imposed fixed transport delay. See the [IMU README](../fdilink_ahrs/README.md).

The current CMake configuration selects the Unix implementation and Linux port enumeration, linking `rt` and `pthread`. Retained Windows/macOS sources do not establish a working or validated build on those platforms. Library tests are disabled in CMake. IMU PTY tests cover the exercised read path, not every API or physical device. Building this library alone opens no ports; this documentation update adds no hardware validation.

`package.xml` identifies the ROS 2 fork as [jinmenglei/serial](https://github.com/jinmenglei/serial), maintained by jinmenglei. Authors are **William Woodall** and **John Harrison**; port enumeration also retains **Craig Lilley** notices. The underlying library originates from [wjwwood/serial](https://github.com/wjwwood/serial). No fixed imported upstream revision is recorded in this directory.

License declarations differ: the package metadata says Apache License 2.0, while source headers and port-enumeration files retain MIT notices; `v8stdint.h` has separate V8 terms. These declarations are preserved without resolving them by relicensing the directory. Keep protocol/IMU changes in `fdilink_ahrs` and motor control changes in the corresponding hardware interface.
