# FDILink IMU 驱动

**简体中文** | [English](#english)

`fdilink_ahrs` 从 FDILink 串口帧发布 ROS 2 IMU、AHRS 和可选导航数据。在本仓库中，控制器使用 `/imu` 作为姿态、角速度和加速度输入；GPS 和 INS 话题存在不代表当前机器人安装了对应硬件，也不代表这些话题已用于控制。

```text
FDILink 设备 → serial 库 → ahrs_driver_node → /imu → custom_dog_control
```

本轮硬件检查范围是 IMU 数据和零力矩、零增益通信，不自动起身或行走。本文更新没有新增真机运动验收。

## 构建和 IMU 独立检查

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to fdilink_ahrs
source install/setup.bash
```

以下入口只启动 IMU 驱动，会打开所选 IMU 串口，不启动电机控制器：

```bash
ros2 launch fdilink_ahrs ahrs_driver.launch.py
```

该 launch 使用下表默认值，没有声明可覆盖的 launch 参数；指定其他串口等配置时直接运行节点：

```bash
ros2 run fdilink_ahrs ahrs_driver_node --ros-args \
  -p serial_port_:=/dev/ttyUSB0 \
  -p serial_baud_:=921600 \
  -p imu_topic:=/imu \
  -p imu_frame_id_:=gyro_link \
  -p device_type_:=1 \
  -p orientation_timeout_s:=0.1
```

在另一个已加载环境的终端检查：

```bash
ros2 topic hz /imu
ros2 topic echo /imu --once
```

`custom_dog_control` 的 `real.launch.py` 可自行启动此驱动，使用 `/imu`、`device_type_=1`、`imu_frame_id_=base`，通过 `imu_port`、`imu_baud` 选择设备。已有驱动时不要再启动第二个节点抢占同一串口。完整控制入口与硬件边界见 [控制包说明](../custom_dog_control/README.md) 和 [硬件说明](../../docs/hardware.md)。

## 参数和输出

| 参数 | 驱动及独立 launch 默认值 | 含义 |
| --- | --- | --- |
| `serial_port_` | `/dev/ttyUSB0` | IMU 串口路径 |
| `serial_baud_` | `921600` | 串口波特率，须与设备配置匹配 |
| `if_debug_` | `false` | 协议调试输出 |
| `device_type_` | `1` | 坐标变换模式；有效使用值为 0 或 1 |
| `imu_topic` | `/imu` | IMU 消息话题 |
| `imu_frame_id_` | `gyro_link` | IMU 消息坐标系名称，不执行安装姿态标定 |
| `orientation_timeout_s` | `0.1` | 最新有效 AHRS 姿态的主机接收年龄上限，须为有限正数 |
| `mag_pose_2d_topic` | `/mag_pose_2d` | 磁航向话题 |
| `Euler_angles_topic` | `/euler_angles` | Roll/Pitch/Heading 话题 |
| `Magnetic_topic` | `/magnetic` | 磁力计话题 |
| `gps_topic` | `/gps/fix` | 地理坐标话题 |
| `twist_topic` | `/system_speed` | 设备输出的机体系速度 |
| `NED_odom_topic` | `/NED_odometry` | 设备输出的 NED 位置和速度 |

| 默认话题 | 消息类型 | 数据来源 |
| --- | --- | --- |
| `/imu` | `sensor_msgs/msg/Imu` | 最新有效 AHRS 姿态，加当前有效 IMU 帧的角速度和加速度 |
| `/mag_pose_2d` | `geometry_msgs/msg/Pose2D` | AHRS Heading |
| `/euler_angles` | `geometry_msgs/msg/Vector3` | AHRS Roll/Pitch/Heading |
| `/magnetic` | `geometry_msgs/msg/Vector3` | 缓存 IMU 磁力计，在 AHRS 帧分支发布 |
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | Geodetic Position 帧 |
| `/system_speed` | `geometry_msgs/msg/Twist` | INS/GPS BodyVelocity |
| `/NED_odometry` | `nav_msgs/msg/Odometry` | INS/GPS North/East/Down 位置和速度 |

输出频率取决于设备帧配置、串口传输和有效帧接收情况；驱动没有设置设备输出频率的 ROS 参数。当前用户设备按 **100 Hz** 使用，不应把它写成厂家所有工作模式的频率上限。提高控制器频率不会生成新的 IMU 测量。

## 时间戳、有效性和坐标约定

- 解析器检查帧类型、长度、CRC8/CRC16 和帧尾，并维护序号丢帧计数。
- 姿态仅由完整、校验有效、四元数有限且模长处于 `[0.5, 1.5]` 的 AHRS 帧更新，缓存前归一化。未收到有效姿态或姿态超过 `orientation_timeout_s` 时不发布 `/imu`；只有新角速度帧不能让旧姿态一直有效。
- `/imu.header.stamp` 保留最近有效 AHRS 的 **ROS 主机接收时间**，不会被后续 IMU 帧刷新；它不是设备采样时间。时钟回退使姿态缓存失效。设备时钟同步、串口缓冲延迟和 IMU/关节对时仍未标定。
- `device_type_=0` 沿用设备四元数、角速度和加速度。`device_type_=1` 使用代码中的 `q_r * q_ahrs * q_rr` 姿态变换，角速度和加速度按 `(x, -y, -z)` 发布。该规则不等于已验证实际安装轴向；实机轴向、零偏和安装外参需单独确认。
- `imu_frame_id_` 只更改标签，不自动变换数值。GPS/NED 辅助话题也不能自动当成四足世界系定位。

`imu_tf_node` / `imu_tf.launch.py` 仅用于独立显示姿态，默认平移为 `(1,1,1)`。当前回调实际广播固定 `world → imu`，没有使用已声明的 frame 名参数，不能用作机器人的正式安装外参。

## 代码与离线回归

| 文件 | 用途 |
| --- | --- |
| `src/ahrs_driver.cpp` / `include/ahrs_driver.h` | 串口读取、帧校验、姿态缓存和发布 |
| `include/fdilink_data_struct.h` | 协议数据结构 |
| `src/crc_table.cpp` / `include/crc_table.h` | CRC 实现 |
| `src/imu_tf.cpp` | 可选 TF 显示辅助节点 |
| `test/test_orientation_freshness.py` | 伪终端注入有效帧、故障帧及超时的回归 |

加载已构建环境后，可分别覆盖两种坐标模式；测试自行创建 PTY，不打开真实设备：

```bash
python3 src/fdilink_ahrs/test/test_orientation_freshness.py \
  --driver install/fdilink_ahrs/lib/fdilink_ahrs/ahrs_driver_node --device-type 0
python3 src/fdilink_ahrs/test/test_orientation_freshness.py \
  --driver install/fdilink_ahrs/lib/fdilink_ahrs/ahrs_driver_node --device-type 1
```

已存档结果位于 [test/results](test/results)，只证明其中记录的 PTY 用例；运行当前脚本才会覆盖后来增加的时间戳断言。它们不替代真机频率、轴向、延迟或运动验证。既有全包 lint 仍有失败项，见 [硬件记录](../../docs/hardware.md)，不把协议回归通过写成全部检查通过。

## 来源和维护

本目录是仓库保留的第三方 FDILink ROS 2 驱动及本地集成修改。当前 [package.xml](package.xml) 保留维护者 `noah <noah@todo.todo>`，许可证仍为 `TODO: License declaration`；目录未提供可核实的上游仓库 URL、固定提交及独立 LICENSE，本文不补造来源或重新许可。串口依赖的作者和许可证说明见 [serial README](../serial_ros2/README.md)。

[wheeltec_udev.sh](wheeltec_udev.sh) 含修改系统 udev 规则的示例，不由上述启动命令执行；复制规则前需按实际设备标识核对。

## English

[返回中文](#fdilink-imu-驱动)

`fdilink_ahrs` parses FDILink serial frames and publishes ROS 2 IMU/AHRS and optional navigation messages. The quadruped consumes `/imu`; the presence of GPS/INS publishers does not establish installed or validated GPS/INS hardware. This README adds no hardware-motion acceptance.

Build from the workspace root with `colcon build --symlink-install --packages-up-to fdilink_ahrs` after sourcing ROS 2 Humble, then source `install/setup.bash`. The standalone `ahrs_driver.launch.py` opens the IMU serial port and starts no motor controller. It declares no launch arguments; use the `ros2 run` command above to override ROS parameters.

Defaults are `/dev/ttyUSB0`, 921600 baud, `/imu`, frame `gyro_link`, `device_type_=1`, and `orientation_timeout_s=0.1`. The controller's `real.launch.py` instead labels the IMU frame `base`, with port/baud launch arguments; do not open the same port twice. Topic and parameter tables above list all exposed names. The device's configured rate determines output; there is no driver parameter that programs the sensor's rate. **100 Hz** describes the current user's device setting, not a manufacturer-wide maximum.

The parser validates frame sizes, CRC, end markers, and attitude quaternions. No `/imu` is published before a valid AHRS or after its host-receive age exceeds the configured timeout. Gyro-only frames cannot refresh attitude freshness. The message stamp remains the latest valid AHRS **host receive time**, not device acquisition time; clock rollback invalidates the cache. Clock synchronization and buffered transport delay remain uncalibrated.

Mode 0 publishes device axes. Mode 1 applies `q_r * q_ahrs * q_rr` to orientation and `(x, -y, -z)` to gyro/acceleration. Frame labels do not calibrate mounting orientation, and actual axes/bias still need hardware verification. The optional TF helper uses an arbitrary `(1,1,1)` translation and hard-coded `world → imu` frames despite its declared frame parameters; it is a visualization aid, not a calibrated robot transform.

The two PTY commands above exercise the actual executable without physical serial access. Archived results cover only their recorded assertions; rerun the current script for the newer timestamp checks. Existing whole-package lint failures are documented in [hardware.md](../../docs/hardware.md).

Upstream provenance remains incomplete: `package.xml` retains maintainer `noah <noah@todo.todo>` and a TODO license declaration; no verifiable upstream URL, pinned revision, or standalone license is present here. Existing notices are preserved without inventing a new license. Serial-library provenance is documented in its [README](../serial_ros2/README.md). The optional udev script changes system rules and is not run by the launch commands.
