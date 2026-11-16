#!/usr/bin/env bash
set -euo pipefail

dog_launcher_dir="$(cd -P "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Desktop defaults precede explicit arguments so an operator can override them.
dog_launch_defaults=(
  require_physical_estop_confirmation:=false
  control_frequency_hz:=250 use_rviz:=false
  startup_alignment_enabled:=true
)
# Introspection remains usable without starting nodes or opening any device.
for argument in "$@"; do
  if [[ "$argument" == --show-args || "$argument" == --help || "$argument" == -h ]]; then
    exec "${dog_launcher_dir}/run_real.sh" "${dog_launch_defaults[@]}" "$@"
  fi
done

echo 'RC2027 · NMPC + WBC'
echo '桌面默认主控/WBC 250 Hz、NMPC 50 Hz，关闭 RViz；显式启动参数可覆盖。'
echo 'START 标定；LT+A 起身/站立；LT+X 使能行走；LT+B 被动/复位；LT+Y 软件急停。'
echo '进入站立模式后按 LT+X，再推动摇杆；摇杆回中时仍保持站立，不原地踏步。'
echo '左摇杆前后：前后行走；右摇杆左右：侧移；左摇杆左右：转向。'
echo '先等待 NMPC 模型准备和控制器激活完成；首次生成模型库可能需要数分钟，后续启动复用持久缓存。'
echo '启动先执行约 0.9 秒的小力矩预对位，等完成后再按 START 记录零点。'
echo 'START 不自动起身；LT+A 才请求起身。'
echo '控制诊断自动保存在本次 ROS 日志目录的 control-diagnostics.jsonl 中。'
echo '逐周期命令、策略序号和下一轮 read 的 SDK 发送包保存在本次 ROS 日志目录 cycle-trace；cycle_trace:=false 可关闭。'
echo '自动录制 rosbag：保留本次启动段与最新数据，约 512 MiB；下一次启动替换上一份。'
# Release only this workspace's diagnostic processes before the controller
# opens the same UARTs. The watcher verifies PID/start-time/executable identity.
dog_watch_tool="${dog_launcher_dir}/../../../tools/motor_comms_watch.py"
if [[ -f "$dog_watch_tool" ]]; then
  python3 "$dog_watch_tool" stop --all
fi
# The desktop operator requested startup without a repeated confirmation.
# This disables only the manual prompt requirement; it does not claim a
# physical stop has been verified. Startup alignment is selected separately.
exec "${dog_launcher_dir}/run_real.sh" "${dog_launch_defaults[@]}" "$@"
