#!/usr/bin/env bash
set -euo pipefail

dog_launcher_dir="$(cd -P "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Introspection remains usable without starting nodes or opening any device.
for argument in "$@"; do
  if [[ "$argument" == --show-args || "$argument" == --help || "$argument" == -h ]]; then
    exec "${dog_launcher_dir}/run_real.sh" "$@"
  fi
done

echo 'RC2027 · NMPC + WBC'
echo 'START 标定；LT+A 起身/站立；LT+X 使能行走；LT+B 被动/复位；LT+Y 软件急停。'
echo '进入站立模式后按 LT+X，再推动摇杆；摇杆回中时仍保持站立，不原地踏步。'
echo '左摇杆前后：前后行走；右摇杆左右：侧移；左摇杆左右：转向。'
echo '先等待 NMPC 模型准备和控制器激活完成；首次生成模型库可能需要数分钟，后续启动复用持久缓存。'
echo '启动先执行约 0.9 秒的小力矩预对位，等完成后再按 START 记录零点。'
echo 'START 不自动起身；LT+A 才请求起身。'
echo '控制诊断自动保存在本次 ROS 日志目录的 control-diagnostics.jsonl 中。'
# Release only this workspace's diagnostic processes before the controller
# opens the same UARTs. The watcher verifies PID/start-time/executable identity.
dog_watch_tool="${dog_launcher_dir}/../../../tools/motor_comms_watch.py"
if [[ -f "$dog_watch_tool" ]]; then
  python3 "$dog_watch_tool" stop --all
fi
# The desktop operator requested startup without a repeated confirmation.
# This disables only the manual prompt requirement; it does not claim a
# physical stop has been verified. Startup alignment is selected separately.
exec "${dog_launcher_dir}/run_real.sh" \
  require_physical_estop_confirmation:=false use_rviz:=true \
  startup_alignment_enabled:=true "$@"
