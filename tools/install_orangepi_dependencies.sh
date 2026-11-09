#!/usr/bin/env bash
# Install build/runtime dependencies only; never starts the robot controller.
set -euo pipefail

if (( EUID != 0 )); then
  exec sudo /bin/bash "$0" "$@"
fi

if [[ "$(dpkg --print-architecture)" != arm64 ]]; then
  echo 'This installer is for the Orange Pi ARM64 host.' >&2
  exit 2
fi

backup_dir="/var/backups/qr_ws-dependencies/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$backup_dir"
key_file=$(mktemp)
trap 'rm -f "$key_file"' EXIT
curl --fail --silent --show-error --location --retry 2 \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$key_file"
fingerprint=$(gpg --batch --show-keys --with-colons "$key_file" 2>/dev/null |
  awk -F: '$1 == "fpr" { print $10; exit }')
if [[ "$fingerprint" != C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 ]]; then
  echo 'Unexpected ROS signing-key fingerprint; no system configuration changed.' >&2
  exit 3
fi

keyring=/usr/share/keyrings/qr-ws-ros-archive-keyring.gpg
install -m 0644 "$key_file" "$keyring"
# Retain this host's existing mirror, but use HTTPS and a scoped current key.
source_file=/etc/apt/sources.list.d/ros-fish.list
if [[ -f "$source_file" ]]; then
  cp -a "$source_file" "$backup_dir/"
fi
printf '%s\n' \
  "deb [arch=arm64 signed-by=$keyring] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/ jammy main" \
  > "$source_file"

apt-get -o Acquire::Retries=2 update
apt-get --simulate --no-remove install \
  python3-colcon-common-extensions python3-vcstool git-lfs \
  python3-evdev python3-numpy python3-yaml \
  libeigen3-dev libyaml-cpp-dev libtinyxml-dev liburdfdom-dev \
  liburdfdom-headers-dev libboost-all-dev \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-urdf ros-humble-urdfdom ros-humble-xacro \
  ros-humble-pinocchio ros-humble-coal ros-humble-rviz2 \
  ros-humble-tf2-eigen ros-humble-ament-cmake-gtest
DEBIAN_FRONTEND=noninteractive apt-get --yes --no-remove install \
  python3-colcon-common-extensions python3-vcstool git-lfs \
  python3-evdev python3-numpy python3-yaml \
  libeigen3-dev libyaml-cpp-dev libtinyxml-dev liburdfdom-dev \
  liburdfdom-headers-dev libboost-all-dev \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-urdf ros-humble-urdfdom ros-humble-xacro \
  ros-humble-pinocchio ros-humble-coal ros-humble-rviz2 \
  ros-humble-tf2-eigen ros-humble-ament-cmake-gtest
echo "System dependencies installed. Previous APT source: $backup_dir"
