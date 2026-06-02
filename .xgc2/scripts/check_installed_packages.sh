#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

required_debs=(
  ros-noetic-xgc2-scout-description
  ros-noetic-xgc2-scout-gazebo-sim
  ros-noetic-xgc2-gz-classic-scout
)
required_ros_packages=(
  scout_description
  scout_gazebo_sim
)

for package in "${required_debs[@]}"; do
  dpkg -s "${package}" >/dev/null
done

for ros_pkg in "${required_ros_packages[@]}"; do
  test "$(rospack find "${ros_pkg}")" = "/opt/ros/${ROS_DISTRO}/share/${ros_pkg}"
done

roslaunch --files scout_description mini_description.launch >/tmp/xgc2-scout-description-files.txt
roslaunch --files scout_gazebo_sim mini_spawn.launch >/tmp/xgc2-scout-spawn-files.txt
roslaunch --files scout_gazebo_sim mini_gz_classic_simple.launch rviz:=false >/tmp/xgc2-scout-simple-files.txt
roslaunch --files scout_gazebo_sim mini_gz_classic_ros_control.launch rviz:=false enable_vrpn_server:=false >/tmp/xgc2-scout-ros-control-files.txt

check_paths=(
  "/opt/ros/${ROS_DISTRO}/lib/scout_gazebo_sim"
  "/opt/ros/${ROS_DISTRO}/lib/libscout_gazebo.a"
)

while IFS= read -r file; do
  if ! file -b "${file}" | grep -q '^ELF'; then
    continue
  fi
  if ! ldd "${file}" | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'; then
    echo "missing shared library dependency in ${file}" >&2
    ldd "${file}" >&2 || true
    exit 1
  fi
done < <(
  for path in "${check_paths[@]}"; do
    if [[ -d "${path}" ]]; then
      find "${path}" -type f \( -perm -0100 -o -name '*.so' \)
    elif [[ -f "${path}" ]]; then
      printf '%s\n' "${path}"
    fi
  done | sort -u
)

echo "Installed package check passed"
