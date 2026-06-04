#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

required_debs=(
  ros-noetic-xgc2-scout-description
  ros-noetic-xgc2-gazebo-sim-scout
)
required_ros_packages=(
  scout_description
  gazebo_sim_scout
)

for package in "${required_debs[@]}"; do
  dpkg -s "${package}" >/dev/null
done

for ros_pkg in "${required_ros_packages[@]}"; do
  test "$(rospack find "${ros_pkg}")" = "/opt/ros/${ROS_DISTRO}/share/${ros_pkg}"
done

roslaunch --files scout_description mini_description.launch >/tmp/xgc2-scout-description-files.txt
roslaunch --files gazebo_sim_scout spawn_simple.launch >/tmp/xgc2-scout-spawn-files.txt
roslaunch --files gazebo_sim_scout simple.launch rviz:=false >/tmp/xgc2-scout-simple-files.txt
roslaunch --files gazebo_sim_scout accurate.launch rviz:=false enable_vrpn_server:=false >/tmp/xgc2-scout-accurate-files.txt

world="/opt/ros/${ROS_DISTRO}/share/gazebo_sim_scout/worlds/weston_robot_empty.world"
grep -q '<max_step_size>0.004</max_step_size>' "${world}"
grep -q '<real_time_update_rate>250</real_time_update_rate>' "${world}"

mini_gazebo="/opt/ros/${ROS_DISTRO}/share/scout_description/urdf/scout_mini.gazebo"
grep -q '<mu1 value="1.0"/>' "${mini_gazebo}"
grep -q '<mu2 value="0.9"/>' "${mini_gazebo}"
grep -q '<kp value="10000000.0"/>' "${mini_gazebo}"
grep -q '<maxContacts value="64"/>' "${mini_gazebo}"
! grep -q '<slip1' "${mini_gazebo}"
! grep -q '<slip2' "${mini_gazebo}"

check_paths=(
  "/opt/ros/${ROS_DISTRO}/lib/gazebo_sim_scout"
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
