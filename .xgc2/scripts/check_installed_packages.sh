#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

log() {
  printf 'check: %s\n' "$*"
}

required_debs=(
  ros-noetic-xgc2-scout-description
  ros-noetic-xgc2-gazebo-sim-scout
  ros-noetic-xgc2-gazebo-sim-worlds
)
required_ros_packages=(
  scout_description
  gazebo_sim_scout
  gazebo_sim_worlds
)

log "checking Debian packages"
for package in "${required_debs[@]}"; do
  dpkg -s "${package}" >/dev/null
done

log "checking ROS package paths"
for ros_pkg in "${required_ros_packages[@]}"; do
  test "$(rospack find "${ros_pkg}")" = "/opt/ros/${ROS_DISTRO}/share/${ros_pkg}"
done

log "checking installed world asset"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds/empty/empty.world"

log "checking launch file resolution"
roslaunch --files scout_description mini_description.launch >/tmp/xgc2-scout-description-files.txt
roslaunch --files gazebo_sim_scout spawn_simple.launch >/tmp/xgc2-scout-spawn-files.txt
roslaunch --files gazebo_sim_scout simple.launch rviz:=false world_name:=/tmp/xgc2-scout-empty.world >/tmp/xgc2-scout-simple-files.txt
roslaunch --files gazebo_sim_scout accurate.launch rviz:=false enable_vrpn_server:=false world_name:=/tmp/xgc2-scout-empty.world >/tmp/xgc2-scout-accurate-files.txt

test ! -d "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_scout/worlds"

log "checking expanded Scout mini URDF defaults"
mini_xacro="/opt/ros/${ROS_DISTRO}/share/scout_description/urdf/mini.xacro"
empty_urdf="/opt/ros/${ROS_DISTRO}/share/scout_description/urdf/empty.urdf"
expanded_urdf="/tmp/xgc2-scout-mini-expanded.urdf"
xacro "${mini_xacro}" urdf_extras:="${empty_urdf}" > "${expanded_urdf}"
grep -q '<mu1 value="1.0"/>' "${expanded_urdf}"
grep -q '<mu2 value="0.35"/>' "${expanded_urdf}"
grep -q '<slip1 value="0.0"/>' "${expanded_urdf}"
grep -q '<slip2 value="0.1"/>' "${expanded_urdf}"
grep -q '<kp value="1000000.0"/>' "${expanded_urdf}"
grep -q '<maxContacts value="16"/>' "${expanded_urdf}"
grep -q '<wheelSeparation>0.490</wheelSeparation>' "${expanded_urdf}"
grep -q '<wheelDiameter>0.16</wheelDiameter>' "${expanded_urdf}"
grep -q '<torque>1000</torque>' "${expanded_urdf}"

log "checking tuned Scout mini URDF arguments"
tuned_params="/tmp/xgc2-scout-spawn-accurate-tuned-params.yaml"
xacro "${mini_xacro}" \
  wheel_contact_mu2:=0.31 \
  wheel_contact_slip2:=0.08 \
  skid_steer_torque:=900 \
  urdf_extras:="${empty_urdf}" > "${tuned_params}"
grep -q '<mu2 value="0.31"/>' "${tuned_params}"
grep -q '<slip2 value="0.08"/>' "${tuned_params}"
grep -q '<torque>900</torque>' "${tuned_params}"

log "checking installed ELF dependencies"
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
