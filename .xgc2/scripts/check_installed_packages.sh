#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

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

for package in "${required_debs[@]}"; do
  dpkg -s "${package}" >/dev/null
done

for ros_pkg in "${required_ros_packages[@]}"; do
  test "$(rospack find "${ros_pkg}")" = "/opt/ros/${ROS_DISTRO}/share/${ros_pkg}"
done

test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds/weston_robot_empty/weston_robot_empty.world"

roslaunch --files scout_description mini_description.launch >/tmp/xgc2-scout-description-files.txt
roslaunch --files gazebo_sim_scout spawn_simple.launch >/tmp/xgc2-scout-spawn-files.txt
roslaunch --files gazebo_sim_scout simple.launch rviz:=false world_name:=/tmp/xgc2-scout-empty.world >/tmp/xgc2-scout-simple-files.txt
roslaunch --files gazebo_sim_scout accurate.launch rviz:=false enable_vrpn_server:=false world_name:=/tmp/xgc2-scout-empty.world >/tmp/xgc2-scout-accurate-files.txt

test ! -d "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_scout/worlds"

mini_xacro="/opt/ros/${ROS_DISTRO}/share/scout_description/urdf/mini.xacro"
expanded_urdf="/tmp/xgc2-scout-mini-expanded.urdf"
xacro "${mini_xacro}" > "${expanded_urdf}"
grep -q '<mu1 value="1.0"/>' "${expanded_urdf}"
grep -q '<mu2 value="0.35"/>' "${expanded_urdf}"
grep -q '<slip1 value="0.0"/>' "${expanded_urdf}"
grep -q '<slip2 value="0.1"/>' "${expanded_urdf}"
grep -q '<kp value="1000000.0"/>' "${expanded_urdf}"
grep -q '<maxContacts value="16"/>' "${expanded_urdf}"
grep -q '<wheelSeparation>0.416503</wheelSeparation>' "${expanded_urdf}"
grep -q '<wheelDiameter>0.16</wheelDiameter>' "${expanded_urdf}"
grep -q '<torque>1000</torque>' "${expanded_urdf}"

default_params="/tmp/xgc2-scout-spawn-accurate-default-params.yaml"
roslaunch --dump-params gazebo_sim_scout spawn_accurate.launch > "${default_params}"
grep -q '/ugv1/scout_motor_fr_controller/pid/p: 1.0' "${default_params}"
grep -q '/ugv1/scout_motor_fl_controller/pid/p: 1.0' "${default_params}"
grep -q '/ugv1/scout_motor_rl_controller/pid/p: 1.0' "${default_params}"
grep -q '/ugv1/scout_motor_rr_controller/pid/p: 1.0' "${default_params}"

tuned_params="/tmp/xgc2-scout-spawn-accurate-tuned-params.yaml"
roslaunch --dump-params gazebo_sim_scout spawn_accurate.launch \
  wheel_contact_mu2:=0.31 \
  wheel_contact_slip2:=0.08 \
  wheel_pid_p:=2.5 \
  wheel_pid_i:=0.1 \
  wheel_pid_d:=0.2 \
  wheel_separation:=0.42 \
  wheel_radius:=0.081 \
  command_gain:=1.4 > "${tuned_params}"
grep -q '<mu2 value=\\"0.31\\"/>' "${tuned_params}"
grep -q '<slip2 value=\\"0.08\\"/>' "${tuned_params}"
grep -q '/ugv1/gazebo_ros_control/pid_gains/front_right_wheel/p: 2.5' "${tuned_params}"
grep -q '/ugv1/scout_motor_fr_controller/pid/p: 2.5' "${tuned_params}"
grep -q '/ugv1_scout_skid_steer_controller/command_gain: 1.4' "${tuned_params}"
grep -q '/ugv1_scout_skid_steer_controller/wheel_radius: 0.081' "${tuned_params}"
grep -q '/ugv1_scout_skid_steer_controller/wheel_separation: 0.42' "${tuned_params}"

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
