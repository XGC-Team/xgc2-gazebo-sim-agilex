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

log "checking simulation-owned model and launch resources"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_scout/urdf/mini.xacro"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_scout/launch/mini_description.launch"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_scout/rviz/navigation.rviz"
test ! -d "/opt/ros/${ROS_DISTRO}/share/scout_description/launch"
test ! -d "/opt/ros/${ROS_DISTRO}/share/scout_description/rviz"
test ! -f "/opt/ros/${ROS_DISTRO}/share/scout_description/urdf/mini.xacro"
roslaunch --files gazebo_sim_scout mini_description.launch >/tmp/xgc2-scout-description-files.txt
roslaunch --files gazebo_sim_scout simple.launch rviz:=false world_name:=/tmp/xgc2-scout-empty.world >/tmp/xgc2-scout-simple-files.txt
roslaunch --files gazebo_sim_scout accurate.launch rviz:=false enable_vrpn_server:=false world_name:=/tmp/xgc2-scout-empty.world >/tmp/xgc2-scout-accurate-files.txt

test ! -d "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_scout/worlds"

log "checking expanded Scout mini URDF defaults"
mini_xacro="/opt/ros/${ROS_DISTRO}/share/gazebo_sim_scout/urdf/mini.xacro"
empty_urdf="/opt/ros/${ROS_DISTRO}/share/gazebo_sim_scout/urdf/empty.urdf"
expanded_urdf="/tmp/xgc2-scout-mini-expanded.urdf"
xacro "${mini_xacro}" urdf_extras:="${empty_urdf}" > "${expanded_urdf}"
grep -q '<mu1 value="0.10"/>' "${expanded_urdf}"
grep -q '<mu2 value="1.0"/>' "${expanded_urdf}"
grep -q '<fdir1 value="0 0 1"/>' "${expanded_urdf}"
grep -q '<slip1>5.0</slip1>' "${expanded_urdf}"
grep -q '<slip2>0.0</slip2>' "${expanded_urdf}"
grep -q '<kp value="1000000.0"/>' "${expanded_urdf}"
grep -q '<maxContacts value="16"/>' "${expanded_urdf}"
! grep -q '<odometryTopic>' "${expanded_urdf}"

log "checking installed wheel contact surfaces after SDFormat conversion"
expanded_sdf="/tmp/xgc2-scout-mini-expanded.sdf"
gz sdf -p "${expanded_urdf}" > "${expanded_sdf}"
python3 - "${expanded_sdf}" <<'PY'
import sys
import xml.etree.ElementTree as ET
root = ET.parse(sys.argv[1]).getroot()
wheels = [link for link in root.findall('.//link') if link.get('name', '').endswith('_wheel_link')]
assert len(wheels) == 4, 'Expected four wheel links in installed Scout model'
for wheel in wheels:
    collisions = wheel.findall('collision')
    assert len(collisions) == 1, wheel.get('name')
    ode = collisions[0].find('surface/friction/ode')
    assert ode is not None, wheel.get('name')
    for name, expected in {'mu': .1, 'mu2': 1., 'slip1': 5., 'slip2': 0.}.items():
        value = ode.findtext(name)
        assert value is not None and abs(float(value) - expected) < 1e-9, (wheel.get('name'), name, value)
PY

log "checking tuned Scout mini URDF arguments"
tuned_params="/tmp/xgc2-scout-spawn-accurate-tuned-params.yaml"
xacro "${mini_xacro}" \
  wheel_contact_mu1:=0.31 \
  wheel_contact_mu2:=0.91 \
  wheel_contact_fdir1:="0 1 0" \
  wheel_contact_slip1:=0.08 \
  wheel_contact_slip2:=0.02 \
  urdf_extras:="${empty_urdf}" > "${tuned_params}"
grep -q '<mu1 value="0.31"/>' "${tuned_params}"
grep -q '<mu2 value="0.91"/>' "${tuned_params}"
grep -q '<fdir1 value="0 1 0"/>' "${tuned_params}"
grep -q '<slip1>0.08</slip1>' "${tuned_params}"
grep -q '<slip2>0.02</slip2>' "${tuned_params}"
! grep -q '<odometryTopic>' "${tuned_params}"

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
