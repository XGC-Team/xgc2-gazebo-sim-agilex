#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"

product_version() {
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml"
}

VERSION="${PACKAGE_VERSION:-$(product_version)}"
if [[ -z "${VERSION}" ]]; then
  echo "package version is missing; set PACKAGE_VERSION or .xgc2/product.yml version" >&2
  exit 1
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
BUILD_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}"
rm -f "${OUTPUT_DIR}"/*.deb

copy_path() {
  local src="$1"
  local dst_root="$2"
  if [[ -e "${src}" ]]; then
    mkdir -p "${dst_root}$(dirname "${src#${INSTALL_ROOT}}")"
    cp -a "${src}" "${dst_root}${src#${INSTALL_ROOT}}"
  fi
}

copy_ros_package_paths() {
  local ros_pkg="$1"
  local dst_root="$2"

  copy_path "${PREFIX_ROOT}/share/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/lib/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/include/${ros_pkg}" "${dst_root}"
}

copy_libs() {
  local dst_root="$1"
  shift
  local lib
  for lib in "$@"; do
    copy_path "${PREFIX_ROOT}/lib/${lib}.a" "${dst_root}"
    copy_path "${PREFIX_ROOT}/lib/${lib}.so" "${dst_root}"
  done
}

write_control() {
  local pkg_root="$1"
  local package="$2"
  local depends="$3"
  local description="$4"
  local extra_fields="${5:-}"

  mkdir -p "${pkg_root}/DEBIAN" "${pkg_root}/usr/share/doc/${package}"
  {
    cat <<EOF
Package: ${package}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: ${depends}
EOF
    if [[ -n "${extra_fields}" ]]; then
      printf '%s\n' "${extra_fields}"
    fi
    cat <<EOF
Description: ${description}
EOF
  } > "${pkg_root}/DEBIAN/control"
  printf 'xgc2 gz-classic scout package\n' > "${pkg_root}/usr/share/doc/${package}/README"
  chmod 0755 "${pkg_root}/DEBIAN"
}

build_deb() {
  local package="$1"
  local ros_pkg="$2"
  local depends="$3"
  local description="$4"
  shift 4
  local extra_fields=""
  if [[ "$#" -gt 0 ]]; then
    extra_fields="$1"
    shift
  fi

  local pkg_root="${BUILD_DIR}/${package}"
  rm -rf "${pkg_root}"
  mkdir -p "${pkg_root}"

  if [[ -n "${ros_pkg}" ]]; then
    copy_ros_package_paths "${ros_pkg}" "${pkg_root}"
  fi
  if [[ "$#" -gt 0 ]]; then
    copy_libs "${pkg_root}" "$@"
  fi

  write_control "${pkg_root}" "${package}" "${depends}" "${description}" "${extra_fields}"
  fakeroot dpkg-deb --build "${pkg_root}" "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

gazebo_pkg="ros-noetic-xgc2-gazebo-sim-scout"
description_dep="ros-noetic-xgc2-scout-description (>= 0.4.10-8)"

build_deb \
  "${gazebo_pkg}" \
  "gazebo_sim_scout" \
  "${description_dep}, ros-noetic-xgc2-gazebo-sim-worlds (>= 1.1.0-10), ros-noetic-xgc2-gazebo-sim-vrpn-bridge (>= 1.1.0-12), ros-noetic-scout-msgs (>= 0.3.3-5), ros-noetic-roscpp, ros-noetic-geometry-msgs, ros-noetic-gazebo-msgs, ros-noetic-nav-msgs, ros-noetic-sensor-msgs, ros-noetic-std-msgs, ros-noetic-tf, ros-noetic-tf2, ros-noetic-tf2-ros, ros-noetic-controller-manager, ros-noetic-gazebo-plugins, ros-noetic-gazebo-ros, ros-noetic-gazebo-ros-control, ros-noetic-joint-state-controller, ros-noetic-joint-state-publisher, ros-noetic-robot-state-publisher, ros-noetic-rostopic, ros-noetic-rviz, ros-noetic-velocity-controllers" \
  "XGC2 AgileX Scout Gazebo Classic simulation" \
  "Replaces: ros-noetic-xgc2-scout-gazebo-sim, ros-noetic-xgc2-gz-classic-scout
Conflicts: ros-noetic-xgc2-scout-gazebo-sim, ros-noetic-xgc2-gz-classic-scout" \
  libscout_gazebo

find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
