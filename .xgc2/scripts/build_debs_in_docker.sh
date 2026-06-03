#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DOCKER_IMAGE="${DOCKER_IMAGE:-ros:noetic-ros-base-focal}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image)
      DOCKER_IMAGE="$2"
      shift 2
      ;;
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --skip-install-check)
      INSTALL_CHECK=false
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"

docker pull "${DOCKER_IMAGE}"
docker run --rm \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -v "${REPO_ROOT}:/workspace/xgc2-gazebo-sim-scout:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" \
  bash -lc '
    set -euo pipefail

    export DEBIAN_FRONTEND=noninteractive
    apt-get update
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      dpkg-dev \
      fakeroot \
      file \
      gazebo11 \
      libgazebo11-dev \
      python3-yaml \
      rsync \
      ros-noetic-controller-manager \
      ros-noetic-gazebo-msgs \
      ros-noetic-gazebo-plugins \
      ros-noetic-gazebo-ros \
      ros-noetic-gazebo-ros-control \
      ros-noetic-geometry-msgs \
      ros-noetic-joint-state-controller \
      ros-noetic-joint-state-publisher \
      ros-noetic-joint-state-publisher-gui \
      ros-noetic-nav-msgs \
      ros-noetic-robot-state-publisher \
      ros-noetic-roslaunch \
      ros-noetic-roslint \
      ros-noetic-rospack \
      ros-noetic-rostopic \
      ros-noetic-rviz \
      ros-noetic-sensor-msgs \
      ros-noetic-std-msgs \
      ros-noetic-tf \
      ros-noetic-tf2 \
      ros-noetic-tf2-ros \
      ros-noetic-urdf \
      ros-noetic-velocity-controllers \
      ros-noetic-xacro

    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
    mkdir -p /workspace/work/src
    rsync -a --delete /workspace/xgc2-gazebo-sim-scout/scout/ /workspace/work/src/

    cd /workspace/work
    source /opt/ros/noetic/setup.bash
    DESTDIR=/workspace/work/install-root catkin_make install \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
      -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"

    /workspace/xgc2-gazebo-sim-scout/.xgc2/scripts/package_debs.sh \
      --install-root /workspace/work/install-root \
      --output-dir /workspace/out

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      apt-get install -y /workspace/out/*.deb
      /workspace/xgc2-gazebo-sim-scout/.xgc2/scripts/check_installed_packages.sh
    fi
  '

echo "Debian package output:"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "*.deb" -print | sort
