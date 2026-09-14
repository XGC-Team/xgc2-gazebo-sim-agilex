#!/usr/bin/env bash
# Build the actual Gazebo 11.15.1 vendored ODE without ROS or a Gazebo install.
# Usage: bash build_native_ode.sh /path/to/pinned/gazebo-classic /new/build/directory
set -euo pipefail
if [[ $# != 2 ]]; then echo "usage: $0 GAZEBO_SOURCE BUILD_DIRECTORY" >&2; exit 2; fi
source_dir=$(realpath "$1")
output_dir=$(realpath -m "$2")
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
expected=b22c6e15e52299865b31093b8feebc9ca19e26e8
[[ $(git -C "$source_dir" rev-parse HEAD) == "$expected" ]] || {
  echo 'Refusing an unfrozen upstream ODE revision' >&2; exit 2;
}
[[ -z $(git -C "$source_dir" status --porcelain -- deps/opende deps/threadpool cmake) ]] || {
  echo 'Refusing modified upstream physics sources' >&2; exit 2;
}
if [[ -e "$output_dir" && ! -f "$output_dir/.issue93-native-build" ]]; then
  echo 'Build directory must be new, or previously created by this script' >&2; exit 2
fi
mkdir -p "$output_dir/source/gazebo"
touch "$output_dir/.issue93-native-build"
ln -sfn "$source_dir/deps" "$output_dir/source/deps"
ln -sfn "$source_dir/cmake" "$output_dir/source/cmake"
# ODE includes this generated Gazebo header only for optional feature switches.
# No GUI/ROS/DART/Bullet/HDF5/SSE override is enabled in this reference build.
cat > "$output_dir/source/gazebo/gazebo_config.h" <<'EOF'
#pragma once
#define GAZEBO_MAJOR_VERSION 11
#define GAZEBO_MINOR_VERSION 15
#define GAZEBO_PATCH_VERSION 1
EOF
cat > "$output_dir/source/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.16)
project(scout_native_ode LANGUAGES C CXX)
include(CheckCXXSourceCompiles)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(BUILD_SHARED_LIBS OFF)
set(GAZEBO_MAJOR_VERSION 11)
set(GAZEBO_VERSION_FULL 11.15.1)
set(INCLUDE_INSTALL_DIR include)
set(LIB_INSTALL_DIR lib)
set(BIN_INSTALL_DIR bin)
set(gazebo_cmake_dir "${CMAKE_SOURCE_DIR}/cmake")
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CCD REQUIRED ccd)
set(HAVE_LIBCCD TRUE)
include_directories("${CMAKE_SOURCE_DIR}" "${CMAKE_BINARY_DIR}")
add_subdirectory(deps/opende)
add_executable(contact_oracle "${SCOUT_VALIDATION_DIR}/contact_oracle.cpp")
target_compile_definitions(contact_oracle PRIVATE dDOUBLE)
target_include_directories(contact_oracle PRIVATE "${CMAKE_SOURCE_DIR}/deps/opende/include")
target_link_libraries(contact_oracle PRIVATE gazebo_ode Threads::Threads)
enable_testing()
add_test(NAME contact_oracle COMMAND contact_oracle)
EOF
cmake -S "$output_dir/source" -B "$output_dir/build" \
  -DCMAKE_BUILD_TYPE=Release -DSCOUT_VALIDATION_DIR="$script_dir"
cmake --build "$output_dir/build" --parallel "${BUILD_JOBS:-2}"
ctest --test-dir "$output_dir/build" --output-on-failure
{
  echo "gazebo_source_sha=$expected"
  echo 'scope=upstream vendored ODE; not Gazebo binary-equivalence certification'
  echo 'optional_features=ROS:none,Gazebo_GUI:none,DART:none,Bullet:none,HDF5:none,SSE_override:none'
  cmake --version | head -n1
  c++ --version | head -n1
  sha256sum "$output_dir/build/contact_oracle"
} | tee "$output_dir/build-receipt.txt"
