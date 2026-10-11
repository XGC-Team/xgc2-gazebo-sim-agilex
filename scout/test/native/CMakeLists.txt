cmake_minimum_required(VERSION 3.16)
project(native_chassis_hold_fixture LANGUAGES CXX)
# The test plugin of verify.py: a world plugin that blocks the chassis command dispatcher on request. It builds
# against the scene product that verify.py exercises:
#   cmake -S scout/test/native -B <build> -DSCENE_INCLUDE=<xgc2_gazebo_scene>/include \
#         -DSIMULATION_LIBRARY=<path of libxgc2_simulation_service.so>
find_package(PkgConfig REQUIRED)
pkg_check_modules(Gazebo REQUIRED IMPORTED_TARGET gazebo)
get_target_property(options PkgConfig::Gazebo INTERFACE_COMPILE_OPTIONS)
list(FILTER options EXCLUDE REGEX "^-std=")
set_property(TARGET PkgConfig::Gazebo PROPERTY INTERFACE_COMPILE_OPTIONS "${options}")
pkg_check_modules(JsonCpp REQUIRED IMPORTED_TARGET jsoncpp)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
foreach(value SCENE_INCLUDE SIMULATION_LIBRARY)
  if(NOT ${value})
    message(FATAL_ERROR "${value} explicit native dependency is required")
  endif()
endforeach()
add_library(simulation_service SHARED IMPORTED)
set_target_properties(simulation_service PROPERTIES IMPORTED_LOCATION "${SIMULATION_LIBRARY}")
add_library(xgc2_dispatcher_fixture SHARED dispatcher_fixture.cpp)
target_compile_features(xgc2_dispatcher_fixture PRIVATE cxx_std_17)
target_include_directories(xgc2_dispatcher_fixture PRIVATE "${SCENE_INCLUDE}")
target_include_directories(xgc2_dispatcher_fixture SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(xgc2_dispatcher_fixture PRIVATE simulation_service PkgConfig::Gazebo PkgConfig::JsonCpp
  ${catkin_LIBRARIES})
