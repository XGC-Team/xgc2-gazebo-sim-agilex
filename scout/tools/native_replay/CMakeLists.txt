cmake_minimum_required(VERSION 3.10)
project(scout_native_replay LANGUAGES CXX)
find_package(gazebo 11.15.1 EXACT REQUIRED)
find_package(yaml-cpp REQUIRED)
add_executable(scout_native_replay replay.cpp)
target_compile_features(scout_native_replay PRIVATE cxx_std_17)
target_compile_options(scout_native_replay PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(scout_native_replay SYSTEM PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_include_directories(scout_native_replay PRIVATE ../../include)
target_link_libraries(scout_native_replay PRIVATE ${GAZEBO_LIBRARIES} yaml-cpp)
# This is an explicitly invoked test tool, not a production catkin node or
# installed controller. No ROS libraries, messages, publisher or master.
