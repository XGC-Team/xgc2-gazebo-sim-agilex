cmake_minimum_required(VERSION 2.8.3)
project(gazebo_sim_scout)

## C++ standard and release flags are set by workspace build defaults.

find_package(catkin REQUIRED COMPONENTS    
    roscpp
    controller_interface
    hardware_interface
    pluginlib
    urdf
    gazebo_msgs
    scout_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2 
    tf2_ros
)
find_package(gazebo REQUIRED)
find_package(xgc2_math 0.5.8 REQUIRED CONFIG)

catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES 
  CATKIN_DEPENDS roscpp gazebo_msgs scout_msgs sensor_msgs std_msgs
#   DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(scout_wheel_velocity_controller src/scout_wheel_velocity_controller.cpp)
target_link_libraries(scout_wheel_velocity_controller ${catkin_LIBRARIES} xgc2_math::control)

add_library(scout_gazebo STATIC src/scout_skid_steer.cpp)
find_package(Threads REQUIRED)
add_library(scout_unicycle_plugin SHARED src/scout_unicycle_plugin.cpp)
target_link_libraries(scout_unicycle_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} Threads::Threads xgc2_math::control)
add_library(scout_wheel_diagnostics SHARED src/scout_wheel_diagnostics.cpp)
target_link_libraries(scout_wheel_diagnostics ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} xgc2_math::control)
target_link_libraries(scout_gazebo ${catkin_LIBRARIES} Threads::Threads xgc2_math::control)

add_executable(scout_skid_steer_controller src/scout_skid_steer_controller.cpp)
target_link_libraries(scout_skid_steer_controller scout_gazebo ${catkin_LIBRARIES})

add_executable(gazebo_model_ground_truth src/gazebo_model_ground_truth.cpp)
target_link_libraries(gazebo_model_ground_truth ${catkin_LIBRARIES})

add_executable(sim_scout_status src/sim_scout_status.cpp)
add_dependencies(sim_scout_status ${catkin_EXPORTED_TARGETS})
target_link_libraries(sim_scout_status ${catkin_LIBRARIES})

#############
## Install ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  add_rostest(test/unicycle_runtime.test)
  roslaunch_add_file_check(launch)
  find_package(PythonInterp REQUIRED)
  add_test(
    NAME scout_stable_defaults
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_stable_defaults.py
  )
  add_test(
    NAME scout_source_rsp_clock
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_source_rsp_clock.py
  )
  set_tests_properties(scout_source_rsp_clock PROPERTIES TIMEOUT 90)
  add_test(
    NAME scout_surface_contract
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/gazebo_surface_contract.py
  )
  set_tests_properties(scout_surface_contract PROPERTIES TIMEOUT 30)
  add_test(
    NAME scout_ground_truth
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ground_truth.py
  )
  set_tests_properties(scout_ground_truth PROPERTIES TIMEOUT 90)
  add_test(NAME scout_dynamics_modes
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_dynamics_modes.py)
endif()

install(
  TARGETS scout_unicycle_plugin scout_wheel_diagnostics scout_wheel_velocity_controller scout_gazebo scout_skid_steer_controller gazebo_model_ground_truth sim_scout_status
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/pointcloud_to_pointcloud2
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scout_controllers.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include(cmake/implicit_wheel.cmake)
