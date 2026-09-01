cmake_minimum_required(VERSION 2.8.3)
project(gazebo_sim_scout)

## C++ standard and release flags are set by workspace build defaults.

find_package(catkin REQUIRED COMPONENTS    
    roscpp 
    gazebo_msgs
    nav_msgs
    scout_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2 
    tf2_ros
)
catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES 
  CATKIN_DEPENDS roscpp gazebo_msgs nav_msgs scout_msgs sensor_msgs std_msgs
#   DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(scout_gazebo STATIC src/scout_skid_steer.cpp)
find_package(Threads REQUIRED)
target_link_libraries(scout_gazebo ${catkin_LIBRARIES} Threads::Threads)

add_executable(scout_skid_steer_controller src/scout_skid_steer_controller.cpp)
target_link_libraries(scout_skid_steer_controller scout_gazebo ${catkin_LIBRARIES})

add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})

add_executable(gazebo_model_odom src/gazebo_model_odom.cpp)
target_link_libraries(gazebo_model_odom ${catkin_LIBRARIES})

add_executable(sim_scout_status src/sim_scout_status.cpp)
add_dependencies(sim_scout_status ${catkin_EXPORTED_TARGETS})
target_link_libraries(sim_scout_status ${catkin_LIBRARIES})

#############
## Install ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
  find_package(PythonInterp REQUIRED)
  add_test(
    NAME scout_stable_defaults
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_stable_defaults.py
  )
endif()

install(
  TARGETS scout_gazebo scout_skid_steer_controller odom_to_tf gazebo_model_odom sim_scout_status
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
