cmake_minimum_required(VERSION 2.8.3)
project(scout_gazebo_sim)

## C++ standard and release flags are set by workspace build defaults.

find_package(catkin REQUIRED COMPONENTS    
    roslaunch 
    roslint 
    roscpp 
    gazebo_msgs
    nav_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2 
    tf2_ros
)
catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES 
  CATKIN_DEPENDS roscpp gazebo_msgs nav_msgs sensor_msgs
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
target_link_libraries(scout_gazebo ${catkin_LIBRARIES})

add_executable(scout_skid_steer_controller src/scout_skid_steer_controller.cpp)
target_link_libraries(scout_skid_steer_controller scout_gazebo ${catkin_LIBRARIES})

add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})

add_executable(gazebo_model_odom src/gazebo_model_odom.cpp)
target_link_libraries(gazebo_model_odom ${catkin_LIBRARIES})

#############
## Install ##
#############

roslaunch_add_file_check(launch)

install(
  DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
