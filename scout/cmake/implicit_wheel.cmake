# Included by the Scout CMakeLists after the existing dependencies are configured.
add_library(scout_implicit_wheel_plugin SHARED src/scout_implicit_wheel_plugin.cpp)
target_link_libraries(scout_implicit_wheel_plugin
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} Threads::Threads xgc2_math::control)
install(TARGETS scout_implicit_wheel_plugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  add_test(NAME scout_implicit_wheel_wiring
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_implicit_wheel_wiring.py)
  add_rostest(test/implicit_wheel_runtime.test)
endif()
