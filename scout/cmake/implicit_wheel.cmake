# Included by the Scout CMakeLists after the existing dependencies are configured.
add_library(scout_implicit_wheel_plugin SHARED src/scout_implicit_wheel_plugin.cpp)
target_compile_features(scout_implicit_wheel_plugin PRIVATE cxx_std_20)
target_link_libraries(scout_implicit_wheel_plugin
  XgcXrpc::http
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} Threads::Threads xgc2_math::control)
set_target_properties(scout_implicit_wheel_plugin PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS scout_implicit_wheel_plugin LIBRARY DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins)
if(CATKIN_ENABLE_TESTING)
  add_test(NAME scout_implicit_wheel_wiring
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_implicit_wheel_wiring.py)
endif()
