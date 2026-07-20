cmake_minimum_required(VERSION 3.0.2)
project(motion_primitive_planner)

add_compile_options(-std=c++17)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  gcopter
  multilink_copilot
  dragon
  pluginlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES motion_primitive_planner_core
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs gcopter multilink_copilot dragon pluginlib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(motion_primitive_planner_core src/planner_core.cpp)
target_link_libraries(motion_primitive_planner_core ${catkin_LIBRARIES})
add_dependencies(motion_primitive_planner_core ${catkin_EXPORTED_TARGETS})

add_executable(motion_primitive_planner_node src/online_planner.cpp)
target_link_libraries(motion_primitive_planner_node motion_primitive_planner_core ${catkin_LIBRARIES})
add_dependencies(motion_primitive_planner_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(motion_primitive_planner_core_test test/planner_core_test.cpp)
  if(TARGET motion_primitive_planner_core_test)
    target_link_libraries(motion_primitive_planner_core_test motion_primitive_planner_core ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS motion_primitive_planner_core motion_primitive_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
