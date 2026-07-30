cmake_minimum_required(VERSION 3.0.2)
project(motion_primitive_planner)

add_compile_options(-std=c++17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  aerial_robot_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  gcopter
  multilink_copilot
  dragon
  pluginlib
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES motion_primitive_planner_core
  CATKIN_DEPENDS aerial_robot_msgs roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs gcopter multilink_copilot dragon pluginlib tf
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

add_library(motion_primitive_planner_core
  src/planner_core.cpp
  src/planner_common.cpp
  src/planner_config.cpp
  src/root_candidate_evaluator.cpp
  src/joint_trajectory_planner.cpp
)
if(TARGET ompl::ompl)
  set(MOTION_PRIMITIVE_OMPL_TARGET ompl::ompl)
else()
  set(MOTION_PRIMITIVE_OMPL_TARGET ${OMPL_LIBRARIES})
endif()
target_link_libraries(motion_primitive_planner_core ${catkin_LIBRARIES} ${MOTION_PRIMITIVE_OMPL_TARGET})
add_dependencies(motion_primitive_planner_core ${catkin_EXPORTED_TARGETS})

add_executable(motion_primitive_planner_node src/online_planner.cpp)
target_link_libraries(motion_primitive_planner_node motion_primitive_planner_core ${catkin_LIBRARIES})
add_dependencies(motion_primitive_planner_node ${catkin_EXPORTED_TARGETS})

add_executable(whole_body_motion_primitive_planner_node src/whole_body_online_planner.cpp)
target_link_libraries(whole_body_motion_primitive_planner_node
  motion_primitive_planner_core
  ${catkin_LIBRARIES}
)
add_dependencies(whole_body_motion_primitive_planner_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(motion_primitive_planner_core_test test/planner_core_test.cpp)
  if(TARGET motion_primitive_planner_core_test)
    target_link_libraries(motion_primitive_planner_core_test motion_primitive_planner_core ${catkin_LIBRARIES})
  endif()
  add_rostest_gtest(whole_body_joint_planner_test
    test/whole_body_joint_planner.test
    test/whole_body_joint_planner_test.cpp
  )
  if(TARGET whole_body_joint_planner_test)
    target_link_libraries(whole_body_joint_planner_test motion_primitive_planner_core ${catkin_LIBRARIES})
    add_dependencies(whole_body_joint_planner_test ${catkin_EXPORTED_TARGETS})
  endif()
  add_rostest_gtest(whole_body_joint_bridge_test
    test/whole_body_joint_bridge.test
    test/whole_body_joint_bridge_test.cpp
  )
  if(TARGET whole_body_joint_bridge_test)
    target_link_libraries(whole_body_joint_bridge_test motion_primitive_planner_core ${catkin_LIBRARIES})
    add_dependencies(whole_body_joint_bridge_test ${catkin_EXPORTED_TARGETS})
  endif()
  add_rostest_gtest(whole_body_node_integration_test
    test/whole_body_node_integration.test
    test/whole_body_node_integration_test.cpp
  )
  if(TARGET whole_body_node_integration_test)
    target_link_libraries(whole_body_node_integration_test ${catkin_LIBRARIES})
    add_dependencies(whole_body_node_integration_test ${catkin_EXPORTED_TARGETS})
  endif()
  add_rostest_gtest(root_node_integration_test
    test/root_node_integration.test
    test/root_node_integration_test.cpp
  )
  if(TARGET root_node_integration_test)
    target_link_libraries(root_node_integration_test ${catkin_LIBRARIES})
    add_dependencies(root_node_integration_test ${catkin_EXPORTED_TARGETS})
  endif()
endif()

install(TARGETS motion_primitive_planner_core motion_primitive_planner_node whole_body_motion_primitive_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
  scripts/root_pose_to_flu_odom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
