cmake_minimum_required(VERSION 3.0.2)
project(motion_primitive_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(Threads REQUIRED)
find_package(coal 3 REQUIRED CONFIG)
find_package(octomap REQUIRED)
find_package(orocos_kdl REQUIRED)
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_LIBRARIES coal::coal)
check_cxx_source_compiles("#include <coal/octree.h>
#ifndef COAL_HAS_OCTOMAP
#error Coal must support OctoMap
#endif
int main() { coal::OcTree tree(0.1); return 0; }" MOTION_PRIMITIVE_COAL_OCTOMAP)
unset(CMAKE_REQUIRED_LIBRARIES)
if(NOT MOTION_PRIMITIVE_COAL_OCTOMAP)
  message(FATAL_ERROR "motion_primitive_planner requires Coal with OctoMap support")
endif()
find_package(catkin REQUIRED COMPONENTS
  aerial_robot_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  octomap_msgs
  rog_map_msgs
  diagnostic_msgs
  tf2_eigen
  tf2_ros
  std_srvs
  visualization_msgs
  gcopter
  multilink_copilot
  dragon
  pluginlib
  urdf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES motion_primitive_planner_core
  CATKIN_DEPENDS aerial_robot_msgs roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs octomap_msgs rog_map_msgs diagnostic_msgs tf2_eigen tf2_ros std_srvs visualization_msgs gcopter multilink_copilot dragon pluginlib urdf
  DEPENDS orocos_kdl
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

add_library(motion_primitive_planner_core
  src/octomap_message.cpp
  src/local_map.cpp
  src/root_primitive_generator.cpp
  src/dragon_geometry.cpp
  src/dragon_collision_checker.cpp
  src/whole_body_planner.cpp
  src/planner_config.cpp
  src/joint_trajectory_planner.cpp
)
if(TARGET ompl::ompl)
  set(MOTION_PRIMITIVE_OMPL_TARGET ompl::ompl)
else()
  set(MOTION_PRIMITIVE_OMPL_TARGET ${OMPL_LIBRARIES})
endif()
# Use Coal's transitive OctoMap libraries so a second installation cannot change the ABI.
target_link_libraries(motion_primitive_planner_core ${catkin_LIBRARIES} ${MOTION_PRIMITIVE_OMPL_TARGET} ${CMAKE_THREAD_LIBS_INIT} coal::coal ${orocos_kdl_LIBRARIES})
add_dependencies(motion_primitive_planner_core ${catkin_EXPORTED_TARGETS})

add_executable(whole_body_motion_primitive_planner_node src/whole_body_planner_node.cpp)
target_link_libraries(whole_body_motion_primitive_planner_node
  motion_primitive_planner_core
  ${catkin_LIBRARIES}
)
add_dependencies(whole_body_motion_primitive_planner_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/local_map_protocol.test)
  catkin_install_python(PROGRAMS test/local_map_protocol_test.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  catkin_add_gtest(local_map_test test/local_map_test.cpp)
  if(TARGET local_map_test)
    target_link_libraries(local_map_test motion_primitive_planner_core ${catkin_LIBRARIES})
    add_dependencies(local_map_test ${catkin_EXPORTED_TARGETS})
  endif()
  add_rostest_gtest(dragon_collision_checker_test
    test/dragon_collision_checker.test test/dragon_collision_checker_test.cpp)
  if(TARGET dragon_collision_checker_test)
    target_include_directories(dragon_collision_checker_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(dragon_collision_checker_test motion_primitive_planner_core ${catkin_LIBRARIES})
    add_dependencies(dragon_collision_checker_test ${catkin_EXPORTED_TARGETS})
  endif()
  catkin_add_gtest(candidate_executor_test test/candidate_executor_test.cpp)
  if(TARGET candidate_executor_test)
    target_include_directories(candidate_executor_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(candidate_executor_test ${CMAKE_THREAD_LIBS_INIT})
  endif()
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
endif()

install(TARGETS motion_primitive_planner_core whole_body_motion_primitive_planner_node
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
