cmake_minimum_required(VERSION 2.8.3)

project(gcopter)

set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

if(TARGET ompl::ompl)
  set(GCOPTER_OMPL_TARGET ompl::ompl)
else()
  set(GCOPTER_OMPL_TARGET ${OMPL_LIBRARIES})
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_eigen
  visualization_msgs
  message_generation
)

add_message_files(
  FILES
  PolyTraj.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    include
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gcopter_planner_common
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2_ros
    tf2_eigen
    visualization_msgs
    message_runtime
)

add_library(gcopter_planner_common src/planner_common.cpp)

add_executable(global_planning src/global_planning.cpp)
add_executable(online_replanning src/online_replanning.cpp)
add_executable(traj_server src/traj_server.cpp)

# Build generated trajectory messages before their consumers.
add_dependencies(gcopter_planner_common ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(global_planning ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(online_replanning ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(global_planning
  gcopter_planner_common
  ${catkin_LIBRARIES}
)

target_link_libraries(online_replanning
  gcopter_planner_common
  ${catkin_LIBRARIES}
)

target_link_libraries(gcopter_planner_common
  ${GCOPTER_OMPL_TARGET}
  ${catkin_LIBRARIES}
)

target_link_libraries(traj_server
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(planner_common_test test/planner_common_test.cpp)
  if(TARGET planner_common_test)
    add_dependencies(planner_common_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
    target_link_libraries(planner_common_test
      gcopter_planner_common
      ${catkin_LIBRARIES}
    )
  endif()
endif()

add_executable(corridor_degeneracy_repro test/corridor_degeneracy_repro.cpp)
target_link_libraries(corridor_degeneracy_repro
  ${GCOPTER_OMPL_TARGET}
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  test/analyze_bad_traj_bag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
