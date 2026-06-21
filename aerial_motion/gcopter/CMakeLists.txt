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
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    include
)

catkin_package(
  INCLUDE_DIRS include
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
)

add_executable(global_planning src/global_planning.cpp)
add_executable(online_replanning src/online_replanning.cpp)
add_executable(livox_mid360_simulator src/livox_mid360_simulator.cpp)

target_link_libraries(global_planning
  ${GCOPTER_OMPL_TARGET}
  ${catkin_LIBRARIES}
)

target_link_libraries(online_replanning
  ${GCOPTER_OMPL_TARGET}
  ${catkin_LIBRARIES}
)

target_link_libraries(livox_mid360_simulator
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/point_robot_model.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
