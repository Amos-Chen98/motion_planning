cmake_minimum_required(VERSION 3.0.2)
project(geo_robot_model)

add_compile_options(-std=c++17)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  tf
  tf2_eigen
  tf2_ros
  visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    aerial_robot_msgs
    geometry_msgs
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    tf
    tf2_eigen
    tf2_ros
    visualization_msgs
)

add_executable(livox_mid360_simulator
  src/livox_mid360_simulator.cpp
)

target_link_libraries(livox_mid360_simulator
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/point_robot_model.py
  scripts/pose_to_flight_nav.py
  scripts/geo_dragon_model.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(TARGETS livox_mid360_simulator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
