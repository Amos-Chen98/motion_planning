cmake_minimum_required(VERSION 3.0.2)
project(geo_robot_model)

set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE
  "-O3 -DNDEBUG -march=native -mtune=native -Wall"
  CACHE STRING "Release C++ compiler flags" FORCE
)

add_compile_options(-std=c++17)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_model
  aerial_robot_msgs
  dragon
  geometry_msgs
  message_filters
  nav_msgs
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_eigen
  tf2_ros
  visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES geo_dragon_stability
  CATKIN_DEPENDS
    aerial_robot_model
    aerial_robot_msgs
    dragon
    geometry_msgs
    message_filters
    nav_msgs
    pluginlib
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    tf2_eigen
    tf2_ros
    visualization_msgs
)

add_executable(livox_mid360_simulator
  src/livox_mid360_simulator.cpp
)

add_library(geo_dragon_stability
  src/dragon_stability_monitor.cpp
)

target_compile_definitions(geo_dragon_stability PRIVATE
  EIGEN_MAX_ALIGN_BYTES=16
  EIGEN_MAX_STATIC_ALIGN_BYTES=16
)

# This package otherwise uses -march=native, while the linked DRAGON model is
# built for the baseline architecture. Keep Eigen's compile-time ABI identical.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  target_compile_options(geo_dragon_stability PRIVATE
    -march=x86-64
    -mtune=generic
  )
endif()

target_link_libraries(geo_dragon_stability
  ${catkin_LIBRARIES}
)

add_executable(geo_dragon_stability_monitor
  src/dragon_stability_monitor_node.cpp
)

target_link_libraries(geo_dragon_stability_monitor
  geo_dragon_stability
  ${catkin_LIBRARIES}
)

target_link_libraries(livox_mid360_simulator
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(dragon_stability_monitor_test
    test/dragon_stability_monitor.test
    test/dragon_stability_monitor_test.cpp
  )
  if(TARGET dragon_stability_monitor_test)
    target_compile_definitions(dragon_stability_monitor_test PRIVATE
      EIGEN_MAX_ALIGN_BYTES=16
      EIGEN_MAX_STATIC_ALIGN_BYTES=16
    )
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
      target_compile_options(dragon_stability_monitor_test PRIVATE
        -march=x86-64
        -mtune=generic
      )
    endif()
    target_link_libraries(dragon_stability_monitor_test
      geo_dragon_stability
      ${catkin_LIBRARIES}
    )
    add_dependencies(dragon_stability_monitor_test
      geo_dragon_stability_monitor
      ${catkin_EXPORTED_TARGETS}
    )
  endif()
endif()

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

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS
  geo_dragon_stability
  geo_dragon_stability_monitor
  livox_mid360_simulator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
