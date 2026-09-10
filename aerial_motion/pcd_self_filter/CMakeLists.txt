cmake_minimum_required(VERSION 3.7.2)
project(pcd_self_filter)

add_compile_options(-std=c++17 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs diagnostic_msgs robot_body_filter filters pluginlib tf2_ros tf2_sensor_msgs urdf)
find_package(PkgConfig REQUIRED)
pkg_check_modules(tinyxml2 REQUIRED tinyxml2)
find_package(PCL REQUIRED COMPONENTS common filters)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pcd_self_filter_model
  CATKIN_DEPENDS roscpp sensor_msgs diagnostic_msgs robot_body_filter filters pluginlib tf2_ros tf2_sensor_msgs urdf
  DEPENDS tinyxml2)
include_directories(include ${catkin_INCLUDE_DIRS} ${tinyxml2_INCLUDE_DIRS})
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

add_library(pcd_self_filter_model src/model_adapter.cpp src/cloud_validation.cpp)
target_link_libraries(pcd_self_filter_model ${catkin_LIBRARIES} ${tinyxml2_LIBRARIES})
add_executable(pcd_self_filter_node src/pcd_self_filter_node.cpp)
# Upstream 1.3.2 selects std::optional on C++17 toolchains but does not export
# this header ABI selection to consumers.
target_compile_definitions(pcd_self_filter_node PRIVATE ROBOT_BODY_FILTER_USE_CXX_OPTIONAL=1)
target_link_libraries(pcd_self_filter_node pcd_self_filter_model ${catkin_LIBRARIES})
add_dependencies(pcd_self_filter_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(pcd_self_filter_model_test test/model_adapter_test.cpp)
  if(TARGET pcd_self_filter_model_test)
    target_link_libraries(pcd_self_filter_model_test pcd_self_filter_model)
  endif()
  add_rostest(test/filter_pipeline.test)
  add_rostest(test/self_filter.test)
  add_rostest(test/paused_clock.test)
  catkin_install_python(PROGRAMS test/self_filter_test.py test/paused_clock_test.py test/filter_pipeline_test.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()

install(TARGETS pcd_self_filter_model pcd_self_filter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
