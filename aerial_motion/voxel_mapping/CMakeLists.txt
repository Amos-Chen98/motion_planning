cmake_minimum_required(VERSION 3.0.2)
project(voxel_mapping)

add_compile_options(-std=c++17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters search)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf2_eigen
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES voxel_mapping_core
  CATKIN_DEPENDS roscpp sensor_msgs tf2_eigen tf2_ros
  DEPENDS PCL
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(voxel_mapping_core src/occupied_voxel_map.cpp)
target_include_directories(voxel_mapping_core SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_options(voxel_mapping_core PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(voxel_mapping_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(voxel_mapping_node src/voxel_mapping_node.cpp)
target_link_libraries(voxel_mapping_node voxel_mapping_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(voxel_mapping_core_test test/occupied_voxel_map_test.cpp)
  if(TARGET voxel_mapping_core_test)
    target_link_libraries(voxel_mapping_core_test voxel_mapping_core ${catkin_LIBRARIES})
  endif()
  add_rostest(test/noise_filter.test)
  catkin_install_python(PROGRAMS test/noise_filter_test.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()

install(TARGETS voxel_mapping_core voxel_mapping_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
