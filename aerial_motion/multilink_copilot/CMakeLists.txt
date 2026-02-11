cmake_minimum_required(VERSION 3.0.2)
project(multilink_copilot)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
    aerial_robot_msgs
    aerial_robot_model
    tf
    tf_conversions
    kdl_parser
    eigen_conversions
)

find_package(Eigen3 REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
    aerial_robot_msgs
    aerial_robot_model
    tf
    tf_conversions
    kdl_parser
    eigen_conversions
)

# ##########
# # Build ##
# ##########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# Copilot planner executable
add_executable(copilot_planner src/copilot_planner.cpp)
target_link_libraries(copilot_planner ${catkin_LIBRARIES})
add_dependencies(copilot_planner aerial_robot_msgs_generate_messages_cpp)

# ############
# # Install ##
# ############
install(TARGETS copilot_planner
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    USE_SOURCE_PERMISSIONS
)
