cmake_minimum_required(VERSION 3.0.2)
project(multilink_copilot)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    geometry_msgs
    visualization_msgs
    aerial_robot_msgs
    differential_kinematics
    dragon
    pluginlib
    tf
    tf_conversions
)

find_package(Eigen3 REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    geometry_msgs
    visualization_msgs
    aerial_robot_msgs
    differential_kinematics
    dragon
    pluginlib
)

# ##########
# # Build ##
# ##########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

set(DIFFERENTIAL_KINEMATICS_QPOASES_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../differential_kinematics/qpoases)
include_directories(${DIFFERENTIAL_KINEMATICS_QPOASES_DIR}/include)
link_directories(${DIFFERENTIAL_KINEMATICS_QPOASES_DIR}/lib)

# Copilot planner executable
set(COPILOT_PLANNER_SOURCES
    src/copilot_node.cpp
    src/copilot_stability.cpp
    src/copilot_stability_qp.cpp
    src/copilot_motion.cpp
)

add_executable(copilot_planner ${COPILOT_PLANNER_SOURCES})
target_link_libraries(copilot_planner ${catkin_LIBRARIES} qpOASES)
add_dependencies(copilot_planner aerial_robot_msgs_generate_messages_cpp)

add_executable(geometry_only_copilot_planner
    src/demo/geometry_only_copilot_planner.cpp
)
target_link_libraries(geometry_only_copilot_planner ${catkin_LIBRARIES})
add_dependencies(geometry_only_copilot_planner aerial_robot_msgs_generate_messages_cpp)

catkin_install_python(PROGRAMS
    scripts/evaluation/envelope_width_evaluator.py
    scripts/evaluation/last_link_tail_pose_publisher.py
    scripts/evaluation/trajectory_marker_publisher.py
    scripts/evaluation/run_waypoint_conditioned_maneuvering_batch.py
    scripts/demo/dragon_dance_node.py
    scripts/demo/root_tail_pose_seed_publisher.py
    scripts/debug/print_dragon_edf_positions.py
    scripts/debug/reset_dragon_joint.py
    scripts/evaluation/replay_rosbag.py
    scripts/evaluation/replay_tf_bridge.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ############
# # Install ##
# ############
install(TARGETS copilot_planner geometry_only_copilot_planner
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    USE_SOURCE_PERMISSIONS
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    USE_SOURCE_PERMISSIONS
)

install(DIRECTORY rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    USE_SOURCE_PERMISSIONS
)

install(FILES
    scripts/demo/dragon_dance_support.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
