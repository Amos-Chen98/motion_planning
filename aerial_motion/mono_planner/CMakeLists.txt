cmake_minimum_required(VERSION 3.0.2)
project(mono_planner)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    mavros_msgs
    octomap_msgs
    tf
)

catkin_package(
    CATKIN_DEPENDS
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    mavros_msgs
    octomap_msgs
    tf
)

# Install Python scripts
catkin_install_python(PROGRAMS
    scripts/ros_node/traj_planner_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)

# Install config files
install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)
