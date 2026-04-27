#!/usr/bin/env python3

import os
import sys
current_path = os.path.abspath(os.path.dirname(__file__))[:-9]  # -9 removes '/ros_node'
sys.path.insert(0, current_path)
from octomap_msgs.msg import Octomap
from nav_msgs.msg import OccupancyGrid
from map_server.octree_server import OctreeServer
from map_server.esdf import ESDF
from traj_planner.geo_planner import GeoPlanner
import rospy
import numpy as np
import time
from map_server.pcl_server import PCLServer
import tf.transformations as tf_trans
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray


class AstarConfig():
    def __init__(self):
        self.resolution = rospy.get_param("~resolution", 0.2)
        self.min_flight_height = rospy.get_param("~min_flight_height", 2.0)
        self.max_flight_height = rospy.get_param("~max_flight_height", 4.0)
        self.dim = rospy.get_param("~dim", 3)  # 2 for 2D planning, 3 for 3D planning


class TrajPlanner():
    def __init__(self, node_name="traj_planner"):
        # Node
        rospy.init_node(node_name, anonymous=False)

        # Parameters
        a_star_config = AstarConfig()
        collision_threshold = rospy.get_param("~collision_threshold", 0.4)
        self.target_reach_threshold = rospy.get_param("~target_reach_threshold", 0.2)
        self.cmd_hz = rospy.get_param("~cmd_hz", 60)
        self.move_vel = rospy.get_param("~move_vel", 1.0)
        self.traj_opt = rospy.get_param("~traj_opt", False)
        map_server = rospy.get_param("~map_server", "octomap")  # available options: octomap, pcl, esdf
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.robot_pose_type = rospy.get_param("~robot_pose_type", "pose")  # available options: pose, odom
        self.pose_cmd_type = rospy.get_param("~pose_cmd_type", "pose")  # available options: pose, nav
        self.dim = a_star_config.dim
        dim = self.dim

        # Validate and setup map server based on dim and map_server parameters
        if map_server == "octomap" and dim == 3:
            rospy.loginfo("Using Octomap (3D) to perform collision check")
            self.map = OctreeServer(collision_threshold)
            self.octomap_sub = rospy.Subscriber('/octomap_binary', Octomap, self.map.octomap_cb)

        elif map_server == "pcl" and dim == 3:
            rospy.loginfo("Using PCL (3D) to perform collision check")
            self.map = PCLServer(collision_threshold)
            self.pcl_sub = rospy.Subscriber('/pointcloud/output', PointCloud2, self.map.pcl_cb)

        elif map_server == "esdf" and dim == 2:
            rospy.loginfo("Using ESDF (2D) to perform collision check")
            self.map = ESDF(collision_threshold)
            self.occupancy_map_sub = rospy.Subscriber('projected_map', OccupancyGrid, self.map.occupancy_map_cb)

        else:
            error_msg = f"Invalid configuration: map_server='{map_server}', dim={dim}. "
            error_msg += "Valid combinations are: (octomap, 3), (pcl, 3), (esdf, 2)"
            rospy.logerr(error_msg)
            raise ValueError(error_msg)

        # Planner
        self.planner = GeoPlanner(a_star_config, self.move_vel, self.cmd_hz, self.traj_opt)
        self.current_pos = np.zeros(3)
        self.current_yaw = 0.0

        # Initialize command message based on pose_cmd_type
        if self.pose_cmd_type == "pose":
            self.pose_cmd = PoseStamped()
            self.pose_cmd.header.frame_id = self.frame_id
        elif self.pose_cmd_type == "nav":
            self.nav_cmd = FlightNav()
            self.nav_cmd.header.frame_id = self.frame_id
            self.nav_cmd.control_frame = FlightNav.WORLD_FRAME
            self.nav_cmd.target = FlightNav.COG
            self.nav_cmd.pos_xy_nav_mode = FlightNav.POS_MODE
            self.nav_cmd.target_vel_x = 0.0
            self.nav_cmd.target_vel_y = 0.0
            self.nav_cmd.target_acc_x = 0.0
            self.nav_cmd.target_acc_y = 0.0
            self.nav_cmd.pos_z_nav_mode = FlightNav.POS_MODE
            self.nav_cmd.target_vel_z = 0.0
            self.nav_cmd.target_pos_diff_z = 0.0
            self.nav_cmd.yaw_nav_mode = FlightNav.POS_MODE
            self.nav_cmd.target_omega_z = 0.0
        else:
            error_msg = f"Invalid pose_cmd_type: '{self.pose_cmd_type}'. Valid options are: 'pose', 'nav'"
            rospy.logerr(error_msg)
            raise ValueError(error_msg)

        # Flags and counters
        self.target_received = False
        self.reached_target = False
        self.pose_received = False
        self.des_state_index = 0
        self.des_state_length = 0

        # Subscribers
        if self.robot_pose_type == "pose":
            rospy.loginfo("Subscribing to robot_pose (PoseStamped)")
            self.robot_pose_sub = rospy.Subscriber('robot_pose', PoseStamped, self.robot_pose_cb)
        elif self.robot_pose_type == "odom":
            rospy.loginfo("Subscribing to robot_pose (Odometry)")
            self.robot_pose_sub = rospy.Subscriber('robot_pose', Odometry, self.robot_odom_cb)
        else:
            error_msg = f"Invalid robot_pose_type: '{self.robot_pose_type}'. Valid options are: 'pose', 'odom'"
            rospy.logerr(error_msg)
            raise ValueError(error_msg)

        self.target_sub = rospy.Subscriber('target_pose', PoseStamped, self.target_cb)

        # Publishers
        if self.pose_cmd_type == "pose":
            self.pose_cmd_pub = rospy.Publisher("robot_pose_cmd", PoseStamped, queue_size=10)
        elif self.pose_cmd_type == "nav":
            self.nav_cmd_pub = rospy.Publisher("robot_pose_cmd", FlightNav, queue_size=10)
        self.traj_marker_pub = rospy.Publisher("trajectory_marker", MarkerArray, queue_size=10)

        rospy.loginfo(f"Trajectory planner initialized!")

    def robot_pose_cb(self, data):
        """Store the robot's position from PoseStamped message"""
        self.pose_received = True

        # Extract and store position
        self.current_pos = np.array([data.pose.position.x,
                                     data.pose.position.y,
                                     data.pose.position.z])

        # Extract and store yaw from quaternion
        qx = data.pose.orientation.x
        qy = data.pose.orientation.y
        qz = data.pose.orientation.z
        qw = data.pose.orientation.w
        self.current_yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        # Check if target reached
        if self.target_received:
            # For 2D planning, only compare x and y
            if self.dim == 2:
                distance = np.linalg.norm(self.current_pos[:2] - self.target_pos[:2])
            else:
                distance = np.linalg.norm(self.current_pos - self.target_pos)

            if distance < self.target_reach_threshold:
                rospy.loginfo("Target reached!\n")
                self.end_mission()

    def robot_odom_cb(self, data):
        """Store the robot's position from Odometry message"""
        self.pose_received = True

        # Extract and store position
        self.current_pos = np.array([data.pose.pose.position.x,
                                     data.pose.pose.position.y,
                                     data.pose.pose.position.z])

        # Extract and store yaw from quaternion
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w
        self.current_yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        # Check if target reached
        if self.target_received:
            # For 2D planning, only compare x and y
            if self.dim == 2:
                distance = np.linalg.norm(self.current_pos[:2] - self.target_pos[:2])
            else:
                distance = np.linalg.norm(self.current_pos - self.target_pos)

            if distance < self.target_reach_threshold:
                rospy.loginfo("Target reached!\n")
                self.end_mission()

    def end_mission(self):
        if hasattr(self, 'tracking_cmd_timer'):
            self.tracking_cmd_timer.shutdown()
        self.target_received = False
        self.reached_target = True
        self.des_state_index = 0

    def target_cb(self, msg):
        """Callback for target_pose topic"""
        self.target_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        rospy.loginfo("Target received: x = %f, y = %f, z = %f",
                      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

        self.target_received = True
        self.reached_target = False
        self.des_state_index = 0

        self.global_planning()

    def global_planning(self):
        if not self.pose_received:
            rospy.logwarn("Waiting for robot pose...")
            return

        try:
            time_start = time.time()

            # Pass velocity information for trajectory optimization
            start_vel = np.zeros(3)  # Assume starting from rest
            target_vel = np.zeros(3)  # Assume ending at rest
            path_positions, sparse_path = self.planner.geo_traj_plan(self.map, self.current_pos, self.target_pos,
                                                                     start_vel, target_vel)

            time_end = time.time()
            rospy.loginfo("Planning time: {}".format(time_end - time_start))

            # Convert 2D path to 3D if necessary by adding current z coordinate
            if len(path_positions) > 0 and len(path_positions[0]) == 2:
                path_positions_array = np.array(path_positions)
                z_column = np.full((len(path_positions), 1), self.current_pos[2])
                path_positions = np.hstack([path_positions_array, z_column])

            if len(sparse_path) > 0 and len(sparse_path[0]) == 2:
                sparse_path_array = np.array(sparse_path)
                z_column = np.full((len(sparse_path), 1), self.current_pos[2])
                sparse_path = np.hstack([sparse_path_array, z_column])

            self.path_positions = path_positions
            self.sparse_path = sparse_path
            self.des_state_length = len(self.path_positions)

            self.publish_trajectory_marker()
            self.start_tracking()

        except Exception as ex:
            rospy.logerr("Planning failed: %s", ex)

    def publish_trajectory_marker(self):
        """Publish trajectory as markers for RViz visualization"""
        marker_array = MarkerArray()

        # Line strip marker for the path
        line_marker = Marker()
        line_marker.header.frame_id = self.frame_id
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "trajectory_line"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.08  # Line width
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0

        for pos in self.path_positions:
            point = PoseStamped().pose.position
            point.x = pos[0]
            point.y = pos[1]
            point.z = pos[2]
            line_marker.points.append(point)

        marker_array.markers.append(line_marker)

        # Sphere markers for sparse path
        sphere_marker = Marker()
        sphere_marker.header.frame_id = self.frame_id
        sphere_marker.header.stamp = rospy.Time.now()
        sphere_marker.ns = "sparse_waypoints"
        sphere_marker.id = 1
        sphere_marker.type = Marker.SPHERE_LIST
        sphere_marker.action = Marker.ADD
        sphere_marker.pose.orientation.w = 1.0
        sphere_marker.scale.x = 0.1
        sphere_marker.scale.y = 0.1
        sphere_marker.scale.z = 0.1
        sphere_marker.color.r = 0.0
        sphere_marker.color.g = 0.0
        sphere_marker.color.b = 1.0
        sphere_marker.color.a = 1.0

        if hasattr(self, 'sparse_path'):
            for pos in self.sparse_path:
                point = PoseStamped().pose.position
                point.x = pos[0]
                point.y = pos[1]
                point.z = pos[2]
                sphere_marker.points.append(point)

        marker_array.markers.append(sphere_marker)

        self.traj_marker_pub.publish(marker_array)
        rospy.loginfo("Published trajectory markers with {} waypoints".format(len(self.path_positions)))

    def start_tracking(self):
        """Start publishing trajectory commands"""
        if hasattr(self, 'tracking_cmd_timer'):
            self.tracking_cmd_timer.shutdown()
        self.tracking_cmd_timer = rospy.Timer(rospy.Duration(1/self.cmd_hz), self.tracking_cmd_timer_cb)

    def tracking_cmd_timer_cb(self, event):
        """Publish trajectory commands"""
        if self.des_state_index >= self.des_state_length:
            return

        # Set position
        pos = self.path_positions[self.des_state_index]

        # Calculate yaw from trajectory direction
        if self.des_state_index > 0:
            prev_pos = self.path_positions[self.des_state_index - 1]
            yaw = np.arctan2(pos[1] - prev_pos[1], pos[0] - prev_pos[0])
        else:
            yaw = self.current_yaw

        if self.pose_cmd_type == "pose":
            # PoseStamped message
            self.pose_cmd.pose.position.x = pos[0]
            self.pose_cmd.pose.position.y = pos[1]
            self.pose_cmd.pose.position.z = pos[2]

            # Convert yaw to quaternion
            self.pose_cmd.pose.orientation.x = 0.0
            self.pose_cmd.pose.orientation.y = 0.0
            self.pose_cmd.pose.orientation.z = np.sin(yaw / 2.0)
            self.pose_cmd.pose.orientation.w = np.cos(yaw / 2.0)

            self.pose_cmd.header.stamp = rospy.Time.now()
            self.pose_cmd_pub.publish(self.pose_cmd)

        elif self.pose_cmd_type == "nav":
            # FlightNav message
            self.nav_cmd.target_pos_x = pos[0]
            self.nav_cmd.target_pos_y = pos[1]
            self.nav_cmd.target_pos_z = pos[2]
            self.nav_cmd.target_yaw = yaw

            self.nav_cmd.header.stamp = rospy.Time.now()
            self.nav_cmd_pub.publish(self.nav_cmd)

        if self.des_state_index < self.des_state_length - 1:
            self.des_state_index += 1


if __name__ == "__main__":

    traj_planner = TrajPlanner()

    rospy.spin()
