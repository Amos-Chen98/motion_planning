import os
import sys
scripts_dir = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, scripts_dir)
from planner.robot_model import Robot
import numpy as np
import rospy
import geometry_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray



class Visualizer:
    """
    A class for handling robot trajectory visualization.
    """

    def __init__(self, robot_urdf):
        """
        Initialize the visualizer with a robot model.

        Args:
            robot_urdf: Path to the robot URDF file
        """
        self.robot_viz = Robot(robot_urdf)

    def get_key_states_viz(self, key_states_list):
        """
        Collect the states for visualization

        Args:
            key_states_list: List of key states for visualization

        Returns:
            all_link_joint_list: each element is a (rotor_num+1, 3) array (root + joints + end effector)
            all_rotor_pos_list: each element is a (rotor_num, 3) array
        """
        all_link_joint_list = []
        all_rotor_pos_list = []

        for i in range(1, len(key_states_list)-1):  # skip the last state
            pos = key_states_list[i][0]

            joint_pos_array = self.robot_viz.get_joint_pos_vis(pos)  # shape: (joint_num, 3)
            rotor_pos_list, _ = self.robot_viz.get_rotor_pos_jac(pos)  # shape: (rotor_num, 3)

            last_joint_pos = joint_pos_array[-1]
            last_rotor_pos = rotor_pos_list[-1]
            last_link_dir = last_rotor_pos - last_joint_pos
            end_effector_pos = last_joint_pos + 2 * last_link_dir

            root_pos = np.array([pos[0], pos[1], last_joint_pos[-1]])  # z-axis is the last joint pos
            current_links_seg_array = np.vstack((root_pos, joint_pos_array, end_effector_pos))
            all_link_joint_list.append(current_links_seg_array)
            all_rotor_pos_list.append(rotor_pos_list)

        return all_link_joint_list, all_rotor_pos_list

    def create_key_states_marker_array(self, key_states_viz, rootlink_pos_z_realtime):
        """
        Create a MarkerArray for visualizing key states of the robot trajectory.

        Args:
            key_states_viz: Tuple containing (all_link_joint_list, all_rotor_pos_list)
            rootlink_pos_z_realtime: Current Z position of the rootlink

        Returns:
            MarkerArray: Visualization markers for links and propellers
        """
        all_link_joint_list, all_rotor_pos_list = key_states_viz

        marker_array = MarkerArray()
        id = int(0)
        now = rospy.Time.now()

        # Delete all existing markers first
        delete_marker = Marker()
        delete_marker.header.frame_id = "world"
        delete_marker.header.stamp = now
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        vis_state_num = len(all_link_joint_list)

        # Get propeller radius from robot parameters
        propeller_R = self.robot_viz.param['propeller_R']

        # Create a vibrant multi-color gradient: Blue -> Cyan -> Green -> Yellow -> Orange -> Red
        key_colors = np.array([
            [0.0, 0.0, 1.0],  # Blue
            [0.0, 1.0, 1.0],  # Cyan
            [0.0, 1.0, 0.0],  # Green
            [1.0, 1.0, 0.0],  # Yellow
            [1.0, 0.5, 0.0],  # Orange
            [1.0, 0.0, 0.0]   # Red
        ])

        # Interpolate through all key colors to create smooth gradient
        if vis_state_num == 1:
            color_gradient = np.array([key_colors[0]])
        else:
            # Create parameter values for interpolation
            key_positions = np.linspace(0, 1, len(key_colors))
            state_positions = np.linspace(0, 1, vis_state_num)

            # Interpolate each RGB channel separately
            color_gradient = np.zeros((vis_state_num, 3))
            for channel in range(3):
                color_gradient[:, channel] = np.interp(state_positions, key_positions, key_colors[:, channel])

        for i, (link_joint_array, rotor_pos_array) in enumerate(zip(all_link_joint_list, all_rotor_pos_list)):
            # Links
            link_marker = Marker()
            link_marker.header.frame_id = "world"
            link_marker.header.stamp = now
            link_marker.ns = "links"
            link_marker.id = id
            id += 1
            link_marker.type = Marker.LINE_STRIP
            link_marker.action = Marker.ADD
            link_marker.pose.orientation.w = 1.0
            link_marker.scale.x = 0.03
            link_marker.color.r = 0.0
            link_marker.color.g = 0.0
            link_marker.color.b = 0.0
            link_marker.color.a = 0.5
            link_marker.points = [geometry_msgs.msg.Point(x=pos[0], y=pos[1], z=rootlink_pos_z_realtime) for pos in link_joint_array]

            marker_array.markers.append(link_marker)

            # Propellers
            for rotor_pos in rotor_pos_array:
                propeller_marker = Marker()
                propeller_marker.header.frame_id = "world"
                propeller_marker.header.stamp = now
                propeller_marker.ns = "rotors"
                propeller_marker.id = id
                id += 1
                propeller_marker.type = Marker.CYLINDER
                propeller_marker.action = Marker.ADD
                propeller_marker.pose.position.x = rotor_pos[0]
                propeller_marker.pose.position.y = rotor_pos[1]
                propeller_marker.pose.position.z = rootlink_pos_z_realtime
                propeller_marker.pose.orientation.w = 1.0
                propeller_marker.scale.x = 2 * propeller_R
                propeller_marker.scale.y = 2 * propeller_R
                propeller_marker.scale.z = 0.02
                propeller_marker.color.r = color_gradient[i][0]
                propeller_marker.color.g = color_gradient[i][1]
                propeller_marker.color.b = color_gradient[i][2]
                propeller_marker.color.a = 0.5

                marker_array.markers.append(propeller_marker)

        return marker_array
