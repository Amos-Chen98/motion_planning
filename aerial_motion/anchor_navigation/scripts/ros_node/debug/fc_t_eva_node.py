#!/usr/bin/env python3

import os
import sys
# Navigate up to the scripts directory (from debug/ -> ros_node/ -> scripts/)
current_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
sys.path.insert(0, current_dir)

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from planner.robot_model import Robot


class FCTEvaNode:
    def __init__(self, node_name="fc_t_eva_node"):
        # Initialize ROS node
        rospy.init_node(node_name, anonymous=False)
        
        # Robot model initialization
        default_robot_urdf = os.path.dirname(current_dir) + "/urdf/hydrus_xi_20241227.urdf"
        robot_urdf = rospy.get_param("~robot_urdf", default_robot_urdf)
        self.robot = Robot(robot_urdf)
        
        # State variables
        self.joint_angles = np.zeros(self.robot.joint_num)
        self.gimbal_angles = np.zeros(4)  # 4 gimbal angles
        self.joint_state_received = False
        
        # Subscribe to joint states (includes both joint angles and gimbal angles)
        # Based on motion_planner_node.py, the joint_state includes gimbal angles as first 4 joints
        self.joint_state_sub = rospy.Subscriber('joint_states', JointState, self.joint_state_cb)
        
        # Publisher for fc_t_min
        self.fc_t_min_pub = rospy.Publisher('fc_t_min', Float64, queue_size=10)
        
        rospy.loginfo("FC T Eva Node initialized!")
        
    def joint_state_cb(self, data):
        """
        Callback for joint state messages
        Based on motion_planner_node.py:
        - gimbal_angles are the first 4 joints: data.position[:4]
        - joint_angles are joints 5-7: data.position[4:]
        """
        if len(data.position) >= 7:  # Ensure we have all required joints
            self.gimbal_angles = np.array(data.position[:4])  # First 4 are gimbal angles
            self.joint_angles = np.array(data.position[4:])   # Next 3 are joint angles
            self.joint_state_received = True
            
            # Calculate and publish fc_t_min
            self.calculate_and_publish_fc_t_min()
        else:
            rospy.logwarn(f"Received joint state with insufficient joints: {len(data.position)} < 7")
    
    def calculate_and_publish_fc_t_min(self):
        """
        Calculate fc_t_min using the Robot class and publish it
        """
        if not self.joint_state_received:
            return
            
        # Create robot configuration with zeros for position and yaw
        # Format: [pos_x, pos_y, yaw, joint1, joint2, joint3, gimbal1, gimbal2, gimbal3, gimbal4]
        config = np.concatenate([
            np.zeros(3),          # [pos_x, pos_y, yaw] - position doesn't affect fc_t_min
            self.joint_angles,    # [joint1, joint2, joint3]
            self.gimbal_angles    # [gimbal1, gimbal2, gimbal3, gimbal4]
        ])
        
        try:
            # Calculate fc_t_min using the Robot class
            fc_t_min = self.robot.get_fc_t_min(config)
            
            # Create and publish Float64 message
            msg = Float64()
            msg.data = fc_t_min
            self.fc_t_min_pub.publish(msg)
            
        except Exception as e:
            rospy.logerr(f"Error calculating fc_t_min: {e}")


def main():
    try:
        node = FCTEvaNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("FC T Eva Node shutting down")


if __name__ == "__main__":
    main()
