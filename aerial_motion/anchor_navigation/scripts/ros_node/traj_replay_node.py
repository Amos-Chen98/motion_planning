#!/usr/bin/env python3
"""
Trajectory Replay Node

This ROS node replays saved trajectories by:
1. Reading robot's real-time state
2. Loading trajectory files from saved_traj directory
3. Moving robot to trajectory start position
4. Replaying the full trajectory on user command

Author: Auto-generated based on motion_planner_node
"""

import os
import sys
import rospy
import numpy as np
import tf2_ros
import geometry_msgs.msg
import tf.transformations
import time
import threading
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav

# Add current path for imports
current_dir = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, current_dir)

from planner.traj_coder import TrajCoder
from planner.robot_model import Robot
from planner.bspline_planner import normalize_target_angle


class TrajReplayNode:
    """ROS node for replaying saved trajectories"""
    
    def __init__(self, node_name="traj_replay_node"):
        # Initialize ROS node
        rospy.init_node(node_name, anonymous=False)
        
        # Get parameters
        self.cmd_hz = rospy.get_param("~cmd_hz", 60)
        self.prepare_speed = rospy.get_param("~prepare_speed", 0.1)  # m/s for moving to start position
        self.speed_scale = rospy.get_param("~speed_scale", 1.0)  # Speed scale factor for trajectory replay
        default_robot_urdf = os.path.dirname(current_dir) + "/urdf/hydrus_xi_20241227.urdf"
        robot_urdf = rospy.get_param("~robot_urdf", default_robot_urdf)
        
        # Initialize robot model and trajectory coder
        self.robot = Robot(robot_urdf)
        self.traj_coder = TrajCoder(robot_urdf)
        
        # Joint names for visualization
        joint_vis_names = ["gimbal1", "gimbal2", "gimbal3", "gimbal4", "joint1", "joint2", "joint3", "rotor1", "rotor2", "rotor3", "rotor4"]
        
        # For visualizing the target state (final goal) in rviz
        self.target_state_tf_bc = tf2_ros.StaticTransformBroadcaster()  # TF broadcaster for visualizing the target state in rviz
        self.target_state_tf = geometry_msgs.msg.TransformStamped()
        self.target_state_tf.header.frame_id = "world"
        self.target_state_tf.child_frame_id = "target_state/root"
        self.target_joint_vis_state = JointState()
        self.target_joint_vis_state.name = joint_vis_names
        
        # For visualizing the start state (trajectory beginning) in rviz
        self.start_state_tf_bc = tf2_ros.StaticTransformBroadcaster()  # TF broadcaster for visualizing the start state in rviz
        self.start_state_tf = geometry_msgs.msg.TransformStamped()
        self.start_state_tf.header.frame_id = "world"
        self.start_state_tf.child_frame_id = "start_state/root"
        self.start_joint_vis_state = JointState()
        self.start_joint_vis_state.name = joint_vis_names
        
        # For visualizing the real-time desired robot's motion in rviz
        self.des_state_tf_bc = tf2_ros.TransformBroadcaster()  # TF broadcaster for visualizing the desired robot's motion in rviz
        self.des_state_tf = geometry_msgs.msg.TransformStamped()
        self.des_state_tf.header.frame_id = "world"
        self.des_state_tf.child_frame_id = "des_state/root"
        self.des_joint_vis_state = JointState()
        self.des_joint_vis_state.name = joint_vis_names
        
        # Robot state variables
        self.joint_state = JointState()
        self.fc_pose = None
        self.rootlink_state_received = False
        self.joint_state_received = False
        
        # Current robot state (similar to motion_planner_node)
        self.current_rootlink_pos_x = 0.0
        self.current_rootlink_pos_y = 0.0
        self.current_rootlink_pos_z = 0.0
        self.current_rootlink_yaw = 0.0
        self.current_joint_angles = []
        
        # Trajectory replay variables
        self.des_states = None
        self.des_state_index = 0
        self.des_state_len = 0
        self.prepare_trajectory = None
        self.prepare_index = 0
        self.prepare_len = 0
        
        # State machine
        self.state = "WAITING_FOR_FILENAME"  # WAITING_FOR_FILENAME, WAITING_FOR_PREPARE, PREPARING, WAITING_FOR_LAUNCH, REPLAYING
        
        # Control command messages
        self.baselink_state_cmd = FlightNav()
        self.joint_state_cmd = JointState()
        self.joint_state_cmd.name = ["joint1", "joint2", "joint3"]
        
        # Configure flight command
        self.baselink_state_cmd.control_frame = FlightNav().WORLD_FRAME
        self.baselink_state_cmd.target = FlightNav().COG
        self.baselink_state_cmd.pos_xy_nav_mode = FlightNav().POS_MODE
        self.baselink_state_cmd.pos_z_nav_mode = FlightNav().NO_NAVIGATION
        self.baselink_state_cmd.yaw_nav_mode = FlightNav().POS_VEL_MODE
        
        # Position threshold for reaching targets
        self.position_threshold = 0.1  # meters
        self.yaw_threshold = 0.1  # radians
        self.joint_threshold = 0.05  # radians
        
        # Subscribers
        self.fc_pose_sub = rospy.Subscriber('uav/baselink/odom', Odometry, self.fc_pose_cb)
        self.joint_state_sub = rospy.Subscriber('joint_states', JointState, self.joint_state_cb)
        
        # Publishers
        self.uav_nav_pub = rospy.Publisher('uav/nav', FlightNav, queue_size=10)
        self.joint_des_state_pub = rospy.Publisher('joints_ctrl', JointState, queue_size=10)
        
        # Visualization publishers
        self.target_joint_vis_state_pub = rospy.Publisher('target_joint_vis_state', JointState, queue_size=10)
        self.start_joint_vis_state_pub = rospy.Publisher('start_joint_vis_state', JointState, queue_size=10)
        self.des_joint_vis_state_pub = rospy.Publisher('des_joint_vis_state', JointState, queue_size=10)
        
        # Control timer
        self.control_timer = None
        
        # Input thread for user commands
        self.input_thread = threading.Thread(target=self.input_handler, daemon=True)
        self.input_thread.start()
        
        rospy.loginfo("Trajectory Replay Node initialized!")
        rospy.loginfo(f"Speed scale factor: {self.speed_scale}")
        rospy.loginfo("Available commands:")
        rospy.loginfo("  - Enter trajectory file ID (number from the list)")
        rospy.loginfo("  - Press 'P' to prepare (move to start position)")
        rospy.loginfo("  - Press 'L' to launch trajectory replay")
        rospy.loginfo("  - Press 'Q' to quit")
        
        self.print_available_trajectories()
        self.prompt_for_file_id()

    def fc_pose_cb(self, data: Odometry):
        """Callback for flight controller pose"""
        self.fc_pose = data
        
        # Extract position and orientation
        fc_pos = np.array([data.pose.pose.position.x,
                          data.pose.pose.position.y,
                          data.pose.pose.position.z])
        fc_orientation = np.array([data.pose.pose.orientation.x,
                                  data.pose.pose.orientation.y,
                                  data.pose.pose.orientation.z,
                                  data.pose.pose.orientation.w])
        
        # Only process if we have joint state data
        if not self.joint_state_received:
            return
            
        # Calculate rootlink state from flight controller position
        rootlink_state = self.robot.get_rootlink_pos_from_fc_pos(
            np.concatenate((fc_pos, fc_orientation)),
            self.current_joint_angles)
        
        # Update current robot state
        self.current_rootlink_pos_x = rootlink_state[0]
        self.current_rootlink_pos_y = rootlink_state[1] 
        self.current_rootlink_pos_z = rootlink_state[2]
        self.current_rootlink_yaw = rootlink_state[3]
        self.rootlink_state_received = True

    def joint_state_cb(self, data: JointState):
        """Callback for joint states"""
        self.joint_state = data
        if len(data.position) >= 7:  # gimbal(4) + joints(3)
            self.current_joint_angles = list(data.position[4:])  # Skip gimbal angles
            self.joint_state_received = True

    def print_available_trajectories(self):
        """Print list of available trajectory files"""
        files = self.traj_coder.list_saved_trajectories()
        if files:
            rospy.loginfo("Available trajectory files:")
            rospy.loginfo(f"Speed scale factor: {self.speed_scale} (1.0 = original speed)")
            for i, filename in enumerate(files):
                # Remove .json extension for display
                display_name = filename[:-5] if filename.endswith('.json') else filename
                info = self.traj_coder.get_trajectory_info(
                    os.path.join(self.traj_coder.save_dir, filename), self.speed_scale)
                if info:
                    if self.speed_scale != 1.0:
                        rospy.loginfo(f"  {i+1:2d}. {display_name} ({info['num_segments']} segments, "
                                    f"{info['total_time']:.1f}s scaled from {info['original_time']:.1f}s)")
                    else:
                        rospy.loginfo(f"  {i+1:2d}. {display_name} ({info['num_segments']} segments, {info['total_time']:.1f}s)")
                else:
                    rospy.loginfo(f"  {i+1:2d}. {display_name}")
        else:
            rospy.logwarn("No trajectory files found in saved_traj directory")

    def prompt_for_file_id(self):
        """Prompt user for trajectory file ID"""
        if self.state == "WAITING_FOR_FILENAME":
            print("\nPlease enter trajectory file ID (number from the list), or 'Q' to quit:")

    def input_handler(self):
        """Handle user input in a separate thread"""
        while not rospy.is_shutdown():
            try:
                user_input = input().strip()
                
                if user_input.upper() == 'Q':
                    rospy.loginfo("Quitting trajectory replay node...")
                    rospy.signal_shutdown("User requested quit")
                    break
                    
                elif self.state == "WAITING_FOR_FILENAME":
                    self.handle_file_id_input(user_input)
                    
                elif self.state == "WAITING_FOR_PREPARE" and user_input.upper() == 'P':
                    self.start_preparation()
                    
                elif self.state == "WAITING_FOR_LAUNCH" and user_input.upper() == 'L':
                    self.start_trajectory_replay()
                    
            except EOFError:
                break
            except Exception as e:
                rospy.logerr(f"Input handler error: {e}")

    def handle_file_id_input(self, file_id_str):
        """Handle trajectory file ID input"""
        if not file_id_str:
            return
            
        try:
            file_id = int(file_id_str)
        except ValueError:
            rospy.logwarn(f"Invalid input '{file_id_str}'. Please enter a number from the list.")
            self.print_available_trajectories()
            self.prompt_for_file_id()
            return
            
        # Get available files
        files = self.traj_coder.list_saved_trajectories()
        
        if not files:
            rospy.logwarn("No trajectory files available")
            self.prompt_for_file_id()
            return
            
        # Check if ID is valid (1-indexed)
        if file_id < 1 or file_id > len(files):
            rospy.logwarn(f"Invalid file ID {file_id}. Please enter a number between 1 and {len(files)}.")
            self.print_available_trajectories()
            self.prompt_for_file_id()
            return
            
        # Get filename (convert to 0-indexed)
        filename = files[file_id - 1]
        filepath = os.path.join(self.traj_coder.save_dir, filename)
        
        # Load trajectory
        rospy.loginfo(f"Loading trajectory {file_id}: {filename}")
        rospy.loginfo(f"Speed scale factor: {self.speed_scale}")
        self.des_states = self.traj_coder.load_and_compute_commands(filepath, self.cmd_hz, self.speed_scale)
        
        if self.des_states is None:
            rospy.logerr("Failed to load trajectory")
            self.prompt_for_file_id()
            return
            
        self.des_state_len = len(self.des_states)
        self.des_state_index = 0
        
        rospy.loginfo(f"Loaded trajectory with {self.des_state_len} command points")
        
        # Visualize start state (first state in trajectory)
        self.vis_start_state()
        
        # Visualize target state (last state in trajectory)
        self.vis_target_state()
        
        # Wait for user to confirm preparation
        self.state = "WAITING_FOR_PREPARE"
        print("\n" + "="*60)
        print("TRAJECTORY LOADED SUCCESSFULLY!")
        print("="*60)
        print("")
        print("Press 'P' to prepare (move robot to start position):")
        print("="*60)

    def wait_for_robot_state(self):
        """Wait for robot state to be available"""
        while not (self.rootlink_state_received and self.joint_state_received) and not rospy.is_shutdown():
            rospy.sleep(0.1)

    def generate_preparation_trajectory(self):
        """Generate trajectory to move robot to start position"""
        if not self.des_states:
            return
            
        # Get current and target configuration vectors (rootlink state)
        current_config = np.array([
            self.current_rootlink_pos_x,
            self.current_rootlink_pos_y,
            self.current_rootlink_yaw,
            *self.current_joint_angles
        ])
        
        target_config = np.array([
            self.des_states[0]['rootlink'].pos_x,
            self.des_states[0]['rootlink'].pos_y,
            self.des_states[0]['rootlink'].yaw,
            *[joint.pos for joint in self.des_states[0]['joints']]
        ])
        
        # Normalize target yaw to avoid unnecessary rotation
        target_config[2] = normalize_target_angle(current_config[2], target_config[2])
        
        # Calculate generalized distance and time
        config_diff = target_config - current_config
        generalized_distance = np.linalg.norm(config_diff)
        prepare_time = max(generalized_distance / self.prepare_speed, 1.0)  # Minimum 1 second
        
        # Generate preparation waypoints using np.linspace
        num_points = max(int(prepare_time * self.cmd_hz), 10)  # Minimum 10 points
        self.prepare_len = num_points
        
        rospy.loginfo(f"Generating preparation trajectory: {prepare_time:.1f}s, {num_points} points")
        rospy.loginfo(f"Generalized distance: {generalized_distance:.3f}")
        
        # Interpolate between current and target configurations
        config_array = np.linspace(current_config, target_config, num_points)
        
        # Convert configurations to proper desired states using robot model
        self.prepare_trajectory = self.robot.get_des_states(config_array)
        
        # Start preparation
        self.state = "PREPARING"
        self.prepare_index = 0
        self.start_control_timer()
        
        rospy.loginfo("Moving robot to trajectory start position...")

    def start_control_timer(self):
        """Start the control command timer"""
        if self.control_timer is not None:
            self.control_timer.shutdown()
        self.control_timer = rospy.Timer(rospy.Duration(1.0 / self.cmd_hz), self.control_timer_cb)

    def control_timer_cb(self, event):
        """Control timer callback"""
        if self.state == "PREPARING":
            self.execute_preparation()
        elif self.state == "REPLAYING":
            self.execute_replay()

    def execute_preparation(self):
        """Execute preparation trajectory"""
        if self.prepare_index >= self.prepare_len:
            # Preparation finished
            rospy.loginfo("Robot reached trajectory start position")
            self.state = "WAITING_FOR_LAUNCH"
            self.control_timer.shutdown()
            self.control_timer = None
            print("\n" + "="*60)
            print("PREPARATION COMPLETED!")
            print("="*60)
            print("✓ Robot is now at the trajectory start position")
            print("")
            print("Press 'L' to launch trajectory replay:")
            print("="*60)
            return
            
        # Publish preparation commands
        self.publish_commands(self.prepare_trajectory[self.prepare_index])
        self.prepare_index += 1

    def execute_replay(self):
        """Execute trajectory replay"""
        if self.des_state_index >= self.des_state_len:
            # Trajectory finished
            rospy.loginfo("Trajectory replay completed!")
            self.state = "WAITING_FOR_FILENAME"
            self.control_timer.shutdown()
            self.control_timer = None
            print("\n" + "="*60)
            print("TRAJECTORY REPLAY COMPLETED!")
            print("="*60)
            print("✓ Robot has reached the target position")
            print("")
            self.print_available_trajectories()
            self.prompt_for_file_id()
            return
            
        # Publish trajectory commands
        self.publish_commands(self.des_states[self.des_state_index])
        self.des_state_index += 1

    def start_trajectory_replay(self):
        """Start trajectory replay"""
        if self.state != "WAITING_FOR_LAUNCH":
            return
            
        rospy.loginfo("Starting trajectory replay...")
        self.state = "REPLAYING"
        self.des_state_index = 0
        self.start_control_timer()

    def publish_commands(self, desired_state):
        """Publish control commands for desired state"""
        # Publish baselink command
        self.baselink_state_cmd.header.stamp = rospy.Time.now()
        self.baselink_state_cmd.target_pos_x = desired_state['cog'].pos_x
        self.baselink_state_cmd.target_pos_y = desired_state['cog'].pos_y
        self.baselink_state_cmd.target_pos_z = self.current_rootlink_pos_z  # Maintain current Z
        self.baselink_state_cmd.target_yaw = desired_state['cog'].yaw
        
        self.baselink_state_cmd.target_vel_x = desired_state['cog'].vel_x
        self.baselink_state_cmd.target_vel_y = desired_state['cog'].vel_y
        self.baselink_state_cmd.target_vel_z = 0.0
        self.baselink_state_cmd.target_omega_z = desired_state['cog'].omega_z
        
        self.uav_nav_pub.publish(self.baselink_state_cmd)
        
        # Publish joint command
        self.joint_state_cmd.header.stamp = rospy.Time.now()
        self.joint_state_cmd.position = [joint.pos for joint in desired_state['joints']]
        self.joint_state_cmd.velocity = [joint.vel for joint in desired_state['joints']]
        
        self.joint_des_state_pub.publish(self.joint_state_cmd)
        
        # Publish visualization
        self.send_des_state_vis_tf(desired_state)
        self.pub_joint_vis_state(desired_state)

    def send_des_state_vis_tf(self, desired_state):
        """Send desired state transform for visualization"""
        self.des_state_tf.header.stamp = rospy.Time.now()
        self.des_state_tf.transform.translation.x = desired_state['rootlink'].pos_x
        self.des_state_tf.transform.translation.y = desired_state['rootlink'].pos_y
        self.des_state_tf.transform.translation.z = self.current_rootlink_pos_z

        quaternion = tf.transformations.quaternion_from_euler(0, 0, desired_state['rootlink'].yaw)

        self.des_state_tf.transform.rotation.x = quaternion[0]
        self.des_state_tf.transform.rotation.y = quaternion[1]
        self.des_state_tf.transform.rotation.z = quaternion[2]
        self.des_state_tf.transform.rotation.w = quaternion[3]

        self.des_state_tf_bc.sendTransform(self.des_state_tf)

    def pub_joint_vis_state(self, desired_state):
        """Publish desired joint state for visualization"""
        self.des_joint_vis_state.header.stamp = rospy.Time.now()
        joint_positions = [desired_state['joints'][i].pos for i in range(len(desired_state['joints']))]
        joint_velocities = [desired_state['joints'][i].vel for i in range(len(desired_state['joints']))]

        # gimbal angles (4) + joint angles (3) + rotor angles (4)
        self.des_joint_vis_state.position = [0, 0, 0, 0] + joint_positions + [0, 0, 0, 0]
        self.des_joint_vis_state.velocity = [0, 0, 0, 0] + joint_velocities + [0, 0, 0, 0]

        self.des_joint_vis_state_pub.publish(self.des_joint_vis_state)

    def vis_target_state(self):
        """Visualize the target state (last state in trajectory) in rviz - called once per trajectory"""
        if not self.des_states or len(self.des_states) == 0:
            return
            
        self.send_target_state_vis_tf()
        self.pub_target_joint_state()

    def send_target_state_vis_tf(self):
        """Send target state transform for visualization"""
        if not self.des_states:
            return
            
        # Use the last state in trajectory as target
        target_state = self.des_states[-1]
        
        self.target_state_tf.header.stamp = rospy.Time.now()
        self.target_state_tf.transform.translation.x = target_state['rootlink'].pos_x
        self.target_state_tf.transform.translation.y = target_state['rootlink'].pos_y
        # Use a default Z height if robot state not available yet
        self.target_state_tf.transform.translation.z = self.current_rootlink_pos_z if self.rootlink_state_received else 0.0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, target_state['rootlink'].yaw)

        self.target_state_tf.transform.rotation.x = quaternion[0]
        self.target_state_tf.transform.rotation.y = quaternion[1]
        self.target_state_tf.transform.rotation.z = quaternion[2]
        self.target_state_tf.transform.rotation.w = quaternion[3]

        self.target_state_tf_bc.sendTransform(self.target_state_tf)

    def pub_target_joint_state(self):
        """Publish target joint state for visualization"""
        if not self.des_states:
            return
            
        # Use the last state in trajectory as target
        target_state = self.des_states[-1]
        
        self.target_joint_vis_state.header.stamp = rospy.Time.now()
        # gimbal angles (4) + joint angles (3) + rotor angles (4)
        target_joint_positions = [target_state['joints'][i].pos for i in range(len(target_state['joints']))]
        self.target_joint_vis_state.position = [0, 0, 0, 0] + target_joint_positions + [0, 0, 0, 0]
        self.target_joint_vis_state.velocity = [0] * 11
        self.target_joint_vis_state_pub.publish(self.target_joint_vis_state)

    def vis_start_state(self):
        """Visualize the start state (first state in trajectory) in rviz - called once per trajectory"""
        if not self.des_states or len(self.des_states) == 0:
            return
            
        self.send_start_state_vis_tf()
        self.pub_start_joint_state()

    def send_start_state_vis_tf(self):
        """Send start state transform for visualization"""
        if not self.des_states:
            return
            
        # Use the first state in trajectory as start
        start_state = self.des_states[0]
        
        self.start_state_tf.header.stamp = rospy.Time.now()
        self.start_state_tf.transform.translation.x = start_state['rootlink'].pos_x
        self.start_state_tf.transform.translation.y = start_state['rootlink'].pos_y
        # Use a default Z height if robot state not available yet
        self.start_state_tf.transform.translation.z = self.current_rootlink_pos_z if self.rootlink_state_received else 0.0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, start_state['rootlink'].yaw)

        self.start_state_tf.transform.rotation.x = quaternion[0]
        self.start_state_tf.transform.rotation.y = quaternion[1]
        self.start_state_tf.transform.rotation.z = quaternion[2]
        self.start_state_tf.transform.rotation.w = quaternion[3]

        self.start_state_tf_bc.sendTransform(self.start_state_tf)

    def pub_start_joint_state(self):
        """Publish start joint state for visualization"""
        if not self.des_states:
            return
            
        # Use the first state in trajectory as start
        start_state = self.des_states[0]
        
        self.start_joint_vis_state.header.stamp = rospy.Time.now()
        # gimbal angles (4) + joint angles (3) + rotor angles (4)
        start_joint_positions = [start_state['joints'][i].pos for i in range(len(start_state['joints']))]
        self.start_joint_vis_state.position = [0, 0, 0, 0] + start_joint_positions + [0, 0, 0, 0]
        self.start_joint_vis_state.velocity = [0] * 11
        self.start_joint_vis_state_pub.publish(self.start_joint_vis_state)

    def start_preparation(self):
        """Start the preparation phase after user confirmation"""
        print("\n" + "="*60)
        print("PREPARATION PHASE STARTED")
        print("="*60)
        rospy.loginfo("User confirmed preparation. Acquiring robot state...")
        
        # Wait for robot state
        self.wait_for_robot_state()
        
        # Generate preparation trajectory
        self.generate_preparation_trajectory()

    def cleanup(self):
        """Cleanup function"""
        if self.control_timer is not None:
            self.control_timer.shutdown()
        rospy.loginfo("Trajectory replay node shutdown")


if __name__ == "__main__":
    try:
        node = TrajReplayNode()
        rospy.on_shutdown(node.cleanup)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received, shutting down...")
