import os
import sys
current_dir = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, current_dir)
from planner.global_planner import GlobalPlanner
from data_review.planning_logger import PlanningLogger
from map_server.esdf import ESDF
from planner.robot_model import Robot
from planner.gimbal_yaw_planner_bspline import GimbalYawPlanner
from planner.bspline_planner import normalize_target_angle, OptLog
from visualizer.visualizer import Visualizer
import copy
import time
import subprocess
from datetime import datetime
import numpy as np
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry, OccupancyGrid
from aerial_robot_msgs.msg import FlightNav
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from anchor_navigation.msg import HydrusConfig


class MotionPlanner():
    def __init__(self, node_name="motion_planner"):
        # Node
        rospy.init_node(node_name, anonymous=False)

        # urdf
        default_robot_urdf = os.path.dirname(current_dir) + "/urdf/hydrus_xi_20241227.urdf"
        robot_urdf = rospy.get_param("~robot_urdf", default_robot_urdf)

        # robot
        self.robot = Robot(robot_urdf)

        # visualizer
        self.visualizer = Visualizer(robot_urdf)

        # Variables
        self.joint_state = JointState()
        self.gimbal_angles = JointState()
        self.robot_state = HydrusConfig()
        self.target_state = HydrusConfig()
        self.baselink_state_cmd = FlightNav()
        self.joint_state_cmd = JointState()
        self.joint_state_cmd.name = ["joint1", "joint2", "joint3"]
        self.gimbal_des_state = JointState()
        self.gimbal_des_state.name = ["gimbal1", "gimbal2", "gimbal3", "gimbal4"]

        self.baselink_state_cmd.control_frame = FlightNav().WORLD_FRAME
        self.baselink_state_cmd.target = FlightNav().COG  # OR BASELINK, NOTE: baselink is defined at the flight controller (fc), NOT the rootlink
        self.baselink_state_cmd.pos_xy_nav_mode = FlightNav().POS_MODE  # POS_MODE = 2, POS_VEL_MODE = 4
        self.baselink_state_cmd.pos_z_nav_mode = FlightNav().NO_NAVIGATION
        self.baselink_state_cmd.yaw_nav_mode = FlightNav().POS_VEL_MODE

        # Joint names for visualization (shared between target and desired state)
        joint_vis_names = ["gimbal1", "gimbal2", "gimbal3", "gimbal4", "joint1", "joint2", "joint3", "rotor1", "rotor2", "rotor3", "rotor4"]

        # For visualizing the target state in rviz
        self.target_state_tf_bc = tf2_ros.StaticTransformBroadcaster()  # TF broadcaster for visualizing the target state in rviz
        self.target_state_tf = geometry_msgs.msg.TransformStamped()
        self.target_state_tf.header.frame_id = "world"
        self.target_state_tf.child_frame_id = "target_state/root"
        self.target_joint_vis_state = JointState()
        self.target_joint_vis_state.name = joint_vis_names

        # For visualizing the real-time desired robot's motion in rviz
        self.des_state_tf_bc = tf2_ros.TransformBroadcaster()  # TF broadcaster for visualizing the desired robot's motion in rviz
        self.des_state_tf = geometry_msgs.msg.TransformStamped()
        self.des_state_tf.header.frame_id = "world"
        self.des_state_tf.child_frame_id = "des_state/root"
        self.des_joint_vis_state = JointState()
        self.des_joint_vis_state.name = joint_vis_names

        planner_config = self.get_planner_config()

        self.planner_config = planner_config

        # Parameters
        self.joint_num = self.robot.param['joint_num']
        self.propeller_R = self.robot.param['propeller_R']
        self.cmd_hz = planner_config['cmd_hz']
        astar_safe_dis = planner_config['dis2obs_min'] + self.propeller_R + planner_config['astar_extra_clearance']
        self.target_reach_threshold = planner_config['target_reach_threshold']

        # Flag
        self.rootlink_state_received = False
        self.joint_state_received = False
        self.target_state_received = False
        self.target_state_reached = False
        self.planning_finished = False
        self.target_pos_z_set = False
        self.should_shutdown = False

        # Initialize position tracking variables
        self.rootlink_pos_z_realtime = 0.0
        self.rootlink_pos_z = 0.6

        # Initialize planning logger
        self.planning_logger = PlanningLogger(mission_timeout=planner_config['mission_timeout'])
        self.planning_initial_state = None  # Capture the exact initial state used for planning
        self.current_opt_logs = []  # Store optimization logs from latest planning

        # map server, planner, and robot model
        self.map = ESDF(safe_dis=astar_safe_dis)  # this safe_dis is only used in A* planner
        self.planner = GlobalPlanner(robot_urdf, planner_config)
        self.gimbal_yaw_planner = GimbalYawPlanner(robot_urdf, planner_config)

        # Subscribers
        self.fc_pose_sub = rospy.Subscriber('uav/baselink/odom', Odometry, self.fc_pose_cb)
        self.joint_state_sub = rospy.Subscriber('joint_states', JointState, self.joint_state_cb)
        self.map_sub = rospy.Subscriber('/projected_map', OccupancyGrid, self.map.occupancy_map_cb)

        # Unified target state subscriber based on goal completion mode
        if planner_config['is_goal_complete']:
            self.target_state_sub = rospy.Subscriber('target_state', HydrusConfig, self.target_state_cb)
        else:
            self.target_state_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.target_state_cb)

        # Publishers
        # For real tracking
        self.uav_nav_pub = rospy.Publisher('uav/nav', FlightNav, queue_size=10)
        self.joint_des_state_pub = rospy.Publisher('joints_ctrl', JointState, queue_size=10)
        if self.planner_config['replace_gimbal_planner']:
            self.gimbal_state_pub = rospy.Publisher('gimbals_ctrl', JointState, queue_size=10)
        # For visualization
        self.target_joint_vis_state_pub = rospy.Publisher('target_joint_vis_state', JointState, queue_size=10)
        self.des_joint_vis_state_pub = rospy.Publisher('des_joint_vis_state', JointState, queue_size=10)
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

        rospy.loginfo(f"Motion planner initialized!")

        # Register cleanup function for graceful shutdown
        rospy.on_shutdown(self.cleanup)

    def get_planner_config(self):
        planner_config = {}

        # Only used in the motion_planner_node
        planner_config['mission_timeout'] = rospy.get_param("~mission_timeout", 300.0)
        planner_config['is_goal_complete'] = rospy.get_param("~is_goal_complete", True)
        planner_config['replace_gimbal_planner'] = rospy.get_param("~replace_gimbal_planner", False)
        planner_config['auto_shutdown'] = rospy.get_param("~auto_shutdown", False)

        # Computation config
        planner_config['run_traj_opt'] = rospy.get_param("~run_traj_opt", True)
        planner_config['find_anchor'] = rospy.get_param("~find_anchor", False)
        planner_config['is_parallel'] = rospy.get_param("~is_parallel", True) and planner_config['find_anchor']

        # Motion planner node
        planner_config['cmd_hz'] = rospy.get_param("~cmd_hz", 60)
        planner_config['target_reach_threshold'] = rospy.get_param("~target_reach_threshold", 0.2)

        # Global planner
        planner_config['astar_resolution'] = float(rospy.get_param("~astar_resolution", 0.1))
        planner_config['astar_extra_clearance'] = rospy.get_param("~astar_extra_clearance", 0.10)  # extra clearance for A* planner
        planner_config['seg_len'] = self.robot.param['link_L'] * rospy.get_param("~seg_len_factor", 1.0)
        planner_config['travel_speed'] = rospy.get_param("~travel_speed", 0.1)  # average speed of the rootlink
        planner_config['bend_attempt_num'] = int(rospy.get_param("~bend_attempt_num", 5))
        planner_config['transform_attempt_num'] = int(rospy.get_param("~transform_attempt_num", 20))
        planner_config['candidate_int_eva_num'] = int(rospy.get_param("~candidate_int_eva_num", 10))
        planner_config['rotate_first'] = rospy.get_param("~rotate_first", True)  # whether to rotate first before bending
        planner_config['one_way_fold'] = rospy.get_param("~one_way_fold", False)

        # Constraints on the motion
        planner_config['min_joint_angle'] = rospy.get_param("~min_joint_angle", 0.5)
        planner_config['joint_angle_bound'] = rospy.get_param("~joint_angle_bound", np.pi / 2)
        planner_config['translational_vel_limit'] = rospy.get_param("~translational_vel_limit", 2.0)
        planner_config['rotational_vel_limit'] = rospy.get_param("~rotational_vel_limit", 1.0)
        planner_config['dis2obs_min'] = rospy.get_param("~dis2obs_min", 0.05)  # minimum allowed distance between robot and obstacles
        planner_config['fc_rp_dis_min'] = self.robot.param['link_L'] * rospy.get_param("~fc_threshold_factor", 0.05)
        planner_config['fc_t_min_thre'] = rospy.get_param("~fc_t_min_thre", 0.3)

        # BSpline planner and recovery parameters
        planner_config['stage_control_points_num'] = int(rospy.get_param("~stage_control_points_num", 5))
        planner_config['sample_density'] = float(rospy.get_param("~sample_density", 100))
        planner_config['collision_cost_tolerance'] = rospy.get_param("~collision_cost_tolerance", 1.0)
        planner_config['stability_violation_tolerance'] = float(rospy.get_param("~stability_violation_tolerance", 5.0))
        planner_config['opt_max_retries'] = int(rospy.get_param("~opt_max_retries", 5))
        planner_config['traj_fc_t_min_thre'] = float(rospy.get_param("~traj_fc_t_min_thre", 0.1))
        planner_config['save_traj'] = rospy.get_param("~save_traj", False)

        # Optimizer
        planner_config['ftol_rel'] = float(rospy.get_param("~ftol_rel", 1e-4))
        planner_config['maxtime'] = rospy.get_param("~maxtime", 120)
        planner_config['constraint_tol'] = float(rospy.get_param("~constraint_tol", 1e-4))
        planner_config['esdf_constraint_weight'] = float(rospy.get_param("~esdf_constraint_weight", 1e3))
        planner_config['vel_constraint_weight'] = float(rospy.get_param("~vel_constraint_weight", 1))
        planner_config['stability_constraint_weight'] = float(rospy.get_param("~stability_constraint_weight", 0))

        return planner_config

    def fc_pose_cb(self, data: Odometry):
        # Skip processing if shutdown is requested
        if self.should_shutdown:
            return

        # Extract position and orientation from the Odometry message
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
            self.robot_state.joint_angles)

        # Update robot state
        self.robot_state.rootlink_pos_x = rootlink_state[0]
        self.robot_state.rootlink_pos_y = rootlink_state[1]
        self.rootlink_pos_z_realtime = rootlink_state[2]
        self.robot_state.rootlink_yaw = rootlink_state[3]
        self.rootlink_state_received = True

        # Set target Z position once planning is finished
        if self.planning_finished and not self.target_pos_z_set:
            self.rootlink_pos_z = self.rootlink_pos_z_realtime
            self.target_pos_z_set = True

        # Check if target state is reached
        if not self.target_state_reached and self.reach_target_state():
            rospy.loginfo("Target state reached!")
            self.target_state_reached = True

            # Update the log entry with successful result
            # The planning logger will handle cases where the log file doesn't exist yet
            if self.planning_logger.current_planning_timestamp is not None:
                self.planning_logger.update_planning_result(mission_success=True)

            # Shutdown the node after target is reached (if auto_shutdown is enabled)
            if self.planner_config['auto_shutdown']:
                self.shutdown_node("Target state reached successfully")
            else:
                rospy.loginfo("Target state reached! (auto_shutdown disabled, keeping node running)")

    def joint_state_cb(self, data):
        # Skip processing if shutdown is requested
        if self.should_shutdown:
            return

        self.joint_state = data
        self.robot_state.joint_angles = self.joint_state.position[4:]
        self.gimbal_angles = self.joint_state.position[:4]  # first 4 joints are gimbal angles
        self.joint_state_received = True

    def _planning_timeout_handler(self, event):
        """Handle planning timeout event

        Args:
            event: ROS timer event
        """
        if not self.planning_logger.current_planning_logged:
            rospy.logerr(f"Planning timeout after {self.planning_logger.mission_timeout} seconds! Marking as failed.")
            computation_time = time.time() - self.planning_logger.current_planning_start_time
            self.planning_logger.log_planning_result(
                self.planning_initial_state if self.planning_initial_state else self.robot_state,
                self.target_state,
                self.planner_config,
                False,  # plan_success = False (timeout)
                computation_time,
                self.current_opt_logs,
                False,  # mission_success = False (failed to plan)
                None,   # root_path_length = None (no valid path)
                None    # generalized_path_length = None (no valid path)
            )
        self.planning_logger.planning_timer = None
        # Shutdown node after timeout (if auto_shutdown is enabled)
        if self.planner_config['auto_shutdown']:
            self.shutdown_node("Planning timeout exceeded")
        else:
            rospy.loginfo("Planning timeout exceeded (auto_shutdown disabled, keeping node running)")

    def target_state_cb(self, data):
        # Skip processing if shutdown is requested
        if self.should_shutdown:
            return

        if self.planner_config['is_goal_complete']:
            # Complete goal mode: data is HydrusConfig, create single candidate
            candidate_target_states = self.generate_complete_target_state(data)
            mode_description = "complete"
        else:
            # Simple goal mode: data is PoseStamped, generate multiple candidates
            candidate_target_states = self.generate_candidate_target_states(data)
            mode_description = "simple"

        self.planning_logger.start_planning_session()
        is_plan_successful = False

        # Start planning timeout timer with custom callback
        self.planning_logger.start_mission_timeout(self._planning_timeout_handler)

        # Try each candidate state, but only log once at the end
        for candidate_state in candidate_target_states:
            self.target_state = candidate_state
            self.target_state_received = True
            print("\n")
            rospy.loginfo(f"Target state [{mode_description} mode]: pos=({candidate_state.rootlink_pos_x:.2f}, {candidate_state.rootlink_pos_y:.2f}), "
                          f"yaw={np.degrees(candidate_state.rootlink_yaw):.0f}°, "
                          f"joints=[{', '.join([f'{np.degrees(angle):.0f}°' for angle in candidate_state.joint_angles])}]")
            print("\n")
            self.vis_target_state()

            is_plan_successful = self.traj_plan()
            if is_plan_successful:
                break

        # Cancel planning timeout timer since planning completed
        self.planning_logger.cancel_mission_timeout()

        # Log the final result for both modes
        if not self.planning_logger.current_planning_logged:
            computation_time = time.time() - self.planning_logger.current_planning_start_time
            # Calculate path lengths if planning was successful
            root_path_length, generalized_path_length = None, None
            if is_plan_successful and self.des_states is not None:
                root_path_length, generalized_path_length = PlanningLogger.calculate_path_length(self.des_states, self.joint_num)

            self.planning_logger.log_planning_result(
                self.planning_initial_state if self.planning_initial_state else self.robot_state,
                self.target_state,
                self.planner_config,
                is_plan_successful,  # plan_success
                computation_time,
                self.current_opt_logs,
                None,  # mission_success = None (mission not yet started/completed)
                root_path_length,  # root_path_length
                generalized_path_length  # generalized_path_length
            )

        # Shutdown the node if planning failed (if auto_shutdown is enabled)
        if not is_plan_successful and self.planner_config['auto_shutdown']:
            self.shutdown_node("Planning failed - unable to find valid trajectory")
        elif not is_plan_successful:
            rospy.loginfo("Planning failed - unable to find valid trajectory (auto_shutdown disabled, keeping node running)")

    def generate_complete_target_state(self, data):
        """Generate single candidate target state for complete goal mode"""
        target_state = data

        # Ensure the shortest shift from current rootlink yaw to target rootlink yaw
        if self.rootlink_state_received:
            target_state.rootlink_yaw = normalize_target_angle(self.robot_state.rootlink_yaw, data.rootlink_yaw)

        return [target_state]

    def generate_candidate_target_states(self, data):
        rootlink_target_pos_x = data.pose.position.x
        rootlink_target_pos_y = data.pose.position.y
        rootlink_target_yaw = tf.transformations.euler_from_quaternion([
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        ])[2]  # Extract yaw (rotation around z-axis)

        # Ensure the shortest shift from current rootlink yaw to target rootlink yaw
        if self.rootlink_state_received:
            opt_target_yaw = normalize_target_angle(self.robot_state.rootlink_yaw, rootlink_target_yaw)
            bending_dir = 1 if np.sum(self.robot_state.joint_angles) >= 0 else -1

        bound = self.planner_config['joint_angle_bound']
        # min_angle = self.planner_config['min_joint_angle']

        # positive_angles = np.linspace(bending_dir * bound, bending_dir * min_angle, 5)
        # negative_angles = np.linspace(-bending_dir * min_angle, -bending_dir * bound, 5)

        # joint_angle_candidates = np.concatenate((positive_angles, negative_angles))

        # Create candidate target states
        # candidate_target_states = []
        # for joint_angle in joint_angle_candidates:
        #     candidate_state = HydrusConfig()
        #     candidate_state.rootlink_pos_x = rootlink_target_pos_x
        #     candidate_state.rootlink_pos_y = rootlink_target_pos_y
        #     candidate_state.rootlink_yaw = opt_target_yaw
        #     candidate_state.joint_angles = [joint_angle] * self.joint_num
        #     candidate_target_states.append(candidate_state)
        candidate_target_states = []
        candidate_state = HydrusConfig()
        candidate_state.rootlink_pos_x = rootlink_target_pos_x
        candidate_state.rootlink_pos_y = rootlink_target_pos_y
        candidate_state.rootlink_yaw = opt_target_yaw
        candidate_state.joint_angles = [bending_dir * bound] * self.joint_num
        candidate_target_states.append(candidate_state)

        return candidate_target_states

    def reach_target_state(self):
        if self.rootlink_state_received and self.joint_state_received and self.target_state_received:
            current_state = np.array([self.robot_state.rootlink_pos_x,
                                      self.robot_state.rootlink_pos_y,
                                      self.robot_state.rootlink_yaw]
                                     + list(self.robot_state.joint_angles))
            target_state = np.array([self.target_state.rootlink_pos_x,
                                     self.target_state.rootlink_pos_y,
                                     self.target_state.rootlink_yaw]
                                    + list(self.target_state.joint_angles))
            return np.linalg.norm(current_state - target_state) < self.target_reach_threshold
        else:
            return False

    def traj_plan(self):
        self.target_state_reached = False

        if self.reach_target_state():
            rospy.loginfo("Target state already reached!")
            self.target_state_reached = True
            return True

        while not self.rootlink_state_received or not self.joint_state_received:
            time.sleep(0.1)

        # Capture the exact initial state used for planning
        self.planning_initial_state = copy.deepcopy(self.robot_state)

        self.des_states, key_states_viz, opt_logs = self.planner.plan(self.map, self.robot_state, self.target_state)

        # Store optimization logs for later use
        self.current_opt_logs = opt_logs if opt_logs else []

        if key_states_viz is not None:
            self.pub_key_states_viz(key_states_viz)

        if self.des_states is not None:
            self.planning_finished = True
            print("\n")
            rospy.loginfo(f"Planning finished")

            if self.planner_config['replace_gimbal_planner']:
                # Extract des_joint_angles array from planned trajectory
                des_joint_angles_array = []
                for state in self.des_states:
                    joint_angles = [state['joints'][i].pos for i in range(self.joint_num)]
                    des_joint_angles_array.append(joint_angles)
                des_joint_angles_array = np.array(des_joint_angles_array)

                rospy.loginfo("Planning gimbal yaw trajectory...")
                trajectory_time = len(self.des_states) / self.cmd_hz

                des_gimbal_angles_array, objective_history = self.gimbal_yaw_planner.plan(
                    des_joint_angles_array,
                    trajectory_time,
                    self.gimbal_angles
                )

                if des_gimbal_angles_array is not None:
                    rospy.loginfo(f"Gimbal yaw planning completed. Shape: {des_gimbal_angles_array.shape}")
                    rospy.loginfo(f"Objective history length: {len(objective_history) if objective_history is not None else 0}")
                    # Store for future processing
                    self.des_gimbal_angles_array = des_gimbal_angles_array
                    self.rotor_num = self.des_gimbal_angles_array.shape[1]

                    # Log the minimum objective value from gimbal planning
                    if objective_history is not None and len(objective_history) > 0:
                        min_objective = np.min(objective_history)
                        rospy.loginfo(f"Gimbal planning minimum objective value: {min_objective:.6f}")
                else:
                    return False

            self.des_state_index = int(0)
            self.des_state_len = len(self.des_states)

            # if there is one tracking_cmd_timer already, delete it
            if hasattr(self, 'tracking_cmd_timer'):
                self.tracking_cmd_timer.shutdown()

            self.tracking_cmd_timer = rospy.Timer(rospy.Duration(1 / self.cmd_hz), self.tracking_cmd_timer_cb)

            return True
        else:
            # Planning failed
            return False

    def tracking_cmd_timer_cb(self, event):
        # Skip processing if shutdown is requested
        if self.should_shutdown:
            return

        if self.des_state_index >= self.des_state_len:
            rospy.loginfo("Trajectory completed, stopping tracking timer")
            self.tracking_cmd_timer.shutdown()
            return

        self.pub_baselink_state_cmd()
        self.pub_joint_state_cmd()
        self.send_des_state_vis_tf()
        self.pub_joint_vis_state()
        if self.planner_config['replace_gimbal_planner']:
            self.pub_gimbal_state_cmd()

        self.des_state_index += 1

    def pub_baselink_state_cmd(self):
        self.baselink_state_cmd.header.stamp = rospy.Time.now()

        # Position commands using COG (Center of Gravity)
        self.baselink_state_cmd.target_pos_x = self.des_states[self.des_state_index]['cog'].pos_x
        self.baselink_state_cmd.target_pos_y = self.des_states[self.des_state_index]['cog'].pos_y
        self.baselink_state_cmd.target_pos_z = self.rootlink_pos_z
        self.baselink_state_cmd.target_yaw = self.des_states[self.des_state_index]['cog'].yaw

        # Velocity commands using COG (Center of Gravity)
        self.baselink_state_cmd.target_vel_x = self.des_states[self.des_state_index]['cog'].vel_x
        self.baselink_state_cmd.target_vel_y = self.des_states[self.des_state_index]['cog'].vel_y
        self.baselink_state_cmd.target_vel_z = 0
        self.baselink_state_cmd.target_omega_z = self.des_states[self.des_state_index]['cog'].omega_z

        self.uav_nav_pub.publish(self.baselink_state_cmd)

    def pub_joint_state_cmd(self):
        self.joint_state_cmd.header.stamp = rospy.Time.now()
        self.joint_state_cmd.position = [self.des_states[self.des_state_index]['joints'][i].pos
                                         for i in range(self.joint_num)]
        self.joint_state_cmd.velocity = [self.des_states[self.des_state_index]['joints'][i].vel
                                         for i in range(self.joint_num)]

        self.joint_des_state_pub.publish(self.joint_state_cmd)

    def pub_gimbal_state_cmd(self):
        self.gimbal_des_state.header.stamp = rospy.Time.now()
        self.gimbal_des_state.position = [self.des_gimbal_angles_array[self.des_state_index][i]
                                          for i in range(self.rotor_num)]
        self.gimbal_des_state.velocity = [0] * self.rotor_num

        self.gimbal_state_pub.publish(self.gimbal_des_state)

    def vis_target_state(self):
        self.send_target_state_vis_tf()
        self.pub_target_joint_state()

    def send_target_state_vis_tf(self):
        self.target_state_tf.header.stamp = rospy.Time.now()
        self.target_state_tf.transform.translation.x = self.target_state.rootlink_pos_x
        self.target_state_tf.transform.translation.y = self.target_state.rootlink_pos_y
        self.target_state_tf.transform.translation.z = self.rootlink_pos_z_realtime

        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.target_state.rootlink_yaw)

        self.target_state_tf.transform.rotation.x = quaternion[0]
        self.target_state_tf.transform.rotation.y = quaternion[1]
        self.target_state_tf.transform.rotation.z = quaternion[2]
        self.target_state_tf.transform.rotation.w = quaternion[3]

        self.target_state_tf_bc.sendTransform(self.target_state_tf)

    def pub_target_joint_state(self):
        self.target_joint_vis_state.header.stamp = rospy.Time.now()
        # gimbal angles (4) + joint angles (3) + rotor angles (4)
        self.target_joint_vis_state.position = [0, 0, 0, 0] + list(self.target_state.joint_angles) + [0, 0, 0, 0]
        self.target_joint_vis_state.velocity = [0] * 11
        self.target_joint_vis_state_pub.publish(self.target_joint_vis_state)

    def send_des_state_vis_tf(self):
        self.des_state_tf.header.stamp = rospy.Time.now()
        self.des_state_tf.transform.translation.x = self.des_states[self.des_state_index]['rootlink'].pos_x
        self.des_state_tf.transform.translation.y = self.des_states[self.des_state_index]['rootlink'].pos_y
        self.des_state_tf.transform.translation.z = self.rootlink_pos_z

        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.des_states[self.des_state_index]['rootlink'].yaw)

        self.des_state_tf.transform.rotation.x = quaternion[0]
        self.des_state_tf.transform.rotation.y = quaternion[1]
        self.des_state_tf.transform.rotation.z = quaternion[2]
        self.des_state_tf.transform.rotation.w = quaternion[3]

        self.des_state_tf_bc.sendTransform(self.des_state_tf)

    def pub_joint_vis_state(self):
        self.des_joint_vis_state.header.stamp = rospy.Time.now()
        joint_positions = [self.des_states[self.des_state_index]['joints'][i].pos for i in range(self.joint_num)]
        joint_velocities = [self.des_states[self.des_state_index]['joints'][i].vel for i in range(self.joint_num)]

        # gimbal angles (4) + joint angles (3) + rotor angles (4)
        self.des_joint_vis_state.position = [0, 0, 0, 0] + joint_positions + [0, 0, 0, 0]
        self.des_joint_vis_state.velocity = [0, 0, 0, 0] + joint_velocities + [0, 0, 0, 0]

        self.des_joint_vis_state_pub.publish(self.des_joint_vis_state)

    def pub_key_states_viz(self, key_states_viz):
        """
        Publish visualization markers for key states in the planned trajectory.

        Args:
            key_states_viz: Tuple containing (all_link_joint_list, all_rotor_pos_list)
        """
        marker_array = self.visualizer.create_key_states_marker_array(
            key_states_viz,
            self.rootlink_pos_z_realtime
        )
        self.marker_pub.publish(marker_array)

    def cleanup(self):
        """Cleanup function called on node shutdown"""
        rospy.loginfo("Motion planner shutting down...")

        # Cleanup planning logger
        self.planning_logger.cleanup()
        
        # Give extra time for all log writes to complete
        rospy.loginfo("Waiting for all log writes to complete...")
        rospy.sleep(1.0)

        if hasattr(self, 'tracking_cmd_timer'):
            self.tracking_cmd_timer.shutdown()

    def shutdown_node(self, reason):
        """Gracefully shutdown the node with logging"""
        if not self.should_shutdown:
            self.should_shutdown = True
            rospy.loginfo(f"Shutting down motion planner: {reason}")

            # Stop any active timers immediately
            if hasattr(self, 'tracking_cmd_timer'):
                self.tracking_cmd_timer.shutdown()

            # Cancel planning timeout timer if it exists
            if hasattr(self.planning_logger, 'planning_timer') and self.planning_logger.planning_timer is not None:
                self.planning_logger.cancel_mission_timeout()

            # Give a brief moment for cleanup
            time.sleep(0.1)

            # Signal ROS to shutdown the entire system
            rospy.loginfo("Initiating full ROS system shutdown...")
            try:
                # Kill all ROS processes including rosmaster
                subprocess.call(['pkill', '-f', 'ros'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                subprocess.call(['pkill', '-f', 'gazebo'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                subprocess.call(['pkill', '-f', 'rviz'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except:
                pass

            # Signal ROS to shutdown
            rospy.signal_shutdown(reason)

            # Wait for ROS shutdown to complete
            while not rospy.is_shutdown():
                time.sleep(0.01)


if __name__ == "__main__":

    motion_planner = MotionPlanner()

    rospy.spin()
