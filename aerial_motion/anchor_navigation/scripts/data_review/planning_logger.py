#!/usr/bin/env python3
"""
Planning Logger Class for Motion Planner

This module provides a standalone logging system for motion planning results.
Extracted from the motion_planner_node to improve modularity and maintainability.
"""

import os
import time
import json
import numpy as np
import rospy
from datetime import datetime


class PlanningLogger:
    """
    A class to handle all planning result logging functionality.

    This class manages:
    - Log file initialization and setup
    - Planning result logging
    - Result updates
    - Planning timeout handling
    """

    @staticmethod
    def calculate_path_length(des_states, joint_num):
        """
        Calculate path length metrics from a trajectory.
        
        Computes two metrics:
        1. Root path length: Sum of Euclidean distances in 2D space (rootlink x, y only)
        2. Generalized path length: Sum of norms in full configuration space 
           (rootlink x, y, yaw, and all joint angles)
        
        Args:
            des_states (list): List of desired states, where each state is a dict
                             with 'rootlink' and 'joints' keys
            joint_num (int): Number of joints in the robot
        
        Returns:
            tuple: (root_path_length, generalized_path_length) or (None, None) if des_states is None/empty
        """
        if des_states is None or len(des_states) < 2:
            return None, None
        
        root_path_length = 0.0
        generalized_path_length = 0.0
        
        for i in range(1, len(des_states)):
            # Extract previous state
            prev_root_pos = np.array([
                des_states[i-1]['rootlink'].pos_x,
                des_states[i-1]['rootlink'].pos_y
            ])
            prev_config = [
                des_states[i-1]['rootlink'].pos_x,
                des_states[i-1]['rootlink'].pos_y,
                des_states[i-1]['rootlink'].yaw
            ] + [des_states[i-1]['joints'][j].pos for j in range(joint_num)]
            
            # Extract current state
            curr_root_pos = np.array([
                des_states[i]['rootlink'].pos_x,
                des_states[i]['rootlink'].pos_y
            ])
            curr_config = [
                des_states[i]['rootlink'].pos_x,
                des_states[i]['rootlink'].pos_y,
                des_states[i]['rootlink'].yaw
            ] + [des_states[i]['joints'][j].pos for j in range(joint_num)]
            
            # Calculate root path length (2D Euclidean distance)
            root_path_length += np.linalg.norm(curr_root_pos - prev_root_pos)
            
            # Calculate generalized path length (full configuration space)
            generalized_path_length += np.linalg.norm(np.array(curr_config) - np.array(prev_config))
        
        return root_path_length, generalized_path_length

    def __init__(self, log_dir=None, mission_timeout=300.0):
        """
        Initialize the planning logger.

        Args:
            log_dir (str, optional): Directory for log files. If None, uses default location.
            mission_timeout (float): Planning timeout in seconds. Default is 300.0.
        """
        # Planning log variables
        self.current_planning_start_time = None
        self.current_planning_timestamp = None
        self.current_planning_computation_time = None
        self.current_planning_logged = False  # Track if current planning has been logged
        self.pending_mission_success = None  # Store mission success if set before initial log
        self.mission_timeout = mission_timeout
        self.planning_timer = None  # Timer for planning timeout

        # Set up log file path
        if log_dir is None:
            # Default log directory relative to motion_planner package
            current_dir = os.path.dirname(os.path.dirname(__file__))
            log_dir = os.path.join(os.path.dirname(current_dir), "logs")

        self.log_dir = log_dir
        self.current_log_file_path = None  # Path to current planning session log file

        # Initialize planning log directory
        self.setup_planning_log_directory()

    def setup_planning_log_directory(self):
        """Initialize the planning log directory"""
        # Create logs directory if it doesn't exist
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            rospy.loginfo(f"Planning log directory created: {self.log_dir}")
        else:
            rospy.loginfo(f"Using existing planning log directory: {self.log_dir}")

    def _generate_log_filename(self, timestamp_str):
        """
        Generate a unique log filename based on timestamp

        Args:
            timestamp_str (str): Timestamp string in format 'YYYY-MM-DD HH:MM:SS'

        Returns:
            str: Generated filename
        """
        # Convert timestamp to filename-safe format (YYYYMMDD_HHMMSS)
        try:
            dt = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S.%f')
        except ValueError:
            dt = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S')

        filename = f"planning_session_{dt.strftime('%Y%m%d_%H%M%S')}.json"
        return filename

    def start_planning_session(self):
        """
        Start a new planning session.

        Returns:
            str: The timestamp for this planning session
        """
        self.current_planning_timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self.current_planning_start_time = time.time()
        self.current_planning_logged = False
        self.pending_mission_success = None  # Reset pending mission success

        # Generate unique log file path for this session
        log_filename = self._generate_log_filename(self.current_planning_timestamp)
        self.current_log_file_path = os.path.join(self.log_dir, log_filename)

        return self.current_planning_timestamp

    def log_planning_result(self, robot_state, target_state, planner_config, plan_success, computation_time, opt_logs=None, mission_success=None, root_path_length=None, generalized_path_length=None):
        """
        Log planning result to JSON file

        Args:
            robot_state: Robot's initial state (should have rootlink_pos_x, rootlink_pos_y, 
                        rootlink_yaw, joint_angles attributes)
            target_state: Target state (should have rootlink_pos_x, rootlink_pos_y, 
                         rootlink_yaw, joint_angles attributes)
            planner_config (dict): Planner configuration containing is_goal_complete, 
                                 find_anchor, is_parallel, run_traj_opt, 
                                 sample_density, transform_attempt_num
            plan_success (bool): Planning result (True for successful plan generation, False for failure)
            computation_time (float): Computation time in seconds
            opt_logs (list, optional): List of OptLog objects containing optimization iteration data
            mission_success (bool, optional): Mission execution result (True if robot reaches target, 
                                            False if tracking errors prevent target achievement, 
                                            None if mission is still ongoing)
            root_path_length (float, optional): Path length in 2D rootlink space (x, y only)
            generalized_path_length (float, optional): Path length in full configuration space
        """
        try:
            # Use pending mission success if mission_success parameter is None
            final_mission_success = mission_success if mission_success is not None else self.pending_mission_success
            
            # Create comprehensive log data structure
            log_data = {
                "session_info": {
                    "timestamp": self.current_planning_timestamp,
                    "world_name": rospy.get_param('/world_name', 'unknown'),
                    "computation_time_sec": round(computation_time, 3),
                    "plan_success": plan_success,
                    "mission_success": final_mission_success,
                    "root_path_length": round(root_path_length, 3) if root_path_length is not None else None,
                    "generalized_path_length": round(generalized_path_length, 3) if generalized_path_length is not None else None
                },
                "initial_state": {
                    "rootlink_position": {
                        "x": round(robot_state.rootlink_pos_x, 3),
                        "y": round(robot_state.rootlink_pos_y, 3)
                    },
                    "rootlink_yaw": round(robot_state.rootlink_yaw, 3),
                    "joint_angles": [round(angle, 3) for angle in robot_state.joint_angles]
                },
                "target_state": {
                    "rootlink_position": {
                        "x": round(target_state.rootlink_pos_x, 3),
                        "y": round(target_state.rootlink_pos_y, 3)
                    },
                    "rootlink_yaw": round(target_state.rootlink_yaw, 3),
                    "joint_angles": [round(angle, 3) for angle in target_state.joint_angles]
                },
                "planner_config": {
                    "is_goal_complete": planner_config['is_goal_complete'],
                    "find_anchor": planner_config['find_anchor'],
                    "is_parallel": planner_config['is_parallel'],
                    "run_traj_opt": planner_config['run_traj_opt'],
                    "sample_density": planner_config['sample_density'],
                    "transform_attempt_num": planner_config['transform_attempt_num']
                }
            }

            # Add optimization logs if provided
            if opt_logs:
                log_data["optimization_logs"] = []
                for i, opt_log in enumerate(opt_logs):
                    segment_log = {
                        "segment_id": i,
                        "iterations": opt_log.iterations,
                        "objective_history": [round(val, 6) for val in opt_log.obj_history]
                    }
                    log_data["optimization_logs"].append(segment_log)

                # Add summary statistics
                total_iterations = sum(opt_log.iterations for opt_log in opt_logs)
                log_data["optimization_summary"] = {
                    "total_segments": len(opt_logs),
                    "total_iterations": total_iterations,
                    "avg_iterations_per_segment": round(total_iterations / len(opt_logs), 2)
                }

            # Write to JSON file
            with open(self.current_log_file_path, 'w') as f:
                json.dump(log_data, f, indent=2)
                # Ensure data is written to disk immediately
                f.flush()
                os.fsync(f.fileno())

            self.current_planning_logged = True
            self.current_planning_computation_time = computation_time
            
            # Clear pending mission success since it's now been logged
            if self.pending_mission_success is not None:
                rospy.loginfo(f"Used pending mission success value: {self.pending_mission_success}")
                self.pending_mission_success = None

            rospy.loginfo(f"Planning result logged to: {self.current_log_file_path}")
            if opt_logs:
                rospy.loginfo(f"Optimization data: {len(opt_logs)} segments, {total_iterations} total iterations")
            if root_path_length is not None and generalized_path_length is not None:
                rospy.loginfo(f"Root path length: {root_path_length:.3f}, Generalized path length: {generalized_path_length:.3f}")

        except Exception as e:
            rospy.logerr(f"Failed to write to planning log: {e}")

    def update_planning_result(self, plan_success=None, mission_success=None):
        """
        Update the result of the current planning session

        Args:
            plan_success (bool, optional): New plan success value to update
            mission_success (bool, optional): New mission success value to update
        """
        try:
            if self.current_log_file_path and os.path.exists(self.current_log_file_path):
                # Read existing log data
                with open(self.current_log_file_path, 'r') as f:
                    log_data = json.load(f)

                # Update the results
                if plan_success is not None:
                    log_data['session_info']['plan_success'] = plan_success
                if mission_success is not None:
                    log_data['session_info']['mission_success'] = mission_success

                # Write back to file
                with open(self.current_log_file_path, 'w') as f:
                    json.dump(log_data, f, indent=2)

                rospy.loginfo(f"Planning result updated in: {self.current_log_file_path}")
                if plan_success is not None:
                    rospy.loginfo(f"  Plan success: {plan_success}")
                if mission_success is not None:
                    rospy.loginfo(f"  Mission success: {mission_success}")
            else:
                # Log file doesn't exist yet, store mission success for later use
                if mission_success is not None:
                    self.pending_mission_success = mission_success
                    rospy.loginfo(f"Log file not ready yet, storing mission success ({mission_success}) for later logging")
                else:
                    rospy.logwarn("No current log file to update")

        except Exception as e:
            rospy.logerr(f"Failed to update planning log: {e}")

    def start_mission_timeout(self, timeout_callback=None):
        """
        Start a timer for planning timeout

        Args:
            timeout_callback (callable, optional): Custom callback function for timeout.
                                                 If None, uses default behavior.
        """
        if self.planning_timer is not None:
            self.planning_timer.shutdown()

        if timeout_callback is None:
            timeout_callback = self._default_mission_timeout_callback

        self.planning_timer = rospy.Timer(
            rospy.Duration(self.mission_timeout),
            timeout_callback,
            oneshot=True
        )

    def cancel_mission_timeout(self):
        """Cancel the planning timeout timer"""
        if self.planning_timer is not None:
            self.planning_timer.shutdown()
            self.planning_timer = None

    def _default_mission_timeout_callback(self, event):
        """Default callback for planning timeout - log failed result"""
        if not self.current_planning_logged:
            rospy.logerr(f"Planning timeout after {self.mission_timeout} seconds! Marking as failed.")
            # Note: This default callback can't log the result because it doesn't have access
            # to robot_state, target_state, and planner_config.
            # Users should provide their own timeout callback that calls log_planning_result.
            self.current_planning_logged = True

        self.planning_timer = None

    def get_planning_session_info(self):
        """
        Get information about the current planning session

        Returns:
            dict: Dictionary containing current session information
        """
        return {
            'timestamp': self.current_planning_timestamp,
            'start_time': self.current_planning_start_time,
            'computation_time': self.current_planning_computation_time,
            'logged': self.current_planning_logged,
            'log_file_path': self.current_log_file_path,
            'log_directory': self.log_dir
        }

    def cleanup(self):
        """Cleanup function called on shutdown"""
        # Cancel planning timeout timer if running
        self.cancel_mission_timeout()
        
        # Give time for any pending file writes to complete
        rospy.loginfo("Waiting for log file operations to complete...")
        rospy.sleep(0.5)
        
        rospy.loginfo("Planning logger cleaned up.")

    def is_current_planning_logged(self):
        """
        Check if the current planning session has been logged

        Returns:
            bool: True if current planning has been logged, False otherwise
        """
        return self.current_planning_logged

    def get_log_file_path(self):
        """
        Get the path to the current log file

        Returns:
            str: Path to the current session's log file, or log directory if no active session
        """
        return self.current_log_file_path if self.current_log_file_path else self.log_dir

    def get_all_log_files(self):
        """
        Get a list of all planning log files in the log directory

        Returns:
            list: List of log file paths
        """
        try:
            log_files = []
            if os.path.exists(self.log_dir):
                for filename in os.listdir(self.log_dir):
                    if filename.startswith('planning_session_') and filename.endswith('.json'):
                        log_files.append(os.path.join(self.log_dir, filename))
            return sorted(log_files)
        except Exception as e:
            rospy.logerr(f"Failed to list log files: {e}")
            return []
