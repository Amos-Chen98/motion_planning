import os
import sys
current_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_path)
from robot_model import Robot  # Import the Robot class from robot_model.py
from bspline_planner import BSplinePlanner
from poly_traj import PolyTraj
import numpy as np
import nlopt
import rospy
from robot_model import Robot



class GimbalYawPlanner(PolyTraj):
    def __init__(self, robot_urdf, planner_config):

        if planner_config is None:
            rospy.logwarn("GimbalYawPlanner: planner_config is None, using default values")

        self.ftol_rel = planner_config['ftol_rel']
        self.maxtime = planner_config['maxtime']
        self.constraint_tol = planner_config['constraint_tol']
        self.gimbal_num = planner_config['rotor_num']
        self.cmd_hz = planner_config['cmd_hz']
        self.rotor_directions = planner_config['rotor_directions']
        self.m_f_rate = planner_config['m_f_rate']
        self.rotor_thrust_min = planner_config['rotor_thrust_min']
        self.rotor_thrust_max = planner_config['rotor_thrust_max']
        self.sample_hz = 5
        self.sample_interval = 1.0 / self.sample_hz
        self.constraint_tol_scalar = planner_config['constraint_tol']  # Tolerance for inequality constraints
        self.num_constraints = 1  # Number of inequality constraints
        self.ineq_constraints_tol = [self.constraint_tol_scalar] * self.num_constraints  # Vector of tolerances
        self.fc_t_min_thres = 1.0

        super().__init__(p=3, dim=self.gimbal_num)

        # Create reference configurations with their corresponding gimbal angles
        self.reference_configs = [
            {
                'name': 'square',
                'joint_ref': np.pi/2 * np.ones(self.gimbal_num - 1),
                'gimbal_angles': np.array([np.pi, 0, np.pi, 0])
            },
            {
                'name': 'straight',
                'joint_ref': np.zeros(self.gimbal_num - 1),
                'gimbal_angles': np.array([np.pi/2, -np.pi/2, -np.pi/2, np.pi/2])
            },
            {
                'name': 'inverse_square',
                'joint_ref': -np.pi/2 * np.ones(self.gimbal_num - 1),
                'gimbal_angles': np.array([0, np.pi, 0, np.pi])
            }
        ]

        # Pre-compute reference arrays for efficient vectorized operations
        self.joint_refs = np.array([config['joint_ref'] for config in self.reference_configs])
        self.gimbal_angles_options = np.array([config['gimbal_angles'] for config in self.reference_configs])

        # Keep legacy attributes for backward compatibility
        self.square_ref = self.reference_configs[0]['joint_ref']
        self.square_gimbal_angles = self.reference_configs[0]['gimbal_angles']
        self.straight_ref = self.reference_configs[1]['joint_ref']
        self.straight_gimbal_angles = self.reference_configs[1]['gimbal_angles']
        self.inverse_square_ref = self.reference_configs[2]['joint_ref']
        self.inverse_square_gimbal_angles = self.reference_configs[2]['gimbal_angles']

        # test flag
        self.printed = False

        # Used for Forward kinematics
        self.robot = Robot(robot_urdf)

    def plan(self, des_joint_angles: np.ndarray, time: float) -> np.ndarray:
        """
        Inputs:
            des_joint_angles: np.ndarray of (n, joint_num), each row a desired joint state at a time step
        Outputs:
            des_gimbal_angles_array: np.ndarray of (n, gimbal_num), each row a desired gimbal angle at a time step
        """
        self.time = time
        self.robot_config_array = self._get_robot_config_array(des_joint_angles)
        self.sampled_time = np.arange(0, self.time, self.sample_interval)

        result = self.get_once_gimbal_angles(self.robot_config_array[0, 3:], None)
        if result is not None:
            self.init_gimbal_angles, _ = result  # Unpack tuple (gimbal_angles, obj_value)
            print(f"Initial gimbal angles: {self.init_gimbal_angles} rad, {np.degrees(self.init_gimbal_angles)} degrees")
        else:
            self.init_gimbal_angles = None

        self._init_optimization()
        gimbal_coeffs = self._init_gimbal_coeffs()
        opt_gimbal_coeffs = self._run_optimization(gimbal_coeffs)
        gimbal_coeffs_full = np.column_stack((self.init_gimbal_angles, opt_gimbal_coeffs))
        des_gimbal_angles_array = self._get_des_gimbal_angles_array(gimbal_coeffs_full)

        return des_gimbal_angles_array

    def _get_robot_config_array(self, des_joint_angles):
        """
        Return a (n, 3+joint_num) array, with the first three columns being the root link position and orientation. Since we only care about the gimbal angles, we can set the first three columns to zero.
        """
        robot_config_array = np.zeros((des_joint_angles.shape[0], 3 + des_joint_angles.shape[1]))
        robot_config_array[:, 3:] = des_joint_angles
        return robot_config_array

    def get_once_gimbal_angles(self, joint_angles, last_gimbal_angles) -> np.ndarray:
        if last_gimbal_angles is not None:
            gimbal_angles = last_gimbal_angles
        else:
            # Vectorized computation to find the closest reference configuration using proper angle difference
            distances = self._calculate_angular_distances(self.joint_refs, joint_angles)
            closest_idx = np.argmin(distances)
            gimbal_angles = self.gimbal_angles_options[closest_idx]

            # Print current joint angles, selected reference, and corresponding gimbal angles (degrees only)
            selected_joint_ref = self.joint_refs[closest_idx]
            joint_angles_deg = np.round(np.degrees(joint_angles)).astype(int)
            selected_joint_ref_deg = np.round(np.degrees(selected_joint_ref)).astype(int)
            gimbal_angles_deg = np.round(np.degrees(gimbal_angles)).astype(int)
            print(f"joint_angles: {joint_angles_deg}°, selected_joint_ref: {selected_joint_ref_deg}°, gimbal_init_angles: {gimbal_angles_deg}°")

        single_robot_config = np.zeros(3 + len(joint_angles))
        single_robot_config[3:] = joint_angles

        self.single_robot_config = single_robot_config

        # opt = nlopt.opt(nlopt.LN_PRAXIS, self.gimbal_num)
        opt = nlopt.opt(nlopt.GN_DIRECT, self.gimbal_num)
        opt.set_max_objective(self._single_objective)
        opt.set_ftol_rel(self.ftol_rel)
        opt.set_maxtime(self.maxtime)

        # Set variable bounds: all variables bounded between -pi and +pi
        lower_bounds = [-np.pi] * self.gimbal_num
        upper_bounds = [np.pi] * self.gimbal_num
        opt.set_upper_bounds(upper_bounds)
        opt.set_lower_bounds(lower_bounds)

        try:
            opt_gimbal_angles = opt.optimize(gimbal_angles)
            obj_value = opt.last_optimum_value()

            if last_gimbal_angles is not None:
                opt_gimbal_angles = self._modify_gimbal_results(last_gimbal_angles, opt_gimbal_angles)

            opt_gimbal_angles_deg = np.round(np.degrees(opt_gimbal_angles)).astype(int)
            print(f"opt_gimbal_angles: {opt_gimbal_angles_deg}°")

            return opt_gimbal_angles, obj_value

        except Exception as e:
            print(f"Exception during gimbal angle optimization: {e}")
            return None

    def _calculate_angular_distances(self, first_angles, second_angles):
        """
        Calculate the proper angular distance between joint_angles and each reference configuration.

        Args:
            first_angles: np.ndarray of shape (n_configs, n_joints) - reference joint configurations
            second_angles: np.ndarray of shape (n_joints,) - current joint angles

        Returns:
            distances: np.ndarray of shape (n_configs,) - angular distances to each reference
        """
        # Calculate angle differences for all configurations at once
        angle_diffs = first_angles - second_angles  # shape: (n_configs, n_joints)

        # Normalize angle differences to (-π, π] using atan2
        # atan2(sin(diff), cos(diff)) gives the principal value in (-π, π]
        normalized_diffs = np.arctan2(np.sin(angle_diffs), np.cos(angle_diffs))

        # Calculate the sum of absolute normalized differences for each configuration
        distances = np.sum(np.abs(normalized_diffs), axis=1)

        return distances

    def _single_objective(self, x, grad):
        """
        Objective function for the initial optimization to find the initial gimbal angles.
        """
        full_config = np.hstack((self.single_robot_config, x))
        return self.robot.get_fc_t_min(full_config)

    def _modify_gimbal_results(self, last_gimbal_angles, opt_gimbal_angles):
        adjusted_gimbal_angles = np.zeros_like(opt_gimbal_angles)
        for i in range(len(opt_gimbal_angles)):
            adjusted_gimbal_angles[i] = self._shift_angle(last_gimbal_angles[i], opt_gimbal_angles[i])

        return adjusted_gimbal_angles

    def _shift_angle(self, angle1, angle2):
        """
        Adjust angle2 to an equivalent angle such that the absolute difference from angle1 is minimized.
        """
        period = 2 * np.pi
        delta = angle1 - angle2
        n = round(delta / period)
        return angle2 + n * period

    def _init_optimization(self):
        opt_variable_num = self.gimbal_num * self.p

        # Use a faster local optimization algorithm for testing
        self.opt = nlopt.opt(nlopt.LN_COBYLA, opt_variable_num)
        self.opt.set_min_objective(self._objective)
        # self.opt.add_inequality_constraint(self._ineq_constraints, self.ineq_constraints_tol)

        # Set variable bounds: all variables bounded between -10 and +10
        # lower_bounds = [-10.0] * opt_variable_num
        # upper_bounds = [10.0] * opt_variable_num
        # self.opt.set_upper_bounds(upper_bounds)
        # self.opt.set_lower_bounds(lower_bounds)
        self.opt.set_ftol_rel(self.ftol_rel)
        self.opt.set_maxtime(10)  # Reduce maxtime for testing

    def _init_gimbal_coeffs(self):
        return np.zeros((self.gimbal_num, self.p))

    def _run_optimization(self, gimbal_coeffs):
        x = gimbal_coeffs.flatten()

        optimized_coeffs = self.opt.optimize(x)

        opt_result = self.opt.last_optimize_result()
        print(f"Optimization result: {opt_result}")

        if opt_result < 0:
            rospy.logwarn("Gimbal angle optimization failed")
            return None
        else:
            print("Gimbal angle optimization succeeded")
            optimized_coeffs = optimized_coeffs.reshape((self.gimbal_num, self.p))
            opt_val = self.opt.last_optimum_value()
            print(f"Optimal value: {opt_val}")

        return optimized_coeffs

    def _get_fc_dis_min_array(self, gimbal_coeffs):
        gimbal_coeffs_full = np.column_stack((self.init_gimbal_angles, gimbal_coeffs))
        self.update_coeffs(gimbal_coeffs_full)
        fc_t_min_array = np.zeros(len(self.sampled_time))

        for i, t in enumerate(self.sampled_time):
            config_index = int(t * self.cmd_hz)
            # Ensure we don't exceed array bounds
            if config_index >= self.robot_config_array.shape[0]:
                config_index = self.robot_config_array.shape[0] - 1

            robot_config = self.robot_config_array[config_index]
            gimbal_angles = self.get_pos(t)  # shape (gimbal_num,)
            full_config = np.hstack((robot_config, gimbal_angles))
            fc_t_min_array[i] = self.robot.get_fc_t_min(full_config)

        return fc_t_min_array

    def _objective(self, x: np.ndarray, grad: np.ndarray):
        """
        Optimized objective function using the existing _get_fc_dis_min_array method
        to avoid code duplication and improve maintainability.
        """
        gimbal_coeffs = x.reshape((self.gimbal_num, self.p))

        # Use the existing method to get all fc_t_min values at once
        fc_t_min_array = self._get_fc_dis_min_array(gimbal_coeffs)

        return np.sum(fc_t_min_array)

    def _ineq_constraints(self, result, x: np.ndarray, grad):
        """
        Inequality constraints for the optimization problem.
        result should be filled with constraint values where result[i] <= 0 means constraint i is satisfied.
        Optimized to use the existing _get_fc_dis_min_array method for consistency.
        """
        gimbal_coeffs = x.reshape((self.gimbal_num, self.p))

        # Use the existing method to get all fc_t_min values at once
        fc_t_min_array = self._get_fc_dis_min_array(gimbal_coeffs)

        # Calculate penalty sum for all time samples
        penalty_sum = 0.0
        for fc_t_min in fc_t_min_array:
            penalty, _ = BSplinePlanner.get_penalty(fc_t_min, self.fc_t_min_thres)
            penalty_sum += penalty

        result[0] = penalty_sum

    def _get_des_gimbal_angles_array(self, coeffs) -> np.ndarray:
        # Generate desired gimbal angle angles based on polynomial coefficients from 0 to self.time
        self.update_coeffs(coeffs)
        des_gimbal_angles_array = self.get_pos_array(self.cmd_hz, self.time)
        return des_gimbal_angles_array
