import os
import sys
current_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_path)
from bspline_planner import BSplinePlanner
from clamped_bspline import ClampedBSpline
import numpy as np
import nlopt
import rospy
from robot_model import Robot



class GimbalYawPlanner(ClampedBSpline):
    def __init__(self, robot_urdf, planner_config):

        # robot
        self.robot = Robot(robot_urdf)

        if planner_config is None:
            rospy.logwarn("GimbalYawPlanner: planner_config is None, using default values")

        self.maxtime = 30
        self.gimbal_num = self.robot.param['rotor_num']
        self.fc_t_min_thres = 1.0
        self.ftol_rel = 1e-6  # Relative tolerance for optimization

        super().__init__(p=3)

        # Control points configuration for B-spline
        self.control_points_num = 5  # Number of intermediate control points (adjustable)
        self.gimbal_angle_bound = 2 * np.pi  # Bound for gimbal angles
        # Include initial gimbal angles, final gimbal angles, and control points in optimization
        self.opt_variable_num = self.gimbal_num + self.control_points_num * self.gimbal_num + self.gimbal_num
        self.sample_hz = 10
        self.output_hz = 60

        # Used for Forward kinematics

        # Initialize objective value history for optimization tracking
        self.objective_history = []

        # Create reference configurations with their corresponding gimbal angles
        self.init_reference_configs()

    def init_reference_configs(self):
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
        # opt = nlopt.opt(nlopt.GN_CRS2_LM, self.gimbal_num)

        opt.set_max_objective(self._single_objective)
        opt.set_ftol_rel(self.ftol_rel)
        opt.set_maxtime(self.maxtime)

        # Set variable bounds: all variables bounded between -pi and +pi
        lower_bounds = [-self.gimbal_angle_bound] * self.gimbal_num
        upper_bounds = [self.gimbal_angle_bound] * self.gimbal_num
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

    def plan(self, des_joint_angles: np.ndarray, time: float, init_gimbal_angles_hint: np.ndarray = None) -> tuple:
        """
        Inputs:
            des_joint_angles: np.ndarray of (n, joint_num), each row a desired joint state at a time step
            time: float, total time duration for the trajectory
            init_gimbal_angles_hint: np.ndarray of (gimbal_num,), hint for initial gimbal angles (optional)
        Outputs:
            tuple with three elements:
                des_gimbal_angles_array: np.ndarray of (n, gimbal_num), each row a desired gimbal angle at a time step
                opt_init_gimbal_angles: np.ndarray of (gimbal_num,), optimized initial gimbal angles
                objective_history: np.ndarray of (n_iterations,), objective function values for each optimization iteration
        """
        self.time = time
        input_sample_interval = self.time / des_joint_angles.shape[0]
        self.input_hz = 1.0 / input_sample_interval

        # Use independent sample frequency (must be an integer divisor of input_hz)
        sample_interval = 1.0 / self.sample_hz
        self.sampled_time = np.arange(0, self.time, sample_interval)
        self.robot_config_array = self._build_robot_config_array(des_joint_angles)

        # Store hint for initialization
        self.init_gimbal_angles_hint = init_gimbal_angles_hint

        # Clear objective history for new planning session
        self.objective_history = []

        self._init_optimization()
        opt_variables = self._init_opt_variables()
        opt_result = self._run_optimization(opt_variables)

        # Build B-spline and generate trajectory
        if opt_result is not None:
            opt_init_gimbal_angles, final_gimbal_angles, opt_control_points = opt_result
            des_gimbal_angles_array = self.get_des_gimbal_angles_array(opt_init_gimbal_angles, final_gimbal_angles, opt_control_points)
            # Return trajectory, optimized initial gimbal angles, and objective history as a tuple
            return des_gimbal_angles_array, opt_init_gimbal_angles, np.array(self.objective_history)
        else:
            return None, None, None

    def _build_robot_config_array(self, des_joint_angles):
        """
        Return a (n, 3+joint_num) array, with the first three columns being the root link position and orientation. Since we only care about the gimbal angles, we can set the first three columns to zero.
        """
        input_len = des_joint_angles.shape[0]
        input_dim = des_joint_angles.shape[1]
        robot_config_array = np.zeros((input_len, 3 + input_dim))  # 3 = [x, y, yaw] of the root link
        robot_config_array[:, 3:] = des_joint_angles
        return robot_config_array

    def _init_optimization(self):
        seed = 42  # Fixed seed for reproducible results
        nlopt.srand(seed)
        # self.opt = nlopt.opt(nlopt.GN_DIRECT, self.opt_variable_num)
        # self.opt = nlopt.opt(nlopt.GN_CRS2_LM, self.opt_variable_num)
        self.opt = nlopt.opt(nlopt.GN_ESCH, self.opt_variable_num)
        self.opt.set_min_objective(self._objective)
        self.opt.set_upper_bounds([self.gimbal_angle_bound] * self.opt_variable_num)
        self.opt.set_lower_bounds([-self.gimbal_angle_bound] * self.opt_variable_num)
        self.opt.set_maxtime(self.maxtime)

    def _init_opt_variables(self):
        # opt_variables = [-1.396263, -4.188790, 0.000000, -4.188790, -4.188790, -4.188790, 4.188790,
        #                  -1.396263, -4.188790, -4.188790, 0.000000, -4.188790, 0.000000, -4.188790,
        #                  4.188790, -4.188790, 0.000000, 0.000000, 0.000000, 0.000000, -4.188790,
        #                  4.188790, 0.000000, 0.000000, 0.000000, -4.188790, 4.188790, 0.000000]

        # Initialize optimization variables: [init_gimbal_angles, final_gimbal_angles, control_points_flattened]
        opt_variables = np.random.uniform(-self.gimbal_angle_bound, self.gimbal_angle_bound, self.opt_variable_num)

        # # If a hint is provided for initial gimbal angles, use it
        # if self.init_gimbal_angles_hint is not None:
        #     opt_variables[:self.gimbal_num] = self.init_gimbal_angles_hint
        # else:
        #     # Use reference configurations to get a reasonable initial guess
        #     # Use the first joint configuration from robot_config_array if available
        #     if hasattr(self, 'robot_config_array') and len(self.robot_config_array) > 0:
        #         first_joint_angles = self.robot_config_array[0, 3:]  # Skip x, y, yaw
        #         distances = self._calculate_angular_distances(self.joint_refs, first_joint_angles)
        #         closest_idx = np.argmin(distances)
        #         opt_variables[:self.gimbal_num] = self.gimbal_angles_options[closest_idx]

        return opt_variables

    def _run_optimization(self, opt_variables):
        try:
            x_opt = self.opt.optimize(opt_variables)
            opt_result = self.opt.last_optimize_result()
            print(f"Optimization result: {opt_result}")

            if opt_result < 0:
                rospy.logwarn("Gimbal angle optimization failed")
                return None
            else:
                opt_val = self.opt.last_optimum_value()
                print("Gimbal angle optimization succeeded")
                print("x_opt:")
                print(np.array2string(x_opt, separator=', ', formatter={'float_kind': lambda x: "%.6f" % x}))
                print(f"Optimal value: {opt_val}")

                init_gimbal_angles, final_gimbal_angles, control_points = self._retrieve_x(x_opt)

                return init_gimbal_angles, final_gimbal_angles, control_points

        except Exception as e:
            rospy.logwarn(f"Optimization failed with exception: {e}")
            return None

    def _objective(self, x: np.ndarray, grad: np.ndarray):
        """
        Objective function for B-spline trajectory optimization
        """
        init_gimbal_angles, final_gimbal_angles, control_points = self._retrieve_x(x)

        self.build_full_spline(
            init_gimbal_angles=init_gimbal_angles,
            final_gimbal_angles=final_gimbal_angles,
            time=self.time,
            control_points=control_points
        )

        fc_t_min_array = self._get_fc_t_min_array()
        penalty_sum = 0.0
        for fc_t_min in fc_t_min_array:
            penalty, _ = BSplinePlanner.get_penalty(fc_t_min, self.fc_t_min_thres)
            penalty_sum += penalty

        obj_value = penalty_sum / self.sample_hz

        self.objective_history.append(obj_value)

        return obj_value

    def build_full_spline(self, init_gimbal_angles, final_gimbal_angles, time, control_points):
        self.set_boundary(
            start_pos=init_gimbal_angles,
            end_pos=final_gimbal_angles,
            start_vel=np.zeros(self.gimbal_num),
            end_vel=np.zeros(self.gimbal_num),
            T=time
        )
        self.build_spline(control_points)

    def _retrieve_x(self, x: np.ndarray):
        # Extract variables: [init_gimbal_angles, final_gimbal_angles, control_points_flattened]
        init_gimbal_angles = x[:self.gimbal_num]
        final_gimbal_angles = x[self.gimbal_num:2*self.gimbal_num]
        x_intermediate_control_points = x[2*self.gimbal_num:]
        control_points = x_intermediate_control_points.reshape((self.control_points_num, self.gimbal_num))
        return init_gimbal_angles, final_gimbal_angles, control_points

    def get_des_gimbal_angles_array(self, init_gimbal_angles, final_gimbal_angles, opt_control_points):
        """
        Build B-spline trajectory and generate desired gimbal angles array

        Args:
            init_gimbal_angles: Initial gimbal angles
            final_gimbal_angles: Final gimbal angles  
            opt_control_points: Optimized control points

        Returns:
            des_gimbal_angles_array: Desired gimbal angles trajectory
        """
        self.set_boundary(
            start_pos=init_gimbal_angles,
            end_pos=final_gimbal_angles,
            start_vel=np.zeros(self.gimbal_num),
            end_vel=np.zeros(self.gimbal_num),
            T=self.time
        )
        self.build_spline(opt_control_points)
        des_gimbal_angles_array = self.get_pos_array(self.output_hz)
        self.fc_t_min_trajectory = self._calculate_fc_t_min_trajectory(des_gimbal_angles_array)
        return des_gimbal_angles_array

    def _get_fc_t_min_array(self):
        """return an array of length (n,) where n is the number of sampled time steps"""
        fc_t_min_array = np.zeros(len(self.sampled_time))
        config_idx_max = len(self.robot_config_array) - 1

        for i, t in enumerate(self.sampled_time):
            # Calculate the corresponding index in robot_config_array based on time
            robot_config_idx = round(t * self.input_hz)
            robot_config_idx = min(robot_config_idx, config_idx_max)  # Clamp to valid range
            robot_config = self.robot_config_array[robot_config_idx]
            gimbal_angles = self.get_pos(t)  # shape (gimbal_num,)
            full_config = np.hstack((robot_config, gimbal_angles))
            fc_t_min_array[i] = self.robot.get_fc_t_min(full_config)

        return fc_t_min_array

    def _calculate_fc_t_min_trajectory(self, des_gimbal_angles_array) -> np.ndarray:
        """Calculate fc_t_min values for the entire gimbal angle trajectory"""
        n_steps = len(des_gimbal_angles_array)
        fc_t_min_trajectory = np.zeros(n_steps)
        config_idx_max = len(self.robot_config_array) - 1

        for i in range(n_steps):
            # Calculate time corresponding to this output step
            t = i / self.output_hz
            # Find corresponding robot configuration index based on time
            robot_config_idx = round(t * self.input_hz)
            robot_config_idx = min(robot_config_idx, config_idx_max)  # Clamp to valid range
            robot_config = self.robot_config_array[robot_config_idx]
            gimbal_angles = des_gimbal_angles_array[i]
            full_config = np.hstack((robot_config, gimbal_angles))
            fc_t_min_trajectory[i] = self.robot.get_fc_t_min(full_config)

        return fc_t_min_trajectory

    def get_fc_t_min_trajectory(self) -> np.ndarray:
        """Get the fc_t_min trajectory calculated during planning"""
        if hasattr(self, 'fc_t_min_trajectory'):
            return self.fc_t_min_trajectory
        else:
            return None

    def get_objective_history(self) -> np.ndarray:
        """Get the objective function values for each optimization iteration"""
        return np.array(self.objective_history)

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
