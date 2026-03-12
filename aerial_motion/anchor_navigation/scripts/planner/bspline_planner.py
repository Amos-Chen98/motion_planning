import os
import sys
current_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_path)
import numpy as np
import nlopt
from clamped_bspline import ClampedBSpline
from robot_model import Robot
import rospy



class RobotConfig:
    def __init__(self):
        self.rootlink_pos_x = 0.0
        self.rootlink_pos_y = 0.0
        self.rootlink_yaw = 0.0
        self.joint_angles = []


class Traj:
    def __init__(self, dim=6, control_points_num=4):
        """A trajectory can be defined by the following parameters:
        """
        self.start_pos = np.zeros(dim)
        self.end_pos = np.zeros(dim)
        self.start_vel = np.zeros(dim)
        self.end_vel = np.zeros(dim)
        self.time = 0.0
        self.control_points = np.zeros((control_points_num, dim))


class OptLog:
    def __init__(self):
        self.iterations = 0
        self.obj_history = []


def normalize_target_angle(init_angle, target_angle):
    """
    Normalize the target angle to be within the range of the initial angle ± π.

    Args:
        init_angle: The initial angle (in radians).
        target_angle: The target angle (in radians).

    Returns:
        The normalized target angle (in radians).
    """
    # Compute the difference
    angle_diff = target_angle - init_angle
    # Normalize to [-π, π]
    angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
    return init_angle + angle_diff


class OptError(Exception):
    """Raised when the trajectory optimization fails in a recoverable way."""
    pass


class BSplinePlanner(ClampedBSpline):
    def __init__(self, robot_urdf, planner_config, map):
        # Create speed bounds array from config
        translational_vel_limit = planner_config['translational_vel_limit']
        rotational_vel_limit = planner_config['rotational_vel_limit']

        # For 6-DOF system: [x, y, yaw, joint1, joint2, joint3]
        # First 2 dimensions are translational (x, y), rest are rotational
        speed_bound = np.array([translational_vel_limit, translational_vel_limit,
                                rotational_vel_limit, rotational_vel_limit,
                                rotational_vel_limit, rotational_vel_limit])

        super().__init__(p=3, speed_bound=speed_bound)

        # Robot config and Pinocchio logic
        self.robot = Robot(robot_urdf)
        self.config_dim = self.robot.param['config_dim']
        self.rotor_num = self.robot.param['rotor_num']

        # Constraints on the motion
        self.joint_angle_bound = planner_config['joint_angle_bound']
        self.dis2obs_min = planner_config['dis2obs_min']
        self.obs_clearance = planner_config['dis2obs_min'] + self.robot.param['propeller_R']
        # self.fc_rp_dis_min = planner_config['fc_rp_dis_min']
        self.fc_t_min_thre = planner_config['fc_t_min_thre']

        # Control command
        self.cmd_hz = planner_config['cmd_hz']

        # BSpline planner and recovery parameters
        self.control_points_num = planner_config['stage_control_points_num']
        self.sample_density = planner_config['sample_density']
        self.travel_speed = planner_config['travel_speed']
        self.save_traj = planner_config['save_traj']
        self.run_traj_opt = planner_config['run_traj_opt']

        # Optimizer
        self.ftol_rel = planner_config['ftol_rel']
        self.maxtime = planner_config['maxtime']
        self.constraint_tol = planner_config['constraint_tol']
        self.esdf_constraint_weight = planner_config['esdf_constraint_weight']
        self.vel_constraint_weight = planner_config['vel_constraint_weight']
        self.stability_constraint_weight = planner_config['stability_constraint_weight']

        # reovery config
        self.collision_cost_tolerance = planner_config['collision_cost_tolerance']
        self.stability_violation_tolerance = planner_config['stability_violation_tolerance']
        self.opt_max_retries = planner_config['opt_max_retries']
        self.traj_fc_t_min_thre = planner_config['traj_fc_t_min_thre']

        self.map = map

    def local_plan(self, local_init_state, local_target_state):
        rospy.loginfo("Init state:")
        rospy.loginfo(np.array2string(local_init_state,
                                      formatter={'float_kind': lambda x: "%.2f" % x}))
        # Handle yaw angle periodicity - normalize yaw difference to be within ±π
        local_target_state[0, 2] = normalize_target_angle(local_init_state[0, 2], local_target_state[0, 2])

        rospy.loginfo("Target state:")
        rospy.loginfo(np.array2string(local_target_state,
                                      formatter={'float_kind': lambda x: "%.2f" % x}))

        travel_dis = np.linalg.norm(local_target_state[0] - local_init_state[0])
        self.time = travel_dis / self.travel_speed
        self.sample_num = int(self.sample_density * travel_dis)
        self.t_samples = np.linspace(0, self.time, self.sample_num)

        self.set_boundary(start_pos=local_init_state[0],
                          end_pos=local_target_state[0],
                          start_vel=local_init_state[1],
                          end_vel=local_target_state[1],
                          T=self.time)

        seed = 0

        if not self.run_traj_opt:
            control_points = self.init_control_points(local_init_state,
                                                      local_target_state,
                                                      seed=seed)
            self.opt_log = OptLog()
            self.eva_direct_move_result(control_points)
        else:
            self.init_optimization()
            self.opt_log = OptLog()
            for attempt in range(self.opt_max_retries):
                # re-initialize CPs each time (seed=0 on first attempt)
                control_points = self.init_control_points(local_init_state,
                                                          local_target_state,
                                                          seed=seed)
                try:
                    control_points = self.run_optimization(control_points)
                    break  # success!
                except OptError as ex:
                    rospy.logwarn(f"Optimization attempt {attempt} failed: {ex}")
                    seed += 1
            else:
                raise RuntimeError(f"All {self.opt_max_retries} optimization retries failed")

        self.build_spline(control_points)
        des_pos_array = self.get_pos_array(self.cmd_hz)
        des_vel_array = self.get_vel_array(self.cmd_hz)

        local_des_states = self.robot.get_des_states(des_pos_array, des_vel_array)

        if not self.save_traj:
            return local_des_states, self.opt_log
        else:
            traj = Traj()
            traj.start_pos = local_init_state[0]
            traj.end_pos = local_target_state[0]
            traj.start_vel = local_init_state[1]
            traj.end_vel = local_target_state[1]
            traj.time = self.time
            traj.control_points = control_points
            return local_des_states, traj, self.opt_log

    def init_optimization(self):
        opt_variable_num = self.control_points_num * self.config_dim
        self.esdf_constraint_num = 1
        self.vel_constraint_num = (self.control_points_num + 3) * self.config_dim * 2
        stability_constraint_num = 1
        ineq_constraint_num = self.esdf_constraint_num + self.vel_constraint_num + stability_constraint_num
        ineq_constraints_tol = self.constraint_tol * np.ones(ineq_constraint_num)

        self.opt = nlopt.opt(nlopt.LD_SLSQP, opt_variable_num)
        self.opt.set_min_objective(self.objective)
        self.opt.add_inequality_mconstraint(self.ineq_constraints, ineq_constraints_tol)
        self.opt.set_ftol_rel(self.ftol_rel)
        self.opt.set_maxtime(self.maxtime)

        single_control_point_bound = np.array([np.inf, np.inf, np.inf] +
                                              [self.joint_angle_bound] * 3)

        # Create flattened bounds array directly without intermediate reshaping
        control_points_bound = np.repeat(single_control_point_bound, self.control_points_num)

        self.opt.set_lower_bounds(-control_points_bound)
        self.opt.set_upper_bounds(control_points_bound)

    def init_control_points(self, local_init_state, local_target_state, seed=0):
        '''
        As for calculating the opt control points, this method is identical to the closed form solution using KKT system
        '''
        init_pos = local_init_state[0]
        target_pos = local_target_state[0]
        init_vel = local_init_state[1]
        target_vel = local_target_state[1]

        h = self.time / (self.control_points_num+4 - self.p)  # self.time interval between two control points

        sec_control_point = init_pos + init_vel * h/self.p
        sec2last_control_point = target_pos - target_vel * h/self.p

        control_points = np.linspace(sec_control_point, sec2last_control_point, self.control_points_num+2)[1:-1]

        # add some noise to the control points
        if seed != 0:
            np.random.seed(seed)
            noise = np.random.normal(0, 1, control_points.shape)
            control_points += noise
            joint_angle_control_points = control_points[:, 3:]
            joint_angle_control_points = np.clip(joint_angle_control_points, -self.joint_angle_bound, self.joint_angle_bound)
            control_points[:, 3:] = joint_angle_control_points

        return control_points

    def print_init_cost(self, control_points):
        '''
        print the trajectory at the initial control points
        '''
        self.build_spline(control_points)
        total_cost = self.objective(control_points.flatten('F'), np.array([]))
        collision_cost, _ = self.set_esdf_constraint()
        collision_cost *= self.esdf_constraint_weight

        rospy.loginfo("---------------Init traj---------------")
        rospy.loginfo(f"Energy cost: {total_cost - collision_cost}")
        rospy.loginfo(f"Collision cost: {collision_cost}")

    def run_optimization(self, control_points: np.ndarray) -> np.ndarray:
        init_opt_variable = control_points.flatten('F')
        xopt = self.opt.optimize(init_opt_variable)
        result = self.opt.last_optimize_result()
        opt_val = self.opt.last_optimum_value()

        control_points = xopt.reshape((self.control_points_num, self.config_dim), order='F')

        # Set the actual number of optimization iterations
        if hasattr(self, 'opt_log'):
            self.opt_log.iterations = self.opt.get_numevals()

        self.eva_opt_result(control_points, result, opt_val)

        return control_points
    
    def eva_direct_move_result(self, control_points):
        self.build_spline(control_points)

        # collision
        collision_cost, _ = self.set_esdf_constraint()
        collision_cost *= self.esdf_constraint_weight

        rospy.loginfo("---------------Direct move traj---------------")
        rospy.loginfo(f"Collision cost: {collision_cost:.3f}")

        if collision_cost > self.collision_cost_tolerance:
            raise OptError("Collision cost is too high!")

    def eva_opt_result(self, control_points, result, opt_val):
        self.build_spline(control_points)

        # collision
        collision_cost, _ = self.set_esdf_constraint()
        collision_cost *= self.esdf_constraint_weight

        # Velocity
        vel_constraints, _ = self.set_vel_constraint()
        vel_constraints *= self.vel_constraint_weight

        vel_array = self.get_vel_array(10.0)
        translational_vel_max = np.max(np.abs(vel_array[:, :2]))
        rotational_vel_max = np.max(np.abs(vel_array[:, 2:]))

        traj_fc_t_min = self.calc_traj_fc_t_min()

        rospy.loginfo("---------------Opt traj----------------")
        rospy.loginfo(f"Optimization result: {result}")
        rospy.loginfo(f"Opt steps: {self.opt.get_numevals()}")
        rospy.loginfo(f"Energy cost: {opt_val - collision_cost:.3f}")
        rospy.loginfo(f"Collision cost: {collision_cost:.3f}")
        rospy.loginfo(f"Minimum torque: {traj_fc_t_min:.3f}")
        rospy.loginfo(f"Max translational velocity: {translational_vel_max:.3f} m/s")
        rospy.loginfo(f"Max rotational velocity: {np.rad2deg(rotational_vel_max):.3f} deg/s")

        if collision_cost > self.collision_cost_tolerance:
            raise OptError("Collision cost is too high!")
        if traj_fc_t_min < self.traj_fc_t_min_thre:
            raise OptError("Minimum torque is too low!")
        # Check velocity limits for each dimension using symmetry
        velocity_violated = np.any(np.abs(vel_array) > self.speed_bound)
        if velocity_violated:
            raise OptError("Velocity limits are violated!")

    def objective(self, x: np.ndarray, grad):
        control_points_mat = x.reshape((self.control_points_num, self.config_dim), order='F')
        self.build_spline(control_points_mat)

        # cost
        energy_cost, energy_grad = self.get_vel_energy_grad()
        collision_cost, collision_grad = self.set_esdf_constraint()
        collision_cost *= self.esdf_constraint_weight
        collision_grad *= self.esdf_constraint_weight

        cost = energy_cost + collision_cost

        # Log iteration data
        if hasattr(self, 'opt_log'):
            # Log this iteration
            self.opt_log.obj_history.append(float(cost))

        # grad
        if grad.size > 0:
            grad[:] = energy_grad + collision_grad

        return cost

    def ineq_constraints(self, result, x: np.ndarray, grad):
        control_points_mat = x.reshape((self.control_points_num, self.config_dim), order='F')
        self.build_spline(control_points_mat)

        esdf_constraints, esdf_grad = self.set_esdf_constraint()
        vel_constraints, vel_grad = self.set_vel_constraint()
        stability_constraints, stability_grad = self.set_stability_constraint()

        esdf_constraints *= self.esdf_constraint_weight
        esdf_grad *= self.esdf_constraint_weight
        vel_constraints *= self.vel_constraint_weight
        vel_grad *= self.vel_constraint_weight
        stability_constraints *= self.stability_constraint_weight
        stability_grad *= self.stability_constraint_weight

        # Optimize array assignment using slicing
        esdf_end = self.esdf_constraint_num
        vel_end = esdf_end + self.vel_constraint_num

        result[:esdf_end] = esdf_constraints
        result[esdf_end:vel_end] = vel_constraints
        result[vel_end:] = stability_constraints

        if grad.size > 0:
            grad[:esdf_end, :] = esdf_grad
            grad[esdf_end:vel_end, :] = vel_grad
            grad[vel_end:, :] = stability_grad

    def compute_sampled_constraint(self, get_constraint_data_func, threshold_value):
        """
        Generic constraint computation function that can be used for both ESDF and stability constraints.

        Args:
            get_constraint_data_func: Function that returns constraint values and gradients for a robot config
            threshold_value: Threshold value for determining constraint violations

        Returns:
            constraints: Aggregated constraint value
            grad: Gradient of constraints w.r.t. control points
        """
        # First, determine the number of constraints by calling the function once
        pos_sample = self.get_pos(self.t_samples[0])
        constraint_values_sample, _ = get_constraint_data_func(pos_sample)
        num_constraints_per_sample = len(constraint_values_sample)

        penalty = np.zeros(num_constraints_per_sample * self.sample_num)
        grad = np.zeros((num_constraints_per_sample * self.sample_num, self.control_points_num * self.config_dim))

        for i, t in enumerate(self.t_samples):
            # Get robot configuration at this time point
            pos = self.get_pos(t)

            # Get constraint data for this configuration
            constraint_values, grad_d2q_array = get_constraint_data_func(pos)

            # print(np.max(constraint_values)) # NOTE: Sometimes, it outputs 10000.0 ??

            # Compute gradient of robot configuration w.r.t. control points
            grad_q2c = self.get_grad_q2c(t)  # shape: (D, N*D)

            # Process each constraint
            for j in range(num_constraints_per_sample):
                constraint_value = constraint_values[j]
                grad_d2q = grad_d2q_array[j]  # gradient of constraint w.r.t. q, shape: (D,)

                # Compute penalty and gradient using extracted function
                penalty_value, grad_p2d = self.get_penalty(constraint_value, threshold_value)
                penalty[num_constraints_per_sample*i + j] = penalty_value

                # Compute gradient chain: penalty → distance → q → control points
                grad_d2c = grad_d2q @ grad_q2c  # shape: (N*D, ) = (1, D) @ (D, N*D)
                grad[num_constraints_per_sample*i + j] = grad_p2d * grad_d2c

        constraints = np.sum(penalty) / self.sample_num
        grad = np.sum(grad, axis=0) / self.sample_num

        return constraints, grad

    @staticmethod
    def get_penalty(constraint_value, threshold_value):
        """
        Compute penalty and gradient for constraint violation using smooth approximation.

        Args:
            constraint_value: Current constraint value
            threshold_value: Threshold value for constraint violation

        Returns:
            penalty: Penalty value (0 if constraint satisfied, positive if violated)
            grad_p2d: Gradient of penalty w.r.t. constraint value
        """
        # Compute penalty using smooth approximation
        penalty = np.where(
            constraint_value > threshold_value,
            0,
            1/(2*threshold_value) * (constraint_value-threshold_value)**2
        )

        # Compute gradient of penalty w.r.t. distance
        grad_p2d = np.where(
            constraint_value > threshold_value,
            0,
            1/threshold_value * (constraint_value-threshold_value)
        )

        return penalty, grad_p2d

    def set_esdf_constraint(self):
        """
        Computes ESDF (collision avoidance) constraint and its gradient.
        """
        return self.compute_sampled_constraint(self.get_rotor_edt_grad, self.obs_clearance)

    def set_stability_constraint(self):
        """
        Computes stability constraint and its gradient.
        """
        # return self.compute_sampled_constraint(self.robot.get_fc_rp_dists_grad, self.fc_rp_dis_min)
        return self.compute_sampled_constraint(self.robot.get_fc_t_array, self.fc_t_min_thre)

    def get_rotor_edt_grad(self, config: np.ndarray):
        rotor_pos_array, rotor_jac_array = self.robot.get_rotor_pos_jac(config)
        rotor_edt_array = np.zeros(self.rotor_num)
        grad_d2q_array = np.zeros((self.rotor_num, self.config_dim))
        for i in range(self.rotor_num):
            rotor_pos = rotor_pos_array[i][:2]
            rotor_jac = rotor_jac_array[i][:2]
            rotor_edt_array[i] = self.map.get_edt(rotor_pos)
            edt_grad = np.array(self.map.get_edt_grad(rotor_pos))
            grad_d2q_array[i] = edt_grad @ rotor_jac

        return rotor_edt_array, grad_d2q_array

    def calc_traj_fc_t_min(self):
        """
        calculate the minimum stability margin using robot.get_fc_t_min
        """
        check_freq = 10.0
        pos_array = self.get_pos_array(check_freq)
        fc_t_mins = np.zeros(len(pos_array))
        for i, pos in enumerate(pos_array):
            fc_t_mins[i] = self.robot.get_fc_t_min(pos)
        traj_fc_t_min = np.min(fc_t_mins)
        return traj_fc_t_min
