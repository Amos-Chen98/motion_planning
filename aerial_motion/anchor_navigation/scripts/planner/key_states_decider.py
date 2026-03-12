import os
import sys
current_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_path)
from robot_model import Robot
import gc
from bspline_planner import BSplinePlanner
from astar_planner import AstarPlanner
import copy
import numpy as np
import rospy



class KeyStatesDecider:
    """
    A class responsible for deciding key states in the global planning process.
    Handles rotated state generation, intermediate state generation, and path-based navigation.
    """

    def __init__(self, robot_urdf, planner_config):
        """
        Initialize the KeyStatesDecider.

        Args:
            robot_urdf: Robot URDF for BSplinePlanner initialization
            planner_config: Configuration dictionary containing planning parameters
        """
        # Robot
        self.robot = Robot(robot_urdf)
        self.config_dim = self.robot.param['config_dim']
        self.joint_num = self.robot.param['joint_num']
        self.link_L = self.robot.param['link_L']

        # Constraints
        self.joint_angle_bound = planner_config['joint_angle_bound']

        # Planning parameters
        self.obs_clearance = planner_config['dis2obs_min'] + self.robot.param['propeller_R']
        self.fc_rp_dis_min = planner_config['fc_rp_dis_min']
        self.fc_t_min_thre = planner_config['fc_t_min_thre']
        self.traj_fc_t_min_thre = planner_config['traj_fc_t_min_thre']
        self.esdf_constraint_weight = planner_config['esdf_constraint_weight']
        self.stability_constraint_weight = planner_config['stability_constraint_weight']

        # Intermediate state generation parameters
        self.transform_attempt_num = planner_config['transform_attempt_num']
        self.candidate_int_eva_num = planner_config['candidate_int_eva_num']
        self.rotate_first = planner_config['rotate_first']
        self.one_way_fold = planner_config['one_way_fold']

        # Store configuration and robot model for BSplinePlanner creation
        self.planner_config = planner_config
        self.robot_urdf = robot_urdf

        # Initialize A* path planner
        astar_resolution = planner_config['astar_resolution']
        self.astar_planner = AstarPlanner(astar_resolution)

        # State
        self.path = None
        self.bspline_planner = None
        self.key_states_complete = False

    def init_bspline_planner(self, map):
        self.bspline_planner = BSplinePlanner(self.robot_urdf, self.planner_config, map)

    def set_key_states_list(self, global_init_state: np.ndarray, global_target_state: np.ndarray,
                            map):
        """
        Creates a list of intermediate states between global init and target states using transformed candidates.

        Args:
            global_init_state: (2, 6) array with position and velocity
            global_target_state: (2, 6) array with position and velocity
            map: Map data for A* planning

        Returns:
            Tuple: (key_states_list, key_states_complete)
                key_states_list: List of (2, 6) arrays including the global init, intermediate waypoints, and target state
                key_states_complete: Boolean indicating if all key states are feasible
        """
        # Initialize BSplinePlanner for this planning session
        if not hasattr(self, 'bspline_planner') or self.bspline_planner is None:
            self.init_bspline_planner(map)
        self.map = map

        try:
            # Check if direct move is feasible first
            if self._is_direct_move_feasible(global_init_state, global_target_state):
                self.key_states_complete = True
                return [global_init_state, global_target_state], True

            # Initialize path for planning
            if not self._get_global_path(global_init_state, global_target_state):
                # A* planning failed, terminate the whole planning
                self.key_states_complete = False
                return [global_init_state, global_target_state], False

            # Start with global_init_state
            key_states_list = [global_init_state]

            # Add rotated state
            if self.rotate_first:
                start_state = self._get_rotated_state(global_init_state)
            else:
                start_state = global_init_state

            # Generate intermediate key states starting from rotated state
            intermediate_states = self._generate_intermediate_states(start_state, global_target_state)
            key_states_list.extend(intermediate_states)

            # Ensure target state is included
            if not np.array_equal(key_states_list[-1], global_target_state):
                key_states_list.append(global_target_state)

            return key_states_list, self.key_states_complete
        finally:
            # Clean up BSplinePlanner instance
            self._cleanup()

    def _is_direct_move_feasible(self, global_init_state: np.ndarray, global_target_state: np.ndarray):
        """Check if a direct move from init to target state is feasible."""
        # Number of intermediate points to check along the direct path
        travel_distance = np.linalg.norm(global_target_state[0, :2] - global_init_state[0, :2])

        num_check_points = int(np.ceil(travel_distance / (0.5 * self.link_L)))

        # Generate intermediate states along the direct path
        intermediate_states = np.linspace(global_init_state[0], global_target_state[0], num_check_points)

        # Check feasibility for all intermediate states
        for state in intermediate_states:
            if not self.is_state_feasible(state):
                return False

        return True

    def _get_global_path(self, global_init_state: np.ndarray, global_target_state: np.ndarray):
        """Initialize the A* path for cost evaluation functions."""
        rootlink_target_pos = global_target_state[0, :2]
        cog_pos_init = copy.deepcopy(self.robot.get_cog_pos_jac(global_init_state[0])[0][:2])  # 2d

        # Attempt A* planning
        astar_result = self.astar_planner.plan(self.map, cog_pos_init, rootlink_target_pos)

        if astar_result is None or len(astar_result) == 0:
            rospy.logerr("A* planning failed, terminating global planning")
            self.key_states_complete = False
            self.path = None
            return False
        else:
            self.path = np.array(astar_result)
            return True

    def _get_rotated_state(self, global_init_state):
        """
        Rotate the robot around its center of gravity (COG) until the rootlink's root 
        is placed on the self.path (with tolerance). Returns the rotated state of shape (2, 6).
        This state will serve as the first state in key_states_list.

        Args:
            global_init_state: (2, 6) array with position and velocity

        Returns:
            rotated_state: (2, 6) array with rotated position and zero velocity
        """
        if not self._is_path_available():
            return global_init_state

        init_pos = global_init_state[0]
        cog_pos_2d = self._get_cog_pos_xy(init_pos)

        return self._find_best_rotated_state(init_pos, cog_pos_2d, global_init_state)

    def _is_path_available(self):
        """Check if path is available for start state calculation."""
        if not hasattr(self, 'path') or self.path is None or len(self.path) == 0:
            rospy.logwarn("Path not available, returning original state")
            return False
        return True

    def _get_cog_pos_xy(self, robot_config):
        """Get the center of gravity position in 2D."""
        cog_pos, _ = self.robot.get_cog_pos_jac(robot_config)
        return cog_pos[:2]  # Only x, y components

    def _find_best_rotated_state(self, init_pos, cog_pos_2d, global_init_state):
        """Find the best rotated state by trying different rotation angles."""
        position_tolerance = 0.15  # meters
        max_rotation_attempts = 36  # 10-degree increments

        best_state = None
        min_distance_to_path = float('inf')

        for i in range(max_rotation_attempts):
            rotation_angle = (2 * np.pi * i) / max_rotation_attempts

            # Generate rotated state
            new_pos = self._calculate_rotated_position(init_pos, cog_pos_2d, rotation_angle)

            # Check feasibility and distance to path
            if self.is_state_feasible(new_pos):
                min_dist_to_path = self._calculate_distance_to_path(new_pos[:2])

                # Return immediately if within tolerance
                if min_dist_to_path <= position_tolerance:
                    return np.vstack((new_pos, np.zeros(self.config_dim)))

                # Track best state
                if min_dist_to_path < min_distance_to_path:
                    min_distance_to_path = min_dist_to_path
                    best_state = np.vstack((new_pos, np.zeros(self.config_dim)))

        return self._handle_rotation_search_result(best_state, min_distance_to_path, global_init_state)

    def _calculate_rotated_position(self, init_pos, cog_pos_2d, rotation_angle):
        """Calculate the new rootlink position after rotation around COG."""
        # Vector from COG to original rootlink position
        cog_to_rootlink_vec = init_pos[:2] - cog_pos_2d

        # Create rotation matrix
        cos_theta = np.cos(rotation_angle)
        sin_theta = np.sin(rotation_angle)
        rotation_matrix = np.array([[cos_theta, -sin_theta],
                                    [sin_theta, cos_theta]])

        # Apply rotation
        rotated_vec = rotation_matrix @ cog_to_rootlink_vec
        new_rootlink_pos = cog_pos_2d + rotated_vec

        # Update yaw and normalize
        new_rootlink_yaw = init_pos[2] + rotation_angle
        new_rootlink_yaw = np.arctan2(np.sin(new_rootlink_yaw), np.cos(new_rootlink_yaw))

        # Create new position array (keep joint angles unchanged)
        return np.array([new_rootlink_pos[0], new_rootlink_pos[1], new_rootlink_yaw,
                        init_pos[3], init_pos[4], init_pos[5]])

    def _calculate_distance_to_path(self, position_2d):
        """Calculate minimum distance from position to path."""
        distances = np.linalg.norm(self.path[:, :2] - position_2d, axis=1)
        return np.min(distances)

    def _handle_rotation_search_result(self, best_state, min_distance_to_path, global_init_state):
        """Handle the result of rotation search and return appropriate state."""
        if best_state is not None:
            rospy.logwarn(f"No rotated state within tolerance found. Using best feasible state with distance to path {min_distance_to_path:.3f}m")
            return best_state

        rospy.logwarn("No feasible rotated state found, returning original state")
        return global_init_state

    def _generate_intermediate_states(self, start_state: np.ndarray, global_target_state: np.ndarray):
        """Generate intermediate key states between start and target."""
        if self.rotate_first:
            intermediate_states = [start_state]
        else:
            intermediate_states = []
        distance_threshold = 0.5 * self.link_L
        local_init_state = start_state

        while True:
            # Check if we've reached the target vicinity
            if self._has_reached_target_vicinity(local_init_state, global_target_state, distance_threshold):
                self.key_states_complete = True
                break

            # Find next best state
            next_state = self._find_next_key_state(local_init_state)
            if next_state is None:
                self.key_states_complete = False
                break

            intermediate_states.append(next_state)
            local_init_state = next_state

            # Safety check to prevent infinite loops
            if self._should_stop_generation(intermediate_states):
                self.key_states_complete = False
                break

        return intermediate_states

    def _has_reached_target_vicinity(self, local_init_state: np.ndarray, global_target_state: np.ndarray, distance_threshold: float):
        """Check if we're close enough to the global target."""
        current_rootlink_pos = local_init_state[0, :2]
        target_rootlink_pos = global_target_state[0, :2]
        distance_to_target = np.linalg.norm(target_rootlink_pos - current_rootlink_pos)

        if distance_to_target <= distance_threshold:
            return True
        return False

    def _find_next_key_state(self, local_init_state: np.ndarray):
        """Find the next key state from current position."""
        # Generate and evaluate candidates
        transformed_candidates = self._generate_transformed_candidates(local_init_state)

        # Select best candidate with finite cost
        best_candidate = self._select_best_candidate(transformed_candidates, local_init_state[0])
        if best_candidate is None:
            print("No valid transformed candidate found.")
            return None

        # # check the best_candidate
        # traj_fc_t_min = self._get_traj_fc_t_min(local_init_state[0], best_candidate)

        # if traj_fc_t_min <= self.traj_fc_t_min_thre:
        #     deformed_candidates = self._generate_deformed_candidates(best_candidate)
        #     best_candidate = self._select_best_deformed_candidate(local_init_state[0], deformed_candidates)
        #     if best_candidate is None:
        #         print("No valid deformed candidate found.")
        #         return None
        #     print("Found a valid deformed candidate.")

        # Create new key state with zero velocity
        return np.vstack((best_candidate, np.zeros(self.config_dim)))

    def _should_stop_generation(self, key_states_list: list):
        """Check if we should stop generating key states."""
        if len(key_states_list) > 20:  # Reasonable upper limit
            rospy.logwarn("Maximum number of key states reached, breaking loop")
            return True
        return False

    def _generate_transformed_candidates(self, local_init_state: np.ndarray):
        """Generate transformed candidates by varying joint angles."""
        if self.one_way_fold:
            extend_joint_angle_candidates = np.linspace(0, self.joint_angle_bound, self.transform_attempt_num)
        else:
            extend_joint_angle_candidates = np.linspace(-self.joint_angle_bound, self.joint_angle_bound, 2 * self.transform_attempt_num)

        transformed_candidates = np.zeros((len(extend_joint_angle_candidates), self.config_dim))

        for i, extend_joint_angle in enumerate(extend_joint_angle_candidates):
            transformed_candidates[i] = self._get_transformed_state(local_init_state, extend_joint_angle)

        return transformed_candidates

    def _get_transformed_state(self, local_init_state: np.ndarray, extend_joint_angle: float):
        """
        Extend one link length from the local_init_state, the joint angle between the new rootlink 
        and the former rootlink is extend_joint_angle.
        """
        init_joint_angles = local_init_state[0, 3:]
        rootlink_init_yaw = local_init_state[0, 2]
        init_root_pos = local_init_state[0, :2]

        new_rootlink_yaw = rootlink_init_yaw - extend_joint_angle
        new_root_pos = init_root_pos - self.link_L * np.array([np.cos(new_rootlink_yaw), np.sin(new_rootlink_yaw)])

        transformed_state = np.hstack((new_root_pos,
                                       new_rootlink_yaw,
                                       extend_joint_angle,
                                       init_joint_angles[:self.joint_num - 1]))

        return transformed_state

    def _generate_deformed_candidates(self, ori_candidate: np.ndarray):
        """
        input shape: (N,)
        return: deformed_candidates of shape (num_candidates, config_dim)
        Generate candidates by sampling the middle joint angle
        """
        middle_joint_candidates = np.linspace(-self.joint_angle_bound, self.joint_angle_bound, self.transform_attempt_num)

        num_candidates = len(middle_joint_candidates)
        deformed_candidates = np.zeros((num_candidates, self.config_dim))

        # Calculate the middle joint index
        if self.joint_num >= 3:
            # For 3 or more joints, use the middle joint (index 1 for 3 joints: 0, 1, 2)
            middle_joint_index = self.joint_num // 2
        elif self.joint_num == 2:
            # For 2 joints, use the first joint (index 0)
            middle_joint_index = 0
        else:
            # For 1 joint, use that joint (index 0)
            middle_joint_index = 0

        for i, middle_joint_angle in enumerate(middle_joint_candidates):
            deformed_candidate = ori_candidate.copy()
            # Modify the middle joint angle (index: 3 + middle_joint_index)
            deformed_candidate[3 + middle_joint_index] = middle_joint_angle
            deformed_candidates[i] = deformed_candidate

        return deformed_candidates

    def _select_best_candidate(self, candidates: np.ndarray, local_init_state: np.ndarray):
        """
        Select the best candidate from the candidates by evaluating their costs.

        Args:
            candidates: Array of candidate states
            local_init_state: Current state for cost evaluation

        Returns:
            Best candidate state or None if no valid candidates found
        """
        # Calculate costs for all candidates
        candidate_cost_array = np.array([
            self._eva_candidate_target_cost(candidate, local_init_state)
            for candidate in candidates
        ])

        # Filter out candidates with infinite cost
        valid_indices = candidate_cost_array != float('inf')
        if not np.any(valid_indices):
            return None

        valid_costs = candidate_cost_array[valid_indices]
        valid_candidates = candidates[valid_indices]

        best_local_index = np.argmin(valid_costs)
        best_candidate = valid_candidates[best_local_index]

        return best_candidate

    def _select_best_deformed_candidate(self, local_init_state: np.ndarray, deformed_candidates: np.ndarray):
        """
        Select the deformed candidate with the lowest trajectory unstable penalty.
        Only considers feasible candidates.

        Args:
            local_init_state: Current state for penalty evaluation
            deformed_candidates: Array of deformed candidate states

        Returns:
            Best feasible deformed candidate with lowest unstable penalty, or None if no feasible candidates
        """
        # First filter for feasible candidates
        feasible_candidates = []
        for candidate in deformed_candidates:
            if self.is_state_feasible(candidate):
                feasible_candidates.append(candidate)

        # If no feasible candidates, return None
        if not feasible_candidates:
            return None

        # Among feasible candidates, find the one with highest fc_t_min value
        best_candidate = None
        max_fc_t_min = -float('inf')

        for candidate in feasible_candidates:
            fc_t_min = self._get_traj_fc_t_min(local_init_state, candidate)
            if fc_t_min > max_fc_t_min:
                max_fc_t_min = fc_t_min
                best_candidate = candidate

        # If the best fc_t_min is still <= threshold, return None
        if max_fc_t_min <= self.traj_fc_t_min_thre:
            return None

        return best_candidate

    def _get_traj_fc_t_min(self, init_config, target_config):
        num_intermediate_states = 50
        inter_states = np.linspace(init_config, target_config, num_intermediate_states)
        fc_t_min_array = np.zeros(num_intermediate_states)
        for i in range(len(inter_states) - 1):
            fc_t_min_array[i] = self.robot.get_fc_t_min(inter_states[i])
        return np.min(fc_t_min_array)

    def _eva_candidate_target_cost(self, candidate_state: np.ndarray, local_init_state: np.ndarray):
        """
        Evaluate the cost of a candidate state.
        input: shape (N,)
        """
        if not self.is_state_feasible(candidate_state):
            return float('inf')

        path_progress = self._get_path_progress(candidate_state)
        dis2path = self._get_dis2path(candidate_state)

        total_cost = dis2path + (1 - path_progress)

        return total_cost

    def _get_path_progress(self, state: np.ndarray):
        """
        For a given state, calculate the progress along the planned path.
        First, find the closest point on the path,
        then retrieve the index of that point.
        The progress is defined as the ratio of the index to the total number of points in the path.
        """
        # Calculate the distance from the state to each point on the path
        distances = np.linalg.norm(self.path[:, :2] - state[:2], axis=1)
        # Find the index of the closest point on the path
        closest_index = np.argmin(distances)
        # Calculate the progress as a ratio of the closest index to the total number of points
        progress = closest_index / (len(self.path) - 1)
        return progress

    def _get_dis2path(self, state: np.ndarray):
        """Calculate the distance from the state to the closest point on the path."""
        # Calculate the distance from the state to each point on the path
        distances = np.linalg.norm(self.path[:, :2] - state[:2], axis=1)
        # Find the minimum distance
        min_distance = np.min(distances)
        return min_distance

    def is_state_feasible(self, state: np.ndarray):
        """Check if the configuration is both collision‐free and stable."""
        # collision check
        if not self._is_state_collision_free(state):
            return False

        # stability check
        if not self._is_state_stable(state):
            return False

        return True

    def _is_state_collision_free(self, state: np.ndarray):
        """
        input: state: (6, )
        Check if the state is collision free by checking the EDT value
        """
        # If ESDF constraint weight is zero, always consider state collision-free
        if self.esdf_constraint_weight == 0:
            return True

        rotor_edt_array, _ = self.bspline_planner.get_rotor_edt_grad(state)
        return np.all(rotor_edt_array > self.obs_clearance)

    def _is_state_stable(self, state: np.ndarray):
        """Check if the state is stable based on force closure to rotor position distances."""
        # If stability constraint weight is zero, always consider state stable
        if self.stability_constraint_weight == 0:
            return True

        # fc_rp_dists, _ = self.robot.get_fc_rp_dists_grad(state)
        fc_t_min = self.robot.get_fc_t_min(state)
        return fc_t_min > self.fc_t_min_thre

    def _cleanup(self):
        """Clean up resources, particularly the BSplinePlanner instance."""
        if hasattr(self, 'bspline_planner') and self.bspline_planner is not None:
            del self.bspline_planner
            self.bspline_planner = None
        gc.collect()
