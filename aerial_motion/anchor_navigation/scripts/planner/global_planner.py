import os
import sys
current_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_path)
import traceback
from visualizer.visualizer import Visualizer
from traj_coder import TrajCoder
from robot_model import Robot
import gc
import multiprocessing
import signal
from key_states_decider import KeyStatesDecider
from bspline_planner import BSplinePlanner
import numpy as np
import rospy


# Global variables for worker processes (shared data to reduce serialization overhead)
_worker_planner_config = None
_worker_map = None
_worker_robot_urdf = None


def _init_worker(planner_config, map_data, robot_urdf):
    """Initialize worker process with shared data to avoid redundant serialization."""
    global _worker_planner_config, _worker_map, _worker_robot_urdf
    
    # Prevent worker processes from interfering with ROS signal handling
    # Reset signal handlers to default to avoid inheriting parent's ROS handlers
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    
    _worker_planner_config = planner_config
    _worker_map = map_data
    _worker_robot_urdf = robot_urdf


def _worker_simplified(idx, local_init_state, local_target_state):
    """Simplified worker function that uses global shared data."""
    global _worker_planner_config, _worker_map, _worker_robot_urdf
    local_planner = None
    try:
        local_planner = BSplinePlanner(_worker_robot_urdf, _worker_planner_config, _worker_map)

        result = local_planner.local_plan(
            local_init_state=local_init_state,
            local_target_state=local_target_state
        )

        return idx, result
    except Exception as e:
        # Log error details but don't let it crash the worker
        error_msg = f"Worker {idx} planning failed: {str(e)}\n{traceback.format_exc()}"
        # Write to stderr so parent process can see it
        sys.stderr.write(f"[WORKER ERROR] {error_msg}\n")
        sys.stderr.flush()
        # Return error result instead of raising
        return idx, None
    finally:
        if local_planner is not None:
            del local_planner
        gc.collect()


class RobotConfig:
    def __init__(self):
        self.rootlink_pos_x = 0.0
        self.rootlink_pos_y = 0.0
        self.rootlink_yaw = 0.0
        self.joint_angles = []


class GlobalPlanner():
    def __init__(self, robot_urdf, planner_config):
        self.robot_urdf = robot_urdf

        # robot
        self.robot = Robot(robot_urdf)
        self.config_dim = self.robot.param['config_dim']

        # visualizer
        self.visualizer = Visualizer(robot_urdf)

        # Constraints on the motion
        self.min_joint_angle = planner_config['min_joint_angle']
        self.joint_angle_bound = planner_config['joint_angle_bound']

        # Mission config
        self.find_anchor = planner_config['find_anchor']
        self.is_parallel = planner_config['is_parallel']
        self.save_traj = planner_config['save_traj']

        # For intermediate state generation
        self.seg_len = planner_config['seg_len']
        self.obs_clearance = planner_config['dis2obs_min'] + self.robot.param['propeller_R']
        self.fc_rp_dis_min = planner_config['fc_rp_dis_min']

        # intermediate state decider
        self.bend_attempt_num = planner_config['bend_attempt_num']
        self.transform_attempt_num = planner_config['transform_attempt_num']
        self.candidate_int_eva_num = planner_config['candidate_int_eva_num']

        self.planner_config = planner_config

        # Initialize KeyStatesDecider
        self.key_states_decider = KeyStatesDecider(robot_urdf, planner_config)

        # Initialize TrajCoder for trajectory saving/loading
        if self.save_traj:
            self.traj_coder = TrajCoder(robot_urdf)

        # Flag
        self.key_states_complete = False

    def plan(self, map, init_state: RobotConfig, target_state: RobotConfig):
        self.map = map
        self.key_states_decider.init_bspline_planner(map)

        global_init_state = np.array([np.array([init_state.rootlink_pos_x,
                                                init_state.rootlink_pos_y,
                                                init_state.rootlink_yaw] + list(init_state.joint_angles)),
                                      np.zeros(self.config_dim)])

        global_target_state = np.array([np.array([target_state.rootlink_pos_x,
                                                  target_state.rootlink_pos_y,
                                                  target_state.rootlink_yaw] + list(target_state.joint_angles)),
                                        np.zeros(self.config_dim)])

        if not self.key_states_decider.is_state_feasible(global_target_state[0]):
            rospy.logerr("The target state is not feasible! Please set a new target state.")
            return None, None, []

        # Try initial planning
        des_states, key_states_viz, opt_logs = self._attempt_plan(global_init_state, global_target_state)

        # If initial planning fails, try fallback with rotated targets
        # if des_states is None:
        #     rospy.logwarn("Attempting fallback with rotated targets")
        #     des_states, key_states_viz = self._plan_with_rotated_targets(global_init_state, global_target_state)

        return des_states, key_states_viz, opt_logs

    def _attempt_plan(self, global_init_state, global_target_state):
        """Attempt to plan from init to target state. Returns (des_states, key_states_viz)."""
        # Log the initial and target states with angles in degrees
        rospy.loginfo(f"Global init state: pos=({global_init_state[0,0]:.2f}, {global_init_state[0,1]:.2f}), "
                      f"yaw={np.degrees(global_init_state[0,2]):.0f}°, "
                      f"joints=[{', '.join([f'{np.degrees(angle):.0f}°' for angle in global_init_state[0,3:]])}]")

        rospy.loginfo(f"Global target state: pos=({global_target_state[0,0]:.2f}, {global_target_state[0,1]:.2f}), "
                      f"yaw={np.degrees(global_target_state[0,2]):.0f}°, "
                      f"joints=[{', '.join([f'{np.degrees(angle):.0f}°' for angle in global_target_state[0,3:]])}]")

        self.global_target_state = global_target_state

        if self.find_anchor:
            all_key_states, self.key_states_complete = self.key_states_decider.set_key_states_list(
                global_init_state, global_target_state, self.map)

            key_states_viz = self.visualizer.get_key_states_viz(all_key_states)

            if self.key_states_complete:
                rospy.loginfo(f"Generated {len(all_key_states)} key states total")
            else:
                rospy.logerr("Some key states are not feasible.")
                return None, key_states_viz
        else:
            all_key_states = [global_init_state, global_target_state]
            key_states_viz = None

        planning_tasks, task_num = self._set_planning_tasks(all_key_states)

        if self.is_parallel:
            results = self._execute_parallel_planning(planning_tasks, task_num)
        else:
            results = self._execute_sequential_planning(planning_tasks)

        if results is None:
            return None, key_states_viz, []
        else:
            # Combine the results into a single list
            des_states, opt_logs = self._combine_des_states(results)
            return des_states, key_states_viz, opt_logs

    def _combine_des_states(self, results):
        """Combine the desired states from all planning segments."""
        des_states = []
        traj_segments = []
        opt_logs = []

        for i, result in enumerate(results):
            if self.save_traj:
                # Result format: (local_des_states, local_traj, opt_log)
                local_des_states = result[0]
                local_traj = result[1]
                opt_log = result[2]
                traj_segments.append(local_traj)
                opt_logs.append(opt_log)
            else:
                # Result format: (local_des_states, opt_log)
                local_des_states = result[0]
                opt_log = result[1]
                opt_logs.append(opt_log)

            if i == 0:
                # First segment, add all trajectory points
                des_states.extend(local_des_states)
            else:
                # Subsequent segments, skip the first point (to avoid duplication)
                des_states.extend(local_des_states[1:])

        # Save trajectory segments if enabled
        if self.save_traj and traj_segments:
            try:
                saved_filepath = self.traj_coder.save_trajectory_segments(traj_segments)
                if not saved_filepath:
                    rospy.logwarn("Failed to save trajectory segments")
            except Exception as e:
                rospy.logerr(f"Error saving trajectory segments: {e}")

        return des_states, opt_logs

    def _execute_parallel_planning(self, planning_tasks, task_num):
        rospy.loginfo("Using parallel planning with shared data optimization")

        # Extract only the varying data (init_state and target_state) for each task
        # planning_tasks format: (planner_config, map, robot_urdf, init_state, target_state)
        simplified_tasks = [
            (i, planning_tasks[i][3], planning_tasks[i][4])  # idx, init_state, target_state
            for i in range(task_num)
        ]

        pool = None
        try:
            # Create pool with safeguards to prevent ROS interference
            # - maxtasksperchild=1: Each worker only handles one task then exits cleanly
            # - initializer: Sets up worker with isolated signal handlers
            rospy.loginfo(f"Creating worker pool with {task_num} processes (maxtasksperchild=1)")
            pool = multiprocessing.Pool(
                processes=task_num,
                initializer=_init_worker,
                initargs=(self.planner_config, self.map, self.robot_urdf),
                maxtasksperchild=1  # Each worker handles only 1 task to avoid accumulation issues
            )
            
            rospy.loginfo("Starting parallel planning tasks...")
            indexed_results = pool.starmap(_worker_simplified, simplified_tasks)
            
            rospy.loginfo("All parallel planning tasks completed")
            
        except Exception as e:
            rospy.logerr(f"Parallel planning failed with exception: {e}\n{traceback.format_exc()}")
            return None
        finally:
            # Explicitly close and join the pool to ensure clean shutdown
            if pool is not None:
                try:
                    rospy.loginfo("Closing worker pool...")
                    pool.close()  # Prevent any more tasks from being submitted
                    
                    # Implement timeout-based join using polling (pool.join() doesn't support timeout)
                    import time
                    join_timeout = 5.0  # seconds
                    start_time = time.time()
                    
                    # Poll worker status until timeout
                    while time.time() - start_time < join_timeout:
                        if all(not p.is_alive() for p in pool._pool):
                            rospy.loginfo("Worker pool closed successfully")
                            break
                        time.sleep(0.1)  # Check every 100ms
                    else:
                        # Timeout reached, some workers still alive
                        rospy.logwarn(f"Some workers did not exit within {join_timeout}s, forcing termination...")
                        pool.terminate()
                        
                        # Wait briefly for termination
                        terminate_start = time.time()
                        while time.time() - terminate_start < 2.0:
                            if all(not p.is_alive() for p in pool._pool):
                                rospy.loginfo("Worker pool terminated successfully")
                                break
                            time.sleep(0.1)
                        else:
                            # Termination timeout, force kill
                            rospy.logerr("Failed to terminate all workers, forcing kill...")
                            for p in pool._pool:
                                if p.is_alive():
                                    try:
                                        os.kill(p.pid, signal.SIGKILL)
                                    except:
                                        pass
                    
                    # Final join to clean up resources
                    pool.join()
                        
                except AttributeError:
                    # pool._pool might not exist if pool creation failed
                    rospy.loginfo("Worker pool cleanup completed (no active workers)")
                except Exception as e:
                    rospy.logwarn(f"Error during pool cleanup: {e}")
                finally:
                    # Force cleanup
                    del pool
                    gc.collect()

        # Check for any failed workers (result is None)
        if any(result is None for _, result in indexed_results):
            rospy.logerr("One or more planning stages failed in parallel execution")
            return None

        indexed_results.sort(key=lambda x: x[0])
        results = [result for _, result in indexed_results]
        return results

    def _execute_sequential_planning(self, planning_tasks):
        rospy.loginfo("Using sequential planning")
        results = []
        for i, task in enumerate(planning_tasks):
            rospy.loginfo(f"=================== Planning stage: {i+1}/{len(planning_tasks)} ===================")
            try:
                result = self._worker(*task)
            except Exception as e:
                rospy.logerr(f"Planning failed: {e}")
                return None
            results.append(result)
            rospy.loginfo("")  # blank line
        return results

    def _set_planning_tasks(self, all_key_states):
        task_num = len(all_key_states) - 1

        # Create all tasks in a single list comprehension
        planning_tasks = [
            (self.planner_config, self.map, self.robot_urdf,
             all_key_states[i], all_key_states[i + 1])
            for i in range(task_num)
        ]

        return planning_tasks, task_num

    def _indexed_worker(self, idx, planner_config, map, robot_urdf, local_init_state,
                        local_target_state):
        try:
            result = self._worker(planner_config, map, robot_urdf, local_init_state,
                                  local_target_state)
            return idx, result
        except Exception as e:
            raise e

    def _worker(self, planner_config, map, robot_urdf, local_init_state, local_target_state):
        local_planner = None
        try:
            local_planner = BSplinePlanner(robot_urdf, planner_config, map)

            result = local_planner.local_plan(
                local_init_state=local_init_state,
                local_target_state=local_target_state
            )

            # Handle different return types based on save_traj setting
            if planner_config['save_traj']:
                # Returns (local_des_states, traj, opt_log)
                return result
            else:
                # Returns (local_des_states, opt_log)
                return result

        except Exception as e:
            raise
        finally:
            if local_planner is not None:
                del local_planner
            gc.collect()

    def _get_cog_pos_xy(self, robot_config):
        """Get the center of gravity position in 2D."""
        cog_pos, _ = self.robot.get_cog_pos_jac(robot_config)
        return cog_pos[:2]  # Only x, y components

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

    def _plan_with_rotated_targets(self, global_init_state, original_target_state):
        """
        Fallback planning using rotated target configurations.

        Args:
            global_init_state: Initial robot state (2, 6)
            original_target_state: Original target state (2, 6)

        Returns:
            Tuple of (des_states, key_states_viz) or (None, None) if all attempts fail
        """
        # Compute target's center of gravity and make a copy to ensure immutability
        target_cog_2d = self._get_cog_pos_xy(original_target_state[0]).copy()

        # Generate rotated targets: +90°, +180°, +270° around CoG
        rotation_angles = [np.pi/2, np.pi, 3*np.pi/2]  # 90°, 180°, 270°

        for i, angle in enumerate(rotation_angles):
            rospy.loginfo(f"Attempting fallback planning with {np.degrees(angle):.0f}° rotated target (attempt {i+1}/3)")

            rotated_pos = self._calculate_rotated_position(original_target_state[0], target_cog_2d, angle)

            rotated_target_state = np.array([rotated_pos, np.zeros(self.config_dim)])

            # Check if rotated target is feasible
            if not self.key_states_decider.is_state_feasible(rotated_target_state[0]):
                rospy.logwarn(f"Rotated target at {np.degrees(angle):.0f}° is not feasible, skipping")
                continue

            # Attempt planning from start to rotated target
            first_segment, first_viz, first_opt_logs = self._attempt_plan(global_init_state, rotated_target_state)

            if first_segment is not None:
                rospy.loginfo(f"Successfully planned to {np.degrees(angle):.0f}° rotated target, now planning back to original")

                # Plan from rotated target back to original target
                second_segment, second_viz, second_opt_logs = self._attempt_plan(rotated_target_state, original_target_state)

                if second_segment is not None:
                    rospy.loginfo("Successfully completed fallback planning with two-segment path")

                    # Concatenate the two segments (avoid duplication of intermediate state)
                    combined_trajectory = first_segment + second_segment[1:]

                    # Combine visualization data
                    combined_viz = self._combine_visualization_data(first_viz, second_viz)

                    # Combine optimization logs
                    combined_opt_logs = first_opt_logs + second_opt_logs

                    return combined_trajectory, combined_viz, combined_opt_logs
                else:
                    rospy.logwarn(f"Failed to plan back from {np.degrees(angle):.0f}° rotated target to original target")
            else:
                rospy.logwarn(f"Failed to plan to {np.degrees(angle):.0f}° rotated target")

        rospy.logerr("All fallback planning attempts failed")
        return None, None, []

    def _combine_visualization_data(self, first_viz, second_viz):
        """
        Combine visualization data from two planning segments.

        Args:
            first_viz: Visualization data from first segment
            second_viz: Visualization data from second segment

        Returns:
            Combined visualization data
        """
        if first_viz is None and second_viz is None:
            return None

        if first_viz is None:
            return second_viz

        if second_viz is None:
            return first_viz

        # Combine the two visualization data sets
        first_links, first_rotors = first_viz
        second_links, second_rotors = second_viz

        combined_links = first_links + second_links
        combined_rotors = first_rotors + second_rotors

        return combined_links, combined_rotors
