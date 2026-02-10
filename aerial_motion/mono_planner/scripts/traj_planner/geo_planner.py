import os
import sys
current_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_path)
import copy
from astar_planner import AstarPlanner
import numpy as np
from traj_planner.traj_utils import TrajUtils


class GeoPlanner(TrajUtils):
    def __init__(self, a_star_config, move_vel, cmd_hz, traj_opt=False):
        super().__init__()
        self.astar_planner = AstarPlanner(a_star_config)
        self.move_vel = move_vel
        self.cmd_hz = cmd_hz
        self.traj_opt = traj_opt
        self.D = a_star_config.dim  # Planning dimension (2D or 3D)
        self.s = 3  # Used in read_planning_condition

    def geo_traj_plan(self, map, start_pos, target_pos, start_vel, target_vel):
        """Generate trajectory waypoints from start to target"""
        path = self.astar_planner.plan(map, start_pos, target_pos)

        if path is None:
            raise Exception("A* planner failed to find a path!")

        path = self.prune_path_nodes(map, path)
        pruned_path = copy.deepcopy(path)

        # Use minimum jerk trajectory optimization only for multi-waypoint paths
        if self.traj_opt and len(path) > 2:
            print("Minimum jerk trajectory!")

            # Match dimensions: use only the first D dimensions
            start_pos_matched = np.array(start_pos[:self.D])
            target_pos_matched = np.array(target_pos[:self.D])
            start_vel_matched = np.array(start_vel[:self.D])
            target_vel_matched = np.array(target_vel[:self.D])

            # Remove first and last elements from path (will be added back via head/tail state)
            path = path[1:-1]
            path_array = np.array(path)

            # Construct full path for time calculation
            path_all = np.concatenate((np.array([start_pos_matched]),
                                       path_array,
                                       np.array([target_pos_matched])), axis=0)

            # Calculate distances and time segments
            distances = np.linalg.norm(np.diff(path_all, axis=0), axis=1)
            ts = distances / self.move_vel

            # Setup trajectory optimization
            head_state = np.array([start_pos_matched, start_vel_matched])
            target_state = np.array([target_pos_matched, target_vel_matched])
            int_wpts = path_array.T

            self.read_planning_condition(head_state, target_state, int_wpts, ts)

            des_pos_array = self.get_pos_array(hz=self.cmd_hz)

            return des_pos_array, pruned_path

        else:
            des_pos_array = []
            for i in range(len(path) - 1):
                seg_start = np.array(path[i])
                seg_end = np.array(path[i + 1])
                seg_length = np.linalg.norm(seg_end - seg_start)
                seg_time = seg_length / self.move_vel
                sample_num = int(seg_time * self.cmd_hz) + 1

                # Interpolate positions along this segment (skip first point to avoid duplicates)
                seg_positions = np.linspace(seg_start, seg_end, sample_num)[1:]
                des_pos_array.extend(seg_positions)

            return des_pos_array, pruned_path

    def read_planning_condition(self, head_state, tail_state, int_wpts, ts):
        """Read and store planning conditions for trajectory generation"""
        self.M = ts.shape[0]

        self.head_state = np.zeros((self.s, self.D))
        self.tail_state = np.zeros((self.s, self.D))

        for i in range(min(self.s, head_state.shape[0])):
            self.head_state[i] = head_state[i]
        for i in range(min(self.s, tail_state.shape[0])):
            self.tail_state[i] = tail_state[i]

        self.int_wpts = int_wpts
        self.ts = ts

    def prune_path_nodes(self, map, path):
        '''
        Prune unnecessary waypoints from path while maintaining feasibility.
        Including the head and tail of the path.
        '''
        key_index = [0]
        head_index = int(0)
        tail_index = int(1)

        while tail_index < len(path):
            while map.seg_feasible_check(path[head_index], path[tail_index]) or tail_index - head_index == 1:
                tail_index += 1
                if tail_index == len(path):
                    break

            # the path from head_index to [tail_index - 1] is feasible
            # the path from head_index to tail_index is not feasible
            key_index.append(tail_index - 1)
            head_index = copy.deepcopy(tail_index - 1)  # reset the head_index

        path_pruned = []
        for i in key_index:
            path_pruned.append(path[i])

        return path_pruned
