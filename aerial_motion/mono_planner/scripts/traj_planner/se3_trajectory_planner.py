import numpy as np

from traj_planner.traj_utils import TrajUtils


class SE3TrajectoryPlanner(TrajUtils):
    def __init__(self):
        super().__init__()
        self.D = 6
        self.s = 3

    def read_planning_condition(self, head_state, tail_state, int_wpts, ts):
        self.M = int(ts.shape[0])

        self.head_state = np.zeros((self.s, self.D))
        self.tail_state = np.zeros((self.s, self.D))

        for index in range(min(self.s, head_state.shape[0])):
            self.head_state[index] = head_state[index]
        for index in range(min(self.s, tail_state.shape[0])):
            self.tail_state[index] = tail_state[index]

        self.int_wpts = int_wpts
        self.ts = np.asarray(ts, dtype=float)
        self.coeffs = None

    def plan(self, states, segment_times):
        if states.shape[0] < 2:
            raise ValueError("At least two states are required to generate a trajectory.")

        head_state = np.zeros((2, self.D))
        tail_state = np.zeros((2, self.D))
        head_state[0] = states[0]
        tail_state[0] = states[-1]

        if states.shape[0] > 2:
            int_wpts = states[1:-1].T
        else:
            int_wpts = np.zeros((self.D, 0))

        self.read_planning_condition(head_state, tail_state, int_wpts, np.asarray(segment_times, dtype=float))

    def sample_positions(self, sample_times):
        self.get_coeffs(self.int_wpts, self.ts)
        return np.vstack([self.get_pos(sample_time).reshape(-1) for sample_time in sample_times])
