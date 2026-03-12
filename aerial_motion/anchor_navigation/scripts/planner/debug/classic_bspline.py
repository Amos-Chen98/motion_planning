import numpy as np


class CubicBSpline:
    def __init__(self, p=3):
        self.p = p
        self.M = (1 / 6) * np.array([
            [1, 4, 1, 0],
            [-3, 0, 3, 0],
            [3, -6, 3, 0],
            [-1, 3, -3, 1]
        ])

    def get_coeffs(self, control_points: np.ndarray, T):
        self.control_points = control_points
        self.T = T
        self.N = len(control_points)
        self.valid_knots = self.T * np.linspace(0, 1, self.N - self.p + 1)
        A = [np.array(control_points[i:i+self.p+1]).T for i in range(self.N-self.p)]
        return A

    def get_index(self, t) -> int:
        if t < self.valid_knots[0] or t > self.valid_knots[-1]:
            return -1
        return max(np.searchsorted(self.valid_knots, t) - 1, 0)

    def get_pos_time_basis(self, t, i: int):
        s = (self.N - self.p) / self.T * (t - self.valid_knots[i])
        return np.array([1, s, s**2, s**3])

    def get_vel_time_basis(self, t, i: int):
        s = (self.N - self.p) / self.T * (t - self.valid_knots[i])
        return np.array([0, 1, 2*s, 3*s**2])

    def get_pos(self, t, A: np.ndarray) -> np.ndarray:
        i = self.get_index(t)
        if i == -1:
            return None
        S = self.get_pos_time_basis(t, i)
        pos = A[i] @ self.M.T @ S
        return pos

    def get_vel(self, t, A: np.ndarray) -> np.ndarray:
        i = self.get_index(t)
        if i == -1:
            return None
        S = self.get_vel_time_basis(t, i)
        vel = A[i] @ self.M.T @ S
        return vel

    def get_pos_array(self, A, total_time, cmd_hz) -> np.ndarray:
        '''
        return a 2D array of desired positions, shape: (sample_num, 6)
        '''
        time_array = np.linspace(0, total_time, int(total_time * cmd_hz))
        pos_array = np.array([self.get_pos(t, A) for t in time_array])

        return pos_array

    def get_vel_array(self, A, total_time, cmd_hz) -> np.ndarray:
        time_array = np.linspace(0, total_time, int(total_time * cmd_hz))
        vel_array = np.array([self.get_vel(t, A) for t in time_array])

        return vel_array
