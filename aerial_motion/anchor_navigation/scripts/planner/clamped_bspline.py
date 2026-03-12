import numpy as np
from scipy.interpolate import BSpline
from scipy.integrate import quad


class ClampedBSpline:
    def __init__(self, p=3, speed_bound=None):
        self.p = p

        self.M_left_0 = np.array([[1, 0, 0, 0],
                                  [-3, 3, 0, 0],
                                  [3, -9/2, 3/2, 0],
                                  [-1, 7/4, -11/12, 1/6]])

        self.M_left_1 = np.array([[1/4, 7/12, 1/6, 0],
                                  [-3/4, 1/4, 1/2, 0],
                                  [3/4, -5/4, 1/2, 0],
                                  [-1/4, 7/12, -1/2, 1/6]])

        self.M_inner = np.array([[1/6, 2/3, 1/6, 0],
                                 [-1/2, 0, 1/2, 0],
                                 [1/2, -1, 1/2, 0],
                                 [-1/6, 1/2, -1/2, 1/6]])

        self.M_right_B1 = np.array([[1/6, 2/3, 1/6, 0],
                                    [-1/2, 0, 1/2, 0],
                                    [1/2, -1, 1/2, 0],
                                    [-1/6, 1/2, -7/12, 1/4]])

        self.M_right_B0 = np.array([[1/6, 7/12, 1/4, 0],
                                    [-1/2, -1/4, 3/4, 0],
                                    [1/2, -5/4, 3/4, 0],
                                    [-1/6, 11/12, -7/4, 1]])

        # Set default velocity bounds if not provided
        if speed_bound is None:
            self.speed_bound = np.inf  # No velocity limits when not specified
        else:
            # Ensure speed_bound is a numpy array without unnecessary wrapping
            self.speed_bound = np.asarray(speed_bound)

    def set_boundary(self, start_pos, end_pos, start_vel, end_vel, T):
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.start_vel = start_vel
        self.end_vel = end_vel
        self.T = T
        self.D = len(start_pos)

    def build_spline(self, control_points: np.ndarray):
        '''
        The number of intermediate control points N should satisfy N >= 4
        This is derived from the matrix Q formulation in get_Q()
        '''
        N = len(control_points)  # number of intermediate control points
        self.N = N
        konts = np.hstack([np.zeros(self.p),
                           np.linspace(0, self.T, N+4 - self.p + 1),
                           np.ones(self.p) * self.T])
        h = konts[self.p+1] - konts[self.p]  # time interval between two control points
        self.h = h
        sec_control_point = self.start_pos + self.start_vel * h/self.p
        sec2last_control_point = self.end_pos - self.end_vel * h/self.p

        full_control_points = np.vstack([self.start_pos, sec_control_point, control_points, sec2last_control_point, self.end_pos])
        self.full_control_points = full_control_points
        identity_control_points = np.eye(N+4)

        self.bspline = BSpline(konts, full_control_points, self.p)
        self.basis_vector = BSpline(konts, identity_control_points, self.p)  # the basis values, shape: (N+4, )
        self.basis_vector_d = self.basis_vector.derivative(1)  # the first derivative of the basis values, shape: (N+4, )

        self.valid_knots = np.linspace(0, self.T, N+4 - self.p + 1)

    def get_index(self, t) -> int:
        if t < self.valid_knots[0] or t > self.valid_knots[-1]:
            return -1
        return max(np.searchsorted(self.valid_knots, t) - 1, 0)

    def get_M(self, i):
        if i == 0:
            return self.M_left_0

        elif i == 1:
            return self.M_left_1

        elif i == self.N+4 - self.p - 2:
            return self.M_right_B1

        elif i == self.N+4 - self.p - 1:
            return self.M_right_B0

        else:
            return self.M_inner

    def get_pos_time_basis(self, t, i: int):
        s = (self.N+4 - self.p) / self.T * (t - self.valid_knots[i])
        return np.array([1, s, s**2, s**3])

    def get_vel_time_basis(self, t, i: int):
        s = (self.N+4 - self.p) / self.T * (t - self.valid_knots[i])
        return (self.N+4 - self.p) / self.T * np.array([0, 1, 2*s, 3*s**2])

    def get_basis_vector(self, t):
        i = self.get_index(t)
        S = self.get_pos_time_basis(t, i)

        if i == -1:
            return None

        return S @ self.get_M(i)

    def get_basis_derivatives(self, t):
        i = self.get_index(t)
        S = self.get_vel_time_basis(t, i)
        if i == -1:
            return None

        return S @ self.get_M(i)

    def get_pos(self, t):
        return self.bspline(t)

    def get_vel(self, t):
        return self.bspline(t, 1)

    def get_pos_array(self, cmd_hz=60):
        '''
        return a pos array from time 0 to time T, with freq of cmd_hz
        '''
        time_array = np.linspace(0, self.T, int(self.T * cmd_hz))
        pos_array = self.bspline(time_array)
        return pos_array

    def get_vel_array(self, cmd_hz=60):
        '''
        return a vel array from time 0 to time T, with freq of cmd_hz
        '''
        time_array = np.linspace(0, self.T, int(self.T * cmd_hz))
        vel_array = self.bspline(time_array, 1)
        return vel_array

    def get_grad_q2c(self, t):
        '''
        Return gradient of position with respect to intermediate control points only
        The shape of q is (D,)
        The shape of intermediate c is (N, D), each row is one control point
        if we flatten c by col into (N*D, ), then the gradient from q to c is a matrix of shape (D, N*D)
        '''
        # Get basis values for all control points (including start and end)
        full_basis_values = self.basis_vector(t)  # shape: (N+4, ), the basis vector at time t

        # Extract only the basis values for intermediate control points
        intermediate_basis_values = full_basis_values[2:-2]  # shape: (N, )

        grad = np.kron(np.eye(self.D), intermediate_basis_values)  # shape: (D, N*D)

        return grad

    def get_grad_v2c(self, t):
        '''
        Return gradient of velocity with respect to intermediate control points only
        The shape of v is (D,)
        The shape of intermediate c is (N, D), each row is one control point
        if we flatten c by col into (N*D, ), then the gradient from v to c is a matrix of shape (D, N*D)
        '''
        # Get first derivative of basis functions for all control points (including start and end)
        full_basis_derivatives = self.basis_vector_d(t)  # shape: (N+4, )

        # Extract only the basis values for intermediate control points
        intermediate_basis_derivatives = full_basis_derivatives[2:-2]  # shape: (N, )

        grad = np.kron(np.eye(self.D), intermediate_basis_derivatives)  # shape: (D, N*D)

        return grad

    def get_vel_energy_grad(self):
        """
        Return both the analytical energy cost and its gradient
        """
        self.Q = self.get_Q()
        cost = self.get_vel_energy()
        grad = self.get_vel_grad().flatten('F')
        return cost, grad

    def get_Q(self):
        Q = np.zeros((self.N+4, self.N+4))

        # Assign values to the upper triangular part (including diagonal)
        Q[0, 0:4] = [9/5, -51/40, -19/40, -1/20]
        Q[1, 1:5] = [3/2, 3/80, -1/4, -1/80]
        Q[2, 2:6] = [27/40, -1/30, -47/240, -1/120]

        for i in range(3, self.N-1):
            Q[i, i:i+4] = [2/3, -1/8, -1/5, -1/120]

        Q[-5, -5:-1] = [2/3, -1/8, -47/240, -1/80]
        Q[-4, -4:] = [2/3, -1/30, -1/4, -1/20]
        Q[-3, -3:] = [27/40, 3/80, -19/40]
        Q[-2, -2:] = [3/2, -51/40]
        Q[-1, -1] = 9/5

        # Use symmetry to fill lower triangle
        Q = Q + Q.T - np.diag(Q.diagonal())

        return Q / self.h

    def get_vel_energy(self):
        """
        Calculate the velocity energy analytically using the properties of cubic B-splines.
        Energy is defined as the integral of squared velocity along the trajectory.
        """
        # Calculate energy using the quadratic form
        energy = 0.5 * np.trace(self.full_control_points.T @ self.Q @ self.full_control_points)

        return energy

    def get_vel_grad(self):
        """
        Calculate the gradient of velocity energy with respect to intermediate control points.
        Returns a gradient matrix of shape (N, D)
        """
        # Calculate the gradient (derivative of the energy with respect to control points)
        # Note that we only return the gradient for intermediate control points
        grad = self.Q @ self.full_control_points
        return grad[2:-2, :]

    def get_Q_integral(self):
        Q = np.zeros((self.N+4, self.N+4))
        for i in range(self.N+4):
            for j in range(self.N+4):
                Q[i, j], _ = quad(lambda t: self.basis_vector_d(t)[i] * self.basis_vector_d(t)[j], 0, self.T)

        return Q

    def get_vel_energy_integral(self):
        '''
        Return the energy of velocity along the whole trajectory
        '''
        vel_energy, _ = quad(lambda t: np.sum(self.get_vel(t)**2, axis=0), 0, self.T)
        return vel_energy

    def set_vel_constraint(self):
        A = 1 / self.h * np.eye(self.N+4-1, self.N+4, 1) - np.eye(self.N+4-1, self.N + 4, 0)  # shape: (N+3, N+4)
        # A_ij=-1 when i=j, A_ij=1 when j=i+1, A_ij=0 otherwise

        vel_control_points = A @ self.full_control_points  # shape: (N+3, D)

        vel_control_points_vec = vel_control_points.flatten('F')

        speed_bound_vec = np.tile(self.speed_bound, (self.N+3, 1)).flatten(order='F')

        # Inequality constraints: -speed_bound <= velocity <= speed_bound
        # This translates to: vel - speed_bound <= 0 and -vel - speed_bound <= 0
        vel_constraint = np.hstack((vel_control_points_vec - speed_bound_vec,
                                    -vel_control_points_vec - speed_bound_vec))

        G = np.kron(np.eye(self.D), A)  # shape: ((N+3)*D, (N+4)*D), vel_control_points.flatten('F') = G @ full_control_points.flatten('F')

        # Extract gradient for intermediate control points only
        grad = G[:, 2*self.D:-2*self.D]  # shape: ((N+3)*D, N*D)

        grad = np.vstack([grad, -grad])

        return vel_constraint, grad
