import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import BSpline
from sympy import *
import pprint

M_left_0 = np.array([[1, 0, 0, 0],
                     [-3, 3, 0, 0],
                     [3, -9/2, 3/2, 0],
                     [-1, 7/4, -11/12, 1/6]])

M_left_1 = np.array([[1/4, 7/12, 1/6, 0],
                     [-3/4, 1/4, 1/2, 0],
                     [3/4, -5/4, 1/2, 0],
                     [-1/4, 7/12, -1/2, 1/6]])

M_inner = np.array([[1/6, 2/3, 1/6, 0],
                    [-1/2, 0, 1/2, 0],
                    [1/2, -1, 1/2, 0],
                    [-1/6, 1/2, -1/2, 1/6]])

M_right_B1 = np.array([[1/6, 2/3, 1/6, 0],
                       [-1/2, 0, 1/2, 0],
                       [1/2, -1, 1/2, 0],
                       [-1/6, 1/2, -7/12, 1/4]])

M_right_B0 = np.array([[1/6, 7/12, 1/4, 0],
                       [-1/2, -1/4, 3/4, 0],
                       [1/2, -5/4, 3/4, 0],
                       [-1/6, 11/12, -7/4, 1]])


p = 3
t = symbols('t')
h = symbols('h')
x = symbols('x')

# Example usage with ref_index=4
u = [0, 0, 0, 0, h, 2*h, 3*h, 4*h, 5*h, 6*h, 7*h, 8*h, 9*h, 10*h, 11*h, 12*h, 12*h, 12*h, 12*h]

# u = [0, 0, 0, 0, h, 2*h, 3*h, 4*h, 5*h, 6*h, 7*h, 8*h, 7*h, 7*h]
S = [1, t, t**2, t**3]

M = len(u)
N = M - 4


def get_M(i):
    if i == 0:
        return M_left_0

    elif i == 1:
        return M_left_1

    elif i == N - p - 2:
        return M_right_B1

    elif i == N - p - 1:
        return M_right_B0

    else:
        return M_inner


basis_list = [0] * N
basis_d_list = [0] * N
basis_d_integral_list = [0] * N
# init Q mat as a N*N zero mat using list
Q = [[0 for i in range(N)] for j in range(N)]
# u: the ref index in the valid knot vector, (0 <= u <= N-p-1)
# i: the index of the basis function, i = p, ..., N-1, for i, we obtain [N_{i-3}, N_{i-2}, N_{i-1}, N_{i}]

for u in range(N-3):  # accumulate the whole time along the traj
    i = u + 3
    x = h * (i + t - 3)
    M = get_M(u)

    basis_vector = S @ M  # [N_{i-3}, N_{i-2}, N_{i-1}, N_{i}]

    for k in range(4):
        basis_list[u+k] = basis_vector[k]
        basis_d_list[u+k] = diff(basis_vector[k], t)

    for k in range(4):
        Q[u+k][u+k] += 1/h * integrate(basis_d_list[u+k]**2, (t, 0, 1))
        if u+k+1 < N:
            Q[u+k][u+k+1] += 1/h * integrate(basis_d_list[u+k] * basis_d_list[u+k+1], (t, 0, 1))
        if u+k+2 < N:
            Q[u+k][u+k+2] += 1/h * integrate(basis_d_list[u+k] * basis_d_list[u+k+2], (t, 0, 1))
        if u+k+3 < N:
            Q[u+k][u+k+3] += 1/h * integrate(basis_d_list[u+k] * basis_d_list[u+k+3], (t, 0, 1))

for i in range(N):
    for j in range(i+1, N):
        Q[j][i] = Q[i][j]

Q = Matrix(Q)

pprint.pprint(Q)
