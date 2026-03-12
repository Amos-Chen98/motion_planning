import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import BSpline
from sympy import *
import pprint


def compute_basis_functions(ref_index, u):
    N = [[0 for _ in range(50)] for _ in range(50)]

    # Initialize the base case
    N[ref_index][0] = 1

    # Compute B-spline basis functions iteratively
    for p in range(1, 4):  # p = 1, 2, 3
        for j in range(ref_index-p, ref_index+1):
            N[j][p] = get_basis(j, p, u, N)

    # Print the ordered coefficients
    for i in range(3, -1, -1):
        current_basis = N[ref_index-i][3]
        coefs_dict = current_basis.as_coefficients_dict()

        # For cubic polynomials, we need coefficients for t^0, t^1, t^2, t^3
        ordered_coeffs = []
        for power in range(4):
            term = t**power if power > 0 else 1
            coef = coefs_dict.get(term, 0)
            ordered_coeffs.append(coef)

        # Print coefficients without labels
        print(f"{ref_index - i}: {', '.join(str(coef) for coef in ordered_coeffs)}")

    print("\n")
    return N[ref_index-3][3], N[ref_index-2][3], N[ref_index-1][3], N[ref_index][3]


def get_basis(j, p, u, N):
    l_deno = u[j+p] - u[j]
    if l_deno != 0:
        l = ((x - u[j]) / l_deno).simplify()
    else:
        l = 0

    r_deno = u[j+p+1] - u[j+1]
    if r_deno != 0:
        r = ((u[j+p+1] - x) / r_deno).simplify()
    else:
        r = 0

    basis = (l * N[j][p-1] + r * N[j+1][p-1]).simplify().expand()

    return basis


if __name__ == "__main__":
    t = symbols('t')
    h = symbols('h')
    x = symbols('x')

    # Example usage with ref_index=4
    u = [0, 0, 0, 0, h, 2*h, 3*h, 4*h, 5*h, 6*h, 7*h, 8*h, 9*h, 10*h, 11*h, 12*h, 12*h, 12*h, 12*h]

    ref_index = 3
    x = h * (t+ref_index-3)
    B1, B2, B3, B4 = compute_basis_functions(ref_index, u)

    print(B1)
    print(B2)
    print(B3)
    print(B4)

    # Plot the basis functions
    # t_array = np.linspace(0, 1, 100)
    # B1_array = np.zeros_like(t_array)
    # B2_array = np.zeros_like(t_array)
    # B3_array = np.zeros_like(t_array)
    # B4_array = np.zeros_like(t_array)

    # for i in range(len(t_array)):
    #     B1_array[i] = B1.subs(t, t_array[i])
    #     B2_array[i] = B2.subs(t, t_array[i])
    #     B3_array[i] = B3.subs(t, t_array[i])
    #     B4_array[i] = B4.subs(t, t_array[i])

    # plt.plot(t_array, B1_array, label="B1")
    # plt.plot(t_array, B2_array, label="B2")
    # plt.plot(t_array, B3_array, label="B3")
    # plt.plot(t_array, B4_array, label="B4")
    # # plt.ylim(0, 1)
    # plt.legend()
    # plt.show()
