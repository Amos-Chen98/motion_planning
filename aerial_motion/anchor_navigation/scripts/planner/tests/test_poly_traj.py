import os
import sys
current_path = os.path.abspath(os.path.dirname(__file__))
parent_path = os.path.dirname(current_path)
sys.path.insert(0, parent_path)

import numpy as np
import matplotlib.pyplot as plt
from poly_traj import PolyTraj

poly_traj = PolyTraj(p=3, dim=2)

coeffs = np.array([[0, 0, 1, 0], [0, 0, 0, 1]])  # Simple linear trajectory in 2D
poly_traj.update_coeffs(coeffs)

pos_array = poly_traj.get_pos_array(cmd_hz=10, duration=1.0)

# print the position array
print("Position Array:\n", pos_array)

