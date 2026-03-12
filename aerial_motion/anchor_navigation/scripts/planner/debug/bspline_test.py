import sys
from pathlib import Path
path_of_interest = str(Path(__file__).resolve().parents[1])
sys.path.insert(0, path_of_interest)
from clamped_bspline import ClampedBSpline
import numpy as np


start_pos = np.array([-1, 0, 0, 0, 0, 0])
end_pos = np.array([10, 0, 0, 0, 0, 0])
start_vel = np.array([1, 0, 0, 0, 0, 0])
end_vel = np.array([0, -5, 0, 0, 0, 0])

control_points = np.random.rand(10, 6)

T = 10

time_array = np.linspace(0, T, 1000)

time_interval = T / (10+4-3)
sec_control_point = start_pos + start_vel * time_interval/3

sec2last_control_point = end_pos - end_vel * time_interval/3

full_control_points = np.vstack([start_pos, sec_control_point, control_points, sec2last_control_point, end_pos])

bspline = ClampedBSpline(p=3)
bspline.set_boundary(start_pos, end_pos, start_vel, end_vel, T)
bspline.build_spline(control_points)

constraint, grad = bspline.set_vel_constraint()

print("shape = ", constraint.shape)
print("shape = ", grad.shape)

print(grad[-1])
