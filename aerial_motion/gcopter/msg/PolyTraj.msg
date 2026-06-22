# Time-stamped piecewise-polynomial trajectory.

time      start_time   # trajectory t=0
int32     traj_id      # monotonic sequence
int32     order        # polynomial degree
float64[] durations    # one per piece
float64[] coef_x       # flattened by piece and coefficient column
float64[] coef_y
float64[] coef_z
