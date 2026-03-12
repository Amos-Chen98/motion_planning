import os
import sys
current_path = os.path.abspath(os.path.dirname(__file__))
parent_path = os.path.dirname(current_path)
sys.path.insert(0, parent_path)
from gimbal_yaw_planner import GimbalYawPlanner
import matplotlib.pyplot as plt
import numpy as np



def create_mock_planner_config():
    """
    Create a mock planner configuration based on the get_planner_config function
    from motion_planner_node.py
    """
    planner_config = {}

    # Robot config from URDF (default values)
    planner_config['config_dim'] = 6  # 3 for position/yaw + 3 joints
    planner_config['joint_num'] = 3
    planner_config['rotor_num'] = 4
    planner_config['rotor_directions'] = [1, -1, 1, -1]  # CCW, CW, CCW, CW
    planner_config['rotor_thrust_min'] = 1.0
    planner_config['rotor_thrust_max'] = 35.5
    planner_config['m_f_rate'] = -0.0182
    planner_config['link_L'] = 0.6
    planner_config['propeller_R'] = 0.2025

    # Motion planner node parameters
    planner_config['cmd_hz'] = 60
    planner_config['target_reach_threshold'] = 0.2

    # Global planner parameters
    planner_config['astar_resolution'] = 0.1
    planner_config['astar_extra_clearance'] = 0.10
    planner_config['seg_len'] = planner_config['link_L'] * 1.0  # seg_len_factor = 1.0
    planner_config['travel_speed'] = 0.1
    planner_config['bend_attempt_num'] = 5
    planner_config['transform_attempt_num'] = 20
    planner_config['candidate_int_eva_num'] = 10

    # Motion constraints
    planner_config['min_joint_angle'] = 0.5
    planner_config['joint_angle_bound'] = np.pi / 2
    planner_config['translational_vel_limit'] = 2.0
    planner_config['rotational_vel_limit'] = 1.0
    planner_config['dis2obs_min'] = 0.05
    planner_config['fc_rp_dis_min'] = planner_config['link_L'] * 0.05  # fc_threshold_factor = 0.05

    # BSpline planner and recovery parameters
    planner_config['stage_control_points_num'] = 5
    planner_config['collision_cost_tolerance'] = 1.0
    planner_config['stability_violation_tolerance'] = 5.0
    planner_config['opt_max_retries'] = 5

    # Optimizer parameters
    planner_config['ftol_rel'] = 1e-4
    planner_config['maxtime'] = 120
    planner_config['constraint_tol'] = 1e-4
    planner_config['esdf_constraint_weight'] = 1e3
    planner_config['vel_constraint_weight'] = 1
    planner_config['stability_constraint_weight'] = 0

    # Computation config
    planner_config['find_anchor'] = False
    planner_config['is_parallel'] = True

    # UI config
    planner_config['is_goal_complete'] = True

    return planner_config


# Create mock planner config
mock_planner_config = create_mock_planner_config()

# Build correct URDF path
scripts_path = os.path.dirname(parent_path)
motion_planner_path = os.path.dirname(scripts_path)
urdf_path = os.path.join(motion_planner_path, 'urdf', 'hydrus_xi_20241227.urdf')

gimbal_yaw_planner = GimbalYawPlanner(
    robot_urdf=urdf_path, planner_config=mock_planner_config)

joint_num = 3

time = 10.0

cmd_hz = 60

# Define waypoints in degrees for smooth transition
waypoints_deg = [
    [0, 0, 0],      # Initial state
    [90, 90, 90],   # First target
    [45, 45, 45],   # Second target  
    [90, 90, 90],   # Third target
    [0, 0, 0]       # Final state
]

# Convert waypoints to radians
waypoints_rad = [np.radians(wp) for wp in waypoints_deg]

# Create time indices for each waypoint
total_steps = int(time * cmd_hz)
num_segments = len(waypoints_rad) - 1
steps_per_segment = total_steps // num_segments

# Initialize the desired joint angles array
des_joint_angles = np.zeros((total_steps, joint_num))

# Generate smooth transitions between waypoints using linear interpolation
current_step = 0
for i in range(num_segments):
    start_angles = waypoints_rad[i]
    end_angles = waypoints_rad[i + 1]
    
    # Determine the number of steps for this segment
    if i == num_segments - 1:  # Last segment gets remaining steps
        segment_steps = total_steps - current_step
    else:
        segment_steps = steps_per_segment
    
    # Linear interpolation between start and end angles
    for step in range(segment_steps):
        alpha = step / (segment_steps - 1) if segment_steps > 1 else 1.0
        interpolated_angles = [(1 - alpha) * start + alpha * end 
                             for start, end in zip(start_angles, end_angles)]
        des_joint_angles[current_step + step] = interpolated_angles
    
    current_step += segment_steps



print("First 5 rows of desired joint angles (in radians):")
print(des_joint_angles[:5])

des_gimbal_angles_array = gimbal_yaw_planner.plan(des_joint_angles, time)

# print the first 5 rows of the desired gimbal angles array
print("First 5 rows of desired gimbal angles (in radians):")
# print in .2f format, both in rad and degrees
for i in range(5):
    angles_rad = des_gimbal_angles_array[i]
    angles_deg = np.degrees(angles_rad)
    print(f"Time step {i}: Gimbal angles (rad) = {angles_rad}, (degrees) = {angles_deg}")

# print with degree formatting, round to 2 decimal places
for i, angles in enumerate(des_gimbal_angles_array):
    formatted_angles = [f"{angle:.2f}" for angle in angles]
    # print(f"Time step {i}: Gimbal angles (degrees) = {formatted_angles}")


# Plot gimbal angle curves
def plot_gimbal_angles(gimbal_angles, des_joint_angles, time_duration, cmd_hz):
    """
    Plot the gimbal angle curves over time along with desired joint angles.

    Args:
        gimbal_angles: numpy array of shape (time_steps, num_gimbals) containing gimbal angles in radians
        des_joint_angles: numpy array of shape (time_steps, num_joints) containing joint angles in radians
        time_duration: total time duration in seconds
        cmd_hz: command frequency in Hz
    """
    time_steps = len(gimbal_angles)
    time_array = np.linspace(0, time_duration, time_steps)
    num_gimbals = gimbal_angles.shape[1]

    plt.figure(figsize=(12, 8))

    # Define colors for different gimbals
    colors = ['b', 'r', 'g', 'c', 'm', 'y']

    # Plot gimbal angles
    for gimbal_idx in range(num_gimbals):
        plt.plot(time_array, gimbal_angles[:, gimbal_idx],
                 color=colors[gimbal_idx % len(colors)],
                 linewidth=2,
                 label=f'Gimbal {gimbal_idx + 1}',
                 marker='o' if time_steps <= 50 else None,
                 markersize=4 if time_steps <= 50 else 0)

    # Plot desired joint angle (first joint only, as all joints have the same angle)
    plt.plot(time_array, des_joint_angles[:, 0],
             color='black',
             linewidth=3,
             linestyle='--',
             label='Desired Joint Angle',
             marker='s' if time_steps <= 50 else None,
             markersize=3 if time_steps <= 50 else 0)

    plt.xlabel('Time (seconds)', fontsize=12)
    plt.ylabel('Angle (rad)', fontsize=12)
    plt.title('Gimbal Angles and Desired Joint Angle Trajectories Over Time', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=10)

    # Add some formatting
    plt.tight_layout()

    # Show statistics
    plt.figtext(0.02, 0.02, f'Duration: {time_duration}s | Frequency: {cmd_hz}Hz | Steps: {time_steps}',
                fontsize=8, style='italic')

    # Show the plot directly
    plt.show()


# Create the plot
plot_gimbal_angles(des_gimbal_angles_array, des_joint_angles, time, cmd_hz)

# Print summary statistics
print("\n=== Gimbal Angle Summary ===")
print(f"Number of gimbals: {des_gimbal_angles_array.shape[1]}")
print(f"Time steps: {des_gimbal_angles_array.shape[0]}")
print(f"Duration: {time}s at {cmd_hz}Hz")

for gimbal_idx in range(des_gimbal_angles_array.shape[1]):
    angles = des_gimbal_angles_array[:, gimbal_idx]
    print(f"\nGimbal {gimbal_idx + 1}:")
    print(f"  Range: {np.min(angles):.2f}° to {np.max(angles):.2f}°")
    print(f"  Mean: {np.mean(angles):.2f}°")
    print(f"  Std Dev: {np.std(angles):.2f}°")
    print(f"  Final angle: {angles[-1]:.2f}°")


# TODO
