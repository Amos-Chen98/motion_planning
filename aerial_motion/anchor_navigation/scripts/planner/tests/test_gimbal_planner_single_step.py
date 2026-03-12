#!/usr/bin/env python3

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

# Add parent directory to sys.path to import planner modules
current_path = os.path.abspath(os.path.dirname(__file__))
parent_path = os.path.dirname(current_path)
sys.path.insert(0, parent_path)

# Import parent directory modules
from gimbal_yaw_planner import GimbalYawPlanner


def create_mock_planner_config():
    """
    Create a mock planner configuration
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


def create_joint_angle_array(num_points=100):
    """
    Create joint angle array from +90° to 0° to -90°, uniformly distributed
    Each element is a 3D vector with identical values
    
    Args:
        num_points: Total number of points (default: 100)
    
    Returns:
        np.array of shape (num_points, 3): Joint angles in radians
    """
    # Create angle transition from +90 to 0 to -90 degrees
    angles_deg = np.linspace(90, -90, num_points)
    angles_rad = np.radians(angles_deg)
    
    # Create joint angle array where each row has 3 identical values
    joint_angles = np.zeros((num_points, 3))
    for i in range(num_points):
        joint_angles[i] = [angles_rad[i], angles_rad[i], angles_rad[i]]
    
    return joint_angles, angles_deg


def test_gimbal_planner_single_step():
    """
    Test GimbalYawPlanner.get_once_gimbal_angles method with varying joint angles
    """
    print("=== Gimbal Planner Single Step Test ===")
    
    # Create mock planner config and initialize planner
    mock_planner_config = create_mock_planner_config()
    
    # Build correct URDF path: tests -> planner -> scripts -> motion_planner -> urdf
    # From tests, go up 3 levels to get to motion_planner
    tests_path = current_path
    planner_path = parent_path
    scripts_path = os.path.dirname(planner_path)
    motion_planner_path = os.path.dirname(scripts_path)
    urdf_path = os.path.join(motion_planner_path, 'urdf', 'hydrus_xi_20241227.urdf')
    print(f"Using URDF: {urdf_path}")
    
    # Verify the file exists
    if not os.path.exists(urdf_path):
        print(f"Error: URDF file not found at {urdf_path}")
        print(f"Current path: {current_path}")
        print(f"Motion planner path: {motion_planner_path}")
        return None, None, None, []
    
    gimbal_yaw_planner = GimbalYawPlanner(
        planner_config=mock_planner_config, 
        robot_urdf=urdf_path
    )
    
    # Create joint angle array
    num_points = 100
    joint_angles_array, angles_deg_array = create_joint_angle_array(num_points)
        
    print(f"Created {num_points} joint angle configurations")
    print(f"Joint angle range: {angles_deg_array[0]:.1f}° to {angles_deg_array[-1]:.1f}°")
    
    # Test get_once_gimbal_angles for each joint configuration
    gimbal_angles_results = []
    obj_value_results = []
    last_gimbal_angles = None
    failed_steps = []
    
    print("\nTesting gimbal angle computation...")
    for i, joint_angles in enumerate(joint_angles_array):
        
        try:
            # Call get_once_gimbal_angles
            gimbal_angles, obj_value = gimbal_yaw_planner.get_once_gimbal_angles(
                joint_angles, None
            )
            
            # Check if the result is valid (not None and not containing NaN)
            if gimbal_angles is not None and not np.any(np.isnan(gimbal_angles)):
                gimbal_angles_results.append(gimbal_angles.copy())
                obj_value_results.append(obj_value if obj_value is not None else np.nan)
                last_gimbal_angles = gimbal_angles  # Use previous result as initial guess for next iteration
            else:
                print(f"Warning: Invalid result at step {i} (joint angle: {angles_deg_array[i]:.1f}°)")
                gimbal_angles_results.append(np.full(4, np.nan))
                obj_value_results.append(np.nan)
                failed_steps.append(i)
            
        except Exception as e:
            print(f"Error at step {i} (joint angle: {angles_deg_array[i]:.1f}°): {e}")
            # Fill with NaN for failed computations
            gimbal_angles_results.append(np.full(4, np.nan))
            obj_value_results.append(np.nan)
            failed_steps.append(i)
    
    # Convert results to numpy array
    gimbal_angles_array = np.array(gimbal_angles_results)  # Shape: (num_points, 4)
    obj_value_array = np.array(obj_value_results)  # Shape: (num_points,)
    
    print(f"\nCompleted computation for {num_points} configurations")
    
    if failed_steps:
        print(f"Failed steps: {len(failed_steps)} out of {num_points}")
        print(f"Failed at joint angles (degrees): {[f'{angles_deg_array[i]:.1f}' for i in failed_steps[:10]]}" + 
              (f" ... and {len(failed_steps)-10} more" if len(failed_steps) > 10 else ""))
    
    return joint_angles_array, gimbal_angles_array, obj_value_array, angles_deg_array, failed_steps


def plot_results(joint_angles_array, gimbal_angles_array, obj_value_array, angles_deg_array, failed_steps):
    """
    Plot joint angles, gimbal angles, and objective function values
    
    Args:
        joint_angles_array: Joint angles array (num_points, 3)
        gimbal_angles_array: Gimbal angles array (num_points, 4) 
        obj_value_array: Objective function values array (num_points,)
        angles_deg_array: Joint angles in degrees for x-axis
        failed_steps: List of failed computation steps 
    """
    # Create subplots: 2 rows, 1 column
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 12))
    
    # Convert gimbal angles to degrees for plotting
    gimbal_angles_deg = np.degrees(gimbal_angles_array)
    joint_angles_deg = np.degrees(joint_angles_array[:, 0])  # All joints have same angle
    
    # Define colors
    joint_color = 'black'
    gimbal_colors = ['red', 'blue', 'green', 'orange']
    
    # ==================== First subplot: Joint and Gimbal Angles ====================
    # Plot joint angle (one curve since all joints have same value)
    ax1.plot(range(len(joint_angles_deg)), joint_angles_deg, 
             color=joint_color, linewidth=3, linestyle='--', 
             label='Joint Angles (all joints)', marker='s', markersize=2, alpha=0.8)
    
    # Plot gimbal angles (four curves), skip NaN values
    for i in range(4):
        # Create mask for non-NaN values
        valid_mask = ~np.isnan(gimbal_angles_deg[:, i])
        valid_indices = np.where(valid_mask)[0]
        valid_angles = gimbal_angles_deg[valid_mask, i]
        
        if len(valid_angles) > 0:
            ax1.plot(valid_indices, valid_angles, 
                     color=gimbal_colors[i], linewidth=2, 
                     label=f'Gimbal {i+1}', marker='o', markersize=1.5, alpha=0.9)
    
    # Highlight failed computation points on first subplot
    if failed_steps:
        failed_y_values = [joint_angles_deg[i] for i in failed_steps]
        ax1.scatter(failed_steps, failed_y_values, 
                   color='red', marker='x', s=50, alpha=0.8, 
                   label='Failed Computations', zorder=5)
    
    # Customize first subplot
    ax1.set_xlabel('Sample Index', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Angle (degrees)', fontsize=12, fontweight='bold')
    ax1.set_title('Joint Angles and Gimbal Angles vs Sample Index', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=10, loc='upper right')
    
    # Add secondary x-axis for first subplot
    ax1_top = ax1.twiny()
    step_size = len(angles_deg_array) // 10  # Show 10 ticks
    tick_indices = range(0, len(angles_deg_array), step_size)
    tick_labels = [f"{angles_deg_array[i]:.0f}°" for i in tick_indices]
    
    ax1_top.set_xlim(ax1.get_xlim())
    ax1_top.set_xticks(tick_indices)
    ax1_top.set_xticklabels(tick_labels)
    ax1_top.set_xlabel('Joint Angle Value', fontsize=12, fontweight='bold')
    
    # ==================== Second subplot: Objective Function Values ====================
    # Create mask for non-NaN objective values
    valid_obj_mask = ~np.isnan(obj_value_array)
    valid_obj_indices = np.where(valid_obj_mask)[0]
    valid_obj_values = obj_value_array[valid_obj_mask]
    
    if len(valid_obj_values) > 0:
        ax2.plot(valid_obj_indices, valid_obj_values, 
                 color='purple', linewidth=2, 
                 label='Objective Function Value', marker='o', markersize=1.5, alpha=0.9)
        
        # Add horizontal line for mean value
        mean_obj_value = np.mean(valid_obj_values)
        ax2.axhline(y=mean_obj_value, color='purple', linestyle=':', alpha=0.7, 
                   label=f'Mean = {mean_obj_value:.4f}')
    
    # Highlight failed computation points on second subplot
    if failed_steps:
        # For failed points, we don't have valid obj_values, so we'll just mark them at y=0
        ax2.scatter(failed_steps, [0] * len(failed_steps), 
                   color='red', marker='x', s=50, alpha=0.8, 
                   label='Failed Computations', zorder=5)
    
    # Customize second subplot
    ax2.set_xlabel('Sample Index', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Objective Function Value', fontsize=12, fontweight='bold')
    ax2.set_title('Objective Function Value vs Sample Index', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=10, loc='upper right')
    
    # Add secondary x-axis for second subplot
    ax2_top = ax2.twiny()
    ax2_top.set_xlim(ax2.get_xlim())
    ax2_top.set_xticks(tick_indices)
    ax2_top.set_xticklabels(tick_labels)
    ax2_top.set_xlabel('Joint Angle Value', fontsize=12, fontweight='bold')
    
    # Adjust layout and display
    plt.tight_layout()

    plt.show()

def print_summary_statistics(joint_angles_array, gimbal_angles_array, obj_value_array, angles_deg_array, failed_steps):
    """
    Print summary statistics of the test results
    """
    print("\n=== Summary Statistics ===")
    print(f"Total sample points: {len(joint_angles_array)}")
    print(f"Joint angle transition: {angles_deg_array[0]:.1f}° → {angles_deg_array[-1]:.1f}°")
    
    # Convert to degrees for readability
    gimbal_angles_deg = np.degrees(gimbal_angles_array)
    
    print("\nGimbal angle statistics (degrees):")
    for i in range(4):
        gimbal_data = gimbal_angles_deg[:, i]
        valid_data = gimbal_data[~np.isnan(gimbal_data)]
        
        if len(valid_data) > 0:
            print(f"  Gimbal {i+1}:")
            print(f"    Range: {np.min(valid_data):.2f}° to {np.max(valid_data):.2f}°")
            print(f"    Mean: {np.mean(valid_data):.2f}°")
            print(f"    Std Dev: {np.std(valid_data):.2f}°")
        else:
            print(f"  Gimbal {i+1}: No valid data")
    
    # Add objective function statistics
    valid_obj_values = obj_value_array[~np.isnan(obj_value_array)]
    print("\nObjective function statistics:")
    if len(valid_obj_values) > 0:
        print(f"  Range: {np.min(valid_obj_values):.6f} to {np.max(valid_obj_values):.6f}")
        print(f"  Mean: {np.mean(valid_obj_values):.6f}")
        print(f"  Std Dev: {np.std(valid_obj_values):.6f}")
        print(f"  Valid samples: {len(valid_obj_values)} out of {len(obj_value_array)}")
    else:
        print("  No valid objective function values")
    
    # Check for failed computations
    failed_count = len(failed_steps)
    success_rate = (len(joint_angles_array) - failed_count) / len(joint_angles_array) * 100
    
    if failed_count > 0:
        print(f"\nWarning: {failed_count} out of {len(joint_angles_array)} computations failed")
        print(f"Success rate: {success_rate:.1f}%")
        if failed_count <= 10:
            failed_angles = [f"{angles_deg_array[i]:.1f}°" for i in failed_steps]
            print(f"Failed at joint angles: {', '.join(failed_angles)}")
        else:
            print(f"Failed at joint angles ranging from {angles_deg_array[failed_steps[0]]:.1f}° to {angles_deg_array[failed_steps[-1]]:.1f}°")
    else:
        print(f"\nAll {len(joint_angles_array)} computations completed successfully! (100% success rate)")


if __name__ == "__main__":
    try:
        # Run the test
        joint_angles_array, gimbal_angles_array, obj_value_array, angles_deg_array, failed_steps = test_gimbal_planner_single_step()
        
        # Print summary statistics
        print_summary_statistics(joint_angles_array, gimbal_angles_array, obj_value_array, angles_deg_array, failed_steps)
        
        # Create plots
        plot_results(joint_angles_array, gimbal_angles_array, obj_value_array, angles_deg_array, failed_steps)
        
        print("\n=== Test Completed Successfully ===")
        
    except Exception as e:
        print(f"Test failed with error: {e}")
        import traceback
        traceback.print_exc()
