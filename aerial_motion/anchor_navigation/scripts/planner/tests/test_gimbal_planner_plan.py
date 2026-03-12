import os
import sys
current_path = os.path.abspath(os.path.dirname(__file__))
parent_path = os.path.dirname(current_path)
sys.path.insert(0, parent_path)
import matplotlib.pyplot as plt
import numpy as np
from gimbal_yaw_planner_bspline import GimbalYawPlanner
import time



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


def create_joint_angle_trajectory(num_points=100, trajectory_time=5.0):
    """
    Create a joint angle trajectory with smooth continuous transition from 90° to 0° to -90°

    Args:
        num_points: Number of trajectory points (default: 100)
        trajectory_time: Total trajectory time in seconds (default: 5.0)

    Returns:
        tuple: (joint_angles_array, time_array, config_descriptions)
            - joint_angles_array: np.array of shape (num_points, 3) with joint angles in radians
            - time_array: np.array of time stamps
            - config_descriptions: list of configuration descriptions for the trajectory
    """
    # Create time array
    time_array = np.linspace(0, trajectory_time, num_points)

    # Initialize joint angles array
    joint_angles = np.zeros((num_points, 3))

    # Create smooth transition from 90° to -90° (180° total range)
    # Using linear interpolation for uniform change
    angle_range = np.linspace(np.pi/2, -np.pi/2, num_points)  # 90° to -90°

    # Apply the same angle to all three joints for uniform motion
    for i in range(num_points):
        joint_angles[i] = [angle_range[i], angle_range[i], angle_range[i]]

    # Create description for the smooth trajectory
    config_descriptions = [
        f"Smooth continuous transition: 90° → 0° → -90° over {trajectory_time:.1f}s"
    ]

    return joint_angles, time_array, config_descriptions


def test_gimbal_planner_plan_function():
    """
    Test GimbalYawPlanner.plan method with a trajectory of joint angles
    """
    print("=== Gimbal Planner Plan Function Test ===")

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
        return None, None, None, None, None

    # Initialize gimbal planner
    gimbal_yaw_planner = GimbalYawPlanner(
        robot_urdf=urdf_path,
        planner_config=mock_planner_config,
    )

    # Create joint angle trajectory
    trajectory_time = 5.0
    input_freq = 60  # 60Hz frequency for input joint angles
    num_points = int(trajectory_time * input_freq)  # Calculate points for 60Hz
    joint_angles_array, time_array, config_descriptions = create_joint_angle_trajectory(
        num_points=num_points,
        trajectory_time=trajectory_time
    )

    print(f"Created trajectory with {num_points} points over {trajectory_time} seconds")
    print(f"Trajectory segments: {config_descriptions}")
    print(f"Joint angle range: {np.degrees(np.min(joint_angles_array)):.1f}° to {np.degrees(np.max(joint_angles_array)):.1f}°")

    # Test the plan function
    print("\nTesting gimbal planner plan function...")
    try:
        # Call the plan function with timing
        init_gimbal_angles = np.array([np.pi, 0, np.pi, 0])

        # Start timing
        start_time = time.time()
        plan_result = gimbal_yaw_planner.plan(
            des_joint_angles=joint_angles_array,
            time=trajectory_time,
            init_gimbal_angles=init_gimbal_angles
        )
        # End timing
        execution_time = time.time() - start_time

        # Extract results from the new return format (tuple)
        if plan_result is not None and isinstance(plan_result, tuple) and len(plan_result) == 2:
            des_gimbal_angles_array, objective_history = plan_result
        else:
            des_gimbal_angles_array = None
            objective_history = None

        # Check if the result is valid
        if des_gimbal_angles_array is not None and not np.any(np.isnan(des_gimbal_angles_array)):
            print(f"✓ Plan function succeeded!")
            print(f"Execution time: {execution_time:.4f} seconds")
            print(f"Optimization iterations: {len(objective_history) if objective_history is not None else 'N/A'}")
            success = True
            error_msg = None

        else:
            print("✗ Plan function returned invalid result (None or NaN)")
            des_gimbal_angles_array = None
            objective_history = None
            success = False
            error_msg = "Invalid result (None or NaN)"
            execution_time = 0.0

    except Exception as e:
        print(f"✗ Plan function failed with error: {e}")
        des_gimbal_angles_array = None
        objective_history = None
        success = False
        error_msg = str(e)
        execution_time = 0.0
        import traceback
        traceback.print_exc()

    return joint_angles_array, des_gimbal_angles_array, objective_history, time_array, config_descriptions, success, error_msg, gimbal_yaw_planner, execution_time


def plot_plan_results(joint_angles_array, des_gimbal_angles_array, objective_history, time_array, config_descriptions, success, gimbal_yaw_planner):
    """
    Plot the joint angle trajectory, resulting gimbal angle trajectory, fc_t_min values, and objective function convergence

    Args:
        joint_angles_array: Input joint angles array (num_points, 3)
        des_gimbal_angles_array: Output gimbal angles array (time_steps, 4) or None
        objective_history: Objective function values for each optimization iteration
        time_array: Time array for input trajectory
        config_descriptions: Description of trajectory segments
        success: Whether the plan function succeeded
        gimbal_yaw_planner: The planner instance to get fc_t_min data
    """
    if not success or des_gimbal_angles_array is None:
        print("Cannot plot results: Plan function failed or returned invalid data")
        return

    # Create output time array based on cmd_hz
    cmd_hz = 60  # From mock config
    output_time_array = np.arange(0, len(des_gimbal_angles_array) / cmd_hz, 1.0 / cmd_hz)

    # Get fc_t_min trajectory from the planner
    fc_t_min_trajectory = gimbal_yaw_planner.get_fc_t_min_trajectory()

    # Convert angles to degrees for plotting
    joint_angles_deg = np.degrees(joint_angles_array)
    gimbal_angles_deg = np.degrees(des_gimbal_angles_array)

    # Create subplots: 4 rows, 1 column
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(16, 20))

    # Define colors
    joint_colors = ['red', 'green', 'blue']
    gimbal_colors = ['crimson', 'darkorange', 'gold', 'purple']

    # ==================== First subplot: Input Joint Angles ====================
    for i in range(3):
        ax1.plot(time_array, joint_angles_deg[:, i],
                 color=joint_colors[i], linewidth=2.5,
                 label=f'Joint {i+1}', marker='o', markersize=2, alpha=0.9)

    # Add horizontal reference lines for key angles
    ax1.axhline(y=90, color='red', linestyle=':', alpha=0.6, linewidth=1.5, label='90° (Square)')
    ax1.axhline(y=0, color='green', linestyle=':', alpha=0.6, linewidth=1.5, label='0° (Straight)')
    ax1.axhline(y=-90, color='blue', linestyle=':', alpha=0.6, linewidth=1.5, label='-90° (Inverse)')

    # Customize first subplot
    ax1.set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Joint Angles (degrees)', fontsize=12, fontweight='bold')
    ax1.set_title('Input Joint Angle Trajectory (Smooth 90° → 0° → -90°)', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax1.legend(fontsize=10, loc='upper right', framealpha=0.9)
    ax1.set_ylim(-100, 100)

    # Add text annotation for trajectory description
    ax1.text(time_array[-1] * 0.02, 85, config_descriptions[0],
             fontsize=10, ha='left', va='top',
             bbox=dict(boxstyle='round,pad=0.4', facecolor='lightblue', alpha=0.8))

    # ==================== Second subplot: Output Gimbal Angles ====================
    for i in range(4):
        ax2.plot(output_time_array, gimbal_angles_deg[:, i],
                 color=gimbal_colors[i], linewidth=2.5,
                 label=f'Gimbal {i+1}', marker='o', markersize=1.5, alpha=0.9)

    # Customize second subplot
    ax2.set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Gimbal Angles (degrees)', fontsize=12, fontweight='bold')
    ax2.set_title('Output Gimbal Angle Trajectory (B-spline Optimized)', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax2.legend(fontsize=10, loc='upper right', framealpha=0.9)

    # Add statistics text
    gimbal_stats = []
    for i in range(4):
        gimbal_data = gimbal_angles_deg[:, i]
        range_val = np.max(gimbal_data) - np.min(gimbal_data)
        gimbal_stats.append(f'G{i+1}: {range_val:.1f}° range')

    stats_text = ', '.join(gimbal_stats)
    ax2.text(output_time_array[-1] * 0.02, ax2.get_ylim()[1] * 0.95, f'Angle ranges: {stats_text}',
             fontsize=9, ha='left', va='top',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='lightyellow', alpha=0.8))

    # ==================== Third subplot: FC Distance Minimum ====================
    if fc_t_min_trajectory is not None:
        ax3.plot(output_time_array, fc_t_min_trajectory,
                 color='darkred', linewidth=3, alpha=0.9,
                 label='FC Distance Minimum', marker='o', markersize=1.5)

        # Add threshold line if it exists
        if hasattr(gimbal_yaw_planner, 'fc_t_min_thres'):
            ax3.axhline(y=gimbal_yaw_planner.fc_t_min_thres,
                        color='red', linestyle='--', alpha=0.7, linewidth=2,
                        label=f'Threshold = {gimbal_yaw_planner.fc_t_min_thres:.2f}')

        # Color coding: green for safe, yellow for warning, red for critical
        safe_mask = fc_t_min_trajectory >= gimbal_yaw_planner.fc_t_min_thres
        warning_mask = (fc_t_min_trajectory < gimbal_yaw_planner.fc_t_min_thres) & (
            fc_t_min_trajectory >= gimbal_yaw_planner.fc_t_min_thres * 0.5)
        critical_mask = fc_t_min_trajectory < gimbal_yaw_planner.fc_t_min_thres * 0.5

        # Fill areas based on safety levels
        if np.any(safe_mask):
            ax3.fill_between(output_time_array, 0, fc_t_min_trajectory,
                             where=safe_mask, alpha=0.2, color='green', label='Safe zone')
        if np.any(warning_mask):
            ax3.fill_between(output_time_array, 0, fc_t_min_trajectory,
                             where=warning_mask, alpha=0.3, color='yellow', label='Warning zone')
        if np.any(critical_mask):
            ax3.fill_between(output_time_array, 0, fc_t_min_trajectory,
                             where=critical_mask, alpha=0.4, color='red', label='Critical zone')

        # Statistics
        min_fc_t = np.min(fc_t_min_trajectory)
        max_fc_t = np.max(fc_t_min_trajectory)
        mean_fc_t = np.mean(fc_t_min_trajectory)
        violation_count = np.sum(fc_t_min_trajectory < gimbal_yaw_planner.fc_t_min_thres)
        violation_percent = (violation_count / len(fc_t_min_trajectory)) * 100

        # Add statistics text
        stats_text = f'Min: {min_fc_t:.3f}, Max: {max_fc_t:.3f}, Mean: {mean_fc_t:.3f}\nViolations: {violation_count}/{len(fc_t_min_trajectory)} ({violation_percent:.1f}%)'
        ax3.text(output_time_array[-1] * 0.02, ax3.get_ylim()[1] * 0.95, stats_text,
                 fontsize=9, ha='left', va='top',
                 bbox=dict(boxstyle='round,pad=0.4', facecolor='lightcoral', alpha=0.8))

    else:
        ax3.text(0.5, 0.5, 'FC Distance Minimum data not available',
                 ha='center', va='center', transform=ax3.transAxes,
                 fontsize=12, style='italic')

    # Customize third subplot
    ax3.set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')
    ax3.set_ylabel('FC Distance Minimum', fontsize=12, fontweight='bold')
    ax3.set_title('Minimum Feasible Control Distance', fontsize=14, fontweight='bold')
    ax3.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax3.legend(fontsize=10, loc='upper right', framealpha=0.9)
    ax3.set_ylim(bottom=0)  # Start y-axis from 0

    # ==================== Fourth subplot: Objective Function Convergence ====================
    if objective_history is not None and len(objective_history) > 0:
        iteration_array = np.arange(1, len(objective_history) + 1)

        # Plot objective function values
        ax4.plot(iteration_array, objective_history,
                 color='darkblue', linewidth=3, marker='o', markersize=4,
                 alpha=0.9, label='Objective Function Value')

        # Add trend line if there are enough points
        if len(objective_history) > 5:
            z = np.polyfit(iteration_array, objective_history, 1)
            p = np.poly1d(z)
            ax4.plot(iteration_array, p(iteration_array),
                     color='red', linestyle='--', linewidth=2, alpha=0.7,
                     label=f'Trend (slope: {z[0]:.3f})')

        # Use linear scale for better readability of objective function values

    else:
        ax4.text(0.5, 0.5, 'Objective function history not available',
                 ha='center', va='center', transform=ax4.transAxes,
                 fontsize=12, style='italic')

    # Customize fourth subplot
    ax4.set_xlabel('Optimization Iteration', fontsize=12, fontweight='bold')
    ax4.set_ylabel('Objective Function Value', fontsize=12, fontweight='bold')
    ax4.set_title('Optimization Convergence History', fontsize=14, fontweight='bold')
    ax4.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax4.legend(fontsize=10, loc='upper right', framealpha=0.9)

    # ==================== Overall Figure Formatting ====================
    # Add overall title
    fig.suptitle('Gimbal Yaw Planner: Complete Trajectory Analysis\n(Joint Angles → Gimbal Angles → Safety Metrics → Optimization Convergence)',
                 fontsize=16, fontweight='bold', y=0.98)

    # Adjust layout to prevent overlap
    plt.tight_layout(rect=[0, 0, 1, 0.96])  # Leave space for suptitle

    plt.show()


def print_plan_summary_statistics(joint_angles_array, des_gimbal_angles_array, objective_history, time_array, success, error_msg, execution_time):
    """
    Print summary statistics of the plan function test results
    """
    print("\n=== Plan Function Test Summary ===")
    print(f"Input trajectory points: {len(joint_angles_array)}")
    print(f"Input trajectory time: {time_array[-1]:.2f} seconds")
    print(f"Joint angle range: {np.degrees(np.min(joint_angles_array)):.1f}° to {np.degrees(np.max(joint_angles_array)):.1f}°")
    print(f"Plan execution time: {execution_time:.4f} seconds")

    if success and des_gimbal_angles_array is not None:
        print(f"✓ Plan function execution: SUCCESS")
        print(f"Output trajectory points: {len(des_gimbal_angles_array)}")
        print(f"Output shape: {des_gimbal_angles_array.shape}")

    else:
        print(f"✗ Plan function execution: FAILED")
        if error_msg:
            print(f"Error: {error_msg}")


def run_multiple_test_scenarios():
    """
    Run multiple test scenarios with different parameters
    """
    print("\n=== Running Multiple Test Scenarios ===")

    scenarios = [
        {"num_points": 50, "trajectory_time": 3.0, "name": "Short trajectory"},
        {"num_points": 100, "trajectory_time": 5.0, "name": "Medium trajectory"},
        {"num_points": 200, "trajectory_time": 10.0, "name": "Long trajectory"},
    ]

    results = []

    for i, scenario in enumerate(scenarios):
        print(f"\n--- Scenario {i+1}: {scenario['name']} ---")
        print(f"Parameters: {scenario['num_points']} points, {scenario['trajectory_time']} seconds")

        try:
            # Update the global function to use scenario parameters
            global create_joint_angle_trajectory

            # Create mock planner config
            mock_planner_config = create_mock_planner_config()

            # Build URDF path
            tests_path = current_path
            planner_path = parent_path
            scripts_path = os.path.dirname(planner_path)
            motion_planner_path = os.path.dirname(scripts_path)
            urdf_path = os.path.join(motion_planner_path, 'urdf', 'hydrus_xi_20241227.urdf')

            if not os.path.exists(urdf_path):
                print(f"Error: URDF file not found at {urdf_path}")
                results.append({"scenario": scenario["name"], "success": False, "error": "URDF not found"})
                continue

            # Initialize planner
            gimbal_yaw_planner = GimbalYawPlanner(urdf_path, mock_planner_config)

            # Create trajectory
            joint_angles_array, time_array, config_descriptions = create_joint_angle_trajectory(
                num_points=scenario['num_points'],
                trajectory_time=scenario['trajectory_time']
            )

            # Test plan function
            init_gimbal_angles = np.array([np.pi, 0, np.pi, 0])
            plan_result = gimbal_yaw_planner.plan(
                des_joint_angles=joint_angles_array,
                time=scenario['trajectory_time'],
                init_gimbal_angles=init_gimbal_angles
            )

            # Extract results from the new return format (tuple)
            if plan_result is not None and isinstance(plan_result, tuple) and len(plan_result) == 2:
                des_gimbal_angles_array, objective_history = plan_result
            else:
                des_gimbal_angles_array = None
                objective_history = None

            if des_gimbal_angles_array is not None and not np.any(np.isnan(des_gimbal_angles_array)):
                print(f"✓ Scenario {i+1} SUCCESS")
                optimization_iterations = len(objective_history) if objective_history is not None else 0
                results.append({
                    "scenario": scenario["name"],
                    "success": True,
                    "output_shape": des_gimbal_angles_array.shape,
                    "input_points": scenario['num_points'],
                    "trajectory_time": scenario['trajectory_time'],
                    "optimization_iterations": optimization_iterations
                })
            else:
                print(f"✗ Scenario {i+1} FAILED - Invalid result")
                results.append({"scenario": scenario["name"], "success": False, "error": "Invalid result"})

        except Exception as e:
            print(f"✗ Scenario {i+1} FAILED - Exception: {e}")
            results.append({"scenario": scenario["name"], "success": False, "error": str(e)})

    # Print summary of all scenarios
    print(f"\n=== Multi-Scenario Test Summary ===")
    success_count = sum(1 for r in results if r["success"])
    total_count = len(results)
    print(f"Overall success rate: {success_count}/{total_count} ({success_count/total_count*100:.1f}%)")

    for result in results:
        status = "✓ PASS" if result["success"] else "✗ FAIL"
        print(f"{status} {result['scenario']}")
        if result["success"]:
            print(f"    Output shape: {result['output_shape']}")
            print(f"    Optimization iterations: {result['optimization_iterations']}")
        else:
            print(f"    Error: {result['error']}")


if __name__ == "__main__":
    try:
        # Run the main test
        joint_angles_array, des_gimbal_angles_array, objective_history, time_array, config_descriptions, success, error_msg, gimbal_yaw_planner, execution_time = test_gimbal_planner_plan_function()

        # Print summary statistics
        print_plan_summary_statistics(joint_angles_array, des_gimbal_angles_array, objective_history, time_array, success, error_msg, execution_time)

        # Create plots if successful
        if success:
            plot_plan_results(joint_angles_array, des_gimbal_angles_array, objective_history,
                              time_array, config_descriptions, success, gimbal_yaw_planner)
            print("\n=== Test and Plotting Completed ===")
        else:
            print("\n=== Test Failed - No Plotting ===")

    except Exception as e:
        print(f"Test failed with error: {e}")
        import traceback
        traceback.print_exc()
