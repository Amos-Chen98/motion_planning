#!/usr/bin/env python3
"""
FC_T_MIN Analysis Script

This script processes experiment data from CSV files and computes the feasible control
torque minimum (fc_t_min) throughout the experiment. It visualizes fc_t_min over time
and reports the minimum value encountered.
"""

import os
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Add the planner directory to path to import robot_model
current_path = os.path.abspath(os.path.dirname(__file__))
planner_path = os.path.join(os.path.dirname(current_path), 'planner')
sys.path.insert(0, planner_path)

from robot_model import Robot

# Import IEEE figure configuration
from ieee_figure_config import (
    configure_ieee_figure_style,
    FIG_WIDTH,
    LINE_WIDTH,
    IEEE_COLORS,
    apply_ieee_layout
)

# =============================================================================
# CONFIGURATION - Change these settings as needed
# =============================================================================
CSV_FILENAME = "gap.csv"  # CSV file to process
URDF_FILENAME = "hydrus_xi_20241227.urdf"  # Robot URDF file
SAVE_PLOTS = False  # Set to True if you want to save plots to files
LEFT_MARGIN = 0.14  # Left margin for plots
RIGHT_MARGIN = 0.96  # Right margin for plots
TOP_MARGIN = 0.96  # Top margin for plots
BOTTOM_MARGIN = 0.15  # Bottom margin for plots

# Figure dimensions
FIGURE_HEIGHT_SCALE = 0.5  # Height scale factor

# Line styling
LIGHT_LINE_WIDTH = 2.0
LINE_ALPHA = 1.0

# =============================================================================
# IEEE JOURNAL FIGURE CONFIGURATION
# =============================================================================
ieee_config = configure_ieee_figure_style(target_column_width=2.05, target_font_size=8,
                                          matplotlib_width=7.0)


def load_robot_model(urdf_path):
    """
    Load robot model from URDF file.

    Args:
        urdf_path: Path to the URDF file

    Returns:
        Robot: Initialized robot model
    """
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")
    
    robot = Robot(urdf_path)
    print(f"✓ Robot model loaded")
    print(f"  - Config dimension: {robot.config_dim}")
    print(f"  - Joint number: {robot.joint_num}")
    print(f"  - Rotor number: {robot.rotor_num}")
    
    return robot


def load_csv_data(filepath):
    """
    Load CSV data and extract configuration data over time.
    
    Args:
        filepath: Path to the CSV file

    Returns:
        dict: Dictionary containing time array and config arrays
    """
    # Read CSV file
    df = pd.read_csv(filepath)

    # Extract base time data
    time_data = df['__time'].values
    # Convert to relative time starting from 0
    time_base = time_data - time_data[0]

    # Extract joint states data
    joint_states_mask = df['/hydrus_xi/joint_states/joint1/position'].notna()
    joint_states_data = df[joint_states_mask]
    joint_states_time = time_base[joint_states_mask]
    
    # Extract joint angles from joint states
    joint1 = joint_states_data['/hydrus_xi/joint_states/joint1/position'].values
    joint2 = joint_states_data['/hydrus_xi/joint_states/joint2/position'].values
    joint3 = joint_states_data['/hydrus_xi/joint_states/joint3/position'].values
    
    # Construct config array: [x, y, yaw, joint1, joint2, joint3]
    # Note: fc_t_min only depends on joint configuration, not global position/orientation
    # Therefore, we set x, y, yaw to 0
    num_points = len(joint_states_time)
    config_array = np.zeros((num_points, 6))
    config_array[:, 0] = 0.0  # x position (fixed at 0)
    config_array[:, 1] = 0.0  # y position (fixed at 0)
    config_array[:, 2] = 0.0  # yaw angle (fixed at 0)
    config_array[:, 3] = joint1
    config_array[:, 4] = joint2
    config_array[:, 5] = joint3
    
    data = {
        'time': joint_states_time,
        'config_array': config_array
    }
    
    return data


def compute_fc_t_min_trajectory(robot, config_array):
    """
    Compute fc_t_min for each configuration in the trajectory.
    
    Args:
        robot: Robot model instance
        config_array: Array of configurations, shape (N, 6)
        
    Returns:
        numpy array: fc_t_min values for each configuration
    """
    num_points = config_array.shape[0]
    fc_t_min_array = np.zeros(num_points)
    
    print(f"Computing fc_t_min for {num_points} configurations...")
    
    # Compute fc_t_min for each configuration
    for i in range(num_points):
        config = config_array[i]
        fc_t_min_array[i] = robot.get_fc_t_min(config)
        
        # Progress indicator
        if (i + 1) % 100 == 0 or i == num_points - 1:
            progress = (i + 1) / num_points * 100
            print(f"  Progress: {progress:.1f}% ({i+1}/{num_points})", end='\r')
    
    print()  # New line after progress
    return fc_t_min_array


def create_fc_t_min_plot(time, fc_t_min_array):
    """
    Create figure showing fc_t_min vs time.

    Args:
        time: Time array
        fc_t_min_array: fc_t_min values

    Returns:
        matplotlib figure object
    """
    fig_height = FIG_WIDTH * FIGURE_HEIGHT_SCALE
    fig, ax = plt.subplots(1, 1, figsize=(FIG_WIDTH, fig_height))

    # Plot fc_t_min
    line = ax.plot(time, fc_t_min_array, color=IEEE_COLORS['navy_blue'], 
                   linestyle='-', linewidth=LIGHT_LINE_WIDTH, alpha=LINE_ALPHA, 
                   label='$f_{c,t,min}$', zorder=3)

    # Add horizontal line at y=0 for reference
    ax.axhline(y=0, color='black', linestyle='--', linewidth=1.0, alpha=0.5, zorder=2)

    # Set labels and formatting
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('$f_{c,t,min}$ (N·m)')
    ax.grid(True, alpha=0.3)
    
    # Set x-axis limits
    ax.set_xlim(np.min(time), np.max(time))

    # Add legend
    legend = ax.legend(loc='best', frameon=False)
    legend.set_zorder(4)

    # Apply IEEE layout
    apply_ieee_layout(fig)
    plt.tight_layout(pad=0.1)
    fig.subplots_adjust(
        left=LEFT_MARGIN,
        right=RIGHT_MARGIN,
        top=TOP_MARGIN,
        bottom=BOTTOM_MARGIN,
    )

    return fig


def main():
    """
    Main function to compute and visualize fc_t_min from CSV data.
    """
    print("FC_T_MIN Analysis from CSV Data")
    print("=" * 60)
    print(f"Processing CSV file: {CSV_FILENAME}")
    print(f"Using URDF file: {URDF_FILENAME}")
    print(f"Save plots: {'Enabled' if SAVE_PLOTS else 'Disabled'}")
    print()

    # Construct file paths
    motion_planner_dir = os.path.dirname(os.path.dirname(current_path))
    data_dir = os.path.join(motion_planner_dir, 'real_exp_data')
    csv_path = os.path.join(data_dir, CSV_FILENAME)
    urdf_dir = os.path.join(motion_planner_dir, 'urdf')
    urdf_path = os.path.join(urdf_dir, URDF_FILENAME)

    # Load robot model
    print(f"Loading robot model from: {urdf_path}")
    robot = load_robot_model(urdf_path)
    print()

    # Load CSV data
    print(f"Loading data from: {csv_path}")
    data = load_csv_data(csv_path)
    time = data['time']
    config_array = data['config_array']
    print(f"✓ Data loaded successfully")
    print(f"  - Number of data points: {len(time)}")
    print(f"  - Time range: {time[0]:.2f}s to {time[-1]:.2f}s (duration: {time[-1] - time[0]:.2f}s)")
    print()

    # Compute fc_t_min trajectory
    fc_t_min_array = compute_fc_t_min_trajectory(robot, config_array)
    print(f"✓ fc_t_min computation completed")
    print()

    # Analyze results
    min_fc_t_min = np.min(fc_t_min_array)
    min_fc_t_min_idx = np.argmin(fc_t_min_array)
    min_fc_t_min_time = time[min_fc_t_min_idx]
    mean_fc_t_min = np.mean(fc_t_min_array)
    max_fc_t_min = np.max(fc_t_min_array)

    print("Analysis Results:")
    print("-" * 60)
    print(f"Minimum fc_t_min: {min_fc_t_min:.3g} N·m (at t = {min_fc_t_min_time:.2f}s)")
    print(f"Mean fc_t_min:    {mean_fc_t_min:.3g} N·m")
    print(f"Maximum fc_t_min: {max_fc_t_min:.3g} N·m")
    print("-" * 60)
    print()

    # Create plot
    print("Creating visualization...")
    fig = create_fc_t_min_plot(time, fc_t_min_array)

    # Optionally save plot
    if SAVE_PLOTS:
        output_dir = os.path.join(current_path, 'output_plots')
        os.makedirs(output_dir, exist_ok=True)

        base_name = os.path.splitext(CSV_FILENAME)[0]
        pdf_filename = os.path.join(output_dir, f"{base_name}_fc_t_min.pdf")
        png_filename = os.path.join(output_dir, f"{base_name}_fc_t_min.png")

        # Save as high-quality PDF for IEEE publication
        fig.savefig(pdf_filename, dpi=300, bbox_inches='tight',
                    format='pdf', facecolor='white', edgecolor='none')
        
        # Also save PNG version for quick preview
        fig.savefig(png_filename, dpi=300, bbox_inches='tight')

        print(f"✓ Plots saved (IEEE format):")
        print(f"  - PDF: {pdf_filename}")
        print(f"  - PNG: {png_filename}")
        print()

    # Show plot
    plt.show()

    print("✓ Analysis completed successfully!")
    return 0


if __name__ == "__main__":
    exit(main())
