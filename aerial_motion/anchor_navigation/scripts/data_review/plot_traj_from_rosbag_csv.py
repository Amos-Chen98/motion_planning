#!/usr/bin/env python3
"""
CSV Data Visualization Script

This script processes experiment data from CSV files and creates IEEE-style
visualizations showing joint angles and position/orientation data over time.
"""

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

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
CSV_FILENAME = "turn.csv"  # CSV file to process
SAVE_PLOTS = False  # Set to True if you want to save plots to files
LEFT_MARGIN = 0.14  # Left margin for plots
RIGHT_MARGIN = 0.83  # Right margin for plots
TOP_MARGIN = 0.77  # Top margin for plots
BOTTOM_MARGIN = 0.15  # Bottom margin for plots

# Figure dimensions
FIGURE_HEIGHT_SCALE = 0.9  # Height scale factor (figure_height = FIG_WIDTH * FIGURE_HEIGHT_SCALE)
                          # Increase for taller figures (e.g., 0.5, 0.6), decrease for shorter (e.g., 0.3, 0.35)

# Angle processing
UNWRAP_YAW_ANGLES = True   # Unwrap yaw angles to remove 2π jumps
UNWRAP_JOINT_ANGLES = False  # Set to True if joint angles also have 2π jumps

# Line styling for lighter appearance
LIGHT_LINE_WIDTH = 2.0  # Lighter line width (reduced from default)
LINE_ALPHA = 1.0  # Transparency for lighter appearance

# legend styling
NCOL = 2  # Number of columns in legend
BBOX_TO_ANCHER = (0.5, 0.95)  # Legend position

# =============================================================================
# IEEE JOURNAL FIGURE CONFIGURATION
# =============================================================================
ieee_config = configure_ieee_figure_style(target_column_width=2.05, target_font_size=8,
                                          matplotlib_width=7.0)


def normalize_yaw_to_target(yaw_actual, yaw_desired):
    """
    Normalize actual yaw angles to be within π radians of the desired yaw.
    This handles the 2π periodicity issue where angles near ±π can appear
    very far apart when they're actually close.

    Args:
        yaw_actual: Array of actual yaw angles in radians
        yaw_desired: Array of desired yaw angles in radians (should be same length or scalar)

    Returns:
        numpy array: Normalized actual yaw angles
    """
    yaw_normalized = yaw_actual.copy()
    
    # Handle case where desired yaw is a scalar (constant target)
    if np.isscalar(yaw_desired):
        target_yaw = yaw_desired
    else:
        # Use the first desired yaw value as reference for initial normalization
        target_yaw = yaw_desired[0]
    
    # Normalize the initial actual yaw to be within π of the target
    initial_diff = yaw_actual[0] - target_yaw
    
    if initial_diff > np.pi:
        # Actual yaw is more than π above target, subtract 2π
        yaw_normalized = yaw_actual - 2*np.pi
        print(f"Normalized yaw: subtracted 2π from actual yaw (initial diff was {np.rad2deg(initial_diff):.1f}°)")
    elif initial_diff < -np.pi:
        # Actual yaw is more than π below target, add 2π
        yaw_normalized = yaw_actual + 2*np.pi
        print(f"Normalized yaw: added 2π to actual yaw (initial diff was {np.rad2deg(initial_diff):.1f}°)")
    else:
        print(f"No yaw normalization needed (initial diff was {np.rad2deg(initial_diff):.1f}°)")
    
    return yaw_normalized


def load_csv_data(filepath):
    """
    Load CSV data and extract required columns with aliases.
    Handle synchronized groups separately since different topics publish at different times.

    Args:
        filepath: Path to the CSV file

    Returns:
        dict: Dictionary containing synchronized data groups with their respective time arrays
    """
    # Read CSV file
    df = pd.read_csv(filepath)

    # Extract base time data
    time_data = df['__time'].values
    # Convert to relative time starting from 0
    time_base = time_data - time_data[0]

    # Group 1: Joint control (desired) data - synchronized together
    joints_ctrl_mask = df['/hydrus_xi/joints_ctrl/joint1/position'].notna()
    joints_ctrl_data = df[joints_ctrl_mask]
    joints_ctrl_time = time_base[joints_ctrl_mask]

    # Group 2: Joint states (actual) data - synchronized together
    joint_states_mask = df['/hydrus_xi/joint_states/joint1/position'].notna()
    joint_states_data = df[joint_states_mask]
    joint_states_time = time_base[joint_states_mask]

    # Group 3: Navigation target (desired) data - synchronized together
    nav_target_mask = df['/hydrus_xi/uav/nav/target_pos_x'].notna()
    nav_target_data = df[nav_target_mask]
    nav_target_time = time_base[nav_target_mask]

    # Group 4: Odometry (actual) data - synchronized together
    odom_mask = df['/hydrus_xi/uav/cog/odom/pose/pose/position/x'].notna()
    odom_data = df[odom_mask]
    odom_time = time_base[odom_mask]

    # Extract raw yaw data
    desired_yaw_raw = nav_target_data['/hydrus_xi/uav/nav/target_yaw'].values
    actual_yaw_raw = odom_data['/hydrus_xi/uav/cog/odom/pose/pose/orientation/yaw'].values

    # Normalize actual yaw to be within π of desired yaw (specifically for turn.csv)
    if CSV_FILENAME == "turn.csv":
        actual_yaw_normalized = normalize_yaw_to_target(actual_yaw_raw, desired_yaw_raw)
    else:
        actual_yaw_normalized = actual_yaw_raw

    # Extract synchronized data groups
    data = {
        # Joint control group (desired joint angles)
        'joints_ctrl_time': joints_ctrl_time,
        'desired_joint1_angle': joints_ctrl_data['/hydrus_xi/joints_ctrl/joint1/position'].values,
        'desired_joint2_angle': joints_ctrl_data['/hydrus_xi/joints_ctrl/joint2/position'].values,
        'desired_joint3_angle': joints_ctrl_data['/hydrus_xi/joints_ctrl/joint3/position'].values,
        
        # Joint states group (actual joint angles)
        'joint_states_time': joint_states_time,
        'joint1_angle': joint_states_data['/hydrus_xi/joint_states/joint1/position'].values,
        'joint2_angle': joint_states_data['/hydrus_xi/joint_states/joint2/position'].values,
        'joint3_angle': joint_states_data['/hydrus_xi/joint_states/joint3/position'].values,
        
        # Navigation target group (desired position/yaw)
        'nav_target_time': nav_target_time,
        'desired_x_position': nav_target_data['/hydrus_xi/uav/nav/target_pos_x'].values,
        'desired_y_position': nav_target_data['/hydrus_xi/uav/nav/target_pos_y'].values,
        'desired_yaw_angle': desired_yaw_raw,
        
        # Odometry group (actual position/yaw) - using normalized yaw
        'odom_time': odom_time,
        'x_position': odom_data['/hydrus_xi/uav/cog/odom/pose/pose/position/x'].values,
        'y_position': odom_data['/hydrus_xi/uav/cog/odom/pose/pose/position/y'].values,
        'yaw_angle': actual_yaw_normalized,
    }

    return data


def unwrap_angle(angles):
    """
    Unwrap angular data to remove 2π jumps due to periodicity.
    
    This function removes discontinuities in angular data by adding or subtracting
    2π whenever the difference between consecutive angles exceeds π radians.
    This is particularly important for yaw angles that may wrap around at ±π.
    
    Args:
        angles: Array of angles in radians
        
    Returns:
        numpy array: Unwrapped angles without 2π jumps
    """
    return np.unwrap(angles)


def create_joint_angles_plot(data):
    """
    Create Figure 1: Joint angles (desired vs actual) vs time.
    Uses separate time arrays for each synchronized group.

    Args:
        data: Dictionary containing synchronized data groups with their respective time arrays

    Returns:
        matplotlib figure object
    """
    fig_height = FIG_WIDTH * FIGURE_HEIGHT_SCALE
    fig, ax = plt.subplots(1, 1, figsize=(FIG_WIDTH, fig_height))

    # Extract time arrays for each synchronized group
    joints_ctrl_time = data['joints_ctrl_time']
    joint_states_time = data['joint_states_time']

    # Convert angles from radians to degrees for better readability
    # Optionally unwrap joint angles to remove 2π jumps
    if UNWRAP_JOINT_ANGLES:
        desired_joint1_deg = np.rad2deg(unwrap_angle(data['desired_joint1_angle']))
        desired_joint2_deg = np.rad2deg(unwrap_angle(data['desired_joint2_angle']))
        desired_joint3_deg = np.rad2deg(unwrap_angle(data['desired_joint3_angle']))
        joint1_deg = np.rad2deg(unwrap_angle(data['joint1_angle']))
        joint2_deg = np.rad2deg(unwrap_angle(data['joint2_angle']))
        joint3_deg = np.rad2deg(unwrap_angle(data['joint3_angle']))
    else:
        desired_joint1_deg = np.rad2deg(data['desired_joint1_angle'])
        desired_joint2_deg = np.rad2deg(data['desired_joint2_angle'])
        desired_joint3_deg = np.rad2deg(data['desired_joint3_angle'])
        joint1_deg = np.rad2deg(data['joint1_angle'])
        joint2_deg = np.rad2deg(data['joint2_angle'])
        joint3_deg = np.rad2deg(data['joint3_angle'])

    # Plot desired joint angles (dashed lines) - using joints_ctrl_time
    # Using professional, sophisticated colors for mature visualization
    line1 = ax.plot(joints_ctrl_time, desired_joint1_deg, color=IEEE_COLORS['orange_yellow'], linestyle='--',
            linewidth=LIGHT_LINE_WIDTH, alpha=LINE_ALPHA, label='Desired Joint 1 Angle', zorder=3)
    line4 = ax.plot(joint_states_time, joint1_deg, color=IEEE_COLORS['orange_yellow'], linestyle='-',
            linewidth=LIGHT_LINE_WIDTH, alpha=LINE_ALPHA, label='Joint 1 Angle', zorder=3)
    line2 = ax.plot(joints_ctrl_time, desired_joint2_deg, color=IEEE_COLORS['purple'], linestyle='--',
            linewidth=LIGHT_LINE_WIDTH, alpha=LINE_ALPHA, label='Desired Joint 2 Angle', zorder=3)
    line5 = ax.plot(joint_states_time, joint2_deg, color=IEEE_COLORS['purple'], linestyle='-',
            linewidth=LIGHT_LINE_WIDTH, alpha=LINE_ALPHA, label='Joint 2 Angle', zorder=3)
    line3 = ax.plot(joints_ctrl_time, desired_joint3_deg, color=IEEE_COLORS['teal_green'], linestyle='--',
            linewidth=LIGHT_LINE_WIDTH, alpha=LINE_ALPHA, label='Desired Joint 3 Angle', zorder=3)
    line6 = ax.plot(joint_states_time, joint3_deg, color=IEEE_COLORS['teal_green'], linestyle='-',
            linewidth=LIGHT_LINE_WIDTH, alpha=LINE_ALPHA, label='Joint 3 Angle', zorder=3)

    # Set labels and formatting
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Joint Angles (deg)')
    ax.grid(True, alpha=0.3)
    
    # Set x-axis limits based on the overall time range
    all_times = np.concatenate([joints_ctrl_time, joint_states_time])
    ax.set_xlim(np.min(all_times), np.max(all_times))

    # Combine legends from all plots
    lines = line1 + line2 + line3 + line4 + line5 + line6
    labels = [l.get_label() for l in lines]
    legend = ax.legend(lines, labels, bbox_to_anchor=BBOX_TO_ANCHER, loc='lower center',
                       ncol=NCOL, frameon=False, columnspacing=1.0)
    legend.set_zorder(2)

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


def create_position_plot(data):
    """
    Create Figure 2: Position and yaw (desired vs actual) vs time with dual y-axes.
    Uses separate time arrays for each synchronized group.

    Args:
        data: Dictionary containing synchronized data groups with their respective time arrays

    Returns:
        matplotlib figure object
    """
    fig_height = FIG_WIDTH * FIGURE_HEIGHT_SCALE
    fig, ax1 = plt.subplots(1, 1, figsize=(FIG_WIDTH, fig_height))

    # Extract time arrays for each synchronized group
    nav_target_time = data['nav_target_time']
    odom_time = data['odom_time']

    # Extract position data
    desired_x = data['desired_x_position']
    desired_y = data['desired_y_position']
    x_pos = data['x_position']
    y_pos = data['y_position']

    # Convert yaw from radians to degrees and optionally unwrap to remove 2π jumps
    if UNWRAP_YAW_ANGLES:
        desired_yaw_unwrapped = unwrap_angle(data['desired_yaw_angle'])
        yaw_unwrapped = unwrap_angle(data['yaw_angle'])
        desired_yaw_deg = np.rad2deg(desired_yaw_unwrapped)
        yaw_deg = np.rad2deg(yaw_unwrapped)
    else:
        desired_yaw_deg = np.rad2deg(data['desired_yaw_angle'])
        yaw_deg = np.rad2deg(data['yaw_angle'])

    # Plot position data on left y-axis (ax1)
    line1 = ax1.plot(nav_target_time, desired_x, color=IEEE_COLORS['navy_blue'], linestyle='--',
                     linewidth=LIGHT_LINE_WIDTH, alpha=LINE_ALPHA, label='Desired X Position', zorder=3)
    line2 = ax1.plot(nav_target_time, desired_y, color=IEEE_COLORS['orange_red'], linestyle='--',
                     linewidth=LIGHT_LINE_WIDTH, alpha=LINE_ALPHA, label='Desired Y Position', zorder=3)
    line4 = ax1.plot(odom_time, x_pos, color=IEEE_COLORS['navy_blue'], linestyle='-',
                     linewidth=LIGHT_LINE_WIDTH, alpha=LINE_ALPHA, label='X Position', zorder=3)
    line5 = ax1.plot(odom_time, y_pos, color=IEEE_COLORS['orange_red'], linestyle='-',
                     linewidth=LIGHT_LINE_WIDTH, alpha=LINE_ALPHA, label='Y Position', zorder=3)

    # Configure left y-axis (position)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (m)', color='black')
    ax1.tick_params(axis='y', labelcolor='black')
    ax1.grid(True, alpha=0.3)
    
    # Set x-axis limits based on the overall time range
    all_times = np.concatenate([nav_target_time, odom_time])
    ax1.set_xlim(np.min(all_times), np.max(all_times))

    # Create second y-axis for yaw angle
    ax2 = ax1.twinx()

    # Plot yaw data on right y-axis (ax2)
    line3 = ax2.plot(nav_target_time, desired_yaw_deg, color=IEEE_COLORS['dark_red'], linestyle='--',
                     linewidth=LIGHT_LINE_WIDTH, alpha=LINE_ALPHA, label='Desired Yaw Angle', zorder=3)
    line6 = ax2.plot(odom_time, yaw_deg, color=IEEE_COLORS['dark_red'], linestyle='-',
                     linewidth=LIGHT_LINE_WIDTH, alpha=LINE_ALPHA, label='Yaw Angle', zorder=3)

    # Configure right y-axis (angle)
    ax2.set_ylabel('Yaw Angle (deg)', color='black')
    ax2.tick_params(axis='y', labelcolor='black')

    # Combine legends from both axes
    lines = line1 + line2 + line3 + line4 + line5 + line6
    labels = [l.get_label() for l in lines]
    legend = ax1.legend(lines, labels, bbox_to_anchor=BBOX_TO_ANCHER, loc='lower center',
                        ncol=NCOL, frameon=False, columnspacing=1.0)
    legend.set_zorder(2)

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
    Main function to visualize CSV data.
    """
    print("CSV Data Visualization")
    print("=" * 50)
    print(f"Processing CSV file: {CSV_FILENAME}")
    print(f"Save plots: {'Enabled' if SAVE_PLOTS else 'Disabled'}")
    print()

    # Construct file path
    current_path = os.path.abspath(os.path.dirname(__file__))
    motion_planner_dir = os.path.dirname(os.path.dirname(current_path))
    data_dir = os.path.join(motion_planner_dir, 'real_exp_data')
    filepath = os.path.join(data_dir, CSV_FILENAME)

    # Load data
    print(f"Loading data from: {filepath}")
    data = load_csv_data(filepath)
    print(f"✓ Data loaded successfully")
    
    # Print information about each synchronized group
    print(f"  - Joint control data: {len(data['joints_ctrl_time'])} points over {data['joints_ctrl_time'][-1] - data['joints_ctrl_time'][0]:.2f}s")
    print(f"  - Joint states data: {len(data['joint_states_time'])} points over {data['joint_states_time'][-1] - data['joint_states_time'][0]:.2f}s")
    print(f"  - Navigation target data: {len(data['nav_target_time'])} points over {data['nav_target_time'][-1] - data['nav_target_time'][0]:.2f}s")
    print(f"  - Odometry data: {len(data['odom_time'])} points over {data['odom_time'][-1] - data['odom_time'][0]:.2f}s")
    print()

    # Create plots
    print("Creating visualizations...")
    fig1 = create_joint_angles_plot(data)
    fig2 = create_position_plot(data)

    # Optionally save plots
    if SAVE_PLOTS:
        output_dir = os.path.join(current_path, 'output_plots')
        os.makedirs(output_dir, exist_ok=True)

        base_name = os.path.splitext(CSV_FILENAME)[0]
        joint_filename = os.path.join(output_dir, f"{base_name}_joint_angles.pdf")
        position_filename = os.path.join(output_dir, f"{base_name}_position_yaw.pdf")

        # Save as high-quality PDF for IEEE publication
        fig1.savefig(joint_filename, dpi=300, bbox_inches='tight',
                     format='pdf', facecolor='white', edgecolor='none')
        fig2.savefig(position_filename, dpi=300, bbox_inches='tight',
                     format='pdf', facecolor='white', edgecolor='none')

        # Also save PNG versions for quick preview
        joint_png = os.path.join(output_dir, f"{base_name}_joint_angles.png")
        position_png = os.path.join(output_dir, f"{base_name}_position_yaw.png")
        fig1.savefig(joint_png, dpi=300, bbox_inches='tight')
        fig2.savefig(position_png, dpi=300, bbox_inches='tight')

        print(f"✓ Plots saved (IEEE format):")
        print(f"  - Joint angles: {joint_filename}")
        print(f"  - Position/Yaw: {position_filename}")
        print(f"  - PNG previews also saved")

    # Show plots
    plt.show()

    print("✓ Visualization completed successfully!")
    return 0


if __name__ == "__main__":
    exit(main())
