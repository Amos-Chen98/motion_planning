#!/usr/bin/env python3
"""
Trajectory Data Visualization Script

This script processes experiment data from saved trajectory JSON files and creates
visualizations showing the desired rootlink pose and joint angles over time.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

# Import IEEE figure configuration
from ieee_figure_config import (
    configure_ieee_style, 
    configure_ieee_figure_style,
    IEEE_CONFIG, 
    IEEE_COLUMN_WIDTH, 
    FIG_WIDTH, 
    SCALE_FACTOR, 
    BASE_FONT_SIZE,
    LINE_WIDTH,
    SEGMENT_COLORS,
    IEEE_COLORS,
    apply_ieee_layout
)

# =============================================================================
# IEEE JOURNAL FIGURE CONFIGURATION
# =============================================================================

# Configure IEEE style and display configuration info
# ieee_config = configure_ieee_style(target_font_size=8, verbose=True)
ieee_config = configure_ieee_figure_style(target_column_width=3.0, target_font_size=8, 
                                matplotlib_width=7.0)

# Add current path and planner path for imports
current_path = os.path.abspath(os.path.dirname(__file__))
planner_path = os.path.join(os.path.dirname(current_path), 'planner')
sys.path.insert(0, current_path)
sys.path.insert(0, planner_path)

from traj_coder import TrajCoder

# =============================================================================
# CONFIGURATION - Change these settings as needed
# =============================================================================
TRAJECTORY_FILENAME = "20250826_sim_example/traj_20250826_180110.json"  # Change this to your desired trajectory file
SAVE_PLOTS = False  # Set to True if you want to save plots to files
LEFT_MARGIN = 0.12  # Left margin for plots
RIGHT_MARGIN = 0.88  # Right margin for plots
TOP_MARGIN = 0.88  # Top margin for plots
BOTTOM_MARGIN = 0.20  # Bottom margin for plots

def load_and_process_trajectory(traj_coder, filepath, cmd_hz=50.0, speed_scale=1.0):
    """
    Load trajectory from file and process it for visualization.
    
    Args:
        traj_coder: TrajCoder instance
        filepath: Path to the trajectory JSON file
        cmd_hz: Command frequency in Hz (higher for smoother plots)
        speed_scale: Speed scaling factor
        
    Returns:
        tuple: (time_array, rootlink_data, joint_data, segment_times)
    """
    # Use TrajCoder to load and compute full desired commands
    print(f"Loading trajectory from: {filepath}")
    des_states = traj_coder.load_and_compute_commands(filepath, cmd_hz, speed_scale)
    if des_states is None:
        return None, None, None, None
    
    print(f"Generated {len(des_states)} desired state points")
    
    # Extract position and velocity data from desired states
    rootlink_data = []
    joint_data = []
    
    for state in des_states:
        # Extract root link pose (x, y, yaw)
        rootlink_state = state['rootlink']
        rootlink_pose = [rootlink_state.pos_x, rootlink_state.pos_y, rootlink_state.yaw]
        
        # Extract joint positions
        joint_positions = [joint.pos for joint in state['joints']]
        
        # Combine rootlink and joint data: [x, y, yaw, joint1, joint2, joint3]
        full_state = rootlink_pose + joint_positions
        rootlink_data.append(full_state)
        joint_data.append(full_state)
    
    # Convert to numpy arrays
    rootlink_data = np.array(rootlink_data)
    joint_data = np.array(joint_data)
    
    # Calculate segment timing information
    # Load trajectory segments separately to get segment boundaries
    traj_segments = traj_coder.load_trajectory_segments(filepath, speed_scale)
    if traj_segments is None:
        return None, None, None, None
    
    # Calculate segment time boundaries and generate time array
    segment_times = []
    current_time = 0.0
    
    for i, traj in enumerate(traj_segments):
        segment_start_time = current_time
        current_time += traj.time
        segment_times.append((segment_start_time, current_time, i))
    
    # Generate time array based on the number of points and total duration
    total_time = current_time
    time_array = np.linspace(0, total_time, len(des_states))
    
    print(f"Processed {len(traj_segments)} segments over {total_time:.2f} seconds")
    
    return time_array, rootlink_data, joint_data, segment_times


def create_rootlink_plot(time_array, rootlink_data, segment_times, title_suffix=""):
    """
    Create Figure 1: Rootlink pose (x, y, yaw) vs time with dual y-axes.
    Position (x, y) on left axis, yaw angle on right axis.
    Sized for IEEE single-column figure.
    
    Args:
        time_array: Array of time points
        rootlink_data: Array of rootlink states (x, y, yaw, joint1, joint2, joint3)
        segment_times: List of (start_time, end_time, segment_id) tuples
        title_suffix: Additional title information
    """
    # IEEE single-column figure dimensions (single plot instead of 3 subplots)
    fig_height = FIG_WIDTH * 0.4  # Adjust aspect ratio for single plot
    fig, ax1 = plt.subplots(1, 1, figsize=(FIG_WIDTH, fig_height))
    
    # Extract rootlink pose data
    x_pos = rootlink_data[:, 0]
    y_pos = rootlink_data[:, 1]
    yaw = rootlink_data[:, 2]
    
    # Convert yaw from radians to degrees for better readability
    yaw_deg = np.rad2deg(yaw)
    
    # Add segment background colors (applied to both axes)
    for start_time, end_time, seg_id in segment_times:
        color = SEGMENT_COLORS[seg_id % len(SEGMENT_COLORS)]
        ax1.axvspan(start_time, end_time, alpha=0.3, color=color, zorder=0)
    
    # Plot position data on left y-axis (ax1) with different line styles and high-contrast colors
    line1 = ax1.plot(time_array, x_pos, color=IEEE_COLORS['navy_blue'], linestyle='-', 
                     linewidth=LINE_WIDTH, label='X Position', zorder=3)
    line2 = ax1.plot(time_array, y_pos, color=IEEE_COLORS['orange_red'], linestyle='--', 
                     linewidth=LINE_WIDTH, label='Y Position', zorder=3)
    
    # Configure left y-axis (position)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (m)', color='black')
    ax1.tick_params(axis='y', labelcolor='black')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(time_array[0], time_array[-1])
    
    # Create second y-axis for yaw angle
    ax2 = ax1.twinx()
    
    # Plot yaw data on right y-axis (ax2) with distinct line style and high-contrast color
    line3 = ax2.plot(time_array, yaw_deg, color=IEEE_COLORS['dark_red'], linestyle='-.', 
                     linewidth=LINE_WIDTH, label='Yaw Angle', zorder=3)
    
    # Configure right y-axis (angle)
    ax2.set_ylabel('Yaw Angle (deg)', color='black')
    ax2.tick_params(axis='y', labelcolor='black')
    
    # Add segment boundaries as vertical lines
    for start_time, end_time, seg_id in segment_times[1:]:  # Skip first boundary
        ax1.axvline(x=start_time, color='red', linestyle='--', alpha=0.6, linewidth=1, zorder=2)
    
    # Combine legends from both axes
    lines = line1 + line2 + line3
    labels = [l.get_label() for l in lines]
    # Create legend above the plot area in a single row
    legend = ax1.legend(lines, labels, bbox_to_anchor=(0.5, 0.95), loc='lower center', 
                       ncol=3, frameon=False, columnspacing=1.0)
    # Set the legend's zorder to place it behind the curves
    legend.set_zorder(2)
    
    # Remove white borders on left and right sides
    apply_ieee_layout(fig)

    plt.tight_layout(pad=0.1)  # Minimal padding
    fig.subplots_adjust(
        left=LEFT_MARGIN,    # Minimal left margin
        right=RIGHT_MARGIN,   # Minimal right margin
        top=TOP_MARGIN,     # Minimal top margin (leave space for subplot titles)
        bottom=BOTTOM_MARGIN,  # Minimal bottom margin (leave space for x-axis labels)
    )
    return fig


def create_joint_plot(time_array, joint_data, segment_times, title_suffix=""):
    """
    Create Figure 2: Joint angles vs time with segment backgrounds.
    Sized for IEEE single-column figure.
    
    Args:
        time_array: Array of time points
        joint_data: Array of joint states (x, y, yaw, joint1, joint2, joint3)
        segment_times: List of (start_time, end_time, segment_id) tuples
        title_suffix: Additional title information
    """
    # IEEE single-column figure dimensions (will be scaled down to column width)
    fig_height = FIG_WIDTH * 0.4  # Adjust aspect ratio for single plot
    fig, ax = plt.subplots(1, 1, figsize=(FIG_WIDTH, fig_height))
    
    # Extract joint angle data (indices 3, 4, 5)
    joint1 = joint_data[:, 3]
    joint2 = joint_data[:, 4]
    joint3 = joint_data[:, 5]
    
    # Convert from radians to degrees for better readability
    joint1_deg = np.rad2deg(joint1)
    joint2_deg = np.rad2deg(joint2)
    joint3_deg = np.rad2deg(joint3)
    
    # Add segment background colors (without legend)
    for start_time, end_time, seg_id in segment_times:
        color = SEGMENT_COLORS[seg_id % len(SEGMENT_COLORS)]
        ax.axvspan(start_time, end_time, alpha=0.3, color=color, zorder=0)
    
    # Plot all joint angles on the same axes with high-contrast colors and styles
    joint_colors = [IEEE_COLORS['navy_blue'], IEEE_COLORS['orange_red'], IEEE_COLORS['dark_red']]
    joint_styles = ['-', '--', '-.']  # Solid, Dashed, Dotted (more distinct than dash-dot)
    
    ax.plot(time_array, joint1_deg, color=joint_colors[0], linestyle=joint_styles[0], 
           linewidth=LINE_WIDTH, label='Joint 1 Angle', zorder=3)
    ax.plot(time_array, joint2_deg, color=joint_colors[1], linestyle=joint_styles[1], 
           linewidth=LINE_WIDTH, label='Joint 2 Angle', zorder=3)
    ax.plot(time_array, joint3_deg, color=joint_colors[2], linestyle=joint_styles[2], 
           linewidth=LINE_WIDTH, label='Joint 3 Angle', zorder=3)
    
    # Add segment boundaries as vertical lines
    for start_time, end_time, seg_id in segment_times[1:]:  # Skip first boundary
        ax.axvline(x=start_time, color='red', linestyle=':', alpha=0.6, linewidth=1.5, zorder=2)
    
    # Set labels and formatting
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Joint Angles (deg)')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(time_array[0], time_array[-1])
    
    # Create legend above the plot area in a single row (no segment types)
    legend = ax.legend(bbox_to_anchor=(0.5, 0.95), loc='lower center', 
                      ncol=3, frameon=False, columnspacing=1.0)
    # Set the legend's zorder to place it behind the curves
    legend.set_zorder(2)
    
    # Remove white borders on left and right sides
    apply_ieee_layout(fig)
    plt.tight_layout(pad=0.1)  # Minimal padding
    fig.subplots_adjust(
        left=LEFT_MARGIN,    # Minimal left margin
        right=RIGHT_MARGIN,   # Minimal right margin
        top=TOP_MARGIN,     # Minimal top margin (leave space for subplot titles)
        bottom=BOTTOM_MARGIN,  # Minimal bottom margin (leave space for x-axis labels)
    )
    return fig


def main():
    """
    Main function to visualize trajectory data.
    
    This script is configured for IEEE journal publications:
    - Figures are sized for single-column width (3.5 inches)
    - Text will be 9pt when scaled to column width
    - Saves both PDF (for publication) and PNG (for preview)
    - Uses Times New Roman font (IEEE standard)
    
    To use in LaTeX:
    \\usepackage{graphicx}
    \\begin{figure}[htbp]
        \\centering
        \\includegraphics[width=\\columnwidth]{figure_name.pdf}
        \\caption{Your caption here}
        \\label{fig:your_label}
    \\end{figure}
    """
    print("Trajectory Data Visualization")
    print("=" * 50)
    print(f"Processing trajectory file: {TRAJECTORY_FILENAME}")
    print("(Change TRAJECTORY_FILENAME variable at the top of this script to use a different file)")
    print(f"Save plots: {'Enabled' if SAVE_PLOTS else 'Disabled'} (change SAVE_PLOTS variable to toggle)")
    print()
    
    # Initialize TrajCoder
    try:
        # Default URDF path
        motion_planner_dir = os.path.dirname(os.path.dirname(current_path))
        robot_urdf_path = os.path.join(motion_planner_dir, 'urdf', 'hydrus_xi_20241227.urdf')
        
        traj_coder = TrajCoder(robot_urdf_path)
        print(f"✓ TrajCoder initialized with URDF: {robot_urdf_path}")
    except Exception as e:
        print(f"✗ Failed to initialize TrajCoder: {e}")
        return 1
    
    # Construct file path
    filepath = os.path.join(traj_coder.save_dir, TRAJECTORY_FILENAME)
    if not os.path.exists(filepath):
        print(f"✗ Trajectory file not found: {filepath}")
        print("Available files in saved_traj directory:")
        files = traj_coder.list_saved_trajectories()
        if files:
            for i, filename in enumerate(files[-10:], 1):  # Show last 10 files
                print(f"  {i:2d}. {filename}")
        return 1
    
    # Get trajectory info
    traj_info = traj_coder.get_trajectory_info(filepath)
    if traj_info is None:
        print("✗ Failed to get trajectory info")
        return 1
    
    print(f"✓ Trajectory information:")
    print(f"  - Timestamp: {traj_info['timestamp']}")
    print(f"  - Segments: {traj_info['num_segments']}")
    print(f"  - Duration: {traj_info['total_time']:.2f}s")
    print(f"  - Config dimension: {traj_info['config_dim']}")
    print()
    
    # Load and process trajectory
    cmd_hz = 50.0  # Command frequency
    speed_scale = 1.0  # Speed scaling factor
    
    time_array, rootlink_data, joint_data, segment_times = load_and_process_trajectory(
        traj_coder, filepath, cmd_hz, speed_scale
    )
    
    if time_array is None:
        print("✗ Failed to load trajectory")
        return 1
    
    # Create title suffix with info
    title_suffix = f" ({traj_info['timestamp']}, {traj_info['num_segments']} segments)"
    
    # Create plots
    print("Creating visualizations...")
    fig1 = create_rootlink_plot(time_array, rootlink_data, segment_times, title_suffix)
    fig2 = create_joint_plot(time_array, joint_data, segment_times, title_suffix)
    
    # Optionally save plots
    if SAVE_PLOTS:
        output_dir = os.path.join(current_path, 'output_plots')
        os.makedirs(output_dir, exist_ok=True)
        
        base_name = os.path.splitext(TRAJECTORY_FILENAME)[0]
        rootlink_filename = os.path.join(output_dir, f"{base_name}_rootlink_pose.pdf")
        joint_filename = os.path.join(output_dir, f"{base_name}_joint_angles.pdf")
        
        # Save as high-quality PDF for IEEE publication
        # DPI 300 is standard for IEEE figures
        fig1.savefig(rootlink_filename, dpi=300, bbox_inches='tight', 
                    format='pdf', facecolor='white', edgecolor='none')
        fig2.savefig(joint_filename, dpi=300, bbox_inches='tight', 
                    format='pdf', facecolor='white', edgecolor='none')
        
        # Also save PNG versions for quick preview
        rootlink_png = os.path.join(output_dir, f"{base_name}_rootlink_pose.png")
        joint_png = os.path.join(output_dir, f"{base_name}_joint_angles.png")
        fig1.savefig(rootlink_png, dpi=300, bbox_inches='tight')
        fig2.savefig(joint_png, dpi=300, bbox_inches='tight')
        
        print(f"✓ Plots saved (IEEE format):")
        print(f"  - Rootlink pose: {rootlink_filename}")
        print(f"  - Joint angles: {joint_filename}")
        print(f"  - PNG previews also saved")
        print(f"  - Figure width: {FIG_WIDTH}\" -> scales to {IEEE_COLUMN_WIDTH}\" in paper")
        print(f"  - Text will be {BASE_FONT_SIZE*SCALE_FACTOR:.1f}pt when scaled to column width")
    
    # Show plots
    plt.show()
    
    print("✓ Visualization completed successfully!")
    return 0


if __name__ == "__main__":
    exit(main())
