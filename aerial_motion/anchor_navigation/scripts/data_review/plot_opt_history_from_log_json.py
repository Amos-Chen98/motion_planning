#!/usr/bin/env python3
"""
Optimization Data Visualization Script

This script processes optimization data from saved planning session JSON files and creates
visualizations showing the objective function convergence over iterations for each segment.
"""

import os
import sys
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.ticker import LogLocator
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
    IEEE_COLORS,
    apply_ieee_layout
)

# =============================================================================
# IEEE JOURNAL FIGURE CONFIGURATION
# =============================================================================

# Configure IEEE style and display configuration info for page width
ieee_config = configure_ieee_figure_style(
    target_column_width=4.0,  # IEEE page width (double column)
    target_font_size=8,
    matplotlib_width=14.0     # 2x final size for better text scaling
)

# Extract figure width from custom config
PAGE_WIDTH = ieee_config['fig_width']
PAGE_FONT_SIZE = ieee_config['base_font_size']

print(f"IEEE Page Width Configuration:")
print(f"  - Target page width: {ieee_config['column_width']}\"")
print(f"  - Matplotlib figure width: {PAGE_WIDTH}\"")
print(f"  - Scale factor: {ieee_config['scale_factor']:.2f}")
print(f"  - Font sizes (matplotlib -> final): {PAGE_FONT_SIZE:.1f}pt -> {PAGE_FONT_SIZE*ieee_config['scale_factor']:.1f}pt")
print()

# =============================================================================
# CONFIGURATION - Change these settings as needed
# =============================================================================
PLANNING_SESSION_FILENAME = "20250826_sim_example/planning_session_20250826_180100.json"  # Change this to your desired planning session file
SAVE_PLOTS = False  # Set to True if you want to save plots to files


def load_planning_session(filepath):
    """
    Load planning session data from JSON file.
    
    Args:
        filepath: Path to the planning session JSON file
        
    Returns:
        dict: Planning session data, or None if failed to load
    """
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
        return data
    except Exception as e:
        print(f"✗ Failed to load planning session: {e}")
        return None


def extract_optimization_data(session_data):
    """
    Extract optimization data from planning session.
    
    Args:
        session_data: Loaded planning session data
        
    Returns:
        tuple: (optimization_logs, session_info) or (None, None) if failed
    """
    try:
        optimization_logs = session_data.get('optimization_logs', [])
        session_info = session_data.get('session_info', {})
        
        if not optimization_logs:
            print("✗ No optimization logs found in session data")
            return None, None
            
        print(f"✓ Found optimization data for {len(optimization_logs)} segments")
        return optimization_logs, session_info
        
    except Exception as e:
        print(f"✗ Failed to extract optimization data: {e}")
        return None, None


def create_all_segments_plot(optimization_logs, session_info):
    """
    Create Figure: Objective function convergence for all segments in separate subplots.
    All segments arranged horizontally in one row for IEEE page width.
    Sized for IEEE page width figure.
    
    Args:
        optimization_logs: List of optimization data for all segments
        session_info: Session information
        
    Returns:
        matplotlib.figure.Figure: The created figure
    """
    num_segments = len(optimization_logs)
    if num_segments == 0:
        return None
    
    # Arrange all segments horizontally in one row
    rows, cols = 1, num_segments
    
    # IEEE page width figure dimensions - fixed height for horizontal layout
    fig_height = PAGE_WIDTH * 0.15  # Aspect ratio optimized for horizontal layout
    fig, axes = plt.subplots(rows, cols, figsize=(PAGE_WIDTH, fig_height))
    
    # Ensure axes is always an array for consistent indexing
    if num_segments == 1:
        axes = [axes]
    
    # Use same color for all curves since each subplot has only one curve
    curve_color = IEEE_COLORS['navy_blue']
    min_marker_color = 'red'
    
    # Plot each segment in its own subplot
    for i, segment_data in enumerate(optimization_logs):
        ax = axes[i]
        
        # Number segments from 1 instead of 0
        segment_id = segment_data.get('segment_id', i) + 1
        objective_history = segment_data.get('objective_history', [])
        iterations = segment_data.get('iterations', len(objective_history))
        
        if not objective_history:
            ax.text(0.5, 0.5, f'Segment {segment_id}\nNo data', 
                   ha='center', va='center', transform=ax.transAxes)
            ax.set_xticks([])
            ax.set_yticks([])
            continue
        
        # Create iteration array (1-indexed)
        iter_array = np.arange(1, len(objective_history) + 1)
        obj_values = np.array(objective_history)
        
        # Plot convergence curve with same color for all segments
        ax.plot(iter_array, obj_values, color=curve_color, linestyle='-', 
               linewidth=LINE_WIDTH, marker='o', markersize=1.5, 
               markerfacecolor=curve_color, markeredgecolor='white', 
               markeredgewidth=0.2, alpha=0.8, zorder=3)
        
        # Add red dot at minimum cost point - made more obvious
        min_idx = np.argmin(obj_values)
        min_iter = iter_array[min_idx]
        min_obj = obj_values[min_idx]
        # Use a larger, more prominent marker with better contrast
        ax.plot(min_iter, min_obj, marker='o', markersize=8, 
               color=min_marker_color, markerfacecolor=min_marker_color, 
               markeredgecolor='white', markeredgewidth=1.5, zorder=5)
        
        # Formatting for each subplot
        ax.set_title(f'Segment {segment_id}', fontsize=PAGE_FONT_SIZE*0.9)
        ax.grid(True, alpha=0.3)
        ax.set_xlim(1, len(objective_history))
        
        # Set y-axis to log scale
        ax.set_yscale('log')
        
        # Set y-axis range with padding (for log scale, use multiplicative padding)
        y_min, y_max = np.min(obj_values), np.max(obj_values)
        if y_min > 0 and y_max > 0:  # Ensure positive values for log scale
            # Increase padding to compress the curve appearance (make it less sharp)
            log_padding_lower = 0.5  # 50% padding below (more compression)
            log_padding_upper = 0.3  # 30% padding above
            ax.set_ylim(y_min * (1 - log_padding_lower), y_max * (1 + log_padding_upper))
        else:
            # Fallback for non-positive values - filter out non-positive values
            positive_values = obj_values[obj_values > 0]
            if len(positive_values) > 0:
                y_min_pos = np.min(positive_values)
                y_max_pos = np.max(positive_values)
                # Apply more aggressive padding for compression
                ax.set_ylim(y_min_pos * 0.5, y_max_pos * 1.3)
        
        # Set axis labels - only leftmost subplot gets y-label, all get x-labels
        ax.set_xlabel('Iteration', fontsize=PAGE_FONT_SIZE*0.9)
        if i == 0:
            ax.set_ylabel('Objective Value', fontsize=PAGE_FONT_SIZE*0.9)
        else:
            # Remove y-axis labels for non-leftmost subplots to prevent overlap
            ax.set_ylabel('')
        
        # Reduce tick label font size for better fit and prevent overlap
        ax.tick_params(axis='both', which='major', labelsize=PAGE_FONT_SIZE*0.8)
        
        # For non-leftmost subplots, reduce y-tick label visibility to prevent overlap
        if i > 0:
            # Keep y-ticks but make labels smaller and fewer
            ax.tick_params(axis='y', which='major', labelsize=PAGE_FONT_SIZE*0.7)
            # For log scale, use LogLocator with appropriate settings
            ax.yaxis.set_major_locator(LogLocator(base=10, numticks=4))
    
    # Remove all white margins and adjust layout to prevent overlap
    plt.tight_layout(pad=0.1)  # Minimal padding
    fig.subplots_adjust(
        left=0.10,    # Minimal left margin
        right=0.99,   # Minimal right margin  
        top=0.80,     # Minimal top margin (leave space for subplot titles)
        bottom=0.30,  # Minimal bottom margin (leave space for x-axis labels)
        wspace=0.35   # Horizontal spacing between subplots to prevent y-axis overlap
    )
    
    return fig


def main():
    """
    Main function to visualize optimization data.
    
    This script is configured for IEEE journal publications:
    - Figures are sized for single-column width (3.5 inches)
    - Text will be 8pt when scaled to column width
    - Saves both PDF (for publication) and PNG (for preview)
    - Uses Times New Roman font (IEEE standard)
    """
    print("Optimization Data Visualization")
    print("=" * 50)
    print(f"Processing planning session file: {PLANNING_SESSION_FILENAME}")
    print("(Change PLANNING_SESSION_FILENAME variable at the top of this script to use a different file)")
    print(f"Save plots: {'Enabled' if SAVE_PLOTS else 'Disabled'} (change SAVE_PLOTS variable to toggle)")
    print()
    
    # Get current script directory and construct file path
    current_path = os.path.abspath(os.path.dirname(__file__))
    logs_dir = os.path.join(os.path.dirname(os.path.dirname(current_path)), 'logs')
    filepath = os.path.join(logs_dir, PLANNING_SESSION_FILENAME)
    
    if not os.path.exists(filepath):
        print(f"✗ Planning session file not found: {filepath}")
        print("Available files in logs directory:")
        try:
            files = [f for f in os.listdir(logs_dir) if f.endswith('.json')]
            for i, filename in enumerate(files, 1):
                print(f"  {i:2d}. {filename}")
        except:
            print("  Could not list log files")
        return 1
    
    # Load planning session data
    session_data = load_planning_session(filepath)
    if session_data is None:
        return 1
    
    # Extract optimization data
    optimization_logs, session_info = extract_optimization_data(session_data)
    if optimization_logs is None:
        return 1
    
    # Display session information
    print(f"✓ Planning session information:")
    print(f"  - Timestamp: {session_info.get('timestamp', 'Unknown')}")
    print(f"  - World: {session_info.get('world_name', 'Unknown')}")
    print(f"  - Computation time: {session_info.get('computation_time_sec', 0):.2f}s")
    print(f"  - Plan success: {session_info.get('plan_success', 'Unknown')}")
    print(f"  - Total segments: {len(optimization_logs)}")
    
    # Display optimization summary
    total_iterations = sum(seg.get('iterations', len(seg.get('objective_history', []))) 
                          for seg in optimization_logs)
    print(f"  - Total iterations: {total_iterations}")
    print()
    
    # Create the visualization
    print("Creating optimization convergence visualization...")
    
    # Only create the all segments subplot figure
    fig = create_all_segments_plot(optimization_logs, session_info)
    if fig is None:
        print("✗ Failed to create visualization")
        return 1
    
    # Optionally save plot
    if SAVE_PLOTS:
        output_dir = os.path.join(current_path, 'output_plots')
        os.makedirs(output_dir, exist_ok=True)
        
        base_name = os.path.splitext(PLANNING_SESSION_FILENAME)[0]
        
        # Save as high-quality PDF for IEEE publication
        fig_filename = os.path.join(output_dir, f"{base_name}_all_segments_convergence.pdf")
        
        fig.savefig(fig_filename, dpi=300, bbox_inches='tight', 
                    format='pdf', facecolor='white', edgecolor='none')
        
        # Also save PNG version for quick preview
        fig_png = os.path.join(output_dir, f"{base_name}_all_segments_convergence.png")
        fig.savefig(fig_png, dpi=300, bbox_inches='tight')
        
        print(f"✓ Plot saved (IEEE format):")
        print(f"  - All segments convergence: {fig_filename}")
        print(f"  - PNG preview also saved")
        print(f"  - Figure width: {PAGE_WIDTH}\" -> scales to 7.0\" in paper")
        print(f"  - Text will be {PAGE_FONT_SIZE*ieee_config['scale_factor']:.1f}pt when scaled to page width")
    
    # Show plot
    plt.show()
    
    print("✓ Optimization visualization completed successfully!")
    return 0


if __name__ == "__main__":
    exit(main())
