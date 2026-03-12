#!/usr/bin/env python3
"""
IEEE Journal Figure Configuration Module

This module provides standardized matplotlib configuration for IEEE journal publications.
It ensures consistent figure styling across all visualization scripts.

Usage:
    from ieee_figure_config import configure_ieee_style, IEEE_CONFIG
    
    # Apply configuration
    config = configure_ieee_style()
    
    # Use predefined constants
    fig_width = IEEE_CONFIG['fig_width']
    column_width = IEEE_CONFIG['column_width']
"""

import matplotlib.pyplot as plt


def configure_ieee_figure_style(target_column_width=3.5, target_font_size=9, 
                                matplotlib_width=7.0):
    """
    Configure matplotlib for IEEE journal figures.
    
    Args:
        target_column_width: Target width in inches for single column (default: 3.5")
        target_font_size: Desired font size in points when scaled (default: 9pt)
        matplotlib_width: Width in matplotlib figure (default: 7.0")
        
    Returns:
        dict: Configuration parameters including scale factor and font sizes
    """
    scale_factor = target_column_width / matplotlib_width
    
    config = {
        'column_width': target_column_width,
        'fig_width': matplotlib_width,
        'scale_factor': scale_factor,
        'base_font_size': target_font_size / scale_factor,
        'title_font_size': (target_font_size + 1) / scale_factor,
        'label_font_size': target_font_size / scale_factor,
        'tick_font_size': (target_font_size - 1) / scale_factor,
        'legend_font_size': (target_font_size - 1) / scale_factor,
    }
    
    # Apply configuration to matplotlib
    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['font.serif'] = ['Times New Roman']
    plt.rcParams['mathtext.fontset'] = 'stix'  # For math text to match Times New Roman
    plt.rcParams['font.size'] = config['base_font_size']
    plt.rcParams['axes.titlesize'] = config['title_font_size']
    plt.rcParams['axes.labelsize'] = config['label_font_size']
    plt.rcParams['xtick.labelsize'] = config['tick_font_size']
    plt.rcParams['ytick.labelsize'] = config['tick_font_size']
    plt.rcParams['legend.fontsize'] = config['legend_font_size']
    
    return config


def configure_ieee_style(target_font_size=8, verbose=True):
    """
    Convenience function to configure IEEE style with common settings.
    
    Args:
        target_font_size: Target font size in final figure (default: 8pt)
        verbose: Whether to print configuration information
        
    Returns:
        dict: Configuration parameters
    """
    config = configure_ieee_figure_style(
        target_column_width=3.5,    # IEEE single column width
        target_font_size=target_font_size,
        matplotlib_width=7.0        # 2x final size for better text scaling
    )
    
    if verbose:
        print(f"IEEE Figure Configuration:")
        print(f"  - Target column width: {config['column_width']}\"")
        print(f"  - Matplotlib figure width: {config['fig_width']}\"")
        print(f"  - Scale factor: {config['scale_factor']:.2f}")
        print(f"  - Font sizes (matplotlib -> final): {config['base_font_size']:.1f}pt -> {config['base_font_size']*config['scale_factor']:.1f}pt")
        print()
    
    return config


# IEEE Journal Figure Settings
# Column width: 3.5 inches (single column in IEEE double-column format)
# Target font sizes when figure is scaled to column width:
# - Main text in paper: 10pt
# - Figure text should be: 8-10pt for readability

# Default configuration - can be imported directly
IEEE_CONFIG = configure_ieee_style(target_font_size=8, verbose=False)

# Extract commonly used values as module-level constants
IEEE_COLUMN_WIDTH = IEEE_CONFIG['column_width']
FIG_WIDTH = IEEE_CONFIG['fig_width']
SCALE_FACTOR = IEEE_CONFIG['scale_factor']
BASE_FONT_SIZE = IEEE_CONFIG['base_font_size']

# Additional styling constants
LINE_WIDTH = 2.0  # Standard line width for plots
SEGMENT_COLORS = ['lightblue', 'lightgray']  # Colors for alternating segment backgrounds

# Color palette for high-contrast plots (IEEE-friendly)
IEEE_COLORS = {
    'navy_blue': '#000080',
    'orange_red': '#FF4500', 
    'dark_red': "#AB0808",
    'purple': '#800080',
    'brown': '#8B4513',
    'malachite': '#0BDA51',
    'teal_green': '#009E73',
    'orange_yellow': '#FFA500',
    'violet': '#9400D3'
}

# Line styles for distinguishing multiple data series
IEEE_LINE_STYLES = ['-', '--', ':', '-.', (0, (3, 1, 1, 1)), (0, (5, 1))]

# Standard plot layout margins for IEEE figures
IEEE_LAYOUT = {
    'left': 0.13,
    'right': 0.87, 
    'top': 0.95,
    'bottom': 0.15
}


def apply_ieee_layout(fig):
    """
    Apply standard IEEE layout margins to a figure.
    
    Args:
        fig: matplotlib figure object
    """
    fig.tight_layout()
    fig.subplots_adjust(**IEEE_LAYOUT)


def get_ieee_colors(n_colors=None):
    """
    Get a list of IEEE-friendly colors.
    
    Args:
        n_colors: Number of colors to return (if None, returns all)
        
    Returns:
        list: List of color hex codes
    """
    colors = list(IEEE_COLORS.values())
    if n_colors is not None:
        return colors[:n_colors]
    return colors


def get_ieee_line_styles(n_styles=None):
    """
    Get a list of IEEE-friendly line styles.
    
    Args:
        n_styles: Number of line styles to return (if None, returns all)
        
    Returns:
        list: List of matplotlib line style specifications
    """
    if n_styles is not None:
        return IEEE_LINE_STYLES[:n_styles]
    return IEEE_LINE_STYLES


if __name__ == "__main__":
    # Demo/test of the configuration
    print("IEEE Figure Configuration Module")
    print("=" * 40)
    
    # Show default configuration
    config = configure_ieee_style(verbose=True)
    
    print("Available colors:")
    for name, color in IEEE_COLORS.items():
        print(f"  - {name}: {color}")
    
    print(f"\nAvailable line styles: {len(IEEE_LINE_STYLES)} styles")
    print(f"Default line width: {LINE_WIDTH}")
    print(f"Segment background colors: {SEGMENT_COLORS}")
