#!/usr/bin/env python3
"""
Script to analyze CSV data from motion planning experiments.
Calculates:
- Maximum and minimum values of initial_x
- Average value of initial_y
- Average value of initial_yaw
"""

import pandas as pd
import numpy as np

def analyze_csv_data(csv_file_path):
    """
    Analyze CSV data and calculate required statistics.
    
    Args:
        csv_file_path (str): Path to the CSV file
    """
    # Read the CSV file
    try:
        df = pd.read_csv(csv_file_path)
    except FileNotFoundError:
        print(f"Error: File '{csv_file_path}' not found.")
        return
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return
    
    # Check if required columns exist
    required_columns = ['initial_x', 'initial_y', 'initial_yaw']
    missing_columns = [col for col in required_columns if col not in df.columns]
    
    if missing_columns:
        print(f"Error: Missing required columns: {missing_columns}")
        print(f"Available columns: {list(df.columns)}")
        return
    
    # Calculate statistics
    print("=== CSV Data Analysis ===")
    print(f"Data file: {csv_file_path}")
    print(f"Total number of rows: {len(df)}")
    print()
    
    # Initial X statistics
    initial_x_max = df['initial_x'].max()
    initial_x_min = df['initial_x'].min()
    
    print(f"Initial X Statistics:")
    print(f"  Maximum value: {initial_x_max:.6f}")
    print(f"  Minimum value: {initial_x_min:.6f}")
    print(f"  Range: {initial_x_max - initial_x_min:.6f}")
    print()
    
    # Initial Y statistics
    initial_y_avg = df['initial_y'].mean()
    initial_y_std = df['initial_y'].std()
    
    print(f"Initial Y Statistics:")
    print(f"  Average value: {initial_y_avg:.6f}")
    print(f"  Standard deviation: {initial_y_std:.6f}")
    print()
    
    # Initial YAW statistics
    initial_yaw_avg = df['initial_yaw'].mean()
    initial_yaw_std = df['initial_yaw'].std()
    
    print(f"Initial YAW Statistics:")
    print(f"  Average value: {initial_yaw_avg:.6f}")
    print(f"  Standard deviation: {initial_yaw_std:.6f}")
    print()
    
    # Additional summary
    print("=== Summary ===")
    print(f"Max initial_x: {initial_x_max:.6f}")
    print(f"Min initial_x: {initial_x_min:.6f}")
    print(f"Avg initial_y: {initial_y_avg:.6f}")
    print(f"Avg initial_yaw: {initial_yaw_avg:.6f}")


if __name__ == "__main__":
    # Path to the CSV file
    csv_file_path = "20250918_ablation_batch_200_analysis.csv"
    
    # Run analysis
    analyze_csv_data(csv_file_path)
