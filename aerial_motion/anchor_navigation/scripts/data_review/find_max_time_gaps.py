#!/usr/bin/env python3
"""
Script to find files with the largest time intervals between consecutive sessions.
Analyzes planning session files and identifies gaps in the data collection.
"""

import os
from datetime import datetime
from pathlib import Path
from typing import List, Tuple


# Configuration
LOG_FOLDER_PATH = "../../logs/20251221_proposed_batch_200"  # Relative to this script's location
TOP_N_GAPS = 3  # Number of largest gaps to report


def extract_timestamp_from_filename(filename: str) -> datetime:
    """
    Extract timestamp from filename pattern: planning_session_YYYYMMDD_HHMMSS.json
    
    Args:
        filename: The filename to parse
        
    Returns:
        datetime object representing the timestamp
    """
    try:
        # Remove file extension
        name_without_ext = filename.replace('.json', '')
        
        # Extract timestamp part (last two parts after splitting by '_')
        parts = name_without_ext.split('_')
        date_str = parts[-2]  # YYYYMMDD
        time_str = parts[-1]  # HHMMSS
        
        # Parse timestamp
        timestamp_str = date_str + time_str
        timestamp = datetime.strptime(timestamp_str, '%Y%m%d%H%M%S')
        
        return timestamp
    except Exception as e:
        raise ValueError(f"Failed to parse timestamp from '{filename}': {e}")


def analyze_time_gaps(folder_path: str, top_n: int = 3) -> List[Tuple[str, float]]:
    """
    Analyze time gaps between consecutive files and find the largest ones.
    
    Args:
        folder_path: Path to the folder containing log files
        top_n: Number of largest gaps to return
        
    Returns:
        List of tuples (filename, gap_in_seconds) for the files after the largest gaps
    """
    # Get absolute path relative to this script's location
    if not os.path.isabs(folder_path):
        script_dir = Path(__file__).resolve().parent
        folder_path = script_dir / folder_path
    else:
        folder_path = Path(folder_path)
    
    if not folder_path.exists():
        raise FileNotFoundError(f"Folder not found: {folder_path}")
    
    # Read all files and extract timestamps
    files_with_timestamps = []
    for filename in os.listdir(folder_path):
        if filename.startswith('planning_session_') and filename.endswith('.json'):
            try:
                timestamp = extract_timestamp_from_filename(filename)
                files_with_timestamps.append((filename, timestamp))
            except ValueError as e:
                print(f"Warning: {e}")
                continue
    
    if len(files_with_timestamps) < 2:
        print("Not enough files to analyze time gaps")
        return []
    
    # Sort by timestamp
    files_with_timestamps.sort(key=lambda x: x[1])
    
    # Calculate time gaps between consecutive files
    gaps = []
    for i in range(1, len(files_with_timestamps)):
        prev_file, prev_time = files_with_timestamps[i-1]
        curr_file, curr_time = files_with_timestamps[i]
        
        time_diff = (curr_time - prev_time).total_seconds()
        gaps.append((curr_file, time_diff, prev_file, prev_time, curr_time))
    
    # Sort by gap size (descending) and get top N
    gaps.sort(key=lambda x: x[1], reverse=True)
    top_gaps = gaps[:top_n]
    
    return top_gaps


def main():
    """Main function to run the analysis."""
    print(f"Analyzing time gaps in: {LOG_FOLDER_PATH}")
    print(f"Looking for top {TOP_N_GAPS} largest time gaps\n")
    print("=" * 80)
    
    try:
        top_gaps = analyze_time_gaps(LOG_FOLDER_PATH, TOP_N_GAPS)
        
        if not top_gaps:
            print("No gaps found or insufficient data")
            return
        
        print(f"\nTop {len(top_gaps)} files with largest time gaps:\n")
        
        for i, (curr_file, gap_seconds, prev_file, prev_time, curr_time) in enumerate(top_gaps, 1):
            gap_minutes = gap_seconds / 60.0
            gap_hours = gap_minutes / 60.0
            
            print(f"{i}. File after gap: {curr_file}")
            print(f"   Previous file: {prev_file}")
            print(f"   Previous time: {prev_time.strftime('%Y-%m-%d %H:%M:%S')}")
            print(f"   Current time:  {curr_time.strftime('%Y-%m-%d %H:%M:%S')}")
            print(f"   Gap duration:  {gap_seconds:.1f} seconds ({gap_minutes:.2f} minutes / {gap_hours:.3f} hours)")
            print()
        
        print("=" * 80)
        print("\nSummary - Files after largest gaps:")
        for i, (curr_file, _, _, _, _) in enumerate(top_gaps, 1):
            print(f"  {i}. {curr_file}")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
