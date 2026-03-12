#!/usr/bin/env python3
"""
Data analyzer script for hydrus planning batch results.
Reads all JSON files and generates a CSV summary.
"""

import json
import csv
import os
import glob
import math
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
from typing import Dict, List, Any, Optional
from ieee_figure_config import configure_ieee_style, IEEE_COLORS


class BatchDataAnalyzer:
    """Analyzer for hydrus planning batch data."""
    
    def __init__(self, batch_folder: str = "20250901_batch"):
        """
        Initialize the analyzer.
        
        Args:
            batch_folder: Name of the batch folder to analyze
        """
        self.batch_folder = batch_folder
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.logs_dir = os.path.join(self.script_dir, "..", "..", "logs", batch_folder)
        self.output_csv = os.path.join(self.script_dir, f"{batch_folder}_analysis.csv")
    
    def normalize_angle(self, angle: float) -> float:
        """
        Normalize angle to (-pi, pi] range.
        
        Args:
            angle: Input angle in radians
            
        Returns:
            Normalized angle in (-pi, pi] range
        """
        if not isinstance(angle, (int, float)):
            return angle  # Return as-is if not a number
        
        # Normalize to (-pi, pi]
        normalized = angle
        while normalized > math.pi:
            normalized -= 2 * math.pi
        while normalized <= -math.pi:
            normalized += 2 * math.pi
        
        return normalized
        
    def load_json_file(self, file_path: str) -> Optional[Dict[str, Any]]:
        """
        Load and parse a JSON file.
        
        Args:
            file_path: Path to the JSON file
            
        Returns:
            Parsed JSON data or None if error
        """
        try:
            with open(file_path, 'r') as f:
                return json.load(f)
        except (json.JSONDecodeError, FileNotFoundError) as e:
            print(f"Error loading {file_path}: {e}")
            return None
    
    def determine_algorithm(self, data: Dict[str, Any]) -> str:
        """
        Determine the algorithm type based on planner_config.
        
        Args:
            data: JSON data from the planning session
            
        Returns:
            Algorithm description string
        """
        # Check if planner_config exists
        if 'planner_config' not in data:
            return "DK"
        
        config = data['planner_config']
        
        # Extract configuration flags
        find_anchor = config.get('find_anchor', False)
        is_parallel = config.get('is_parallel', False)
        run_traj_opt = config.get('run_traj_opt', False)
        
        # Determine algorithm based on flags
        if not find_anchor:
            return "w/o AS"
        elif find_anchor and is_parallel and not run_traj_opt:
            return "w/o LP"
        elif find_anchor and not is_parallel and run_traj_opt:
            return "w/o PC"
        elif find_anchor and is_parallel and run_traj_opt:
            return "Ours"
        else:
            # Fallback case - shouldn't normally happen
            return f"Unknown ({find_anchor}, {is_parallel}, {run_traj_opt})"
    
    def extract_data_from_json(self, data: Dict[str, Any], filename: str) -> Dict[str, Any]:
        """
        Extract relevant data from a JSON file.
        
        Args:
            data: JSON data from the planning session
            filename: Name of the JSON file
            
        Returns:
            Dictionary with extracted data
        """
        # Extract initial pose
        initial_state = data.get('initial_state', {})
        initial_pos = initial_state.get('rootlink_position', {})
        initial_x = initial_pos.get('x', '')
        initial_y = initial_pos.get('y', '')
        initial_yaw = initial_state.get('rootlink_yaw', '')
        
        # Normalize initial yaw angle
        if initial_yaw != '':
            initial_yaw = self.normalize_angle(initial_yaw)
        
        # Extract target pose (may not exist for DK-based or failed cases)
        target_state = data.get('target_state', {})
        if target_state:
            target_pos = target_state.get('rootlink_position', {})
            target_x = target_pos.get('x', '')
            target_y = target_pos.get('y', '')
            target_yaw = target_state.get('rootlink_yaw', '')
            
            # Normalize target yaw angle
            if target_yaw != '':
                target_yaw = self.normalize_angle(target_yaw)
        else:
            target_x = ''
            target_y = ''
            target_yaw = ''
        
        # Extract session info
        session_info = data.get('session_info', {})
        mission_success = session_info.get('mission_success', False)
        # Handle null values for mission_success
        if mission_success is None:
            mission_success = False
        computation_time = session_info.get('computation_time_sec', '')
        
        # If mission failed, set computation time to '-'
        if not mission_success:
            computation_time = '-'
        
        # Determine algorithm
        algorithm = self.determine_algorithm(data)
        
        # Apply 180-degree rotation transformation for DK algorithm
        if algorithm == "DK":
            # Rotate initial pose by 180 degrees around origin
            if initial_x != '':
                initial_x = -initial_x
            if initial_y != '':
                initial_y = -initial_y
            if initial_yaw != '':
                initial_yaw = self.normalize_angle(initial_yaw + math.pi)
        
        return {
            'initial_x': initial_x,
            'initial_y': initial_y,
            'initial_yaw': initial_yaw,
            'target_x': target_x,
            'target_y': target_y,
            'target_yaw': target_yaw,
            'algorithm': algorithm,
            'mission_success': mission_success,
            'computation_time_sec': computation_time
        }
    
    def analyze_batch(self) -> List[Dict[str, Any]]:
        """
        Analyze all JSON files in the batch directory.
        
        Returns:
            List of extracted data from all files
        """
        # Find all JSON files in the batch directory
        json_pattern = os.path.join(self.logs_dir, "*.json")
        json_files = glob.glob(json_pattern)
        
        if not json_files:
            print(f"No JSON files found in {self.logs_dir}")
            return []
        
        print(f"Found {len(json_files)} JSON files to analyze")
        
        extracted_data = []
        for json_file in sorted(json_files):
            filename = os.path.basename(json_file)
            
            data = self.load_json_file(json_file)
            if data is not None:
                extracted_info = self.extract_data_from_json(data, filename)
                extracted_data.append(extracted_info)
        
        return extracted_data
    
    def write_csv(self, data: List[Dict[str, Any]]) -> None:
        """
        Write extracted data to CSV file.
        
        Args:
            data: List of extracted data dictionaries
        """
        if not data:
            print("No data to write to CSV")
            return
        
        # Define CSV headers
        headers = [
            'initial_x',
            'initial_y', 
            'initial_yaw',
            'target_x',
            'target_y',
            'target_yaw',
            'algorithm',
            'mission_success',
            'computation_time_sec'
        ]
        
        try:
            with open(self.output_csv, 'w', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=headers)
                writer.writeheader()
                writer.writerows(data)
            
            print(f"CSV file written to: {self.output_csv}")
            print(f"Total records: {len(data)}")
            
        except IOError as e:
            print(f"Error writing CSV file: {e}")
    
    def print_summary(self, data: List[Dict[str, Any]]) -> None:
        """
        Print a summary of the analyzed data.
        
        Args:
            data: List of extracted data dictionaries
        """
        if not data:
            return
        
        # Count algorithms and collect computation times
        algorithm_counts = {}
        success_counts = {}
        algorithm_times = {}
        total_successful = 0
        
        for record in data:
            algorithm = record['algorithm']
            mission_success = record['mission_success']
            
            algorithm_counts[algorithm] = algorithm_counts.get(algorithm, 0) + 1
            
            if mission_success:
                total_successful += 1
                success_counts[algorithm] = success_counts.get(algorithm, 0) + 1
                
                # Collect computation times for successful cases
                if (record['computation_time_sec'] != '' and 
                    record['computation_time_sec'] != '-' and
                    isinstance(record['computation_time_sec'], (int, float))):
                    
                    if algorithm not in algorithm_times:
                        algorithm_times[algorithm] = []
                    algorithm_times[algorithm].append(float(record['computation_time_sec']))
        
        print("\n=== ANALYSIS SUMMARY ===")
        print(f"Total files analyzed: {len(data)}")
        print(f"Total successful missions: {total_successful}/{len(data)} ({100*total_successful/len(data):.1f}%)")
        
        print("\nAlgorithm performance:")
        for algorithm in sorted(algorithm_counts.keys()):
            count = algorithm_counts[algorithm]
            successful = success_counts.get(algorithm, 0)
            success_rate = 100 * successful / count if count > 0 else 0
            
            print(f"\n{algorithm}:")
            print(f"  Total trials: {count}")
            print(f"  Successful trials: {successful}")
            print(f"  Success rate: {success_rate:.1f}%")
            
            # Print computation time statistics for successful cases
            if algorithm in algorithm_times and algorithm_times[algorithm]:
                times = algorithm_times[algorithm]
                mean_time = sum(times) / len(times)
                variance = sum((t - mean_time) ** 2 for t in times) / len(times)
                std_dev = variance ** 0.5  # Standard deviation is square root of variance
                
                print(f"  Mean computation time: {mean_time:.3f} seconds")
                print(f"  Standard deviation: {std_dev:.6f} seconds")
            else:
                print(f"  Mean computation time: N/A (no successful trials)")
                print(f"  Standard deviation: N/A (no successful trials)")
    
    def create_scatter_plot(self, data: List[Dict[str, Any]]) -> None:
        """
        Create a scatter plot for 'Ours' and 'w/o PC' algorithms' computation times.
        
        Args:
            data: List of extracted data dictionaries
        """
        # Filter for only 'Ours' and 'w/o PC' algorithms with successful trials
        target_algorithms = {'Ours', 'w/o PC'}
        successful_data = []
        
        for record in data:
            if (record['algorithm'] in target_algorithms and
                record['mission_success'] and 
                record['computation_time_sec'] != '' and 
                record['computation_time_sec'] != '-' and
                isinstance(record['computation_time_sec'], (int, float))):
                successful_data.append({
                    'algorithm': record['algorithm'],
                    'computation_time': float(record['computation_time_sec'])
                })
        
        if not successful_data:
            print("No successful trials found for 'Ours' and 'w/o PC' algorithms")
            return
        
        # Group data by algorithm
        algorithm_times = {}
        for record in successful_data:
            algo = record['algorithm']
            if algo not in algorithm_times:
                algorithm_times[algo] = []
            algorithm_times[algo].append(record['computation_time'])
        
        # Check if we have data for both algorithms
        if not algorithm_times:
            print("No data available for scatter plot")
            return
        
        try:
            # Apply IEEE figure configuration for one-column width with 8pt font
            config = configure_ieee_style(target_font_size=8, verbose=False)
            
            # Create scatter plot with IEEE configuration
            fig, ax = plt.subplots(figsize=(config['fig_width'], config['fig_width'] * 0.6))
            
            # Use IEEE-friendly colors
            colors = {'Ours': IEEE_COLORS['navy_blue'], 'w/o PC': IEEE_COLORS['orange_red']}
            markers = {'Ours': 'o', 'w/o PC': 's'}
            
            # Plot scatter points for each algorithm
            for algo in sorted(algorithm_times.keys()):
                times_list = algorithm_times[algo]
                # Create x-coordinates (trial indices)
                x_coords = list(range(len(times_list)))
                
                ax.scatter(x_coords, times_list, 
                          c=colors[algo], marker=markers[algo], 
                          label=f'{algo}', 
                          alpha=0.7, s=25, edgecolors='black', linewidth=0.5)

            ax.set_xlabel('Trial Index')
            ax.set_ylabel('Computation Time (s)')
            ax.legend()
            ax.grid(True, alpha=0.3)
            
            # Add horizontal lines for mean values
            for algo in sorted(algorithm_times.keys()):
                times_list = algorithm_times[algo]
                mean_time = np.mean(times_list)
                ax.axhline(y=mean_time, color=colors[algo], linestyle='--', alpha=0.8, 
                          linewidth=1.5)
            
            # Apply tight layout
            plt.tight_layout()
            
            plt.show()
            
            print(f"\nFigure configured for IEEE one-column width ({config['column_width']}\") with {config['base_font_size']*config['scale_factor']:.0f}pt fonts")
            
            # Print detailed statistics for plotted algorithms
            print("\n=== SCATTER PLOT STATISTICS ===")
            for algo in sorted(algorithm_times.keys()):
                times_list = algorithm_times[algo]
                print(f"\n{algo}:")
                print(f"  Count: {len(times_list)}")
                print(f"  Mean: {np.mean(times_list):.3f} seconds")
                print(f"  Std:  {np.std(times_list):.3f} seconds")
                print(f"  Min:  {np.min(times_list):.3f} seconds")
                print(f"  Max:  {np.max(times_list):.3f} seconds")
                print(f"  Median: {np.median(times_list):.3f} seconds")
        
        except ImportError as e:
            print(f"Error creating scatter plot: {e}")
            print("Please install required packages: pip install matplotlib seaborn")
        except Exception as e:
            print(f"Error creating scatter plot: {e}")
    
    
    def run_analysis(self) -> None:
        """Run the complete analysis pipeline."""
        print(f"Starting analysis of batch: {self.batch_folder}")
        print(f"Looking for JSON files in: {self.logs_dir}")
        
        # Check if logs directory exists
        if not os.path.exists(self.logs_dir):
            print(f"Error: Logs directory does not exist: {self.logs_dir}")
            return
        
        # Analyze all files
        extracted_data = self.analyze_batch()
        
        if extracted_data:
            # Write CSV
            self.write_csv(extracted_data)
            
            # Print summary
            self.print_summary(extracted_data)
            
            # Create scatter plot for 'Ours' and 'w/o PC' only
            self.create_scatter_plot(extracted_data)
        else:
            print("No data extracted from JSON files")


def main():
    """Main function."""
    analyzer = BatchDataAnalyzer("20250905_batch_200_all")
    analyzer.run_analysis()


if __name__ == "__main__":
    main()
