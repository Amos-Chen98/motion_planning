#!/usr/bin/env python3
"""
Hyperparameter Analysis Script for Motion Planner

This script analyzes JSON files from hyperparameter testing to extract combinations
of sample_density and transform_attempt_num, calculating trial numbers, success rates,
and average computation times for successful cases.
"""

import json
import os
import glob
from collections import defaultdict
from typing import Dict, List, Any, Tuple, Optional


class HyperparamAnalyzer:
    """Analyzer for hyperparameter testing results."""
    
    def __init__(self, data_folder: str = "20250921_hyperparam_test"):
        """
        Initialize the hyperparameter analyzer.
        
        Args:
            data_folder: Name of the data folder containing JSON files
        """
        self.data_folder = data_folder
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Hard-coded data folder path
        self.data_dir = os.path.join(self.script_dir, "..", "..", "logs", data_folder)
        
        # Dictionary to store results grouped by parameter combinations
        # Key: (sample_density, transform_attempt_num), Value: list of trial data
        self.parameter_combinations = defaultdict(list)
    
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
    
    def extract_trial_data(self, data: Dict[str, Any], filename: str) -> Optional[Dict[str, Any]]:
        """
        Extract relevant trial data from a JSON file.
        
        Args:
            data: JSON data from the planning session
            filename: Name of the JSON file
            
        Returns:
            Dictionary with extracted trial data or None if invalid
        """
        # Check if planner_config exists
        if 'planner_config' not in data:
            print(f"Warning: No planner_config found in {filename}")
            return None
        
        planner_config = data['planner_config']
        
        # Extract required parameters
        sample_density = planner_config.get('sample_density', None)
        transform_attempt_num = planner_config.get('transform_attempt_num', None)
        
        if sample_density is None or transform_attempt_num is None:
            print(f"Warning: Missing required parameters in {filename}")
            return None
        
        # Extract session info
        session_info = data.get('session_info', {})
        mission_success = session_info.get('mission_success', False)
        
        # Handle null values for mission_success
        if mission_success is None:
            mission_success = False
        
        computation_time = session_info.get('computation_time_sec', None)
        
        # If mission failed or computation time is missing, mark computation time as None
        if not mission_success or computation_time is None or computation_time == '' or computation_time == '-':
            computation_time = None
        else:
            try:
                computation_time = float(computation_time)
            except (ValueError, TypeError):
                computation_time = None
        
        return {
            'filename': filename,
            'sample_density': sample_density,
            'transform_attempt_num': transform_attempt_num,
            'mission_success': mission_success,
            'computation_time_sec': computation_time
        }
    
    def analyze_json_files(self) -> None:
        """
        Analyze all JSON files in the data directory.
        """
        # Find all JSON files in the data directory
        json_pattern = os.path.join(self.data_dir, "*.json")
        json_files = glob.glob(json_pattern)
        
        if not json_files:
            print(f"No JSON files found in {self.data_dir}")
            return
        
        print(f"Found {len(json_files)} JSON files to analyze")
        
        # Process each file
        for json_file in sorted(json_files):
            filename = os.path.basename(json_file)
            
            data = self.load_json_file(json_file)
            if data is not None:
                trial_data = self.extract_trial_data(data, filename)
                if trial_data is not None:
                    # Group by parameter combination
                    key = (trial_data['sample_density'], trial_data['transform_attempt_num'])
                    self.parameter_combinations[key].append(trial_data)
    
    def calculate_statistics(self) -> Dict[Tuple[Any, Any], Dict[str, Any]]:
        """
        Calculate statistics for each parameter combination.
        
        Returns:
            Dictionary with statistics for each parameter combination
        """
        statistics = {}
        
        for params, trials in self.parameter_combinations.items():
            sample_density, transform_attempt_num = params
            
            # Calculate basic statistics
            total_trials = len(trials)
            successful_trials = [trial for trial in trials if trial['mission_success']]
            success_count = len(successful_trials)
            success_rate = (success_count / total_trials * 100) if total_trials > 0 else 0.0
            
            # Calculate average computation time for successful cases
            computation_times = [trial['computation_time_sec'] for trial in successful_trials 
                               if trial['computation_time_sec'] is not None]
            
            if computation_times:
                avg_computation_time = sum(computation_times) / len(computation_times)
            else:
                avg_computation_time = None
            
            statistics[params] = {
                'sample_density': sample_density,
                'transform_attempt_num': transform_attempt_num,
                'total_trials': total_trials,
                'successful_trials': success_count,
                'success_rate': success_rate,
                'avg_computation_time_sec': avg_computation_time,
                'valid_computation_times': len(computation_times)
            }
        
        return statistics
    
    def print_results(self, statistics: Dict[Tuple[Any, Any], Dict[str, Any]]) -> None:
        """
        Print the analysis results in a formatted table.
        
        Args:
            statistics: Dictionary with statistics for each parameter combination
        """
        if not statistics:
            print("No parameter combinations found to analyze.")
            return
        
        print("\n" + "="*80)
        print("HYPERPARAMETER ANALYSIS RESULTS")
        print("="*80)
        print(f"Data folder: {self.data_folder}")
        print(f"Total parameter combinations: {len(statistics)}")
        
        # Sort by sample_density first, then by transform_attempt_num
        sorted_params = sorted(statistics.keys(), key=lambda x: (x[0], x[1]))
        
        print("\n" + "-"*80)
        print(f"{'Sample':<8} {'Transform':<10} {'Total':<7} {'Success':<8} {'Success':<8} {'Avg Compute':<12}")
        print(f"{'Density':<8} {'Attempts':<10} {'Trials':<7} {'Trials':<8} {'Rate (%)':<8} {'Time (s)':<12}")
        print("-"*80)
        
        for params in sorted_params:
            stats = statistics[params]
            
            # Format the values
            sample_density = stats['sample_density']
            transform_attempts = stats['transform_attempt_num']
            total_trials = stats['total_trials']
            successful_trials = stats['successful_trials']
            success_rate = f"{stats['success_rate']:.1f}"
            
            if stats['avg_computation_time_sec'] is not None:
                avg_time = f"{stats['avg_computation_time_sec']:.3f}"
            else:
                avg_time = "N/A"
            
            print(f"{sample_density:<8} {transform_attempts:<10} {total_trials:<7} "
                  f"{successful_trials:<8} {success_rate:<8} {avg_time:<12}")
        
        print("-"*80)
        
        # Print additional summary statistics
        total_all_trials = sum(stats['total_trials'] for stats in statistics.values())
        total_all_successful = sum(stats['successful_trials'] for stats in statistics.values())
        overall_success_rate = (total_all_successful / total_all_trials * 100) if total_all_trials > 0 else 0.0
        
        print(f"\nOVERALL SUMMARY:")
        print(f"Total trials across all combinations: {total_all_trials}")
        print(f"Total successful trials: {total_all_successful}")
        print(f"Overall success rate: {overall_success_rate:.1f}%")
        
        # Find best performing combination
        best_success_rate = max((stats['success_rate'] for stats in statistics.values()), default=0)
        best_combinations = [params for params, stats in statistics.items() 
                           if stats['success_rate'] == best_success_rate]
        
        if best_combinations:
            print(f"\nBest performing combination(s) ({best_success_rate:.1f}% success rate):")
            for params in best_combinations:
                stats = statistics[params]
                avg_time_str = f"{stats['avg_computation_time_sec']:.3f}s" if stats['avg_computation_time_sec'] is not None else "N/A"
                print(f"  Sample Density: {params[0]}, Transform Attempts: {params[1]} "
                      f"(Avg time: {avg_time_str})")
    
    def run_analysis(self) -> None:
        """Run the complete hyperparameter analysis pipeline."""
        print(f"Starting hyperparameter analysis...")
        print(f"Looking for JSON files in: {self.data_dir}")
        
        # Check if data directory exists
        if not os.path.exists(self.data_dir):
            print(f"Error: Data directory does not exist: {self.data_dir}")
            print(f"Please ensure the folder '{self.data_folder}' exists in the logs directory.")
            return
        
        # Analyze all files
        self.analyze_json_files()
        
        if self.parameter_combinations:
            # Calculate statistics
            statistics = self.calculate_statistics()
            
            # Print results
            self.print_results(statistics)
        else:
            print("No valid parameter combinations found in JSON files.")


def main():
    """Main function."""
    # Hard-coded data folder as requested
    analyzer = HyperparamAnalyzer("20250923_hyperparam_test")
    analyzer.run_analysis()


if __name__ == "__main__":
    main()