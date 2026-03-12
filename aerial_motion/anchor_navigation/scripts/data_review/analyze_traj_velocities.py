#!/usr/bin/env python3
"""
Trajectory Velocity Analysis Script

This script processes trajectory JSON files and calculates maximum velocities:
- Maximum linear velocity of rootlink (max of vel_x and vel_y)
- Maximum yaw angular velocity of rootlink
- Maximum angular velocity for each joint
"""

import sys
import os
current_path = os.path.abspath(os.path.dirname(__file__))
planner_path = os.path.join(os.path.dirname(current_path), 'planner')
sys.path.insert(0, current_path)
sys.path.insert(0, planner_path)

# Add current path and planner path for imports
from traj_coder import TrajCoder
import numpy as np


# =============================================================================
# CONFIGURATION - Change these settings as needed
# =============================================================================
TRAJECTORY_FILENAME = "xxx"
CMD_HZ = 40.0
SPEED_SCALE = 1.0


def analyze_trajectory_velocities(traj_coder, filepath, cmd_hz=50.0, speed_scale=1.0):
    """
    Analyze trajectory velocities and find maximum values.

    Args:
        traj_coder: TrajCoder instance
        filepath: Path to the trajectory JSON file
        cmd_hz: Command frequency in Hz
        speed_scale: Speed scaling factor

    Returns:
        dict: Dictionary containing maximum velocity values
    """
    # Load and compute full desired commands
    print(f"Loading trajectory from: {filepath}")
    des_states = traj_coder.load_and_compute_commands(filepath, cmd_hz, speed_scale)
    if des_states is None:
        print("Failed to load trajectory!")
        return None

    print(f"Generated {len(des_states)} desired state points")

    # Initialize tracking variables
    max_rootlink_vel_x = 0.0
    max_rootlink_vel_y = 0.0
    max_rootlink_linear_vel = 0.0
    max_rootlink_yaw_vel = 0.0
    joint_num = len(des_states[0]['joints'])
    max_joint_vels = [0.0] * joint_num

    # Analyze each state point
    for state in des_states:
        rootlink_state = state['rootlink']

        # Get rootlink velocities
        vel_x = abs(rootlink_state.vel_x)
        vel_y = abs(rootlink_state.vel_y)
        omega_z = abs(rootlink_state.omega_z)

        # Update maximum velocity in x dimension
        if vel_x > max_rootlink_vel_x:
            max_rootlink_vel_x = vel_x

        # Update maximum velocity in y dimension
        if vel_y > max_rootlink_vel_y:
            max_rootlink_vel_y = vel_y

        # Update maximum linear velocity (take max of vel_x and vel_y)
        max_linear_vel_at_point = max(vel_x, vel_y)
        if max_linear_vel_at_point > max_rootlink_linear_vel:
            max_rootlink_linear_vel = max_linear_vel_at_point

        # Update maximum yaw angular velocity
        if omega_z > max_rootlink_yaw_vel:
            max_rootlink_yaw_vel = omega_z

        # Update maximum joint velocities
        for j in range(joint_num):
            joint_vel = abs(state['joints'][j].vel)
            if joint_vel > max_joint_vels[j]:
                max_joint_vels[j] = joint_vel

    # Compile results
    results = {
        'max_rootlink_vel_x': max_rootlink_vel_x,
        'max_rootlink_vel_y': max_rootlink_vel_y,
        'max_rootlink_linear_vel': max_rootlink_linear_vel,
        'max_rootlink_yaw_vel': max_rootlink_yaw_vel,
        'max_joint_vels': max_joint_vels,
        'num_joints': joint_num,
        'num_points': len(des_states)
    }

    return results


def print_results(results):
    """
    Print analysis results in a formatted manner.

    Args:
        results: Dictionary containing analysis results
    """
    print("\n" + "="*60)
    print("TRAJECTORY VELOCITY ANALYSIS RESULTS")
    print("="*60)

    print(f"\nNumber of trajectory points: {results['num_points']}")

    print("\n--- Rootlink Velocities ---")
    print(f"Maximum linear velocity in X: {results['max_rootlink_vel_x']:.3g} m/s")
    print(f"Maximum linear velocity in Y: {results['max_rootlink_vel_y']:.3g} m/s")
    print(f"Maximum linear velocity (max of vel_x, vel_y): {results['max_rootlink_linear_vel']:.3g} m/s")
    print(f"Maximum yaw angular velocity:                  {results['max_rootlink_yaw_vel']:.3g} rad/s")
    print(f"                                                {np.rad2deg(results['max_rootlink_yaw_vel']):.3g} deg/s")

    print("\n--- Joint Angular Velocities ---")
    for j in range(results['num_joints']):
        print(f"Joint {j+1} maximum angular velocity: {results['max_joint_vels'][j]:.3g} rad/s")
        print(f"                                     {np.rad2deg(results['max_joint_vels'][j]):.3g} deg/s")

    print("\n" + "="*60)


def main():
    """
    Main function to run the trajectory velocity analysis.
    """
    motion_planner_root = os.path.abspath(
        os.path.join(current_path, '..', '..')
    )
    urdf_dir = os.path.join(motion_planner_root, 'urdf')
    urdf_files = [f for f in os.listdir(urdf_dir) if f.endswith('.urdf')]
    if not urdf_files:
        print(f"Error: No URDF files found in {urdf_dir}")
        return
    urdf_path = os.path.join(urdf_dir, urdf_files[0])

    print(f"Using URDF: {urdf_path}")

    # Construct trajectory file path relative to saved_traj directory
    saved_traj_dir = os.path.join(motion_planner_root, 'saved_traj')
    trajectory_file = os.path.join(saved_traj_dir, TRAJECTORY_FILENAME)

    if not os.path.exists(trajectory_file):
        print(f"Error: Trajectory file not found: {trajectory_file}")
        return

    # Initialize TrajCoder and analyze
    traj_coder = TrajCoder(urdf_path)
    results = analyze_trajectory_velocities(
        traj_coder,
        trajectory_file,
        cmd_hz=CMD_HZ,
        speed_scale=SPEED_SCALE
    )

    if results is not None:
        print_results(results)
    else:
        print("Failed to analyze trajectory!")


if __name__ == '__main__':
    main()
