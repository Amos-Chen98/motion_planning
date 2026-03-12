#!/usr/bin/env python3
"""
Configurable transformation path test script

This script allows users to test arbitrary coordinate transformation sequences 
by simply modifying the path list. It can be used to verify the consistency 
of Jacobian transformations.

Usage:
1. Modify the CUSTOM_TEST_PATHS list to define the paths you want to test
2. Run the script to view results
3. All paths should produce the same point transformation and Jacobian transformation results

Example paths:
- ['thrust1', 'cog'] - Direct from thrust frame to center of gravity
- ['thrust1', 'world', 'cog'] - Through world coordinate system to center of gravity
- ['thrust1', 'fc', 'world', 'cog'] - Multi-step transformation
"""


import os
import sys
import numpy as np

# Add parent directory to path for imports
current_path = os.path.abspath(os.path.dirname(__file__))
parent_path = os.path.dirname(current_path)
sys.path.insert(0, parent_path)
from robot_model import Robot


# =============================================================================
# Configuration section - Modify here to test different transformation paths
# =============================================================================

# Define transformation paths to test
# CUSTOM_TEST_PATHS = [
#     ['thrust1', 'world'],                    # Direct path
#     ['thrust1', 'fc', 'world'],
#     ['thrust1', 'link1', 'world'],
#     ['thrust1', 'cog', 'world', 'fc', 'world']
# ]

CUSTOM_TEST_PATHS = [
    ['thrust1', 'world','cog',  'link1','cog'],                    # Direct path
    ['thrust1', 'fc', 'world', 'cog',  'link1', 'cog'],
    ['thrust1', 'cog', 'link1', 'world', 'world', 'cog',  'link1', 'cog'],
    ['thrust1', 'fc', 'world', 'cog',  'link1', 'cog'],
    ['thrust1', 'gimbal1', 'joint1', 'world', 'world', 'cog', 'link1', 'cog'],
    ['thrust1', 'gimbal1', 'world', 'world', 'joint1', 'cog', 'link1', 'cog']
]

# Test point
TEST_POINT = np.array([0.0, 0.0, 1.0])

# Robot configuration
TEST_CONFIG = np.array([1.0, 2.0, 0.5, 0.2, -0.3, 0.1])

# =============================================================================


def find_urdf_file():
    """Find available URDF file"""
    search_paths = [
        os.path.join(current_path, "..", "..", "..", "urdf"),
        os.path.join(current_path, "..", "..", "..", "..", "urdf"),
        os.path.join(current_path, "..", "..", "..", "..", "..", "urdf"),
    ]

    urdf_files = ["hydrus_xi_20241227.urdf", "2d_opening.urdf"]

    for search_path in search_paths:
        for urdf_file in urdf_files:
            full_path = os.path.join(search_path, urdf_file)
            if os.path.exists(full_path):
                return full_path
    return None


def execute_transformation_path(robot, initial_point, path, initial_jacobian):
    """
    Execute a transformation path sequence

    Args:
        robot: Robot object
        initial_point: Starting 3D point
        path: List of frame names [src, intermediate1, ..., target]
        initial_jacobian: Initial Jacobian matrix (6 x config_dim)

    Returns:
        (final_point, final_jacobian): Final point and Jacobian matrix
    """
    if len(path) < 2:
        raise ValueError("Path must contain at least 2 frames (source and target)")

    current_point = initial_point.copy()
    current_jacobian = initial_jacobian.copy()

    # Execute transformations step by step
    for i in range(len(path) - 1):
        src_frame = path[i]
        target_frame = path[i + 1]

        # Execute single-step transformation
        current_point, step_jacobian = robot.transform_between_frames(
            current_point, src_frame, target_frame,
            return_jacobian=True, src_jacobian=current_jacobian
        )

        # Update Jacobian matrix for next step
        if i < len(path) - 2:  # Not the last step
            # Pad Jacobian matrix back to 6D format
            next_jacobian = np.zeros((6, robot.config_dim))
            next_jacobian[:3, :] = step_jacobian
            current_jacobian = next_jacobian
        else:
            # Last step, keep 3D Jacobian matrix
            current_jacobian = step_jacobian

    return current_point, current_jacobian


def test_custom_paths():
    """Test custom transformation paths"""
    urdf_path = find_urdf_file()
    if not urdf_path:
        print("Error: Unable to find URDF file")
        return False

    print("Configurable Transformation Path Test")
    print("=" * 60)
    print(f"Using URDF file: {os.path.basename(urdf_path)}")
    print(f"Test point: {TEST_POINT}")
    print(f"Robot configuration: {TEST_CONFIG}")

    # Initialize robot
    robot = Robot(urdf_path)
    robot_q = robot.get_robot_q(TEST_CONFIG)
    robot.forward_kinematics(robot_q)

    # Check available frames
    available_frames = ['world', 'cog']
    frame_candidates = ['fc', 'link1', 'thrust1', 'thrust2', 'thrust3', 'thrust4', 'gimbal1', 'gimbal2', 'bat1', 'joint1']
    for frame_name in frame_candidates:
        try:
            robot.robot_model.getFrameId(frame_name)
            available_frames.append(frame_name)
        except:
            pass

    print(f"Available frames: {available_frames}")

    # Filter valid paths
    valid_paths = []
    for path in CUSTOM_TEST_PATHS:
        if all(frame in available_frames for frame in path):
            valid_paths.append(path)
        else:
            missing_frames = [frame for frame in path if frame not in available_frames]
            print(f"⚠ Skipping path {' -> '.join(path)}: missing frames {missing_frames}")

    if len(valid_paths) < 2:
        print("Error: Need at least 2 valid paths for comparison")
        return False

    print(f"\nTesting {len(valid_paths)} valid paths:")
    for i, path in enumerate(valid_paths):
        print(f"  Path {i+1}: {' -> '.join(path)}")

    # Execute all transformation paths
    results = []
    initial_jacobian = np.zeros((6, robot.config_dim))

    for i, path in enumerate(valid_paths):
        try:
            point_result, jac_result = execute_transformation_path(
                robot, TEST_POINT, path, initial_jacobian
            )
            results.append((path, point_result, jac_result))
            print(f"✓ Path {i+1} executed successfully")
        except Exception as e:
            print(f"⚠ Path {i+1} failed: {e}")
            continue

    if len(results) < 2:
        print("Error: Need at least 2 successful paths for comparison")
        return False

    # Compare all paths with the first (reference) path
    ref_path, ref_point, ref_jac = results[0]
    print(f"\nUsing reference path: {' -> '.join(ref_path)}")

    all_consistent = True
    max_point_diff = 0.0
    max_jac_diff = 0.0

    print(f"\n{'Path Comparison Results':=^60}")

    for i, (path, point, jac) in enumerate(results[1:], 1):
        point_diff_norm = np.linalg.norm(point - ref_point)
        jac_diff_norm = np.linalg.norm(jac - ref_jac)

        max_point_diff = max(max_point_diff, point_diff_norm)
        max_jac_diff = max(max_jac_diff, jac_diff_norm)

        path_str = ' -> '.join(path)
        print(f"\nPath {i+1}: {path_str}")
        print(f"  Point difference norm:     {point_diff_norm:.2e}")
        print(f"  Jacobian difference norm: {jac_diff_norm:.2e}")

        if point_diff_norm > 1e-12:
            print(f"  ❌ Point transformation inconsistent!")
            all_consistent = False
        elif jac_diff_norm > 1e-6:
            print(f"  ⚠ Jacobian inconsistent!")
            all_consistent = False
        else:
            print(f"  ✓ Path consistent")

    # Summary report
    print(f"\n{'Test Summary':=^60}")
    print(f"Number of test paths: {len(results)}")
    print(f"Maximum point difference: {max_point_diff:.2e}")
    print(f"Maximum Jacobian difference: {max_jac_diff:.2e}")

    if all_consistent:
        print(f"✅ All transformation paths are consistent")
        return True
    else:
        print(f"❌ Detected transformation path inconsistencies")
        print(f"This indicates a BUG in the Jacobian transformation logic")
        return False


if __name__ == '__main__':
    print(__doc__)
    success = test_custom_paths()
    sys.exit(0 if success else 1)
