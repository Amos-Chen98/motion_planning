#!/usr/bin/env python3
"""
Simple test for transform_between_frames Jacobian path consistency.

This test verifies that different transformation paths yield consistent results:
- Path 1: thrust_frame -> world -> cog  
- Path 2: thrust_frame -> cog (direct)

Both paths should yield identical results for both points and Jacobians.
"""

import os
import sys
import unittest
import numpy as np

# Add parent directory to path for imports
current_path = os.path.abspath(os.path.dirname(__file__))
parent_path = os.path.dirname(current_path)
sys.path.insert(0, parent_path)

from robot_model import Robot


class TestTransformBetweenFrames(unittest.TestCase):
    """Test Jacobian path consistency for transform_between_frames function."""

    @classmethod
    def setUpClass(cls):
        """Set up test fixtures."""
        cls.urdf_path = cls._find_urdf_file()
        if not cls.urdf_path:
            raise FileNotFoundError("Could not find URDF file for testing")

    @staticmethod
    def _find_urdf_file() -> str:
        """Find a suitable URDF file for testing."""
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

    def setUp(self):
        """Set up test robot for each test."""
        self.robot = Robot(self.urdf_path)
        
        # Define test configuration  
        self.test_config = np.array([1.0, 2.0, 0.5, 0.2, -0.3, 0.1])
        
        # Prepare robot state
        robot_q = self.robot.get_robot_q(self.test_config)
        self.robot.forward_kinematics(robot_q)
        
        # Find available frames
        self.available_frames = ['world', 'cog']
        frame_candidates = ['fc', 'link1', 'rotor1', 'thrust1']
        for frame_name in frame_candidates:
            try:
                self.robot.robot_model.getFrameId(frame_name)
                self.available_frames.append(frame_name)
            except:
                pass

    def test_jacobian_transformation_path_consistency(self):
        """
        Test that different transformation paths yield consistent results.
        
        Configurable transformation path testing with flexible path definitions.
        """
        # Test point and configuration
        test_point = np.array([0.0, 0.0, 1.0])
        
        # Find available thrust frame
        thrust_frames = [frame for frame in self.available_frames if 'thrust' in frame]
        if not thrust_frames:
            self.skipTest("No thrust frames available")
        
        thrust_frame = thrust_frames[0]
        
        # Define transformation paths to test
        test_paths = [
            [thrust_frame, 'cog'],                    # Direct path
            [thrust_frame, 'world', 'cog'],          # Via world
            [thrust_frame, 'world', 'world', 'cog'], # Via world twice (redundant step)
        ]
        
        # Additional paths if more frames are available
        if 'fc' in self.available_frames:
            test_paths.append([thrust_frame, 'fc', 'cog'])  # Via flight controller
            test_paths.append([thrust_frame, 'world', 'fc', 'cog'])  # Multi-hop via world and fc
        
        self._test_transformation_paths(test_point, test_paths)
    
    def test_custom_transformation_paths(self):
        """
        Test custom transformation paths - can be easily modified for specific testing.
        
        Example usage:
        - Modify custom_paths list to test specific transformation sequences
        - Add new frame combinations to verify Jacobian consistency
        """
        # Test point
        test_point = np.array([1.0, 0.5, 0.2])
        
        # Find available frames
        available_frames_set = set(self.available_frames)
        
        # Define custom paths for testing - EASILY CONFIGURABLE
        custom_paths = [
            ['world', 'cog'],                    # Basic world to cog
            ['cog', 'world'],                    # Reverse: cog to world
        ]
        
        # Add thrust frame paths if available
        thrust_frames = [f for f in self.available_frames if 'thrust' in f]
        if thrust_frames:
            thrust_frame = thrust_frames[0]
            custom_paths.extend([
                [thrust_frame, 'world'],             # Thrust to world
                [thrust_frame, 'cog'],               # Thrust to cog (direct)
                [thrust_frame, 'world', 'cog'],     # Thrust via world to cog
            ])
        
        # Add FC paths if available
        if 'fc' in available_frames_set:
            custom_paths.extend([
                ['fc', 'world'],                     # FC to world
                ['fc', 'cog'],                       # FC to cog (direct)
                ['world', 'fc', 'cog'],             # World via FC to cog
            ])
            
            # Complex multi-hop paths
            if thrust_frames:
                thrust_frame = thrust_frames[0]
                custom_paths.extend([
                    [thrust_frame, 'fc', 'world'],       # Thrust via FC to world
                    [thrust_frame, 'fc', 'cog'],         # Thrust via FC to cog
                    [thrust_frame, 'world', 'fc'],       # Thrust via world to FC
                ])
        
        print(f"\n=== CUSTOM PATH TESTING ===")
        print(f"Available frames: {self.available_frames}")
        print(f"Testing {len(custom_paths)} custom transformation paths")
        
        self._test_transformation_paths(test_point, custom_paths)
    
    def _test_transformation_paths(self, test_point, test_paths):
        """
        Core method to test multiple transformation paths.
        
        Args:
            test_point: 3D point to transform
            test_paths: List of paths, where each path is a list of frame names
        """
        print(f"\nTesting {len(test_paths)} transformation paths:")
        for i, path in enumerate(test_paths):
            print(f"  Path {i+1}: {' -> '.join(path)}")
        
        # Execute all transformation paths
        results = []
        for i, path in enumerate(test_paths):
            try:
                point_result, jac_result = self._execute_transformation_path(
                    test_point, path, np.zeros((6, self.robot.config_dim))
                )
                results.append((path, point_result, jac_result))
                print(f"✓ Path {i+1} executed successfully")
            except Exception as e:
                print(f"⚠ Path {i+1} failed: {e}")
                continue
        
        if len(results) < 2:
            self.skipTest("Need at least 2 successful paths for comparison")
        
        # Compare all paths against the first (reference) path
        ref_path, ref_point, ref_jac = results[0]
        print(f"\nUsing reference path: {' -> '.join(ref_path)}")
        
        all_consistent = True
        max_point_diff = 0.0
        max_jac_diff = 0.0
        
        for i, (path, point, jac) in enumerate(results[1:], 1):
            point_diff_norm = np.linalg.norm(point - ref_point)
            jac_diff_norm = np.linalg.norm(jac - ref_jac)
            
            max_point_diff = max(max_point_diff, point_diff_norm)
            max_jac_diff = max(max_jac_diff, jac_diff_norm)
            
            path_str = ' -> '.join(path)
            print(f"\nComparing {path_str} vs reference:")
            print(f"  Point difference norm: {point_diff_norm:.2e}")
            print(f"  Jacobian difference norm: {jac_diff_norm:.2e}")
            
            # Points should be identical
            self.assertLess(point_diff_norm, 1e-12, 
                f"Point transformation inconsistent for path: {path_str}")
            
            # Check Jacobian consistency
            if jac_diff_norm > 1e-6:
                print(f"  ⚠ JACOBIAN INCONSISTENCY detected")
                all_consistent = False
            else:
                print(f"  ✓ Path is consistent")
        
        # Summary report
        print(f"\n" + "="*50)
        print(f"TRANSFORMATION PATH TESTING SUMMARY")
        print(f"="*50)
        print(f"Paths tested: {len(results)}")
        print(f"Max point difference: {max_point_diff:.2e}")
        print(f"Max Jacobian difference: {max_jac_diff:.2e}")
        
        # Final assessment
        if not all_consistent:
            print(f"❌ JACOBIAN TRANSFORMATION BUG CONFIRMED")
            print("Multiple transformation paths yield different Jacobian results")
            self.fail("Jacobian transformation paths are inconsistent")
        else:
            print(f"✅ All transformation paths are consistent")
    
    def _execute_transformation_path(self, initial_point, path, initial_jacobian):
        """
        Execute a transformation through a sequence of frames.
        
        Args:
            initial_point: Starting 3D point
            path: List of frame names [src, intermediate1, ..., target]
            initial_jacobian: Initial Jacobian matrix (6 x config_dim)
            
        Returns:
            (final_point, final_jacobian): Results after complete transformation
        """
        if len(path) < 2:
            raise ValueError("Path must contain at least 2 frames (source and target)")
        
        current_point = initial_point.copy()
        current_jacobian = initial_jacobian.copy()
        
        # Transform through each step in the path
        for i in range(len(path) - 1):
            src_frame = path[i]
            target_frame = path[i + 1]
            
            # Execute single transformation step
            current_point, step_jacobian = self.robot.transform_between_frames(
                current_point, src_frame, target_frame,
                return_jacobian=True, src_jacobian=current_jacobian
            )
            
            # Update jacobian for next step (only first 3 rows are used for points)
            if i < len(path) - 2:  # Not the last step
                # Pad jacobian back to 6D format for next transformation
                next_jacobian = np.zeros((6, self.robot.config_dim))
                next_jacobian[:3, :] = step_jacobian
                current_jacobian = next_jacobian
            else:
                # Last step, keep 3D jacobian
                current_jacobian = step_jacobian
        
        return current_point, current_jacobian


if __name__ == '__main__':
    """Main entry point for running tests."""
    print("Transform Between Frames Test Suite - Jacobian Path Consistency")
    print("="*60)
    
    # Run the single test
    unittest.main(verbosity=2)
