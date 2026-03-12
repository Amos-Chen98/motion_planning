import os
import sys
import json
import numpy as np
from datetime import datetime
import rospy

# Add current path for imports
current_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_path)

from bspline_planner import Traj
from robot_model import Robot


class TrajCoder:
    """
    A class for saving and loading trajectory data to/from files.
    Handles serialization of Traj objects and computation of desired control commands.
    """
    
    def __init__(self, robot_urdf):
        """
        Initialize TrajCoder with robot model.
        
        Args:
            robot_urdf: Path to robot URDF file
        """
        self.robot = Robot(robot_urdf)
        # Navigate from scripts/planner/ to motion_planner/saved_traj/
        # current_path is scripts/planner/, so we go up 2 levels to motion_planner/
        self.save_dir = os.path.join(
            os.path.dirname(os.path.dirname(current_path)), 
            'saved_traj'
        )
        self.save_dir = os.path.abspath(self.save_dir)
        
        # Log the save directory path for verification
        rospy.loginfo(f"TrajCoder save directory: {self.save_dir}")
        
        # Ensure save directory exists
        os.makedirs(self.save_dir, exist_ok=True)
        
    def save_trajectory_segments(self, traj_segments):
        """
        Save all trajectory segments to a JSON file with timestamp name.
        
        Args:
            traj_segments: List of Traj objects from each planning segment
            
        Returns:
            str: Path to the saved file
        """
        # Generate timestamp-based filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")  # Include seconds only
        filename = f"traj_{timestamp}.json"
        filepath = os.path.join(self.save_dir, filename)
        
        # Convert trajectory segments to serializable format
        serialized_segments = []
        for i, traj in enumerate(traj_segments):
            if not isinstance(traj, Traj):
                rospy.logerr(f"Segment {i} is not a Traj object: {type(traj)}")
                continue
                
            segment_data = {
                'segment_id': i,
                'start_pos': traj.start_pos.tolist(),
                'end_pos': traj.end_pos.tolist(),
                'start_vel': traj.start_vel.tolist(),
                'end_vel': traj.end_vel.tolist(),
                'time': float(traj.time),
                'control_points': traj.control_points.tolist()
            }
            serialized_segments.append(segment_data)
        
        # Create complete trajectory data structure
        trajectory_data = {
            'metadata': {
                'timestamp': timestamp,
                'num_segments': len(serialized_segments),
                'config_dim': self.robot.config_dim,
                'saved_time': datetime.now().isoformat()
            },
            'segments': serialized_segments
        }
        
        # Save to JSON file
        try:
            with open(filepath, 'w') as f:
                json.dump(trajectory_data, f, indent=2)
            rospy.loginfo(f"Trajectory saved to: {filepath}")
            return filepath
        except Exception as e:
            rospy.logerr(f"Failed to save trajectory: {e}")
            return None
    
    def load_trajectory_segments(self, filepath, speed_scale=1.0):
        """
        Load trajectory segments from a JSON file.
        
        Args:
            filepath: Path to the trajectory file
            speed_scale: Speed scaling factor (0.5 = half speed, 2.0 = double speed)
            
        Returns:
            List[Traj]: List of Traj objects loaded from file
        """
        try:
            with open(filepath, 'r') as f:
                trajectory_data = json.load(f)
        except Exception as e:
            rospy.logerr(f"Failed to load trajectory from {filepath}: {e}")
            return None
        
        # Validate metadata
        metadata = trajectory_data.get('metadata', {})
        if metadata.get('config_dim') != self.robot.config_dim:
            rospy.logwarn(f"Config dimension mismatch: file has {metadata.get('config_dim')}, "
                         f"robot has {self.robot.config_dim}")
        
        # Load trajectory segments
        traj_segments = []
        segments_data = trajectory_data.get('segments', [])
        
        # Log speed scaling information
        if speed_scale != 1.0:
            original_total_time = sum(segment['time'] for segment in segments_data)
            scaled_total_time = original_total_time / speed_scale
            rospy.loginfo(f"Applying speed scale {speed_scale}: "
                         f"trajectory duration {original_total_time:.2f}s -> {scaled_total_time:.2f}s")
        
        for segment_data in segments_data:
            traj = Traj(dim=self.robot.config_dim)
            traj.start_pos = np.array(segment_data['start_pos'])
            traj.end_pos = np.array(segment_data['end_pos'])
            traj.start_vel = np.array(segment_data['start_vel'])
            traj.end_vel = np.array(segment_data['end_vel'])
            
            # Apply speed scaling to segment duration
            # If speed_scale = 0.5, trajectory should take 2x longer (time = original_time / speed_scale)
            # If speed_scale = 2.0, trajectory should take 0.5x time (time = original_time / speed_scale)
            original_time = float(segment_data['time'])
            traj.time = original_time / speed_scale
            
            # Velocities should be scaled by speed_scale to maintain continuity
            # If trajectory takes longer, velocities should be proportionally slower
            traj.start_vel = traj.start_vel * speed_scale
            traj.end_vel = traj.end_vel * speed_scale
            
            traj.control_points = np.array(segment_data['control_points'])
            
            traj_segments.append(traj)
        
        rospy.loginfo(f"Loaded {len(traj_segments)} trajectory segments from {filepath}")
        return traj_segments
    
    def compute_full_desired_commands(self, traj_segments, cmd_hz=10.0):
        """
        Compute full desired control commands from all trajectory segments.
        
        Args:
            traj_segments: List of Traj objects
            cmd_hz: Command frequency in Hz
            
        Returns:
            List[dict]: List of desired states for the complete trajectory
        """
        if not traj_segments:
            rospy.logwarn("No trajectory segments provided")
            return []
        
        # Import here to avoid circular imports
        from clamped_bspline import ClampedBSpline
        
        all_des_states = []
        
        for i, traj in enumerate(traj_segments):
            rospy.loginfo(f"Computing commands for segment {i+1}/{len(traj_segments)}")
            
            # Create B-spline for this segment
            bspline = ClampedBSpline(p=3)  # Cubic B-spline
            
            # Set boundary conditions
            bspline.set_boundary(
                start_pos=traj.start_pos,
                end_pos=traj.end_pos,
                start_vel=traj.start_vel,
                end_vel=traj.end_vel,
                T=traj.time
            )
            
            # Build spline with control points
            bspline.build_spline(traj.control_points)
            
            # Generate position and velocity arrays
            des_pos_array = bspline.get_pos_array(cmd_hz)
            des_vel_array = bspline.get_vel_array(cmd_hz)
            
            # Get desired states from robot model
            segment_des_states = self.robot.get_des_states(des_pos_array, des_vel_array)
            
            if i == 0:
                # First segment: add all points
                all_des_states.extend(segment_des_states)
            else:
                # Subsequent segments: skip first point to avoid duplication
                all_des_states.extend(segment_des_states[1:])
        
        rospy.loginfo(f"Generated {len(all_des_states)} total command points")
        return all_des_states
    
    def list_saved_trajectories(self):
        """
        List all saved trajectory files in the save directory.
        
        Returns:
            List[str]: List of trajectory filenames
        """
        try:
            files = [f for f in os.listdir(self.save_dir) if f.endswith('.json')]
            files.sort()  # Sort by filename (includes timestamp)
            return files
        except Exception as e:
            rospy.logerr(f"Failed to list trajectories: {e}")
            return []
    
    def get_trajectory_info(self, filepath, speed_scale=1.0):
        """
        Get basic information about a trajectory file without fully loading it.
        
        Args:
            filepath: Path to the trajectory file
            speed_scale: Speed scaling factor for time calculation
            
        Returns:
            dict: Basic trajectory information
        """
        try:
            with open(filepath, 'r') as f:
                trajectory_data = json.load(f)
            
            metadata = trajectory_data.get('metadata', {})
            original_total_time = sum(segment['time'] for segment in trajectory_data.get('segments', []))
            scaled_total_time = original_total_time / speed_scale
            
            return {
                'timestamp': metadata.get('timestamp', 'unknown'),
                'num_segments': metadata.get('num_segments', 0),
                'config_dim': metadata.get('config_dim', 0),
                'saved_time': metadata.get('saved_time', 'unknown'),
                'total_time': scaled_total_time,
                'original_time': original_total_time,
                'file_size': os.path.getsize(filepath)
            }
        except Exception as e:
            rospy.logerr(f"Failed to get trajectory info from {filepath}: {e}")
            return None

    def load_and_compute_commands(self, filepath, cmd_hz=10.0, speed_scale=1.0):
        """
        Load trajectory from file and compute full desired control commands.
        
        Args:
            filepath: Path to the trajectory file
            cmd_hz: Command frequency in Hz
            speed_scale: Speed scaling factor (0.5 = half speed, 2.0 = double speed)
            
        Returns:
            List[dict]: Desired states for complete trajectory
        """
        traj_segments = self.load_trajectory_segments(filepath, speed_scale)
        if traj_segments is None:
            return None
        return self.compute_full_desired_commands(traj_segments, cmd_hz)