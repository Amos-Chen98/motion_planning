#!/usr/bin/env python3

import os
import sys

import numpy as np
import rospy
import tf.transformations as tf_trans
from geometry_msgs.msg import PoseStamped

SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PACKAGE_SCRIPT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if PACKAGE_SCRIPT_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_SCRIPT_DIR)

from traj_planner.se3_trajectory_planner import SE3TrajectoryPlanner
from traj_planner.waypoint_planner_common import BaseWaypointConditionedPlannerNode, PlanningArtifacts


class WaypointConditionedPlannerNode(BaseWaypointConditionedPlannerNode):
    NODE_NAME = "waypoint_conditioned_planner_node"
    READY_LOG_MESSAGE = "Waypoint-conditioned planner ready: publish_rate_hz=%.1f total_trajectory_time=%.2f"

    def __init__(self):
        self.traj_planner = SE3TrajectoryPlanner()
        super().__init__()

    def build_plan(self, start_root_pose_snapshot, terminal_root_pose_snapshot, ordered_waypoints, _frame_id):
        state_sequence = self.build_state_sequence(
            start_root_pose_snapshot,
            terminal_root_pose_snapshot,
            ordered_waypoints,
        )
        segment_times = self.compute_segment_times(state_sequence[:, :3])
        sample_times = self.build_sample_times()

        self.traj_planner.plan(state_sequence, segment_times)
        sampled_states = self.traj_planner.sample_positions(sample_times)
        return PlanningArtifacts(
            samples=sampled_states,
            trajectory_positions=sampled_states[:, :3],
            control_positions=state_sequence[:, :3],
        )

    def sample_to_pose(self, sample):
        return self.state_to_pose_stamped(np.asarray(sample, dtype=float).copy(), self.trajectory_frame_id)

    def log_planning_success(self, waypoint_count, sample_count, reason):
        rospy.loginfo(
            "Planned %d-waypoint trajectory from current root to startup root (%d samples, %.2f s total) because %s.",
            waypoint_count,
            sample_count,
            self.total_trajectory_time,
            reason,
        )

    def build_state_sequence(self, start_root_pose_snapshot, terminal_root_pose_snapshot, ordered_waypoints):
        states = [self.pose_to_state(start_root_pose_snapshot)]
        states.extend(self.pose_to_state(waypoint) for waypoint in ordered_waypoints)
        states.append(self.pose_to_state(terminal_root_pose_snapshot))

        state_array = np.vstack(states)
        state_array[:, 3] = np.unwrap(state_array[:, 3])
        state_array[:, 4] = np.unwrap(state_array[:, 4])
        state_array[:, 5] = np.unwrap(state_array[:, 5])
        return state_array

    def compute_segment_times(self, positions):
        segment_distances = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        if segment_distances.size == 0:
            raise ValueError("At least one trajectory segment is required.")

        if np.all(segment_distances < self.POSE_EPSILON):
            weights = np.ones_like(segment_distances)
        else:
            weights = np.maximum(segment_distances, 1e-3)

        return self.total_trajectory_time * weights / np.sum(weights)

    @staticmethod
    def pose_to_state(pose_stamped):
        quaternion = [
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w,
        ]
        roll, pitch, yaw = tf_trans.euler_from_quaternion(quaternion)
        return np.array(
            [
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                pose_stamped.pose.position.z,
                roll,
                pitch,
                yaw,
            ],
            dtype=float,
        )

    @staticmethod
    def state_to_pose_stamped(state, frame_id):
        quaternion = tf_trans.quaternion_from_euler(state[3], state[4], state[5])

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = state[0]
        pose_msg.pose.position.y = state[1]
        pose_msg.pose.position.z = state[2]
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        return pose_msg


if __name__ == "__main__":
    try:
        WaypointConditionedPlannerNode().run()
    except Exception as exc:
        rospy.logerr("Failed to start waypoint_conditioned_planner_node: %s", exc)
