#!/usr/bin/env python3

import copy
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PACKAGE_SCRIPT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if PACKAGE_SCRIPT_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_SCRIPT_DIR)

from traj_planner.nonholonomic_trajectory import (
    NonholonomicQuinticTrajectory,
    matrix_to_quaternion,
    quaternion_to_rotation_matrix,
    rotation_about_x,
    rotation_matrix_from_tangent,
)
from traj_planner.waypoint_planner_common import BaseWaypointConditionedPlannerNode, PlanningArtifacts


class NonholonomicWaypointConditionedPlannerNode(BaseWaypointConditionedPlannerNode):
    NODE_NAME = "nonholo_wpt_cond_planner"
    READY_LOG_MESSAGE = "Nonholonomic waypoint-conditioned planner ready: publish_rate_hz=%.1f total_trajectory_time=%.2f"
    TANGENT_EPSILON = 1e-6
    AXIS_EPSILON = 1e-3
    FEASIBILITY_EPSILON = 1e-6
    ROTATION_TOLERANCE = 1e-5
    cache_waypoint_before_signature = True

    def build_plan(self, start_root_pose_snapshot, terminal_root_pose_snapshot, ordered_waypoints, frame_id):
        trajectory = self.build_trajectory(start_root_pose_snapshot, terminal_root_pose_snapshot, ordered_waypoints)
        sample_times = self.build_sample_times()
        sampled_positions, sampled_poses = self.sample_trajectory(trajectory, sample_times, frame_id)
        return PlanningArtifacts(
            samples=sampled_poses,
            trajectory_positions=sampled_positions,
            control_positions=trajectory.knot_positions,
        )

    def sample_to_pose(self, sample):
        return copy.deepcopy(sample)

    def on_invalid_pose_signature(self, source_name, waypoint_index, exc):
        if source_name == "root/tail_pose":
            rospy.logerr("Received invalid root/tail_pose orientation: %s", exc)
        else:
            rospy.logerr("Received invalid orientation on %s: %s", source_name, exc)
        return None

    def on_planning_failure(self, log_prefix, exc):
        rospy.logerr("%s: %s", log_prefix, exc)
        self.invalidate_current_trajectory()

    def log_planning_success(self, waypoint_count, sample_count, reason):
        rospy.loginfo(
            "Planned %d-waypoint nonholonomic trajectory from current root to terminal pose (%d samples, %.2f s total) because %s.",
            waypoint_count,
            sample_count,
            self.total_trajectory_time,
            reason,
        )

    def build_trajectory(self, start_root_pose_snapshot, terminal_root_pose_snapshot, ordered_waypoints):
        knot_poses = [start_root_pose_snapshot]
        knot_poses.extend(ordered_waypoints)
        knot_poses.append(terminal_root_pose_snapshot)

        knot_positions = []
        knot_rotations = []
        for pose_stamped in knot_poses:
            position, rotation = self.pose_to_position_and_rotation(pose_stamped)
            knot_positions.append(position)
            knot_rotations.append(rotation)

        return NonholonomicQuinticTrajectory(
            np.vstack(knot_positions),
            np.stack(knot_rotations, axis=0),
            self.total_trajectory_time,
            self.TANGENT_EPSILON,
            self.AXIS_EPSILON,
            self.FEASIBILITY_EPSILON,
            self.ROTATION_TOLERANCE,
        )

    def sample_trajectory(self, trajectory, sample_times, frame_id):
        sampled_positions = []
        sampled_poses = []

        for sample_time in sample_times:
            position, velocity, twist_angle = trajectory.sample(sample_time)
            base_rotation = rotation_matrix_from_tangent(velocity, self.TANGENT_EPSILON, self.AXIS_EPSILON)
            full_rotation = np.dot(base_rotation, rotation_about_x(twist_angle))
            sampled_positions.append(position)
            sampled_poses.append(self.position_and_rotation_to_pose_stamped(position, full_rotation, frame_id))

        return np.vstack(sampled_positions), sampled_poses

    @staticmethod
    def position_and_rotation_to_pose_stamped(position, rotation_matrix, frame_id):
        quaternion = matrix_to_quaternion(rotation_matrix)

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        return pose_msg

    @staticmethod
    def pose_to_position_and_rotation(pose_stamped):
        quaternion = [
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w,
        ]
        return (
            np.array(
                [
                    pose_stamped.pose.position.x,
                    pose_stamped.pose.position.y,
                    pose_stamped.pose.position.z,
                ],
                dtype=float,
            ),
            quaternion_to_rotation_matrix(quaternion),
        )


if __name__ == "__main__":
    try:
        NonholonomicWaypointConditionedPlannerNode().run()
    except Exception as exc:
        rospy.logerr("Failed to start %s: %s", NonholonomicWaypointConditionedPlannerNode.NODE_NAME, exc)
        raise
