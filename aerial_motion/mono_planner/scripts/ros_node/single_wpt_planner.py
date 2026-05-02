#!/usr/bin/env python3

import copy
import math
import os
import select
import sys
import termios
import threading
import tty
from dataclasses import dataclass

import numpy as np
import rospy
import tf.transformations as tf_trans
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PACKAGE_SCRIPT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if PACKAGE_SCRIPT_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_SCRIPT_DIR)

from traj_planner.nonholonomic_trajectory import matrix_to_quaternion, rotation_matrix_from_tangent
from traj_planner.waypoint_planner_common import canonicalize_quaternion, position_to_point


@dataclass
class CircularTrajectoryModel:
    center: np.ndarray
    radius: float
    basis_u: np.ndarray
    basis_v: np.ndarray
    start_phase: float
    total_time: float
    start_position: np.ndarray
    waypoint_position: np.ndarray
    waypoint_yaw: float
    waypoint_time: float

    @property
    def angular_rate(self):
        return 2.0 * math.pi / self.total_time


class SingleWaypointPlanner:
    NODE_NAME = "single_wpt_planner"
    TRAJECTORY_MARKER_TOPIC = "mono_planner/traj_marker"
    KEYBOARD_POLL_PERIOD = 0.05
    TANGENT_EPSILON = 1e-6
    AXIS_EPSILON = 1e-3
    ARROW_COLORS = (
        (1.00, 0.55, 0.10),
        (0.10, 0.35, 1.00),
    )

    def __init__(self):
        rospy.init_node(self.NODE_NAME, anonymous=False)

        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 40.0))
        self.total_trajectory_time = float(rospy.get_param("~total_trajectory_time", 20.0))
        self.robot_frame_type = str(rospy.get_param("~robot_frame_type", "LINK")).upper()
        if self.publish_rate_hz <= 0.0:
            raise ValueError("~publish_rate_hz must be greater than 0.")
        if self.total_trajectory_time <= 0.0:
            raise ValueError("~total_trajectory_time must be greater than 0.")
        if self.robot_frame_type not in {"FLU", "LINK"}:
            raise ValueError("~robot_frame_type must be either 'FLU' or 'LINK'.")

        self.lock = threading.RLock()

        self.latest_root_pose = None
        self.latest_waypoint_pose = None
        self.snapshot_root_pose = None
        self.snapshot_waypoint_pose = None
        self.plan_ready = False
        self.execution_started = False
        self.trajectory_frame_id = "world"
        self.trajectory_samples = []
        self.marker_points = np.zeros((0, 3), dtype=float)
        self.reference_marker_poses = []
        self.publish_index = 0
        self.publish_timer = None

        self.stdin_fd = None
        self.stdin_settings = None
        self.keyboard_enabled = self.configure_keyboard_input()
        self.keyboard_timer = None

        self.target_pose_pub = rospy.Publisher("root/target_pose", PoseStamped, queue_size=1)
        self.trajectory_marker_pub = rospy.Publisher(
            self.TRAJECTORY_MARKER_TOPIC,
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.root_pose_sub = rospy.Subscriber("root/tail_pose", PoseStamped, self.root_pose_cb, queue_size=1)
        self.waypoint_pose_sub = rospy.Subscriber("/waypoint/pose_0", PoseStamped, self.waypoint_pose_cb, queue_size=1)

        if self.keyboard_enabled:
            self.keyboard_timer = rospy.Timer(
                rospy.Duration(self.KEYBOARD_POLL_PERIOD),
                self.keyboard_timer_cb,
            )

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo(
            "%s ready: publish_rate_hz=%.1f total_trajectory_time=%.2f robot_frame_type=%s",
            self.NODE_NAME,
            self.publish_rate_hz,
            self.total_trajectory_time,
            self.robot_frame_type,
        )
        rospy.loginfo(
            "This planner uses only root/tail_pose position and waypoint/pose_0 position+yaw. "
            "~robot_frame_type=%s is retained for interface compatibility and does not affect the circular trajectory geometry.",
            self.robot_frame_type,
        )
        rospy.loginfo("Waiting for root/tail_pose and waypoint/pose_0 to build the circular roundtrip trajectory.")

    def configure_keyboard_input(self):
        if not sys.stdin.isatty():
            rospy.logwarn(
                "No interactive terminal detected; the node will publish visualization markers only and will not send root/target_pose."
            )
            return False

        try:
            self.stdin_fd = sys.stdin.fileno()
            self.stdin_settings = termios.tcgetattr(self.stdin_fd)
            tty.setcbreak(self.stdin_fd)
        except (termios.error, ValueError, OSError) as exc:
            rospy.logwarn(
                "Failed to enable keyboard confirmation input: %s. The node will only publish visualization markers.",
                exc,
            )
            self.stdin_fd = None
            self.stdin_settings = None
            return False

        return True

    def restore_terminal_settings(self):
        if self.stdin_fd is None or self.stdin_settings is None:
            return

        termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.stdin_settings)
        self.stdin_fd = None
        self.stdin_settings = None

    def on_shutdown(self):
        self.stop_publish_timer()
        if self.keyboard_timer is not None:
            self.keyboard_timer.shutdown()
            self.keyboard_timer = None
        self.restore_terminal_settings()

    def root_pose_cb(self, msg):
        with self.lock:
            if self.plan_ready:
                return
            self.latest_root_pose = copy.deepcopy(msg)

        self.try_plan_once()

    def waypoint_pose_cb(self, msg):
        with self.lock:
            if self.plan_ready:
                return
            self.latest_waypoint_pose = copy.deepcopy(msg)

        self.try_plan_once()

    def try_plan_once(self):
        with self.lock:
            if self.plan_ready:
                return
            if self.latest_root_pose is None or self.latest_waypoint_pose is None:
                return

            root_pose_snapshot = copy.deepcopy(self.latest_root_pose)
            waypoint_pose_snapshot = copy.deepcopy(self.latest_waypoint_pose)
            self.snapshot_root_pose = root_pose_snapshot
            self.snapshot_waypoint_pose = waypoint_pose_snapshot

        frame_id = root_pose_snapshot.header.frame_id or waypoint_pose_snapshot.header.frame_id or "world"
        waypoint_frame_id = waypoint_pose_snapshot.header.frame_id or ""
        root_frame_id = root_pose_snapshot.header.frame_id or ""
        if waypoint_frame_id and waypoint_frame_id != root_frame_id:
            rospy.logwarn(
                "Frame mismatch detected: root/tail_pose uses '%s' but waypoint/pose_0 uses '%s'; no TF transform will be applied.",
                root_frame_id or "<empty>",
                waypoint_frame_id,
            )

        try:
            sampled_poses, marker_points, reference_marker_poses = self.build_roundtrip_trajectory(
                root_pose_snapshot,
                waypoint_pose_snapshot,
                frame_id,
            )
        except np.linalg.LinAlgError as exc:
            self.reset_failed_snapshot()
            rospy.logerr("Circular roundtrip solve failed: %s", exc)
            return
        except ValueError as exc:
            self.reset_failed_snapshot()
            rospy.logerr("Circular roundtrip is infeasible: %s", exc)
            return

        with self.lock:
            self.trajectory_samples = sampled_poses
            self.marker_points = marker_points
            self.reference_marker_poses = reference_marker_poses
            self.trajectory_frame_id = frame_id
            self.publish_index = 0
            self.plan_ready = True

        self.publish_trajectory_markers()
        rospy.loginfo(
            "Planned circular roundtrip trajectory root -> waypoint/pose_0 -> root (%d samples, %.2f s total).",
            len(sampled_poses),
            self.total_trajectory_time,
        )
        if self.keyboard_enabled:
            rospy.loginfo("Trajectory markers published. Press L in this terminal to start publishing root/target_pose.")
        else:
            rospy.logwarn("Trajectory markers published, but command publishing is disabled because no interactive terminal is available.")

    def reset_failed_snapshot(self):
        with self.lock:
            self.snapshot_root_pose = None
            self.snapshot_waypoint_pose = None

    def build_roundtrip_trajectory(self, root_pose, waypoint_pose, frame_id):
        circle_model = self.build_trajectory(root_pose, waypoint_pose)
        sample_times = self.build_sample_times(circle_model)
        sampled_positions, sampled_poses = self.sample_trajectory(circle_model, sample_times, frame_id)
        marker_points = np.vstack((circle_model.start_position, circle_model.waypoint_position, circle_model.center))
        reference_marker_poses = self.build_reference_marker_poses(circle_model, sampled_poses[0], frame_id)
        return sampled_poses, marker_points, reference_marker_poses

    def build_trajectory(self, root_pose, waypoint_pose):
        root_position = self.pose_to_position(root_pose)
        waypoint_position, waypoint_yaw = self.pose_to_position_and_yaw(waypoint_pose)
        return self.build_circular_trajectory_model(
            root_position,
            waypoint_position,
            waypoint_yaw,
            self.total_trajectory_time,
        )

    @classmethod
    def build_circular_trajectory_model(cls, root_position, waypoint_position, waypoint_yaw, total_time):
        root_position = np.asarray(root_position, dtype=float)
        waypoint_position = np.asarray(waypoint_position, dtype=float)
        tangent_at_waypoint = np.array([math.cos(waypoint_yaw), math.sin(waypoint_yaw), 0.0], dtype=float)

        root_to_waypoint = root_position - waypoint_position
        root_to_waypoint_distance = np.linalg.norm(root_to_waypoint)
        if root_to_waypoint_distance < cls.TANGENT_EPSILON:
            raise ValueError("root and waypoint positions are coincident")

        root_offset_perpendicular_to_tangent = root_to_waypoint - np.dot(root_to_waypoint, tangent_at_waypoint) * tangent_at_waypoint
        perpendicular_distance = np.linalg.norm(root_offset_perpendicular_to_tangent)
        if perpendicular_distance < cls.TANGENT_EPSILON:
            raise ValueError("root lies on the waypoint tangent line; no finite circle exists")

        radial_direction_at_waypoint = root_offset_perpendicular_to_tangent / perpendicular_distance
        circle_radius = (root_to_waypoint_distance**2) / (2.0 * perpendicular_distance)
        circle_center = waypoint_position + circle_radius * radial_direction_at_waypoint

        basis_u = cls.normalize_vector(
            waypoint_position - circle_center,
            "circle radial basis is degenerate",
        )
        basis_v = cls.normalize_vector(
            tangent_at_waypoint,
            "waypoint yaw produced a degenerate tangent direction",
        )

        start_offset = root_position - circle_center
        start_phase = math.atan2(np.dot(start_offset, basis_v), np.dot(start_offset, basis_u))
        waypoint_phase_offset = float(np.mod(-start_phase, 2.0 * math.pi))
        waypoint_time = total_time * waypoint_phase_offset / (2.0 * math.pi)

        return CircularTrajectoryModel(
            center=circle_center,
            radius=circle_radius,
            basis_u=basis_u,
            basis_v=basis_v,
            start_phase=start_phase,
            total_time=float(total_time),
            start_position=root_position,
            waypoint_position=waypoint_position,
            waypoint_yaw=float(waypoint_yaw),
            waypoint_time=float(waypoint_time),
        )

    def build_sample_times(self, circle_model):
        publish_dt = 1.0 / self.publish_rate_hz
        step_count = max(1, int(math.ceil(circle_model.total_time / publish_dt)))
        uniform_sample_times = np.array(
            [min(step_index * publish_dt, circle_model.total_time) for step_index in range(step_count + 1)],
            dtype=float,
        )
        return np.unique(np.concatenate((uniform_sample_times, np.array([circle_model.waypoint_time], dtype=float))))

    def sample_trajectory(self, circle_model, sample_times, frame_id):
        sampled_positions = []
        sampled_poses = []

        for sample_time in sample_times:
            position, velocity = self.sample_circle(circle_model, sample_time)
            full_rotation = rotation_matrix_from_tangent(
                velocity,
                self.TANGENT_EPSILON,
                self.AXIS_EPSILON,
            )
            sampled_positions.append(position)
            sampled_poses.append(self.position_and_rotation_to_pose_stamped(position, full_rotation, frame_id))

        return np.vstack(sampled_positions), sampled_poses

    @classmethod
    def sample_circle(cls, circle_model, sample_time):
        sample_time = min(max(float(sample_time), 0.0), circle_model.total_time)
        phase = circle_model.start_phase + circle_model.angular_rate * sample_time
        cosine = math.cos(phase)
        sine = math.sin(phase)
        position = circle_model.center + circle_model.radius * (cosine * circle_model.basis_u + sine * circle_model.basis_v)
        velocity = (
            circle_model.radius
            * circle_model.angular_rate
            * (-sine * circle_model.basis_u + cosine * circle_model.basis_v)
        )
        return position, velocity

    def build_reference_marker_poses(self, circle_model, start_sample_pose, frame_id):
        waypoint_yaw_rotation = self.yaw_to_rotation_matrix(circle_model.waypoint_yaw)
        waypoint_yaw_pose = self.position_and_rotation_to_pose_stamped(
            circle_model.waypoint_position,
            waypoint_yaw_rotation,
            frame_id,
        )
        start_target_pose = copy.deepcopy(start_sample_pose)
        return [waypoint_yaw_pose, start_target_pose]

    @staticmethod
    def pose_to_position(pose_stamped):
        return np.array(
            [
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                pose_stamped.pose.position.z,
            ],
            dtype=float,
        )

    @staticmethod
    def pose_to_position_and_yaw(pose_stamped):
        quaternion = canonicalize_quaternion(
            [
                pose_stamped.pose.orientation.x,
                pose_stamped.pose.orientation.y,
                pose_stamped.pose.orientation.z,
                pose_stamped.pose.orientation.w,
            ]
        )
        _, _, yaw = tf_trans.euler_from_quaternion(quaternion)
        return (
            np.array(
                [
                    pose_stamped.pose.position.x,
                    pose_stamped.pose.position.y,
                    pose_stamped.pose.position.z,
                ],
                dtype=float,
            ),
            yaw,
        )

    @staticmethod
    def yaw_to_rotation_matrix(yaw):
        return tf_trans.euler_matrix(0.0, 0.0, yaw)[:3, :3]

    @classmethod
    def normalize_vector(cls, vector, description):
        vector = np.asarray(vector, dtype=float)
        vector_norm = np.linalg.norm(vector)
        if vector_norm < cls.TANGENT_EPSILON:
            raise ValueError(description)
        return vector / vector_norm

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

    def publish_trajectory_markers(self):
        with self.lock:
            frame_id = self.trajectory_frame_id
            trajectory_samples = [copy.deepcopy(sample) for sample in self.trajectory_samples]
            marker_points = self.marker_points.copy()
            reference_marker_poses = [copy.deepcopy(pose) for pose in self.reference_marker_poses]

        stamp = rospy.Time.now()
        delete_all_marker = Marker()
        delete_all_marker.action = Marker.DELETEALL
        self.trajectory_marker_pub.publish(MarkerArray(markers=[delete_all_marker]))

        marker_array = MarkerArray()

        line_marker = Marker()
        line_marker.header.frame_id = frame_id
        line_marker.header.stamp = stamp
        line_marker.ns = "trajectory_line"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.05
        line_marker.color.r = 0.10
        line_marker.color.g = 0.85
        line_marker.color.b = 0.25
        line_marker.color.a = 1.0
        line_marker.points = [
            position_to_point(
                (
                    pose_msg.pose.position.x,
                    pose_msg.pose.position.y,
                    pose_msg.pose.position.z,
                )
            )
            for pose_msg in trajectory_samples
        ]
        marker_array.markers.append(line_marker)

        point_marker = Marker()
        point_marker.header.frame_id = frame_id
        point_marker.header.stamp = stamp
        point_marker.ns = "trajectory_reference_points"
        point_marker.id = 1
        point_marker.type = Marker.SPHERE_LIST
        point_marker.action = Marker.ADD
        point_marker.pose.orientation.w = 1.0
        point_marker.scale.x = 0.10
        point_marker.scale.y = 0.10
        point_marker.scale.z = 0.10
        point_marker.color.r = 0.10
        point_marker.color.g = 0.35
        point_marker.color.b = 1.0
        point_marker.color.a = 1.0
        point_marker.points = [position_to_point(position) for position in marker_points]
        marker_array.markers.append(point_marker)

        for index, (pose_msg, color) in enumerate(zip(reference_marker_poses, self.ARROW_COLORS)):
            arrow_marker = Marker()
            arrow_marker.header.frame_id = frame_id
            arrow_marker.header.stamp = stamp
            arrow_marker.ns = "trajectory_reference_orientations"
            arrow_marker.id = 2 + index
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose.position.x = pose_msg.pose.position.x
            arrow_marker.pose.position.y = pose_msg.pose.position.y
            arrow_marker.pose.position.z = pose_msg.pose.position.z
            arrow_marker.pose.orientation.x = pose_msg.pose.orientation.x
            arrow_marker.pose.orientation.y = pose_msg.pose.orientation.y
            arrow_marker.pose.orientation.z = pose_msg.pose.orientation.z
            arrow_marker.pose.orientation.w = pose_msg.pose.orientation.w
            arrow_marker.scale.x = 0.35
            arrow_marker.scale.y = 0.07
            arrow_marker.scale.z = 0.07
            arrow_marker.color.r = color[0]
            arrow_marker.color.g = color[1]
            arrow_marker.color.b = color[2]
            arrow_marker.color.a = 1.0
            marker_array.markers.append(arrow_marker)

        self.trajectory_marker_pub.publish(marker_array)

    def keyboard_timer_cb(self, _event):
        if not self.keyboard_enabled:
            return

        with self.lock:
            if not self.plan_ready or self.execution_started:
                return

        key = self.read_key()
        if key is None:
            return
        if key.lower() != "l":
            rospy.loginfo("Ignoring key '%s'; press L to start trajectory publishing.", key)
            return

        rospy.loginfo("Received L confirmation; starting root/target_pose publishing.")
        self.start_trajectory_publishing()

    def read_key(self):
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None
        return sys.stdin.read(1)

    def start_trajectory_publishing(self):
        self.stop_publish_timer()

        with self.lock:
            if not self.plan_ready or len(self.trajectory_samples) == 0:
                return
            if self.execution_started:
                return
            self.execution_started = True
            self.publish_index = 0

        self.publish_current_sample()

        with self.lock:
            if self.publish_index >= len(self.trajectory_samples):
                return
            self.publish_timer = rospy.Timer(
                rospy.Duration(1.0 / self.publish_rate_hz),
                self.publish_timer_cb,
            )

    def publish_timer_cb(self, _event):
        if self.publish_current_sample():
            self.stop_publish_timer()
            rospy.loginfo("Completed circular root/target_pose publishing.")

    def publish_current_sample(self):
        with self.lock:
            if self.publish_index >= len(self.trajectory_samples):
                return True

            pose_msg = copy.deepcopy(self.trajectory_samples[self.publish_index])
            self.publish_index += 1
            reached_end = self.publish_index >= len(self.trajectory_samples)

        pose_msg.header.stamp = rospy.Time.now()
        self.target_pose_pub.publish(pose_msg)
        return reached_end

    def stop_publish_timer(self):
        timer_to_stop = None
        with self.lock:
            if self.publish_timer is not None:
                timer_to_stop = self.publish_timer
                self.publish_timer = None

        if timer_to_stop is not None:
            timer_to_stop.shutdown()

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        SingleWaypointPlanner().run()
    except Exception as exc:
        rospy.logerr("Failed to start %s: %s", SingleWaypointPlanner.NODE_NAME, exc)
        raise
