import copy
import math
import re
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np
import rospy
import tf.transformations as tf_trans
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class PlanningArtifacts:
    samples: object
    trajectory_positions: np.ndarray
    control_positions: np.ndarray


LINK_TO_FLU_LOCAL_Z_HALF_TURN = np.array([0.0, 0.0, 1.0, 0.0], dtype=float)


def normalize_topic_name(topic_name):
    if not topic_name.startswith("/"):
        return "/" + topic_name
    return topic_name


def canonicalize_quaternion(quaternion):
    normalized = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(normalized)
    if norm == 0.0:
        raise ValueError("Received zero-norm quaternion.")

    normalized = normalized / norm
    for component in normalized:
        if abs(component) > 1e-12:
            if component < 0.0:
                normalized = -normalized
            break
    return normalized


def pose_signature(pose_stamped):
    orientation = canonicalize_quaternion(
        [
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w,
        ]
    )
    return np.array(
        [
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z,
            orientation[0],
            orientation[1],
            orientation[2],
            orientation[3],
        ],
        dtype=float,
    )


def position_to_point(position):
    point = Point()
    point.x = position[0]
    point.y = position[1]
    point.z = position[2]
    return point


class BaseWaypointConditionedPlannerNode(ABC):
    WAYPOINT_TOPIC_PATTERN = re.compile(r"^/?waypoint/pose_(\d+)$")
    POSE_EPSILON = 0.05
    DISCOVERY_PERIOD = 1.0
    TRAJECTORY_MARKER_TOPIC = "/waypoint_conditioned_trajectory_marker"
    NODE_NAME = None
    READY_LOG_MESSAGE = None
    cache_waypoint_before_signature = False

    def __init__(self):
        if self.NODE_NAME is None:
            raise ValueError("NODE_NAME must be defined by the concrete planner node.")

        rospy.init_node(self.NODE_NAME, anonymous=False)

        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 60.0))
        self.total_trajectory_time = float(rospy.get_param("~total_trajectory_time", 20.0))
        configured_robot_frame_type = str(rospy.get_param("~robot_frame_type", "FLU"))
        self.robot_frame_type = configured_robot_frame_type.upper()
        if self.publish_rate_hz <= 0.0:
            raise ValueError("~publish_rate_hz must be greater than 0.")
        if self.total_trajectory_time <= 0.0:
            raise ValueError("~total_trajectory_time must be greater than 0.")
        if self.robot_frame_type not in {"FLU", "LINK"}:
            raise ValueError("~robot_frame_type must be either 'FLU' or 'LINK'.")

        self.lock = threading.RLock()

        self.startup_root_pose = None
        self.latest_root_pose = None
        self.waypoint_subscribers = {}
        self.waypoint_topic_indices = {}
        self.latest_waypoints = {}
        self.latest_waypoint_signatures = {}

        self.trajectory_samples = []
        self.trajectory_frame_id = "world"
        self.publish_index = 0
        self.publish_timer = None

        self.target_pose_pub = rospy.Publisher("root/target_pose", PoseStamped, queue_size=1)
        self.trajectory_marker_pub = rospy.Publisher(
            self.TRAJECTORY_MARKER_TOPIC,
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.root_pose_sub = rospy.Subscriber("root/tail_pose", PoseStamped, self.root_pose_cb, queue_size=1)
        self.discovery_timer = rospy.Timer(rospy.Duration(self.DISCOVERY_PERIOD), self.discover_waypoint_topics)

        rospy.loginfo(
            self.READY_LOG_MESSAGE or "{} ready: publish_rate_hz=%.1f total_trajectory_time=%.2f".format(self.NODE_NAME),
            self.publish_rate_hz,
            self.total_trajectory_time,
        )
        if self.robot_frame_type == "LINK":
            rospy.loginfo(
                "Interpreting root/tail_pose as LINK and converting to FLU with a local-Z 180-degree rotation."
            )
        else:
            rospy.loginfo("Interpreting root/tail_pose as FLU without conversion.")

    def convert_root_pose_for_planning(self, pose_stamped):
        converted_pose = copy.deepcopy(pose_stamped)
        if self.robot_frame_type != "LINK":
            return converted_pose

        input_quaternion = np.array(
            [
                pose_stamped.pose.orientation.x,
                pose_stamped.pose.orientation.y,
                pose_stamped.pose.orientation.z,
                pose_stamped.pose.orientation.w,
            ],
            dtype=float,
        )
        rotated_quaternion = tf_trans.quaternion_multiply(input_quaternion, LINK_TO_FLU_LOCAL_Z_HALF_TURN)
        norm = np.linalg.norm(rotated_quaternion)
        if norm > 0.0:
            rotated_quaternion = rotated_quaternion / norm

        converted_pose.pose.orientation.x = rotated_quaternion[0]
        converted_pose.pose.orientation.y = rotated_quaternion[1]
        converted_pose.pose.orientation.z = rotated_quaternion[2]
        converted_pose.pose.orientation.w = rotated_quaternion[3]
        return converted_pose

    def root_pose_cb(self, msg):
        root_pose = self.convert_root_pose_for_planning(msg)
        bootstrap_plan = False
        with self.lock:
            if self.startup_root_pose is None:
                self.startup_root_pose = copy.deepcopy(root_pose)
                bootstrap_plan = True
            self.latest_root_pose = root_pose

        if bootstrap_plan:
            rospy.loginfo("Captured startup root/tail_pose; future plans will terminate at this pose.")
            self.try_plan("startup root/tail_pose received")

    def waypoint_pose_cb(self, msg, topic_name):
        topic_name = normalize_topic_name(topic_name)

        with self.lock:
            waypoint_index = self.waypoint_topic_indices.get(topic_name)
            if waypoint_index is None:
                return

            previous_signature = self.latest_waypoint_signatures.get(waypoint_index)
            previous_frame = ""
            if waypoint_index in self.latest_waypoints:
                previous_frame = self.latest_waypoints[waypoint_index].header.frame_id

            if self.cache_waypoint_before_signature:
                self.latest_waypoints[waypoint_index] = copy.deepcopy(msg)
            new_signature = self.compute_pose_signature(
                msg,
                "waypoint/pose_{}".format(waypoint_index),
                waypoint_index=waypoint_index,
            )
            if not self.cache_waypoint_before_signature:
                self.latest_waypoints[waypoint_index] = copy.deepcopy(msg)
            self.latest_waypoint_signatures[waypoint_index] = new_signature

            frame_changed = previous_frame != msg.header.frame_id
            pose_changed = (
                previous_signature is None
                or new_signature is None
                or not np.allclose(previous_signature, new_signature, atol=self.POSE_EPSILON, rtol=0.0)
            )
            if not pose_changed and not frame_changed:
                return

        self.invalidate_current_trajectory()
        self.try_plan(
            "waypoint/pose_{} {}".format(
                waypoint_index,
                "received" if previous_signature is None else "updated",
            )
        )

    def compute_pose_signature(self, pose_stamped, source_name, waypoint_index=None):
        try:
            return pose_signature(pose_stamped)
        except ValueError as exc:
            return self.on_invalid_pose_signature(source_name, waypoint_index, exc)

    def discover_waypoint_topics(self, _event):
        try:
            published_topics = rospy.get_published_topics()
        except rospy.ROSException as exc:
            rospy.logwarn_throttle(5.0, "Failed to query published topics: %s", exc)
            return

        discovered_topics = {}
        for topic_name, topic_type in published_topics:
            if topic_type != "geometry_msgs/PoseStamped":
                continue
            match = self.WAYPOINT_TOPIC_PATTERN.match(topic_name)
            if match is None:
                continue
            discovered_topics[normalize_topic_name(topic_name)] = int(match.group(1))

        topics_changed = False
        topics_to_remove = []
        topics_to_add = []

        with self.lock:
            current_topics = set(self.waypoint_subscribers.keys())
            discovered_topic_names = set(discovered_topics.keys())

            for topic_name in sorted(current_topics - discovered_topic_names):
                topics_to_remove.append(topic_name)
                topics_changed = True

            for topic_name in sorted(discovered_topic_names - current_topics):
                topics_to_add.append(topic_name)
                topics_changed = True

            for topic_name in topics_to_remove:
                subscriber = self.waypoint_subscribers.pop(topic_name)
                subscriber.unregister()
                waypoint_index = self.waypoint_topic_indices.pop(topic_name)
                self.latest_waypoints.pop(waypoint_index, None)
                self.latest_waypoint_signatures.pop(waypoint_index, None)

            for topic_name in topics_to_add:
                self.waypoint_topic_indices[topic_name] = discovered_topics[topic_name]
                self.waypoint_subscribers[topic_name] = rospy.Subscriber(
                    topic_name,
                    PoseStamped,
                    self.waypoint_pose_cb,
                    callback_args=topic_name,
                    queue_size=1,
                )

        if topics_changed:
            self.invalidate_current_trajectory()
            rospy.loginfo(
                "Discovered waypoint topics: %s",
                ", ".join(
                    "{}->{}".format(topic_name, discovered_topics[topic_name])
                    for topic_name in sorted(discovered_topics)
                ) or "none",
            )
            self.try_plan("waypoint topic set changed")

    def try_plan(self, reason):
        planning_inputs = self.collect_planning_inputs()
        if planning_inputs is None:
            return

        start_root_pose_snapshot, terminal_root_pose_snapshot, ordered_waypoint_entries = planning_inputs

        ordered_waypoints = [waypoint for _, waypoint in ordered_waypoint_entries]
        if not ordered_waypoints:
            rospy.loginfo_throttle(2.0, "Waiting for at least one waypoint/pose_x topic before planning.")
            return

        frame_id = (
            start_root_pose_snapshot.header.frame_id
            or terminal_root_pose_snapshot.header.frame_id
            or "world"
        )
        self.warn_if_frame_mismatch(frame_id, ordered_waypoint_entries)

        try:
            planning_artifacts = self.build_plan(
                start_root_pose_snapshot,
                terminal_root_pose_snapshot,
                ordered_waypoints,
                frame_id,
            )
        except np.linalg.LinAlgError as exc:
            self.on_planning_failure("Trajectory solve failed while replanning ({})".format(reason), exc)
            return
        except ValueError as exc:
            self.on_planning_failure("Trajectory setup failed while replanning ({})".format(reason), exc)
            return

        with self.lock:
            self.trajectory_samples = planning_artifacts.samples
            self.trajectory_frame_id = frame_id
            self.publish_index = 0

        self.log_planning_success(len(ordered_waypoints), len(planning_artifacts.samples), reason)
        self.publish_trajectory_markers(
            frame_id,
            planning_artifacts.trajectory_positions,
            planning_artifacts.control_positions,
        )
        self.start_trajectory_publishing()

    def on_invalid_pose_signature(self, _source_name, _waypoint_index, exc):
        raise exc

    def on_planning_failure(self, log_prefix, exc):
        rospy.logerr("%s: %s", log_prefix, exc)

    def log_planning_success(self, waypoint_count, sample_count, reason):
        rospy.loginfo(
            "Planned %d-waypoint trajectory from current root to startup root (%d samples, %.2f s total) because %s.",
            waypoint_count,
            sample_count,
            self.total_trajectory_time,
            reason,
        )

    def collect_planning_inputs(self):
        with self.lock:
            start_root_pose_snapshot = copy.deepcopy(self.latest_root_pose)
            terminal_root_pose_snapshot = copy.deepcopy(self.startup_root_pose)

            discovered_indices = sorted(self.waypoint_topic_indices.values())
            missing_indices = []
            if not discovered_indices:
                ordered_waypoints = []
            else:
                missing_indices = [index for index in discovered_indices if index not in self.latest_waypoints]
                if missing_indices:
                    ordered_waypoints = None
                else:
                    ordered_waypoints = [(index, copy.deepcopy(self.latest_waypoints[index])) for index in discovered_indices]

        if start_root_pose_snapshot is None or terminal_root_pose_snapshot is None:
            rospy.loginfo_throttle(2.0, "Waiting for root/tail_pose before planning.")
            return None

        current_root_frame = start_root_pose_snapshot.header.frame_id or ""
        startup_root_frame = terminal_root_pose_snapshot.header.frame_id or ""
        if current_root_frame != startup_root_frame:
            rospy.logerr_throttle(
                2.0,
                "Frame mismatch detected: current root/tail_pose uses '%s' but startup root/tail_pose uses '%s'; no TF transform will be applied.",
                current_root_frame or "<empty>",
                startup_root_frame or "<empty>",
            )
            return None

        if ordered_waypoints is None:
            rospy.loginfo_throttle(
                2.0,
                "Waiting for initial messages on waypoint indices: %s",
                ", ".join(str(index) for index in missing_indices),
            )
            return None

        return start_root_pose_snapshot, terminal_root_pose_snapshot, ordered_waypoints

    def build_sample_times(self):
        publish_dt = 1.0 / self.publish_rate_hz
        step_count = max(1, int(math.ceil(self.total_trajectory_time / publish_dt)))
        return np.array(
            [min(step_index * publish_dt, self.total_trajectory_time) for step_index in range(step_count + 1)],
            dtype=float,
        )

    def start_trajectory_publishing(self):
        self.stop_publish_timer()

        with self.lock:
            if len(self.trajectory_samples) == 0:
                return
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

    def publish_current_sample(self):
        with self.lock:
            if self.publish_index >= len(self.trajectory_samples):
                return True

            sample = self.trajectory_samples[self.publish_index]
            self.publish_index += 1
            reached_end = self.publish_index >= len(self.trajectory_samples)

        pose_msg = self.sample_to_pose(sample)
        pose_msg.header.stamp = rospy.Time.now()
        self.target_pose_pub.publish(pose_msg)
        return reached_end

    def invalidate_current_trajectory(self):
        self.stop_publish_timer()
        with self.lock:
            self.publish_index = 0
            self.trajectory_samples = []
        self.clear_trajectory_markers()

    def stop_publish_timer(self):
        timer_to_stop = None
        with self.lock:
            if self.publish_timer is not None:
                timer_to_stop = self.publish_timer
                self.publish_timer = None

            self.publish_index = 0

        if timer_to_stop is not None:
            timer_to_stop.shutdown()

    def warn_if_frame_mismatch(self, root_frame_id, ordered_waypoint_entries):
        mismatched_indices = [
            waypoint_index
            for waypoint_index, waypoint in ordered_waypoint_entries
            if waypoint.header.frame_id and waypoint.header.frame_id != root_frame_id
        ]
        if mismatched_indices:
            rospy.logwarn(
                "Frame mismatch detected: root/tail_pose uses '%s' but waypoints %s use different frame_ids; no TF transform will be applied.",
                root_frame_id,
                mismatched_indices,
            )

    def publish_trajectory_markers(self, frame_id, trajectory_positions, control_positions):
        marker_array = MarkerArray()
        stamp = rospy.Time.now()

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
        line_marker.points = [position_to_point(position) for position in trajectory_positions]
        marker_array.markers.append(line_marker)

        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = frame_id
        waypoint_marker.header.stamp = stamp
        waypoint_marker.ns = "trajectory_waypoints"
        waypoint_marker.id = 1
        waypoint_marker.type = Marker.SPHERE_LIST
        waypoint_marker.action = Marker.ADD
        waypoint_marker.pose.orientation.w = 1.0
        waypoint_marker.scale.x = 0.10
        waypoint_marker.scale.y = 0.10
        waypoint_marker.scale.z = 0.10
        waypoint_marker.color.r = 0.10
        waypoint_marker.color.g = 0.35
        waypoint_marker.color.b = 1.0
        waypoint_marker.color.a = 1.0
        waypoint_marker.points = [position_to_point(position) for position in control_positions]
        marker_array.markers.append(waypoint_marker)

        self.trajectory_marker_pub.publish(marker_array)

    def clear_trajectory_markers(self):
        delete_all_marker = Marker()
        delete_all_marker.action = Marker.DELETEALL
        self.trajectory_marker_pub.publish(MarkerArray(markers=[delete_all_marker]))

    @abstractmethod
    def build_plan(self, start_root_pose_snapshot, terminal_root_pose_snapshot, ordered_waypoints, frame_id):
        raise NotImplementedError

    @abstractmethod
    def sample_to_pose(self, sample):
        raise NotImplementedError

    def run(self):
        rospy.spin()
