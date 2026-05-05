#!/usr/bin/env python3

import datetime
import json
import os
import re
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped


GEOMETRY_EPSILON = 1e-12
POSE_EPSILON = 1e-9


def as_float64_vector(values):
    return np.asarray(values, dtype=np.float64)


def vector_to_tuple(values):
    return tuple(float(component) for component in as_float64_vector(values))


def euclidean_distance(point_a, point_b):
    return float(np.linalg.norm(as_float64_vector(point_a) - as_float64_vector(point_b)))


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_quaternion(quaternion):
    quaternion_array = as_float64_vector(quaternion)
    norm = float(np.linalg.norm(quaternion_array))
    if norm <= GEOMETRY_EPSILON:
        raise ValueError("Received zero-norm quaternion.")

    return vector_to_tuple(quaternion_array / norm)


def rotate_vector(quaternion, vector):
    normalized_quaternion = as_float64_vector(normalize_quaternion(quaternion))
    vector_array = as_float64_vector(vector)
    quaternion_vector = normalized_quaternion[:3]
    quaternion_scalar = normalized_quaternion[3]

    twice_cross = 2.0 * np.cross(quaternion_vector, vector_array)
    rotated_vector = vector_array + quaternion_scalar * twice_cross + np.cross(quaternion_vector, twice_cross)
    return vector_to_tuple(rotated_vector)


def radial_distance_to_plane_axis(point, center, normal):
    center_to_point = as_float64_vector(point) - as_float64_vector(center)
    normal_array = as_float64_vector(normal)
    axial_component = float(np.dot(center_to_point, normal_array))
    radial_component = center_to_point - axial_component * normal_array
    return float(np.linalg.norm(radial_component))


def signed_distance_to_plane(point, center, normal):
    return float(np.dot(as_float64_vector(point) - as_float64_vector(center), as_float64_vector(normal)))


def interpolate_point(point_a, point_b, interpolation_ratio):
    point_a_array = as_float64_vector(point_a)
    point_b_array = as_float64_vector(point_b)
    return vector_to_tuple(point_a_array + (point_b_array - point_a_array) * interpolation_ratio)


def time_to_seconds(stamp):
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


class PoseSample(object):
    __slots__ = ("stamp", "position")

    def __init__(self, stamp, position):
        self.stamp = stamp
        self.position = position


class CrossingRecord(object):
    __slots__ = ("stamp_sec", "position", "radial_distance_m")

    def __init__(self, stamp_sec, position, radial_distance_m):
        self.stamp_sec = stamp_sec
        self.position = position
        self.radial_distance_m = radial_distance_m


class WaypointState(object):
    __slots__ = (
        "index",
        "topic_name",
        "center",
        "orientation",
        "normal",
        "root_previous_sample",
        "tail_previous_sample",
        "root_crossing",
        "tail_crossing",
    )

    def __init__(self, index, topic_name):
        self.index = index
        self.topic_name = topic_name
        self.center = None
        self.orientation = None
        self.normal = None
        self.root_previous_sample = None
        self.tail_previous_sample = None
        self.root_crossing = None
        self.tail_crossing = None

    def has_pose(self):
        return self.center is not None and self.orientation is not None and self.normal is not None

    def is_complete(self):
        return self.root_crossing is not None and self.tail_crossing is not None


class EnvelopeWidthEvaluator(object):
    NODE_NAME = "envelope_width_evaluator"
    WAYPOINT_TOPIC_TYPE = "geometry_msgs/PoseStamped"
    DEFAULT_ROOT_TOPIC = "/dragon/root/pose"
    DEFAULT_LAST_LINK_TOPIC = "/dragon/last_link/tail_pose"
    DEFAULT_WAYPOINT_TOPIC_PATTERN = r"^/?waypoint/pose_(\d+)$"
    DEFAULT_RING_RADIUS_M = 0.4
    DEFAULT_WAYPOINT_DISCOVERY_PERIOD_SEC = 1.0
    DEFAULT_WAYPOINT_DISCOVERY_SETTLE_SEC = 2.0
    DEFAULT_OUTPUT_DIR = os.path.normpath(
        os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..",
            "..",
            "data",
            "envelope_width",
        )
    )
    LOG_THROTTLE_SEC = 2.0
    RING_LOCAL_NORMAL = (1.0, 0.0, 0.0)

    def __init__(self):
        rospy.init_node(self.NODE_NAME, anonymous=False)

        self.root_topic = rospy.get_param("~root_topic", self.DEFAULT_ROOT_TOPIC)
        self.last_link_topic = rospy.get_param("~last_link_topic", self.DEFAULT_LAST_LINK_TOPIC)
        self.waypoint_topic_pattern_text = rospy.get_param(
            "~waypoint_topic_pattern",
            self.DEFAULT_WAYPOINT_TOPIC_PATTERN,
        )
        self.ring_radius_m = float(rospy.get_param("~ring_radius_m", self.DEFAULT_RING_RADIUS_M))
        self.discovery_period_sec = float(
            rospy.get_param("~waypoint_discovery_period_sec", self.DEFAULT_WAYPOINT_DISCOVERY_PERIOD_SEC)
        )
        self.discovery_settle_sec = float(
            rospy.get_param("~waypoint_discovery_settle_sec", self.DEFAULT_WAYPOINT_DISCOVERY_SETTLE_SEC)
        )
        self.output_dir = os.path.expanduser(rospy.get_param("~output_dir", self.DEFAULT_OUTPUT_DIR))

        if self.ring_radius_m <= 0.0:
            raise ValueError("~ring_radius_m must be greater than 0.")
        if self.discovery_period_sec <= 0.0:
            raise ValueError("~waypoint_discovery_period_sec must be greater than 0.")
        if self.discovery_settle_sec < 0.0:
            raise ValueError("~waypoint_discovery_settle_sec must be non-negative.")

        try:
            self.waypoint_topic_pattern = re.compile(self.waypoint_topic_pattern_text)
        except re.error as exc:
            raise ValueError(
                "Invalid ~waypoint_topic_pattern '{}': {}".format(self.waypoint_topic_pattern_text, exc)
            )

        self.lock = threading.RLock()
        self.reference_frame_id = None
        self.topic_by_index = {}
        self.subscribers = {}
        self.waypoints = {}
        self.latest_root_sample = None
        self.latest_tail_sample = None
        self.output_written = False
        self.finalizing = False
        self.output_path = None
        self.discovery_change_monotonic = time.monotonic()

        self.root_subscriber = rospy.Subscriber(
            self.root_topic,
            PoseStamped,
            self.root_pose_callback,
            queue_size=200,
        )
        self.last_link_subscriber = rospy.Subscriber(
            self.last_link_topic,
            PoseStamped,
            self.last_link_pose_callback,
            queue_size=200,
        )
        self.discovery_timer = rospy.Timer(
            rospy.Duration.from_sec(self.discovery_period_sec),
            self.discovery_timer_callback,
        )
        rospy.on_shutdown(self.on_shutdown)

        self.discover_waypoint_topics()
        rospy.loginfo(
            "Waypoint ring envelope width evaluator listening on %s and %s; waypoint pattern '%s', ring radius %.3f m, discovery period %.3f s, settle %.3f s, output dir %s",
            rospy.resolve_name(self.root_topic),
            rospy.resolve_name(self.last_link_topic),
            self.waypoint_topic_pattern_text,
            self.ring_radius_m,
            self.discovery_period_sec,
            self.discovery_settle_sec,
            self.output_dir,
        )

    @staticmethod
    def normalize_topic_name(topic_name):
        if topic_name.startswith("/"):
            return topic_name
        return "/" + topic_name

    @staticmethod
    def sample_from_pose(msg):
        return PoseSample(
            msg.header.stamp,
            (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ),
        )

    @staticmethod
    def orientation_from_pose(msg):
        return (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

    @staticmethod
    def pose_matches_waypoint_state(waypoint_state, pose_msg):
        if not waypoint_state.has_pose():
            return False

        center = as_float64_vector(
            (
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z,
            )
        )
        orientation = as_float64_vector(
            (
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w,
            )
        )

        center_matches = np.allclose(
            as_float64_vector(waypoint_state.center),
            center,
            atol=POSE_EPSILON,
            rtol=0.0,
        )
        orientation_matches = np.allclose(
            as_float64_vector(waypoint_state.orientation),
            orientation,
            atol=POSE_EPSILON,
            rtol=0.0,
        )
        return bool(center_matches and orientation_matches)

    def discovery_timer_callback(self, _event):
        self.discover_waypoint_topics()
        self.maybe_finalize_complete()

    def discover_waypoint_topics(self):
        discovered_topics = {}
        for topic_name, topic_type in rospy.get_published_topics():
            if topic_type != self.WAYPOINT_TOPIC_TYPE:
                continue

            normalized_topic_name = self.normalize_topic_name(topic_name)
            match = self.waypoint_topic_pattern.match(normalized_topic_name)
            if match is None:
                continue

            discovered_topics[int(match.group(1))] = normalized_topic_name

        with self.lock:
            previous_topics = dict(self.topic_by_index)
            if previous_topics == discovered_topics:
                return

            removed_indices = set(previous_topics) - set(discovered_topics)
            for waypoint_index in removed_indices:
                removed_topic_name = previous_topics[waypoint_index]
                subscriber = self.subscribers.pop(removed_topic_name, None)
                if subscriber is not None:
                    subscriber.unregister()
                self.waypoints.pop(waypoint_index, None)

            for waypoint_index, topic_name in discovered_topics.items():
                previous_topic_name = previous_topics.get(waypoint_index)
                if previous_topic_name == topic_name:
                    continue

                if previous_topic_name is not None:
                    subscriber = self.subscribers.pop(previous_topic_name, None)
                    if subscriber is not None:
                        subscriber.unregister()

                self.subscribers[topic_name] = rospy.Subscriber(
                    topic_name,
                    PoseStamped,
                    self.waypoint_pose_callback,
                    callback_args=waypoint_index,
                    queue_size=1,
                )
                self.waypoints[waypoint_index] = WaypointState(waypoint_index, topic_name)

            for waypoint_index, topic_name in discovered_topics.items():
                if waypoint_index in self.waypoints:
                    self.waypoints[waypoint_index].topic_name = topic_name

            self.topic_by_index = discovered_topics
            self.discovery_change_monotonic = time.monotonic()

        rospy.loginfo(
            "Discovered %d waypoint pose topics for ring envelope width evaluation.",
            len(discovered_topics),
        )

    def capture_reference_frame(self, frame_id, topic_name):
        with self.lock:
            if self.reference_frame_id is None:
                self.reference_frame_id = frame_id
                rospy.loginfo(
                    "Captured reference frame '%s' from %s.",
                    self.reference_frame_id,
                    topic_name,
                )
                return True

            expected_frame_id = self.reference_frame_id

        if frame_id != expected_frame_id:
            rospy.logwarn_throttle(
                self.LOG_THROTTLE_SEC,
                "Discarding %s sample with frame_id '%s'; expected '%s'.",
                topic_name,
                frame_id,
                expected_frame_id,
            )
            return False

        return True

    def root_pose_callback(self, msg):
        if not self.capture_reference_frame(msg.header.frame_id, rospy.resolve_name(self.root_topic)):
            return

        sample = self.sample_from_pose(msg)
        crossing_count = 0
        with self.lock:
            previous_global_sample = self.latest_root_sample
            if previous_global_sample is not None and sample.stamp < previous_global_sample.stamp:
                rospy.logwarn_throttle(
                    self.LOG_THROTTLE_SEC,
                    "Discarding out-of-order root sample at %.6f; previous stamp is %.6f.",
                    time_to_seconds(sample.stamp),
                    time_to_seconds(previous_global_sample.stamp),
                )
                return

            self.latest_root_sample = sample
            crossing_count = self.process_stream_sample_locked("root", sample)

        if crossing_count > 0:
            rospy.loginfo("Detected %d root ring crossing(s) at stamp %.6f.", crossing_count, time_to_seconds(sample.stamp))
        self.maybe_finalize_complete()

    def last_link_pose_callback(self, msg):
        if not self.capture_reference_frame(msg.header.frame_id, rospy.resolve_name(self.last_link_topic)):
            return

        sample = self.sample_from_pose(msg)
        crossing_count = 0
        with self.lock:
            previous_global_sample = self.latest_tail_sample
            if previous_global_sample is not None and sample.stamp < previous_global_sample.stamp:
                rospy.logwarn_throttle(
                    self.LOG_THROTTLE_SEC,
                    "Discarding out-of-order last-link sample at %.6f; previous stamp is %.6f.",
                    time_to_seconds(sample.stamp),
                    time_to_seconds(previous_global_sample.stamp),
                )
                return

            self.latest_tail_sample = sample
            crossing_count = self.process_stream_sample_locked("tail", sample)

        if crossing_count > 0:
            rospy.loginfo(
                "Detected %d last-link ring crossing(s) at stamp %.6f.",
                crossing_count,
                time_to_seconds(sample.stamp),
            )
        self.maybe_finalize_complete()

    def waypoint_pose_callback(self, pose_msg, waypoint_index):
        if not self.capture_reference_frame(
            pose_msg.header.frame_id,
            self.normalize_topic_name(self.topic_by_index.get(waypoint_index, "waypoint/pose_{}".format(waypoint_index))),
        ):
            return

        with self.lock:
            waypoint_state = self.waypoints.get(waypoint_index)
            if waypoint_state is None:
                return

            if waypoint_state.has_pose():
                if self.pose_matches_waypoint_state(waypoint_state, pose_msg):
                    return

                rospy.logdebug_throttle(
                    self.LOG_THROTTLE_SEC,
                    "Ignoring updated pose for %s after the initial waypoint geometry was captured.",
                    waypoint_state.topic_name,
                )
                return

            center = (
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z,
            )
            orientation = self.orientation_from_pose(pose_msg)
            try:
                normal = rotate_vector(orientation, self.RING_LOCAL_NORMAL)
            except ValueError as exc:
                rospy.logwarn_throttle(
                    self.LOG_THROTTLE_SEC,
                    "Ignoring waypoint pose for %s due to invalid orientation: %s",
                    waypoint_state.topic_name,
                    exc,
                )
                return

            waypoint_state.center = center
            waypoint_state.orientation = orientation
            waypoint_state.normal = normal
            waypoint_state.root_previous_sample = self.latest_root_sample
            waypoint_state.tail_previous_sample = self.latest_tail_sample

        rospy.loginfo(
            "Captured waypoint %d geometry from %s.",
            waypoint_index,
            waypoint_state.topic_name,
        )
        self.maybe_finalize_complete()

    def process_stream_sample_locked(self, stream_name, current_sample):
        crossing_count = 0
        for waypoint_state in self.iter_waypoints_locked():
            if not waypoint_state.has_pose():
                continue

            if stream_name == "root":
                if waypoint_state.root_crossing is not None:
                    continue
                previous_sample = waypoint_state.root_previous_sample
                if previous_sample is None:
                    waypoint_state.root_previous_sample = current_sample
                    continue

                crossing = self.build_crossing_record(previous_sample, current_sample, waypoint_state)
                waypoint_state.root_previous_sample = current_sample
                if crossing is not None:
                    waypoint_state.root_crossing = crossing
                    crossing_count += 1
                continue

            if waypoint_state.tail_crossing is not None:
                continue
            previous_sample = waypoint_state.tail_previous_sample
            if previous_sample is None:
                waypoint_state.tail_previous_sample = current_sample
                continue

            crossing = self.build_crossing_record(previous_sample, current_sample, waypoint_state)
            waypoint_state.tail_previous_sample = current_sample
            if crossing is not None:
                waypoint_state.tail_crossing = crossing
                crossing_count += 1

        return crossing_count

    def build_crossing_record(self, previous_sample, current_sample, waypoint_state):
        start_distance = signed_distance_to_plane(
            previous_sample.position,
            waypoint_state.center,
            waypoint_state.normal,
        )
        end_distance = signed_distance_to_plane(
            current_sample.position,
            waypoint_state.center,
            waypoint_state.normal,
        )

        if abs(start_distance) <= GEOMETRY_EPSILON and abs(end_distance) <= GEOMETRY_EPSILON:
            return None

        if abs(start_distance) <= GEOMETRY_EPSILON:
            interpolation_ratio = 0.0
        elif abs(end_distance) <= GEOMETRY_EPSILON:
            interpolation_ratio = 1.0
        elif start_distance * end_distance > 0.0:
            return None
        else:
            interpolation_ratio = start_distance / (start_distance - end_distance)

        interpolation_ratio = clamp(interpolation_ratio, 0.0, 1.0)
        position = interpolate_point(previous_sample.position, current_sample.position, interpolation_ratio)
        radial_distance_m = radial_distance_to_plane_axis(position, waypoint_state.center, waypoint_state.normal)
        if radial_distance_m > self.ring_radius_m + GEOMETRY_EPSILON:
            return None

        stamp_sec = time_to_seconds(previous_sample.stamp) + (
            time_to_seconds(current_sample.stamp) - time_to_seconds(previous_sample.stamp)
        ) * interpolation_ratio
        return CrossingRecord(stamp_sec, position, radial_distance_m)

    def iter_waypoints_locked(self):
        return [self.waypoints[index] for index in sorted(self.waypoints)]

    def waypoint_topics_stable(self):
        with self.lock:
            discovered_waypoint_count = len(self.topic_by_index)
            stable = (time.monotonic() - self.discovery_change_monotonic) >= self.discovery_settle_sec
        return discovered_waypoint_count > 0 and stable

    def all_waypoints_complete(self):
        with self.lock:
            if not self.topic_by_index:
                return False

            for waypoint_index in self.topic_by_index:
                waypoint_state = self.waypoints.get(waypoint_index)
                if waypoint_state is None or not waypoint_state.is_complete():
                    return False

        return True

    def maybe_finalize_complete(self):
        if not self.waypoint_topics_stable():
            return
        if not self.all_waypoints_complete():
            return
        self.finalize_result("complete")

    def on_shutdown(self):
        self.finalize_result("shutdown")

    def finalize_result(self, reason):
        with self.lock:
            if self.output_written or self.finalizing:
                return

            self.finalizing = True
            result = self.build_result_locked(reason)

        try:
            output_path = self.write_result_file(result)
        except Exception as exc:
            with self.lock:
                self.finalizing = False
            rospy.logerr("Failed to write waypoint ring envelope width JSON: %s", exc)
            return

        with self.lock:
            self.output_written = True
            self.finalizing = False
            self.output_path = output_path

        rospy.loginfo("Wrote waypoint ring envelope width JSON to %s.", output_path)
        if reason == "complete" and not rospy.is_shutdown():
            rospy.signal_shutdown("Waypoint ring envelope width evaluation completed.")

    def build_result_locked(self, reason):
        waypoint_entries = []
        completed_widths = []

        for waypoint_state in self.iter_waypoints_locked():
            status = self.status_for_waypoint(waypoint_state)
            envelope_width_m = None
            if waypoint_state.root_crossing is not None and waypoint_state.tail_crossing is not None:
                envelope_width_m = euclidean_distance(
                    waypoint_state.root_crossing.position,
                    waypoint_state.tail_crossing.position,
                )
                completed_widths.append(envelope_width_m)

            waypoint_entries.append(
                {
                    "index": waypoint_state.index,
                    "topic": waypoint_state.topic_name,
                    "center": self.vector_to_list(waypoint_state.center),
                    "orientation_xyzw": self.vector_to_list(waypoint_state.orientation),
                    "status": status,
                    "root_crossing": self.crossing_to_dict(waypoint_state.root_crossing),
                    "tail_crossing": self.crossing_to_dict(waypoint_state.tail_crossing),
                    "envelope_width_m": envelope_width_m,
                }
            )

        summary = {
            "discovered_waypoint_count": len(self.topic_by_index),
            "completed_waypoint_count": len(completed_widths),
            "incomplete_waypoint_count": len(self.topic_by_index) - len(completed_widths),
            "min_envelope_width_m": min(completed_widths) if completed_widths else None,
            "mean_envelope_width_m": (
                sum(completed_widths) / float(len(completed_widths)) if completed_widths else None
            ),
            "max_envelope_width_m": max(completed_widths) if completed_widths else None,
        }

        return {
            "root_topic": rospy.resolve_name(self.root_topic),
            "last_link_topic": rospy.resolve_name(self.last_link_topic),
            "frame_id": self.reference_frame_id,
            "ring_radius_m": self.ring_radius_m,
            "finalization_reason": reason,
            "generated_at_wall_time": self.generated_at_wall_time(),
            "summary": summary,
            "waypoints": waypoint_entries,
        }

    @staticmethod
    def status_for_waypoint(waypoint_state):
        if waypoint_state.root_crossing is not None and waypoint_state.tail_crossing is not None:
            return "complete"
        if waypoint_state.root_crossing is None and waypoint_state.tail_crossing is None:
            return "missing_both_crossings"
        if waypoint_state.root_crossing is None:
            return "missing_root_crossing"
        return "missing_tail_crossing"

    @staticmethod
    def vector_to_list(vector):
        if vector is None:
            return None
        if len(vector) == 3:
            return [vector[0], vector[1], vector[2]]
        return [vector[0], vector[1], vector[2], vector[3]]

    @staticmethod
    def crossing_to_dict(crossing):
        if crossing is None:
            return None

        return {
            "stamp_sec": crossing.stamp_sec,
            "position": [
                crossing.position[0],
                crossing.position[1],
                crossing.position[2],
            ],
            "radial_distance_m": crossing.radial_distance_m,
        }

    @staticmethod
    def generated_at_wall_time():
        return datetime.datetime.now().astimezone().isoformat(timespec="seconds")

    @staticmethod
    def wall_time_to_filename():
        return "{}.json".format(datetime.datetime.now().strftime("%Y%m%d%H%M%S"))

    def write_result_file(self, result):
        os.makedirs(self.output_dir, exist_ok=True)
        output_path = os.path.join(self.output_dir, self.wall_time_to_filename())
        with open(output_path, "w", encoding="utf-8") as output_file:
            json.dump(result, output_file, indent=2)
            output_file.write("\n")
        return output_path

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        EnvelopeWidthEvaluator().run()
    except rospy.ROSInterruptException:
        pass
