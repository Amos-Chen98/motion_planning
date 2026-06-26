#!/usr/bin/env python3

import datetime
import json
import os
import re
import subprocess
import sys
import warnings

import rosbag
import rospy
import tf.transformations as tf_trans
import tf2_ros

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_FALLBACK_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, ".."))
EVALUATION_SCRIPT_DIR = os.path.join(PACKAGE_FALLBACK_DIR, "scripts", "evaluation")
if EVALUATION_SCRIPT_DIR not in sys.path:
    sys.path.insert(0, EVALUATION_SCRIPT_DIR)

from envelope_width_evaluator import (
    CrossingRecord,
    GEOMETRY_EPSILON,
    PoseSample,
    clamp,
    euclidean_distance,
    interpolate_point,
    normalize_quaternion,
    radial_distance_to_plane_axis,
    rotate_vector,
    signed_distance_to_plane,
    time_to_seconds,
)


warnings.filterwarnings("ignore", message="translation should be of type Vector3")
warnings.filterwarnings("ignore", message="rotation should be of type Quaternion")


PACKAGE_NAME = "multilink_copilot"
BAG_STEM = "2026-05-06-18-14-00_four_ring_success"
ROOT_POSE_TOPIC = "dragon/root/pose"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
ROBOT_NS = "dragon"
WAYPOINT_TOPIC_PATTERN = r"^/?waypoint/pose_(\d+)$"
WAYPOINT_TOPIC_TYPE = "geometry_msgs/PoseStamped"
RING_RADIUS_M = 0.4
TAIL_OFFSET_X_M = 0.455
RING_LOCAL_NORMAL = (1.0, 0.0, 0.0)
TF_CACHE_PADDING_SEC = 30.0

LINK_TAILS = (
    (1, "link1_tail"),
    (2, "link2_tail"),
    (3, "link3_tail"),
    (4, "link4_tail"),
)
SEPARATION_LINK_KEYS = ("link2_tail", "link3_tail", "link4_tail")


def normalize_topic_name(topic_name):
    if topic_name.startswith("/"):
        return topic_name
    return "/" + topic_name


def strip_leading_slash(frame_id):
    return frame_id[1:] if frame_id.startswith("/") else frame_id


def pose_position(msg):
    return (
        float(msg.pose.position.x),
        float(msg.pose.position.y),
        float(msg.pose.position.z),
    )


def pose_orientation(msg):
    return (
        float(msg.pose.orientation.x),
        float(msg.pose.orientation.y),
        float(msg.pose.orientation.z),
        float(msg.pose.orientation.w),
    )


def transform_translation(transform_stamped):
    translation = transform_stamped.transform.translation
    return (float(translation.x), float(translation.y), float(translation.z))


def transform_orientation(transform_stamped):
    rotation = transform_stamped.transform.rotation
    return (
        float(rotation.x),
        float(rotation.y),
        float(rotation.z),
        float(rotation.w),
    )


def vector_to_list(vector):
    if vector is None:
        return None
    return [float(component) for component in vector]


def crossing_to_dict(crossing):
    if crossing is None:
        return None

    return {
        "stamp_sec": float(crossing.stamp_sec),
        "position": vector_to_list(crossing.position),
        "radial_distance_m": float(crossing.radial_distance_m),
    }


def generated_at_wall_time():
    return datetime.datetime.now().astimezone().isoformat(timespec="seconds")


def fallback_package_dir():
    return PACKAGE_FALLBACK_DIR


def find_package_dir():
    try:
        output = subprocess.check_output(["rospack", "find", PACKAGE_NAME], text=True)
        package_dir = output.strip()
        if package_dir:
            return package_dir
    except (OSError, subprocess.CalledProcessError):
        pass

    return fallback_package_dir()


def stats_for_values(values):
    if not values:
        return {
            "count": 0,
            "min_m": None,
            "mean_m": None,
            "max_m": None,
        }

    return {
        "count": len(values),
        "min_m": min(values),
        "mean_m": sum(values) / float(len(values)),
        "max_m": max(values),
    }


class WaypointEvaluation(object):
    __slots__ = (
        "index",
        "topic",
        "center",
        "orientation",
        "normal",
        "previous_sample_by_link",
        "crossing_by_link",
    )

    def __init__(self, index, topic):
        self.index = index
        self.topic = topic
        self.center = None
        self.orientation = None
        self.normal = None
        self.previous_sample_by_link = {link_key: None for _, link_key in LINK_TAILS}
        self.crossing_by_link = {link_key: None for _, link_key in LINK_TAILS}

    def has_pose(self):
        return self.center is not None and self.orientation is not None and self.normal is not None

    def missing_crossings(self):
        return [link_key for _, link_key in LINK_TAILS if self.crossing_by_link[link_key] is None]

    def is_complete(self):
        return not self.missing_crossings()


class LinkTailSeparationEvaluator(object):
    def __init__(self, bag_stem):
        self.bag_stem = bag_stem
        self.package_dir = find_package_dir()
        self.bag_path = self.resolve_bag_path(bag_stem)
        self.output_dir = os.path.join(self.package_dir, "data", "separation")
        self.output_path = os.path.join(self.output_dir, "{}.json".format(os.path.splitext(os.path.basename(self.bag_path))[0]))
        self.waypoint_pattern = re.compile(WAYPOINT_TOPIC_PATTERN)

        self.reference_frame_id = None
        self.tf_buffer = None
        self.root_pose_topic = None
        self.tf_topic = None
        self.tf_static_topic = None
        self.waypoints_by_index = {}

        self.root_pose_sample_count = 0
        self.valid_root_pose_sample_count = 0
        self.root_frame_mismatch_count = 0
        self.root_out_of_order_count = 0
        self.tf_transform_count = 0
        self.tf_static_transform_count = 0
        self.tf_lookup_failures_by_link = {link_key: 0 for _, link_key in LINK_TAILS}
        self.link_tail_sample_count_by_link = {link_key: 0 for _, link_key in LINK_TAILS}

    def resolve_bag_path(self, bag_stem):
        bag_name = bag_stem if bag_stem.endswith(".bag") else "{}.bag".format(bag_stem)
        return os.path.join(self.package_dir, "data", "rosbag", bag_name)

    def run(self):
        if not os.path.isfile(self.bag_path):
            raise RuntimeError("bag file does not exist: {}".format(self.bag_path))

        with rosbag.Bag(self.bag_path) as bag:
            topic_infos = bag.get_type_and_topic_info().topics
            self.root_pose_topic = self.resolve_topic(topic_infos, ROOT_POSE_TOPIC, required=True)
            self.tf_topic = self.resolve_topic(topic_infos, TF_TOPIC, required=True)
            self.tf_static_topic = self.resolve_topic(topic_infos, TF_STATIC_TOPIC, required=False)
            self.waypoints_by_index = self.discover_waypoints(topic_infos)

            if not self.waypoints_by_index:
                raise RuntimeError("no waypoint pose topics matching '{}' found in {}".format(WAYPOINT_TOPIC_PATTERN, self.bag_path))

            self.tf_buffer = tf2_ros.Buffer(
                cache_time=rospy.Duration.from_sec((bag.get_end_time() - bag.get_start_time()) + TF_CACHE_PADDING_SEC),
                debug=False,
            )

            self.load_waypoints_and_tf(bag)
            self.process_root_poses(bag)
            result = self.build_result(bag)

        self.write_result(result)
        return self.output_path

    def resolve_topic(self, topic_infos, expected_topic, required):
        expected_normalized = normalize_topic_name(expected_topic)
        for topic in topic_infos:
            if normalize_topic_name(topic) == expected_normalized:
                return topic

        if required:
            raise RuntimeError("required topic '{}' not found in {}".format(expected_topic, self.bag_path))
        return None

    def discover_waypoints(self, topic_infos):
        waypoints_by_index = {}

        for topic, topic_info in topic_infos.items():
            if topic_info.msg_type != WAYPOINT_TOPIC_TYPE:
                continue

            match = self.waypoint_pattern.match(normalize_topic_name(topic))
            if match is None:
                continue

            index = int(match.group(1))
            if index in waypoints_by_index:
                raise RuntimeError(
                    "duplicate waypoint index {} found for '{}' and '{}'".format(
                        index,
                        waypoints_by_index[index].topic,
                        topic,
                    )
                )
            waypoints_by_index[index] = WaypointEvaluation(index, topic)

        return waypoints_by_index

    def load_waypoints_and_tf(self, bag):
        waypoint_by_topic = {waypoint.topic: waypoint for waypoint in self.waypoints_by_index.values()}
        topics = [self.tf_topic] + list(waypoint_by_topic)
        if self.tf_static_topic is not None:
            topics.insert(0, self.tf_static_topic)

        for topic, msg, _stamp in bag.read_messages(topics=topics):
            if topic == self.tf_static_topic:
                for transform in msg.transforms:
                    self.tf_buffer.set_transform_static(transform, "bag_static")
                    self.tf_static_transform_count += 1
                continue

            if topic == self.tf_topic:
                for transform in msg.transforms:
                    self.tf_buffer.set_transform(transform, "bag")
                    self.tf_transform_count += 1
                continue

            waypoint = waypoint_by_topic.get(topic)
            if waypoint is not None and not waypoint.has_pose():
                self.capture_waypoint_pose(waypoint, msg)

        missing_waypoints = [index for index, waypoint in sorted(self.waypoints_by_index.items()) if not waypoint.has_pose()]
        if missing_waypoints:
            raise RuntimeError("missing pose messages for waypoint indices: {}".format(missing_waypoints))
        if self.tf_transform_count == 0:
            raise RuntimeError("no dynamic TF transforms found on {}".format(self.tf_topic))

    def capture_waypoint_pose(self, waypoint, msg):
        if self.reference_frame_id is None:
            self.reference_frame_id = msg.header.frame_id
        elif msg.header.frame_id != self.reference_frame_id:
            raise RuntimeError(
                "waypoint frame mismatch for {}: got '{}', expected '{}'".format(
                    waypoint.topic,
                    msg.header.frame_id,
                    self.reference_frame_id,
                )
            )

        waypoint.center = pose_position(msg)
        waypoint.orientation = normalize_quaternion(pose_orientation(msg))
        waypoint.normal = rotate_vector(waypoint.orientation, RING_LOCAL_NORMAL)

    def process_root_poses(self, bag):
        previous_root_stamp = None

        for _topic, msg, _stamp in bag.read_messages(topics=[self.root_pose_topic]):
            self.root_pose_sample_count += 1

            if self.reference_frame_id is not None and msg.header.frame_id != self.reference_frame_id:
                self.root_frame_mismatch_count += 1
                continue

            if previous_root_stamp is not None and msg.header.stamp < previous_root_stamp:
                self.root_out_of_order_count += 1
                continue
            previous_root_stamp = msg.header.stamp

            processed_any_link = False
            for link_index, link_key in LINK_TAILS:
                try:
                    tail_position = self.compute_link_tail_position(msg, link_index)
                except (
                    tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException,
                    ValueError,
                ):
                    self.tf_lookup_failures_by_link[link_key] += 1
                    continue

                processed_any_link = True
                self.link_tail_sample_count_by_link[link_key] += 1
                self.process_link_sample(link_key, PoseSample(msg.header.stamp, tail_position))

            if processed_any_link:
                self.valid_root_pose_sample_count += 1

    def compute_link_tail_position(self, root_pose_msg, link_index):
        root_frame = strip_leading_slash("{}/root".format(ROBOT_NS))
        link_frame = strip_leading_slash("{}/link{}".format(ROBOT_NS, link_index))

        relative_transform = self.tf_buffer.lookup_transform(
            root_frame,
            link_frame,
            root_pose_msg.header.stamp,
            rospy.Duration(0),
        )

        root_position = pose_position(root_pose_msg)
        root_orientation = normalize_quaternion(pose_orientation(root_pose_msg))
        relative_position = transform_translation(relative_transform)
        relative_orientation = normalize_quaternion(transform_orientation(relative_transform))

        world_link_orientation = normalize_quaternion(
            tf_trans.quaternion_multiply(root_orientation, relative_orientation)
        )
        relative_position_world = rotate_vector(root_orientation, relative_position)
        world_link_position = (
            root_position[0] + relative_position_world[0],
            root_position[1] + relative_position_world[1],
            root_position[2] + relative_position_world[2],
        )
        tail_offset_world = rotate_vector(world_link_orientation, (TAIL_OFFSET_X_M, 0.0, 0.0))

        return (
            world_link_position[0] + tail_offset_world[0],
            world_link_position[1] + tail_offset_world[1],
            world_link_position[2] + tail_offset_world[2],
        )

    def process_link_sample(self, link_key, current_sample):
        for waypoint in self.iter_waypoints():
            if waypoint.crossing_by_link[link_key] is not None:
                continue

            previous_sample = waypoint.previous_sample_by_link[link_key]
            if previous_sample is None:
                waypoint.previous_sample_by_link[link_key] = current_sample
                continue

            crossing = self.build_crossing_record(previous_sample, current_sample, waypoint)
            waypoint.previous_sample_by_link[link_key] = current_sample
            if crossing is not None:
                waypoint.crossing_by_link[link_key] = crossing

    def build_crossing_record(self, previous_sample, current_sample, waypoint):
        start_distance = signed_distance_to_plane(
            previous_sample.position,
            waypoint.center,
            waypoint.normal,
        )
        end_distance = signed_distance_to_plane(
            current_sample.position,
            waypoint.center,
            waypoint.normal,
        )

        if abs(start_distance) <= GEOMETRY_EPSILON and abs(end_distance) <= GEOMETRY_EPSILON:
            return None

        if start_distance >= -GEOMETRY_EPSILON or end_distance < -GEOMETRY_EPSILON:
            return None

        if abs(end_distance) <= GEOMETRY_EPSILON:
            interpolation_ratio = 1.0
        else:
            interpolation_ratio = start_distance / (start_distance - end_distance)

        interpolation_ratio = clamp(interpolation_ratio, 0.0, 1.0)
        position = interpolate_point(previous_sample.position, current_sample.position, interpolation_ratio)
        radial_distance_m = radial_distance_to_plane_axis(position, waypoint.center, waypoint.normal)
        if radial_distance_m > RING_RADIUS_M + GEOMETRY_EPSILON:
            return None

        stamp_sec = time_to_seconds(previous_sample.stamp) + (
            time_to_seconds(current_sample.stamp) - time_to_seconds(previous_sample.stamp)
        ) * interpolation_ratio
        return CrossingRecord(stamp_sec, position, radial_distance_m)

    def build_result(self, bag):
        waypoint_entries = []
        separation_values = {link_key: [] for link_key in SEPARATION_LINK_KEYS}
        completed_waypoint_count = 0

        for waypoint in self.iter_waypoints():
            separation_from_link1 = self.compute_waypoint_separations(waypoint)
            for link_key, separation in separation_from_link1.items():
                if separation is not None:
                    separation_values[link_key].append(separation)

            if waypoint.is_complete():
                completed_waypoint_count += 1

            waypoint_entries.append(
                {
                    "index": waypoint.index,
                    "topic": waypoint.topic,
                    "center": vector_to_list(waypoint.center),
                    "orientation_xyzw": vector_to_list(waypoint.orientation),
                    "normal": vector_to_list(waypoint.normal),
                    "status": self.status_for_waypoint(waypoint),
                    "missing_crossings": waypoint.missing_crossings(),
                    "crossings": {
                        link_key: crossing_to_dict(waypoint.crossing_by_link[link_key])
                        for _, link_key in LINK_TAILS
                    },
                    "separation_from_link1_m": separation_from_link1,
                }
            )

        waypoint_count = len(self.waypoints_by_index)
        return {
            "bag_name": os.path.basename(self.bag_path),
            "bag_path": self.bag_path,
            "bag_start_time_sec": bag.get_start_time(),
            "bag_end_time_sec": bag.get_end_time(),
            "root_pose_topic": self.root_pose_topic,
            "tf_topic": self.tf_topic,
            "tf_static_topic": self.tf_static_topic,
            "frame_id": self.reference_frame_id,
            "robot_ns": ROBOT_NS,
            "ring_radius_m": RING_RADIUS_M,
            "tail_offset_x_m": TAIL_OFFSET_X_M,
            "generated_at_wall_time": generated_at_wall_time(),
            "summary": {
                "waypoint_count": waypoint_count,
                "complete_waypoint_count": completed_waypoint_count,
                "incomplete_waypoint_count": waypoint_count - completed_waypoint_count,
                "root_pose_sample_count": self.root_pose_sample_count,
                "valid_root_pose_sample_count": self.valid_root_pose_sample_count,
                "root_frame_mismatch_count": self.root_frame_mismatch_count,
                "root_out_of_order_count": self.root_out_of_order_count,
                "tf_transform_count": self.tf_transform_count,
                "tf_static_transform_count": self.tf_static_transform_count,
                "tf_lookup_failures_by_link": self.tf_lookup_failures_by_link,
                "link_tail_sample_count_by_link": self.link_tail_sample_count_by_link,
                "separation_from_link1_m": {
                    link_key: stats_for_values(separation_values[link_key])
                    for link_key in SEPARATION_LINK_KEYS
                },
            },
            "waypoints": waypoint_entries,
        }

    def compute_waypoint_separations(self, waypoint):
        link1_crossing = waypoint.crossing_by_link["link1_tail"]
        separations = {}

        for link_key in SEPARATION_LINK_KEYS:
            link_crossing = waypoint.crossing_by_link[link_key]
            if link1_crossing is None or link_crossing is None:
                separations[link_key] = None
            else:
                separations[link_key] = euclidean_distance(link1_crossing.position, link_crossing.position)

        return separations

    def status_for_waypoint(self, waypoint):
        missing_crossings = waypoint.missing_crossings()
        if not missing_crossings:
            return "complete"
        if len(missing_crossings) == 1:
            return "missing_{}_crossing".format(missing_crossings[0])
        return "missing_multiple_crossings"

    def iter_waypoints(self):
        return [self.waypoints_by_index[index] for index in sorted(self.waypoints_by_index)]

    def write_result(self, result):
        os.makedirs(self.output_dir, exist_ok=True)
        with open(self.output_path, "w", encoding="utf-8") as output_file:
            json.dump(result, output_file, indent=2)
            output_file.write("\n")


def main():
    try:
        output_path = LinkTailSeparationEvaluator(BAG_STEM).run()
    except (RuntimeError, ValueError, rosbag.ROSBagException) as exc:
        print("error: {}".format(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130

    print("Wrote link-tail separation JSON to {}".format(output_path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
