#!/usr/bin/env python3

import json
import math
import os
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

DISTANCE_EPSILON = 1e-12


def euclidean_distance(point_a, point_b):
    return math.sqrt(
        (point_a[0] - point_b[0]) * (point_a[0] - point_b[0])
        + (point_a[1] - point_b[1]) * (point_a[1] - point_b[1])
        + (point_a[2] - point_b[2]) * (point_a[2] - point_b[2])
    )


def point_to_segment_distance(point, segment_start, segment_end):
    segment_vector = (
        segment_end[0] - segment_start[0],
        segment_end[1] - segment_start[1],
        segment_end[2] - segment_start[2],
    )
    point_vector = (
        point[0] - segment_start[0],
        point[1] - segment_start[1],
        point[2] - segment_start[2],
    )
    segment_length_squared = (
        segment_vector[0] * segment_vector[0]
        + segment_vector[1] * segment_vector[1]
        + segment_vector[2] * segment_vector[2]
    )
    if segment_length_squared <= DISTANCE_EPSILON:
        return euclidean_distance(point, segment_start)

    projection = (
        point_vector[0] * segment_vector[0]
        + point_vector[1] * segment_vector[1]
        + point_vector[2] * segment_vector[2]
    ) / segment_length_squared
    projection = max(0.0, min(1.0, projection))

    closest_point = (
        segment_start[0] + projection * segment_vector[0],
        segment_start[1] + projection * segment_vector[1],
        segment_start[2] + projection * segment_vector[2],
    )
    return euclidean_distance(point, closest_point)


def point_to_polyline_distance(point, polyline_points):
    if not polyline_points:
        raise ValueError("Polyline is empty.")
    if len(polyline_points) == 1:
        return euclidean_distance(point, polyline_points[0])

    return min(
        point_to_segment_distance(point, polyline_points[index], polyline_points[index + 1])
        for index in range(len(polyline_points) - 1)
    )


def as_point_array(points):
    point_array = np.asarray(points, dtype=np.float64)
    if point_array.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    if point_array.ndim != 2 or point_array.shape[1] != 3:
        raise ValueError("Expected points with shape (N, 3).")
    return point_array


def iter_point_to_polyline_distance_batches(query_points, polyline_points, batch_size):
    if batch_size <= 0:
        raise ValueError("Batch size must be positive.")

    polyline_array = as_point_array(polyline_points)
    if polyline_array.shape[0] == 0:
        raise ValueError("Polyline is empty.")

    query_array = as_point_array(query_points)
    if query_array.shape[0] == 0:
        return

    if polyline_array.shape[0] == 1:
        anchor = polyline_array[0]
        anchor_squared_norm = float(np.dot(anchor, anchor))
        for start_index in range(0, query_array.shape[0], batch_size):
            batch = query_array[start_index : start_index + batch_size]
            batch_squared_norm = np.einsum("ij,ij->i", batch, batch)
            squared_distance = batch_squared_norm + anchor_squared_norm - 2.0 * (batch @ anchor)
            np.maximum(squared_distance, 0.0, out=squared_distance)
            yield np.sqrt(squared_distance)
        return

    segment_start = polyline_array[:-1]
    segment_vector = polyline_array[1:] - segment_start
    segment_length_squared = np.einsum("ij,ij->i", segment_vector, segment_vector)

    nondegenerate_mask = segment_length_squared > DISTANCE_EPSILON
    regular_segment_start = segment_start[nondegenerate_mask]
    regular_segment_vector = segment_vector[nondegenerate_mask]
    regular_segment_length_squared = segment_length_squared[nondegenerate_mask]
    regular_segment_start_squared_norm = np.einsum(
        "ij,ij->i",
        regular_segment_start,
        regular_segment_start,
    )
    regular_segment_start_dot_vector = np.einsum(
        "ij,ij->i",
        regular_segment_start,
        regular_segment_vector,
    )

    degenerate_segment_points = segment_start[~nondegenerate_mask]
    degenerate_segment_points_squared_norm = np.einsum(
        "ij,ij->i",
        degenerate_segment_points,
        degenerate_segment_points,
    )

    for start_index in range(0, query_array.shape[0], batch_size):
        batch = query_array[start_index : start_index + batch_size]
        batch_squared_norm = np.einsum("ij,ij->i", batch, batch)
        minimum_squared_distance = np.full(batch.shape[0], np.inf, dtype=np.float64)

        if regular_segment_start.shape[0] > 0:
            point_to_start_dot = batch @ regular_segment_start.T
            delta_squared_norm = (
                batch_squared_norm[:, None]
                + regular_segment_start_squared_norm[None, :]
                - 2.0 * point_to_start_dot
            )
            point_to_vector_dot = batch @ regular_segment_vector.T
            projection_numerator = point_to_vector_dot - regular_segment_start_dot_vector[None, :]
            projection = projection_numerator / regular_segment_length_squared[None, :]
            np.clip(projection, 0.0, 1.0, out=projection)

            squared_distance = (
                delta_squared_norm
                - 2.0 * projection * projection_numerator
                + projection * projection * regular_segment_length_squared[None, :]
            )
            np.maximum(squared_distance, 0.0, out=squared_distance)
            minimum_squared_distance = np.minimum(minimum_squared_distance, np.min(squared_distance, axis=1))

        if degenerate_segment_points.shape[0] > 0:
            point_to_degenerate_dot = batch @ degenerate_segment_points.T
            degenerate_squared_distance = (
                batch_squared_norm[:, None]
                + degenerate_segment_points_squared_norm[None, :]
                - 2.0 * point_to_degenerate_dot
            )
            np.maximum(degenerate_squared_distance, 0.0, out=degenerate_squared_distance)
            minimum_squared_distance = np.minimum(
                minimum_squared_distance,
                np.min(degenerate_squared_distance, axis=1),
            )

        yield np.sqrt(minimum_squared_distance)


def point_to_polyline_distance_stats_batched(query_points, polyline_points, batch_size):
    total_count = 0
    total_sum = 0.0
    max_distance = 0.0

    for batch_distance in iter_point_to_polyline_distance_batches(
        query_points,
        polyline_points,
        batch_size,
    ):
        if batch_distance.size == 0:
            continue
        total_count += int(batch_distance.size)
        total_sum += float(np.sum(batch_distance, dtype=np.float64))
        max_distance = max(max_distance, float(np.max(batch_distance)))

    return total_count, total_sum, max_distance


class TrajectorySample(object):
    __slots__ = ("stamp", "position")

    def __init__(self, stamp, position):
        self.stamp = stamp
        self.position = position


class EnvelopeWidthEvaluator(object):
    NODE_NAME = "envelope_width_evaluator"

    WAIT_INIT = "WAIT_INIT"
    WAIT_START = "WAIT_START"
    RECORDING = "RECORDING"
    FINISHING = "FINISHING"
    DONE = "DONE"

    DEFAULT_ROOT_TOPIC = "root/tail_pose"
    DEFAULT_LAST_LINK_TOPIC = "last_link/tail_pose"
    DEFAULT_START_PATH_LENGTH_THRESHOLD_M = 1.422
    DEFAULT_RETURN_POSITION_TOLERANCE_M = 0.10
    DEFAULT_RETURN_REARM_MARGIN_M = 0.05
    DEFAULT_OUTPUT_DIR = "~/.ros/multilink_copilot/envelope_metrics"
    DEFAULT_DISTANCE_BATCH_SIZE = 128
    FINALIZATION_DELAY_SEC = 0.10
    LOG_THROTTLE_SEC = 2.0

    def __init__(self):
        rospy.init_node(self.NODE_NAME, anonymous=False)

        self.root_topic = rospy.get_param("~root_topic", self.DEFAULT_ROOT_TOPIC)
        self.last_link_topic = rospy.get_param("~last_link_topic", self.DEFAULT_LAST_LINK_TOPIC)
        self.start_path_length_threshold_m = float(
            rospy.get_param("~start_path_length_threshold_m", self.DEFAULT_START_PATH_LENGTH_THRESHOLD_M)
        )
        self.return_position_tolerance_m = float(
            rospy.get_param("~return_position_tolerance_m", self.DEFAULT_RETURN_POSITION_TOLERANCE_M)
        )
        self.return_rearm_distance_m = float(
            rospy.get_param("~return_rearm_distance_m", self.default_return_rearm_distance())
        )
        self.output_dir = os.path.expanduser(rospy.get_param("~output_dir", self.DEFAULT_OUTPUT_DIR))
        self.distance_batch_size = max(
            1,
            int(rospy.get_param("~distance_batch_size", self.DEFAULT_DISTANCE_BATCH_SIZE)),
        )

        self.state = self.WAIT_INIT
        self.reference_frame_id = None
        self.initial_root_position = None
        self.previous_root_sample = None
        self.latest_last_link_stamp = None
        self.accumulated_root_path_length = 0.0

        self.start_ros_time = None
        self.end_ros_time = None
        self.root_recording_samples = []
        self.last_link_history = []
        self.finalization_timer = None
        self.return_detection_armed = False

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

        rospy.loginfo(
            "Envelope width evaluator listening on %s and %s; start threshold %.3f m, return tolerance %.3f m, return rearm distance %.3f m, distance batch size %d, output dir %s",
            rospy.resolve_name(self.root_topic),
            rospy.resolve_name(self.last_link_topic),
            self.start_path_length_threshold_m,
            self.return_position_tolerance_m,
            self.return_rearm_distance_m,
            self.distance_batch_size,
            self.output_dir,
        )

    def root_pose_callback(self, msg):
        if self.state in (self.FINISHING, self.DONE):
            return

        if self.state == self.WAIT_INIT:
            sample = self.sample_from_pose(msg)
            self.reference_frame_id = msg.header.frame_id
            self.initial_root_position = sample.position
            self.previous_root_sample = sample
            self.state = self.WAIT_START
            rospy.loginfo(
                "Captured initial root/tail_pose at %.6f in frame '%s'; waiting for %.3f m of accumulated root motion.",
                sample.stamp.to_sec(),
                self.reference_frame_id,
                self.start_path_length_threshold_m,
            )
            return

        if not self.has_expected_frame(msg.header.frame_id, self.root_topic):
            return

        sample = self.sample_from_pose(msg)
        if self.previous_root_sample is not None and sample.stamp < self.previous_root_sample.stamp:
            rospy.logwarn_throttle(
                self.LOG_THROTTLE_SEC,
                "Discarding out-of-order root/tail_pose sample at %.6f; previous stamp is %.6f.",
                sample.stamp.to_sec(),
                self.previous_root_sample.stamp.to_sec(),
            )
            return

        segment_length = 0.0
        if self.previous_root_sample is not None:
            segment_length = euclidean_distance(self.previous_root_sample.position, sample.position)
        self.previous_root_sample = sample

        if self.state == self.WAIT_START:
            self.accumulated_root_path_length += segment_length
            if self.accumulated_root_path_length < self.start_path_length_threshold_m:
                return

            self.start_ros_time = sample.stamp
            self.root_recording_samples = [sample]
            self.state = self.RECORDING
            self.return_detection_armed = self.is_return_detection_armed(sample.position)
            rospy.loginfo(
                "Started envelope recording at %.6f after %.3f m of accumulated root motion (return detection %s).",
                self.start_ros_time.to_sec(),
                self.accumulated_root_path_length,
                "armed" if self.return_detection_armed else "waiting to leave start neighborhood",
            )
            return

        if self.state == self.RECORDING:
            self.root_recording_samples.append(sample)
            if self.should_finish_recording(sample.position):
                self.finish_recording(sample.stamp)

    def last_link_pose_callback(self, msg):
        if self.state in (self.WAIT_INIT, self.DONE):
            return

        if not self.has_expected_frame(msg.header.frame_id, self.last_link_topic):
            return

        sample = self.sample_from_pose(msg)
        if self.latest_last_link_stamp is not None and sample.stamp < self.latest_last_link_stamp:
            rospy.logwarn_throttle(
                self.LOG_THROTTLE_SEC,
                "Discarding out-of-order last_link/tail_pose sample at %.6f; previous stamp is %.6f.",
                sample.stamp.to_sec(),
                self.latest_last_link_stamp.to_sec(),
            )
            return

        self.latest_last_link_stamp = sample.stamp
        self.last_link_history.append(sample)

    def finish_recording(self, end_ros_time):
        if self.state in (self.FINISHING, self.DONE):
            return

        self.end_ros_time = end_ros_time
        self.state = self.FINISHING
        rospy.loginfo(
            "Detected envelope recording end at %.6f; finalizing after %.3f s to collect in-flight last-link samples.",
            self.end_ros_time.to_sec(),
            self.FINALIZATION_DELAY_SEC,
        )
        self.finalization_timer = rospy.Timer(
            rospy.Duration.from_sec(self.FINALIZATION_DELAY_SEC),
            self.finalize_recording,
            oneshot=True,
        )

    def finalize_recording(self, _event):
        if self.state == self.DONE:
            return

        self.state = self.DONE
        if self.start_ros_time is None or self.end_ros_time is None:
            rospy.logerr("Recording interval is incomplete; not writing envelope metric JSON.")
            rospy.signal_shutdown("Envelope width evaluation failed.")
            return
        if self.end_ros_time < self.start_ros_time:
            rospy.logerr(
                "Recording interval is invalid: end time %.6f precedes start time %.6f; not writing JSON.",
                self.end_ros_time.to_sec(),
                self.start_ros_time.to_sec(),
            )
            rospy.signal_shutdown("Envelope width evaluation failed.")
            return
        if not self.root_recording_samples:
            rospy.logerr("No root samples were captured in the recording interval; not writing JSON.")
            rospy.signal_shutdown("Envelope width evaluation failed.")
            return

        filtered_last_link_samples = [
            sample
            for sample in self.last_link_history
            if self.start_ros_time <= sample.stamp <= self.end_ros_time
        ]
        if not filtered_last_link_samples:
            rospy.logerr("No last-link samples fell inside the recording interval; not writing JSON.")
            rospy.signal_shutdown("Envelope width evaluation failed.")
            return

        root_polyline = [sample.position for sample in self.root_recording_samples]
        last_link_positions = [sample.position for sample in filtered_last_link_samples]
        total_count, total_sum, max_distance = point_to_polyline_distance_stats_batched(
            last_link_positions,
            root_polyline,
            self.distance_batch_size,
        )
        if total_count <= 0:
            rospy.logerr("Envelope width calculation produced no distances; not writing JSON.")
            rospy.signal_shutdown("Envelope width evaluation failed.")
            return

        result = {
            "start_ros_time": self.time_to_dict(self.start_ros_time),
            "end_ros_time": self.time_to_dict(self.end_ros_time),
            "envelope_metric": {
                "mean": total_sum / float(total_count),
                "max": max_distance,
            },
        }

        self.write_result_file(result)
        rospy.loginfo(
            "Wrote envelope metrics for %.6f -> %.6f using %d root samples and %d last-link samples.",
            self.start_ros_time.to_sec(),
            self.end_ros_time.to_sec(),
            len(self.root_recording_samples),
            len(filtered_last_link_samples),
        )
        rospy.signal_shutdown("Envelope width evaluation completed.")

    def write_result_file(self, result):
        os.makedirs(self.output_dir, exist_ok=True)
        output_path = os.path.join(
            self.output_dir,
            self.wall_time_to_filename(),
        )
        with open(output_path, "w") as output_file:
            json.dump(result, output_file, indent=2)
            output_file.write("\n")

    def has_expected_frame(self, frame_id, topic_name):
        if frame_id != self.reference_frame_id:
            rospy.logwarn_throttle(
                self.LOG_THROTTLE_SEC,
                "Discarding %s sample with frame_id '%s'; expected '%s'.",
                rospy.resolve_name(topic_name),
                frame_id,
                self.reference_frame_id,
            )
            return False
        return True

    def has_returned_to_initial_position(self, position):
        return euclidean_distance(position, self.initial_root_position) <= self.return_position_tolerance_m

    def is_return_detection_armed(self, position):
        return self.distance_to_initial_position(position) > self.return_rearm_distance_m

    def should_finish_recording(self, position):
        if not self.return_detection_armed:
            if self.is_return_detection_armed(position):
                self.return_detection_armed = True
                rospy.loginfo(
                    "Armed return detection after root left the start neighborhood by %.3f m.",
                    self.distance_to_initial_position(position),
                )
            return False

        return self.has_returned_to_initial_position(position)

    def distance_to_initial_position(self, position):
        return euclidean_distance(position, self.initial_root_position)

    def default_return_rearm_distance(self):
        return max(
            self.return_position_tolerance_m + self.DEFAULT_RETURN_REARM_MARGIN_M,
            1.5 * self.return_position_tolerance_m,
        )

    @staticmethod
    def sample_from_pose(msg):
        return TrajectorySample(
            msg.header.stamp,
            (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ),
        )

    @staticmethod
    def time_to_dict(stamp):
        return {
            "secs": int(stamp.secs),
            "nsecs": int(stamp.nsecs),
        }

    @staticmethod
    def wall_time_to_filename():
        return "{}.json".format(time.strftime("%Y%m%d%H%M%S", time.localtime()))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        EnvelopeWidthEvaluator().run()
    except rospy.ROSInterruptException:
        pass
