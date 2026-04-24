#!/usr/bin/env python3

import json
import math
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped


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
    if segment_length_squared <= 1e-12:
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
            "Envelope width evaluator listening on %s and %s; start threshold %.3f m, return tolerance %.3f m, return rearm distance %.3f m, output dir %s",
            rospy.resolve_name(self.root_topic),
            rospy.resolve_name(self.last_link_topic),
            self.start_path_length_threshold_m,
            self.return_position_tolerance_m,
            self.return_rearm_distance_m,
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
        envelope_widths = [
            point_to_polyline_distance(sample.position, root_polyline)
            for sample in filtered_last_link_samples
        ]

        result = {
            "start_ros_time": self.time_to_dict(self.start_ros_time),
            "end_ros_time": self.time_to_dict(self.end_ros_time),
            "envelope_metric": {
                "mean": sum(envelope_widths) / float(len(envelope_widths)),
                "max": max(envelope_widths),
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
