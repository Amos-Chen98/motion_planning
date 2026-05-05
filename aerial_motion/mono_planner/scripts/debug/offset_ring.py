#!/usr/bin/env python3

import copy
import re
import threading

import rospy
import tf.transformations as tf_trans
from geometry_msgs.msg import PoseStamped


class WaypointYOffsetPublisher:
    DEFAULT_INPUT_TOPIC_PATTERN = r"^/?waypoint/pose_(\d+)$"
    DEFAULT_OUTPUT_TOPIC_TEMPLATE = "/waypoint_offset/pose_{index}"
    DEFAULT_DISCOVERY_PERIOD_SEC = 1.0
    DEFAULT_OFFSET_M = 0.15
    WAYPOINT_TOPIC_TYPE = "geometry_msgs/PoseStamped"

    def __init__(self):
        rospy.init_node("offset_ring", anonymous=False)

        self.offset_m = float(rospy.get_param("~offset_m", self.DEFAULT_OFFSET_M))
        self.input_topic_pattern_text = rospy.get_param(
            "~input_topic_pattern",
            self.DEFAULT_INPUT_TOPIC_PATTERN,
        )
        self.output_topic_template = rospy.get_param(
            "~output_topic_template",
            self.DEFAULT_OUTPUT_TOPIC_TEMPLATE,
        )
        self.discovery_period_sec = float(
            rospy.get_param("~discovery_period_sec", self.DEFAULT_DISCOVERY_PERIOD_SEC)
        )
        if self.discovery_period_sec <= 0.0:
            raise ValueError("~discovery_period_sec must be greater than 0.")

        try:
            self.input_topic_pattern = re.compile(self.input_topic_pattern_text)
        except re.error as exc:
            raise ValueError(
                "Invalid ~input_topic_pattern '{}': {}".format(self.input_topic_pattern_text, exc)
            )

        self.lock = threading.RLock()
        self.subscribers_by_topic = {}
        self.publishers_by_index = {}
        self.topic_by_index = {}

        self.discover_waypoint_topics(None)
        self.discovery_timer = rospy.Timer(
            rospy.Duration(self.discovery_period_sec),
            self.discover_waypoint_topics,
        )

        rospy.loginfo(
            "Offset ring publisher ready: input pattern '%s', output template '%s', offset %.3f m.",
            self.input_topic_pattern_text,
            self.output_topic_template,
            self.offset_m,
        )

    @staticmethod
    def normalize_topic_name(topic_name):
        if topic_name.startswith("/"):
            return topic_name
        return "/" + topic_name

    def build_output_topic_name(self, waypoint_index):
        return self.output_topic_template.format(index=waypoint_index)

    def discover_waypoint_topics(self, _event):
        try:
            published_topics = rospy.get_published_topics()
        except rospy.ROSException as exc:
            rospy.logwarn_throttle(5.0, "Failed to query published topics: %s", exc)
            return

        discovered_topics = {}
        for topic_name, topic_type in published_topics:
            if topic_type != self.WAYPOINT_TOPIC_TYPE:
                continue

            normalized_topic_name = self.normalize_topic_name(topic_name)
            match = self.input_topic_pattern.match(normalized_topic_name)
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
                subscriber = self.subscribers_by_topic.pop(removed_topic_name, None)
                if subscriber is not None:
                    subscriber.unregister()

                publisher = self.publishers_by_index.pop(waypoint_index, None)
                if publisher is not None:
                    publisher.unregister()

            for waypoint_index, topic_name in discovered_topics.items():
                previous_topic_name = previous_topics.get(waypoint_index)
                if previous_topic_name == topic_name:
                    continue

                if previous_topic_name is not None:
                    subscriber = self.subscribers_by_topic.pop(previous_topic_name, None)
                    if subscriber is not None:
                        subscriber.unregister()

                if waypoint_index not in self.publishers_by_index:
                    output_topic_name = self.build_output_topic_name(waypoint_index)
                    self.publishers_by_index[waypoint_index] = rospy.Publisher(
                        output_topic_name,
                        PoseStamped,
                        queue_size=1,
                        latch=True,
                    )
                self.subscribers_by_topic[topic_name] = rospy.Subscriber(
                    topic_name,
                    PoseStamped,
                    self.waypoint_pose_cb,
                    callback_args=waypoint_index,
                    queue_size=1,
                )

            self.topic_by_index = discovered_topics

        rospy.loginfo(
            "Discovered %d waypoint pose topics for offset ring publishing.",
            len(discovered_topics),
        )

    def waypoint_pose_cb(self, pose_msg, waypoint_index):
        offset_pose_msg = self.build_offset_pose(pose_msg)

        with self.lock:
            publisher = self.publishers_by_index.get(waypoint_index)

        if publisher is not None:
            publisher.publish(offset_pose_msg)

    def build_offset_pose(self, pose_msg):
        offset_pose_msg = copy.deepcopy(pose_msg)
        quaternion = [
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        ]
        rotation_matrix = tf_trans.quaternion_matrix(quaternion)

        offset_pose_msg.pose.position.x += self.offset_m * rotation_matrix[0, 1]
        offset_pose_msg.pose.position.y += self.offset_m * rotation_matrix[1, 1]
        offset_pose_msg.pose.position.z += self.offset_m * rotation_matrix[2, 1]
        return offset_pose_msg

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        WaypointYOffsetPublisher().run()
    except Exception as exc:
        rospy.logerr("Failed to start offset ring publisher: %s", exc)
        raise
