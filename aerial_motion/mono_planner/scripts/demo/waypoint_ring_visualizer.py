#!/usr/bin/env python3

import copy
import math
import re
import threading

import rospy
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class WaypointRingVisualizer:
    RING_RADIUS_METERS = 0.4
    RING_THICKNESS_METERS = 0.02
    RING_SEGMENTS = 72
    DISCOVERY_PERIOD_SECONDS = 1.0
    WAYPOINT_TOPIC_PATTERN = re.compile(r"^/?waypoint/pose_(\d+)$")
    WAYPOINT_TOPIC_TYPE = "geometry_msgs/PoseStamped"
    POSE_EPSILON = 1e-9

    def __init__(self):
        rospy.init_node("waypoint_ring_visualizer", anonymous=False)

        self.lock = threading.RLock()
        self.pose_messages = {}
        self.topic_by_index = {}
        self.subscribers = {}
        self.ring_points = self._build_ring_points()
        self.ring_marker_publisher = rospy.Publisher("/waypoint/ring_markers", MarkerArray, queue_size=1, latch=True)
        self.discovery_timer = rospy.Timer(rospy.Duration(self.DISCOVERY_PERIOD_SECONDS), self.discover_waypoint_topics)

        self._publish_ring_markers()
        rospy.loginfo(
            "Waypoint ring visualizer ready: publishing /waypoint/ring_markers from discovered /waypoint/pose_* topics."
        )

    @staticmethod
    def _normalize_topic_name(topic_name):
        if topic_name.startswith("/"):
            return topic_name
        return "/" + topic_name

    @classmethod
    def _build_ring_points(cls):
        points = []
        for step in range(cls.RING_SEGMENTS + 1):
            angle = 2.0 * math.pi * float(step) / float(cls.RING_SEGMENTS)
            points.append(
                Point(
                    x=0.0,
                    y=cls.RING_RADIUS_METERS * math.cos(angle),
                    z=cls.RING_RADIUS_METERS * math.sin(angle),
                )
            )
        return points

    @classmethod
    def _poses_equal(cls, left_pose_msg, right_pose_msg):
        if left_pose_msg.header.frame_id != right_pose_msg.header.frame_id:
            return False

        left_values = [
            left_pose_msg.pose.position.x,
            left_pose_msg.pose.position.y,
            left_pose_msg.pose.position.z,
            left_pose_msg.pose.orientation.x,
            left_pose_msg.pose.orientation.y,
            left_pose_msg.pose.orientation.z,
            left_pose_msg.pose.orientation.w,
        ]
        right_values = [
            right_pose_msg.pose.position.x,
            right_pose_msg.pose.position.y,
            right_pose_msg.pose.position.z,
            right_pose_msg.pose.orientation.x,
            right_pose_msg.pose.orientation.y,
            right_pose_msg.pose.orientation.z,
            right_pose_msg.pose.orientation.w,
        ]
        return all(abs(left - right) <= cls.POSE_EPSILON for left, right in zip(left_values, right_values))

    def discover_waypoint_topics(self, _event):
        discovered_topics = {}
        for topic_name, topic_type in rospy.get_published_topics():
            if topic_type != self.WAYPOINT_TOPIC_TYPE:
                continue

            normalized_topic_name = self._normalize_topic_name(topic_name)
            match = self.WAYPOINT_TOPIC_PATTERN.match(normalized_topic_name)
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
                self.pose_messages.pop(waypoint_index, None)

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
                    self.waypoint_pose_cb,
                    callback_args=waypoint_index,
                    queue_size=1,
                )

            self.topic_by_index = discovered_topics

        rospy.loginfo("Discovered %d waypoint pose topics for ring visualization.", len(discovered_topics))
        self._publish_ring_markers()

    def waypoint_pose_cb(self, pose_msg, waypoint_index):
        with self.lock:
            previous_pose_msg = self.pose_messages.get(waypoint_index)
            if previous_pose_msg is not None and self._poses_equal(previous_pose_msg, pose_msg):
                return

            self.pose_messages[waypoint_index] = copy.deepcopy(pose_msg)

        self._publish_ring_markers()

    def _build_ring_markers(self, stamped_poses):
        marker_array = MarkerArray()
        stamp = rospy.Time.now()

        for waypoint_index, pose_msg in stamped_poses:
            marker = Marker()
            marker.header.frame_id = pose_msg.header.frame_id
            marker.header.stamp = stamp
            marker.ns = "waypoint_rings"
            marker.id = waypoint_index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(0.0)
            marker.pose.position.x = pose_msg.pose.position.x
            marker.pose.position.y = pose_msg.pose.position.y
            marker.pose.position.z = pose_msg.pose.position.z
            marker.pose.orientation.x = pose_msg.pose.orientation.x
            marker.pose.orientation.y = pose_msg.pose.orientation.y
            marker.pose.orientation.z = pose_msg.pose.orientation.z
            marker.pose.orientation.w = pose_msg.pose.orientation.w
            marker.scale.x = self.RING_THICKNESS_METERS
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.points = list(self.ring_points)
            marker_array.markers.append(marker)

        return marker_array

    def _publish_ring_markers(self):
        with self.lock:
            stamped_poses = [
                (waypoint_index, copy.deepcopy(self.pose_messages[waypoint_index]))
                for waypoint_index in sorted(self.pose_messages)
            ]

        delete_all_marker = Marker()
        delete_all_marker.action = Marker.DELETEALL
        self.ring_marker_publisher.publish(MarkerArray(markers=[delete_all_marker]))
        self.ring_marker_publisher.publish(self._build_ring_markers(stamped_poses))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        WaypointRingVisualizer().run()
    except Exception as exc:
        rospy.logerr("Failed to start waypoint ring visualizer: %s", exc)
        raise
